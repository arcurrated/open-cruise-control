/*
Программа предназначена для удержания скорости автомобиля.
В качестве датчика скорости - запрос в CAN-шину,
В качестве исполнительного элемента - положение дроссельной заслонки.

Коэффициенти ПИ-контроллера подобраны эмпирически.
*/

// Работоспособность пока полностью не проверена

#include <SPI.h>
#include <mcp2515.h>
#include <EEPROM.h>

struct CalibrationData {
    uint8_t ch1Min; // Минимальное напряжение на первом канале
    uint8_t ch1Max; // Максимальное напряжение на первом канале

    uint8_t ch2Min;
    uint8_t ch2Max;

    // Выключить симуляцию если напряжение на каналах будет выше
    uint8_t ch1CutOff;
    uint8_t ch2CutOff;
};

#define CUT_OFF_OFFSET 20 // ch1CutOff = ch1Max - CUT_OFF_OFFSET

#define ANALOG_THRESHOLD 100 // Для срабатывания кнопок (они висят на аналоговых входах)

#define CALIBRATION_TIME_MS 3000
#define REQUEST_SPEED_INTERVAL_MS 100
#define SPEED_AVG_NUM 5

#define CLICK_BUTTON_PRESS_MS 500
#define HOLD_BUTTON_PRESS_MS 2000

// Состояния системы
#define STATE_IDLE 0
#define STATE_CRUISE 1
#define STATE_CALIBRATION 2

#define BUTTON_PIN A0
#define CH1_THROTTLE_FEEDBACK_PIN A1
#define CH2_THROTTLE_FEEDBACK_PIN A2
#define BRAKE_PEDAL_PIN A3

#define CH1_THROTTLE_OUT 5
#define CH2_THROTTLE_OUT 6
#define SIGNAL_LIGHT_PIN 3 // PWM

#define CAN_SPEED_CODE 0x0D
#define I_KOEF 0.06 // PI-controller
#define P_KOEF 0.05 // PI-controller

// граничные значения интегратора нужны для того, чтобы он работал в том диапазоне,
// в котором он может влиять на положение дроссельной заслонки.
// например, если не установить нижнюю границу, то при увеличении скорости
// ошибка будет отрицательной - интегратор уйдет в отрицательную область и это не повлияет на дроссельную заслонку
// теперь, чтобы он опять стал равен 0 нужно будет некоторое время держать положительную ошибку
// этого можно избежать, установив границы для интегратора
#define THROTTLE_BIAS 0.1 // дроссельная заслонка должна быть немного приоткрыта для отсутствия торможения двигателем
#define I_LOW_BOUNDARY -THROTTLE_BIAS/I_KOEF // значение интегратора не может быть меньше
#define I_HIGH_BOUNDARY (1.0-THROTTLE_BIAS)/I_KOEF // значение интегратора не может быть выше



#define PERMANENT_CALL_ERROR_MS 50


MCP2515 mcp2515(10); // CAN-iface
struct can_frame canMsg;
CalibrationData calibrationData;


unsigned long previousMillis = 0;
unsigned long previousIntegratorMillis = 0;
double integrator = 0;
int targetSpeed = 0;

bool buttonPressed = false;

bool buttonClicked = false;
bool buttonHolded = false;

uint8_t state = STATE_IDLE;

void setup() {
  Serial.begin(115200);
  if(mcp2515.reset() != MCP2515::ERROR_OK){
    Serial.println("Error reset");
    while(1);
  }
  
  if(mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK){
    Serial.println("Error bitrate");
    while(1);
  }

  if(mcp2515.setNormalMode() != MCP2515::ERROR_OK){
    Serial.println("Error mode");
    while(1);
  }

  // прочитать данные калибровки
  EEPROM.get(0, calibrationData);

  if(calibrationData.ch1Max == 0 || calibrationData.ch1Min == 0){
    startCalibration();
  }
}

void loop() {
  // обработка состояния кнопки
  int buttonVal = analogRead(BUTTON_PIN);
  if(buttonVal > ANALOG_THRESHOLD){
    if(!buttonPressed){
      // возможно, сейчас нажимается кнопка
      delay(10);
      buttonVal = analogRead(BUTTON_PIN);
      if(buttonVal > ANALOG_THRESHOLD){
        // да, кнопка нажата
        previousMillis = millis();
        buttonPressed = true;
      }
    } else {
      unsigned long holdTime = millis() - previousMillis;
      if(holdTime > HOLD_BUTTON_PRESS_MS){
        buttonHolded = true;
      }
    }

    buttonPressed = true;
  } else {
    if(buttonPressed){
      // возможно, сейчас отпускается кнопка
      delay(10);
      if(analogRead(BUTTON_PIN) < ANALOG_THRESHOLD){
        // да, кнопка отпущена
        unsigned long holdTime = millis() - previousMillis;
        if(holdTime <  CLICK_BUTTON_PRESS_MS){
          buttonClicked = true;
        }
      }
    }

    buttonPressed = false;
  }

  switch(state){
    case STATE_IDLE: idleLoop(); break;
    case STATE_CRUISE: cruiseLoop(); break;
    case STATE_CALIBRATION: calibrationLoop(); break;
  }

  // Сбрасываем рабочие переменные кнопки
  buttonClicked = false;
  buttonHolded = false;
}

/*
Свободный ход системы
*/
void idleLoop(){
  // Проверка на нажатие кнопки включения круиз-контроля
  if(buttonClicked){ // enable cruise
    blink(SIGNAL_LIGHT_PIN, 1, 100);

    targetSpeed = 0; // reset speed
    integrator = 0; // reset integrator
    uint8_t _speed;
    bool result;

    uint8_t i = 0;
    uint8_t j = 0;

    // Лучше усреднить, потому что информация 
    // о скорости может быть устаревшей
    while(i < SPEED_AVG_NUM && j < 100){
      j++;

      result = requestSpeed(&_speed);
      if(result){
        i++;
        targetSpeed += _speed;
      }
    }

    // Если всё прошло предсказуемо
    if(i != 0){
      targetSpeed = targetSpeed/i;

      digitalWrite(SIGNAL_LIGHT_PIN, HIGH);
      state = STATE_CRUISE;
    }
  }

  if(buttonHolded){
    startCalibration();
  }
}

/*
Цикл управления круизом
*/
void cruiseLoop(){
  // Проверка на нажатие педали тормоза (отключение круиз-контроля)
  if(analogRead(BRAKE_PEDAL_PIN) > ANALOG_THRESHOLD){ // disable cruise
    digitalWrite(SIGNAL_LIGHT_PIN, LOW);

    // Обнуление ШИМ-сигнала
    analogWrite(CH1_THROTTLE_OUT, 0);
    analogWrite(CH2_THROTTLE_OUT, 0);

    state = STATE_IDLE;
    return;
  }

  // Обновление положения дроссельной заслонки
  controlLoop();

  // Выключение симуляции при превышении значения на выходе
  uint8_t ch1Feedback = analogRead(CH1_THROTTLE_FEEDBACK_PIN)/4;
  delay(20);
  uint8_t ch2Feedback = analogRead(CH2_THROTTLE_FEEDBACK_PIN)/4;
  if(ch1Feedback > calibrationData.ch1CutOff 
    || ch2Feedback > calibrationData.ch2CutOff){
    analogWrite(CH1_THROTTLE_OUT, 0);
    analogWrite(CH2_THROTTLE_OUT, 0);
  }
}

/*
Цикл управления дроссельной заслонкой
*/
void controlLoop(){
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= REQUEST_SPEED_INTERVAL_MS) {
    // 1. извлечение данных о текущей скорости
    uint8_t speed;
    bool result = requestSpeed(&speed);
    if(!result){ return; }

    previousMillis = currentMillis;

    // 2. получение выхода системы автоматического регулирования (0..1)
    float throttlePosition = forwardACS(speed);

    // 3. переход к необходимому уровню ШИМ исходя из граничных значений
    uint8_t throttlePositionInt = throttlePosition*255;

    int ch1Out = map(throttlePositionInt, 0, 255, 
        calibrationData.ch1Min, calibrationData.ch1Max);

    int ch2Out = map(throttlePositionInt, 0, 255, 
        calibrationData.ch2Min, calibrationData.ch2Max);

    // 4. вывод ШИМ на пины
    analogWrite(CH1_THROTTLE_OUT, ch1Out);
    analogWrite(CH2_THROTTLE_OUT, ch2Out);
  }
}

/*
Функция для запроса данных о скорости у ЭБУ
*/
bool requestSpeed(uint8_t *speed){
  struct can_frame request;
  request.can_id = 0x7DF; // Бродкастный адрес
  request.can_dlc = 8;
  request.data[0] = 0x02; // Длина данных
  request.data[1] = 0x01; // Режим 01
  request.data[2] = CAN_SPEED_CODE; // speed 
  mcp2515.sendMessage(&request);

  while (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(canMsg.can_id == 0x7E8){
      // извлечение данных о текущей скорости
      *speed = canMsg.data[3];
      return true;
    }
  }

  return false;
}


/*
Цикл ПИ-контроллера
Возвращает число с плавающей точкой от 0 до 1
*/
float forwardACS(int currentSpeed){
  unsigned long currentMillis = millis();
  unsigned long deltaTime = currentMillis - previousIntegratorMillis;
  
  if(deltaTime < PERMANENT_CALL_ERROR_MS){ return; }

  previousIntegratorMillis = currentMillis;
  
  int error = targetSpeed - currentSpeed;
  
  integrator += error * (deltaTime/1000.0);
  if(integrator > I_HIGH_BOUNDARY){
    integrator = I_HIGH_BOUNDARY;
  }
  if(integrator < I_LOW_BOUNDARY){
    integrator = I_LOW_BOUNDARY;
  }

  float value = THROTTLE_BIAS + (float)error * P_KOEF + integrator * I_KOEF; // TF

  if(value < 0) { value = 0; }
  if(value > 1) { value = 1; }

  return value;
}


/*
Начать калибровку
*/
void startCalibration(){
  blink(SIGNAL_LIGHT_PIN, 2, 100);

  state = STATE_CALIBRATION;
  previousMillis = millis();
  calibrationData.ch1Min = 255;
  calibrationData.ch1Max = 0;

  calibrationData.ch2Min = 255;
  calibrationData.ch2Max = 0;
}

/*
Цикл калибровки
*/
void calibrationLoop(){
  uint8_t ch1Value = analogRead(CH1_THROTTLE_FEEDBACK_PIN)/4;
  uint8_t ch2Value = analogRead(CH2_THROTTLE_FEEDBACK_PIN)/4;

  // Демонстрация корректного прохождения калибровки
  if(ch1Value > ch2Value){
    analogWrite(SIGNAL_LIGHT_PIN, ch1Value);
  } else {
    analogWrite(SIGNAL_LIGHT_PIN, ch2Value);
  }

  if(ch1Value > calibrationData.ch1Max){
    calibrationData.ch1Max = ch1Value;
  }
  if(ch1Value < calibrationData.ch1Min){
    calibrationData.ch1Min = ch1Value;
  }

  if(ch2Value > calibrationData.ch2Max){
    calibrationData.ch2Max = ch1Value;
  }
  if(ch2Value < calibrationData.ch2Min){
    calibrationData.ch2Min = ch1Value;
  }

  if(millis() - previousMillis > CALIBRATION_TIME_MS){
    // Время калибровки прошло, записываем значения
    calibrationData.ch1CutOff = calibrationData.ch1Max - CUT_OFF_OFFSET;
    calibrationData.ch2CutOff = calibrationData.ch2Max - CUT_OFF_OFFSET;

    // записать данные калибровки
    EEPROM.put(0, calibrationData);

    // Выходим из цикла калибровки
    state = STATE_IDLE;

    blink(SIGNAL_LIGHT_PIN, 3, 100);
  }
}

/*
Поморгать цифровым выходом
*/
void blink(uint8_t pin, uint8_t count, int interval){
  for(uint8_t i = 0; i < count; i++){
    digitalWrite(pin, 1);
    delay(interval);
    digitalWrite(pin, 0);
    delay(interval);
  }
}