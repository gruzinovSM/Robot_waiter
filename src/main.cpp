#include "AsyncStream.h"
#include "StringUtils.h"
#include "SoftwareSerial.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "math.h"

enum MOTOR
{
  MOTOR_1,
  MOTOR_2
};

struct WheelState
{
  int speed = 0;
  unsigned long startTime = 0;
  bool isAccelerating = false;
  bool isHighPwm = false;
  bool isBraking = false;
  unsigned long brakeStartTime = 0;
  int startAccelSpeed = 0;
  int targetPwm = 0; // Целевое значение PWM
  int boostPwm = 0;  // Пиковое значение PWM (targetPwm + DELTA_PWM)
  int direction = 1;
};

WheelState rightWheel;
WheelState leftWheel;

AsyncStream<100> serial(&Serial, '\n'); // указали Stream-объект и символ конца
void engine_handler(const int &speed, MOTOR motor);
void speed_conversion();
bool parsing_NRF();
void parsing_COM(char *buf);
int scaleToRange(int value, int start_val);
void scaleSpeeds(int &speedR, int &speedL);
void rotation(int &speedR, int &speedL);
void radioSetup();
int velocity_to_pwm(int vel, WheelState &state);
boolean oppositeDir(boolean state);

//      MOTORS
#define SPEED_M1_PIN 3
#define DIR_M1_PIN 8
#define EN_M1_PIN 3 // LOW - включен
#define SPEED_M2_PIN 6
#define DIR_M2_PIN 7
#define EN_M2_PIN 4

boolean doneParsing, startParsing;
int start_coordX = 500, start_coordY = 510;
int speedR, speedL;
int speedR_before_conv, speedL_before_conv;
bool rotation_flag = 0;
int time_start_rotation = 0;
String string_convert;

#define MAX_SPEED 180
#define MAX_SPEED_FORW 155
#define MAX_SPEED_BACK -175
#define R 0.085
#define L 0.52

RF24 radio(9, 10); // "создать" модуль на пинах 9 и 10 Для Уно

byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // возможные номера труб

int coord[2];

void setup()
{
  Serial.begin(115200); // connection with Jetson

  radioSetup(); // connection with NRF

  pinMode(SPEED_M1_PIN, OUTPUT);
  pinMode(DIR_M1_PIN, OUTPUT);
  pinMode(EN_M1_PIN, OUTPUT);
  pinMode(SPEED_M2_PIN, OUTPUT);
  pinMode(DIR_M2_PIN, OUTPUT);
  pinMode(EN_M2_PIN, OUTPUT);

  digitalWrite(DIR_M1_PIN, HIGH);
  digitalWrite(EN_M1_PIN, LOW);
  digitalWrite(DIR_M2_PIN, HIGH);
  digitalWrite(EN_M2_PIN, LOW);
}

void loop()
{
  bool exist_NRF = parsing_NRF();

  if (exist_NRF == 0) // нет сигнала с NRF
  {
    if (serial.available())
    {
      parsing_COM(serial.buf);
    }
    else
    {
      return;
    }
  }
  else if (exist_NRF != 1)
  {
    return;
  }

  engine_handler(speedL, MOTOR::MOTOR_1); // проверить направление (правое/левое колесо)
  engine_handler(speedR, MOTOR::MOTOR_2);
  /*
    speedR = 0;
    speedL = 0;
  */
}

void engine_handler(const int &speed, MOTOR motor)
{
  if (speed > 255 || speed < -255)
    return;

  uint8_t en_pin;
  uint8_t speed_pin;
  uint8_t dir_pin;
  if (motor == MOTOR::MOTOR_1)
  {
    en_pin = EN_M1_PIN;
    speed_pin = SPEED_M1_PIN;
    dir_pin = DIR_M1_PIN;
  }
  else if (motor == MOTOR::MOTOR_2)
  {
    en_pin = EN_M2_PIN;
    speed_pin = SPEED_M2_PIN;
    dir_pin = DIR_M2_PIN;
  }

  if (speed == 0)
  {
    analogWrite(speed_pin, 0);
  }
  else if (speed > 0)
  {
    digitalWrite(dir_pin, HIGH); // проверить направление (правое/левое колесо)
    analogWrite(speed_pin, speed);
  }
  else
  {
    digitalWrite(dir_pin, LOW);
    analogWrite(speed_pin, -speed);
  }
}

/*
1. Получаем пакет (линейная скорость; угловая скорость) в диапазоне [-100; 100]
2. Пересчитываем в скорости правого и левого колеса
3. При выходе за границы максимально допустимых значений скорости пропорционально масштабируем их
*/
void parsing_COM(char *buf)
{
  int i = 0;
  int dutyX, dutyY;

  while (buf[i])
  {
    char incomingChar = buf[i];
    if (startParsing)
    {
      if (incomingChar == ' ') // принят пакет speed
      {
        dutyX = string_convert.toInt(); // конвертируем принятый пакет в переменную
        string_convert = "";            // очищаем переменную пакета
      }
      else if (incomingChar == ';') // принят пакет dir
      {
        dutyY = string_convert.toInt(); // конвертируем принятый пакет в переменную
        string_convert = "";            // очищаем переменную пакета
        startParsing = false;           // закончить принятие пакетов
        doneParsing = true;             // парсинг окончен, можно переходить к движению
      }
      else
      {
        string_convert += incomingChar; // записываем  принятые данные в переменную
      }
    }

    if (incomingChar == '$') // начало парсинга
    {
      startParsing = true; // начать принятие пакетов
    }
    i++;
  }

  /*
  нужно написать алгоритм, который будет при отправке с Jetson значений скорости
  (скорость находится в окрестности нулевой скорости) плавно(???) выставлять
  ШИМ на конкретном колесе равный 145 (мб 150) и аналогичная задача для торможения

  плавность мб реализована заданием константы TIME_ACCELERATION = 1 секунда
  (соотв 5-7 обновлением информации о препятствиях)

  возможно нужно распределить по условиям движение вперед-назад и вправо-влево
  */

  int velR = ((dutyX + dutyY * L / 2.0)); // / 100.0;     //100 - константа, на которую умножалось значение скорости на Jetson
  int velL = ((dutyX - dutyY * L / 2.0)); // / 100.0;

  speedR = velocity_to_pwm(velR, rightWheel);
  speedL = velocity_to_pwm(velL, leftWheel);
}

const int DELTA = 4;
const int HYSTERESIS = 2;                    // Ширина зоны гистерезиса
const int DIRECTION_HYSTERESIS = 5;          // Гистерезис направления
const unsigned long TIME_ACCELERATION = 750; // 1 секунда в миллисекундах            при 750 - хорошо
const unsigned long TIME_HIGH_PWM = 300;     // 0.5 секунды в миллисекундах           при 250 - хорошо
const int MAX_PWM_FWD = 160;                 // Максимальный ШИМ вперед         
const int MID_PWM_FWD = 150;                 // Рабочий ШИМ вперед
const int MAX_PWM_REV = 160;                 // Максимальный ШИМ назад
const int MID_PWM_REV = 150;                 // Рабочий ШИМ назад
const int DELTA_PWM = 10;                    // Превышение над целевой скоростью при разгоне

/**
 * @brief преобразование скорости вращения колес в ШИМ;
 *        если значение vel находится в DELTA окрестности нуля, тогда соответствующее значение speed равно 0
 *        при получении значения vel не в DELTA окрестности нуля,
 *        будет плавно (в течении TIME_ACCELERATION) изменяться значение speed (ШИМ на соответствующее колесо) от 0 до MAX_PWM
 *        и такое значение speed будет держаться время TIME_HIGH_PWM
 *        после значение speed будет равно MID_PWM
 *
 * @param vel линейная скорость вращения колеса в диапазоне [-100, 100]
 * @return int значение ШИМ для соответствующего колеса
 */
int velocity_to_pwm(int vel, WheelState &state)
{
  bool isForward = (vel >= 0);
  int absVel = abs(vel);

  int MAX_PWM = isForward ? MAX_PWM_FWD : MAX_PWM_REV;
  int MID_PWM = isForward ? MID_PWM_FWD : MID_PWM_REV;

  // Определяем направление
  int newDirection = (vel >= 0) ? 1 : -1;

  // Проверяем, изменилось ли направление
  if (newDirection != state.direction)
  {
    // Если направление изменилось, начинаем торможение
    state.direction = newDirection;
    state.isAccelerating = false;
    state.isHighPwm = false;
    state.isBraking = true;
    state.startTime = millis();
  }

  // Проверяем, находится ли vel в окрестности нуля
  bool isNearZero = (abs(vel) <= DELTA);

  if (!isNearZero)
  {
    // Если vel не в окрестности нуля, вычисляем targetPwm и boostPwm
    state.targetPwm = map(abs(vel), 0, 52, MID_PWM, MAX_PWM);
    state.boostPwm = state.targetPwm + DELTA_PWM;
    state.boostPwm = constrain(state.boostPwm, 0, MAX_PWM);
    // Если не в процессе разгона/торможения, начинаем разгон
    if (!state.isAccelerating && !state.isBraking && !state.isHighPwm)
    {
      state.isAccelerating = true;
      state.startTime = millis();
    }
  }
  else
  {
    // Если vel в окрестности нуля, начинаем торможение
    if (!state.isBraking && (state.speed != 0))
    {
      state.isAccelerating = false;
      state.isHighPwm = false;
      state.isBraking = true;
      state.startAccelSpeed = state.speed;
      state.startTime = millis();
    }
  }

  // Вычисляем текущее время
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - state.startTime;

  if (state.isAccelerating)
  {
    // Плавный разгон от 0 до boostPwm
    if (elapsedTime < TIME_ACCELERATION)
    {
      float progress = (float)elapsedTime / TIME_ACCELERATION;
      state.speed = (int)(state.boostPwm * progress);
    }
    else
    {
      state.speed = state.boostPwm;
      state.isAccelerating = false;
      state.isHighPwm = true;
      state.startTime = currentTime;
    }
  }
  else if (state.isHighPwm)
  {
    // Удержание boostPwm в течение TIME_HIGH_PWM
    if (elapsedTime >= TIME_HIGH_PWM)
    {
      state.isHighPwm = false;
      // После удержания boostPwm переходим к targetPwm
      state.speed = state.targetPwm;
    }
  }
  else if (state.isBraking)
  {
    // Плавное торможение от текущего speed до 0
    if (elapsedTime < TIME_ACCELERATION)
    {
      float progress = (float)elapsedTime / TIME_ACCELERATION;
      state.speed = state.startAccelSpeed - (int)(state.startAccelSpeed * progress);
    }

    else
    {
      state.speed = 0;
      state.isBraking = false;
    }
  }

  // Применяем направление
  int outputSpeed = state.speed * state.direction;

  return outputSpeed;
}

/*
1. Получаем пакет состоящий из двух значений (координата по Ох; координата по Оу) в диапазоне [0; 1023]
2. Преобразуем его в диапазон [-160; 170]
3. Пересчитываем в скорости правого и левого колеса
4. При выходе за границы максимально допустимых значений скорости пропорционально масштабируем их
*/
bool parsing_NRF()
{
  bool exist_parsing = 0;
  byte pipeNo;
  int coordX, coordY;

  if (radio.available(&pipeNo)) // слушаем эфир со всех труб
  {
    exist_parsing = 1;

    radio.read(&coord, sizeof(coord));

    coordX = scaleToRange(coord[0], start_coordX);
    coordY = scaleToRange(coord[1], start_coordY);

    speedR_before_conv = coordX + coordY / 2.5;
    speedL_before_conv = coordX - coordY / 2.5;

    speedR = speedR_before_conv;
    speedL = speedL_before_conv;

    scaleSpeeds(speedR, speedL);

    // speed_conversion();

    // rotation(speedR, speedL);       //опционально
  }

  return exist_parsing;
}

void rotation(int &speedR, int &speedL)
{
  rotation_flag = 0;

  if (speedR < -50 && speedL > 50)
  {
    rotation_flag = 1;
    time_start_rotation = millis();

    speedR = 160;
    speedL = -160;
  }
  else if (speedL < -50 && speedR > 50)
  {
    rotation_flag = 1;
    time_start_rotation = millis();

    speedL = 160;
    speedR = -160;
  }
}

// Функция для масштабирования значения из диапазона 0–1023 в -160–170
int scaleToRange(int val, int start_val)
{
  float scaledVal;

  if (val >= start_val)
  {
    scaledVal = (float)(val - start_val) / (1023 - start_val) * MAX_SPEED_FORW;
  }
  else
  {
    scaledVal = (float)(start_val - val) / start_val * MAX_SPEED_BACK;
  }
  // Округление до целого числа
  return (int)scaledVal;
}

// Функция для пропорционального масштабирования скоростей
// Используется, если какое-либо из значение выходит за пределы максимально допустимых
void scaleSpeeds(int &speedR, int &speedL)
{
  // Находим максимальное значение по модулю
  int maxSpeed = max(speedR, speedL);
  int minSpeed = min(speedR, speedL);

  // Если максимальное значение выходит за пределы диапазона
  if (maxSpeed > MAX_SPEED_FORW)
  {
    // Коэффициент масштабирования
    float scaleFactor = MAX_SPEED_FORW / (float)maxSpeed;

    // Масштабируем оба значения
    speedR = speedR * scaleFactor;
    speedL = speedL * scaleFactor;
  }

  if (minSpeed < MAX_SPEED_BACK)
  {
    // Коэффициент масштабирования
    float scaleFactor = MAX_SPEED_BACK / (float)minSpeed;

    // Масштабируем оба значения
    speedR = speedR * scaleFactor;
    speedL = speedL * scaleFactor;
  }
}

void radioSetup()
{
  radio.begin();            // активировать модуль
  radio.setAutoAck(1);      // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);  // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload(); // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32); // размер пакета, в байтах

  radio.openReadingPipe(1, address[0]); // хотим слушать трубу 0
  radio.setChannel(0x60);               // выбираем канал (в котором нет шумов!)

  radio.setPALevel(RF24_PA_MAX); // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate(RF24_2MBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  // должна быть одинакова на приёмнике и передатчике!
  // при самой низкой скорости имеем самую высокую чувствительность и дальность!!

  radio.powerUp();        // начать работу
  radio.startListening(); // начинаем слушать эфир, мы приёмный модуль
}
