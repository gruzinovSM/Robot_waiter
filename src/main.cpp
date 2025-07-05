#include "AsyncStream.h"
#include "StringUtils.h"
#include "SoftwareSerial.h"
#include "nRF24L01.h"
#include "RF24.h"

enum MOTOR {
  MOTOR_1,
  MOTOR_2
};

AsyncStream<100> serial(&Serial, '\n');  // указали Stream-объект и символ конца
void engine_handler(const int &speed, MOTOR motor);
void speed_conversion();
bool parsing_NRF();
void parsing_COM(char* buf);
int scaleToRange(int value, int start_val);
void scaleSpeeds(int &speedR, int &speedL);
void rotation(int &speedR, int &speedL);
void radioSetup();
int speed_to_pwm(double &speed);
boolean oppositeDir(boolean state);

//      MOTORS
#define SPEED_M1_PIN 3
#define DIR_M1_PIN 8
#define EN_M1_PIN 3  //LOW - включен
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


RF24 radio(9, 10);  // "создать" модуль на пинах 9 и 10 Для Уно

byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"};     //возможные номера труб

int coord[2];


void setup()
{
  Serial.begin(115200);   //connection with Jetson

  radioSetup();           //connection with NRF

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

  if (exist_NRF == 0)               //нет сигнала с NRF
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

  engine_handler(speedL, MOTOR::MOTOR_1);       //проверить направление (правое/левое колесо)
  engine_handler(speedR, MOTOR::MOTOR_2);

  speedR = 0;
  speedL = 0;
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
    digitalWrite(dir_pin, HIGH);        //проверить направление (правое/левое колесо)
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
void parsing_COM(char* buf)
{
  int i = 0;
  int dutyX, dutyY;
  while (buf[i])
  {
    char incomingChar = buf[i];
    if (startParsing)
    {
      if (incomingChar == ' ')  // принят пакет speed
      {
        dutyX = string_convert.toInt();  // конвертируем принятый пакет в переменную
        string_convert = "";          // очищаем переменную пакета
      } 
      else if (incomingChar == ';')  // принят пакет dir
      {
        dutyY = string_convert.toInt();  // конвертируем принятый пакет в переменную
        string_convert = "";             // очищаем переменную пакета
        startParsing = false;            // закончить принятие пакетов
        doneParsing = true;              // парсинг окончен, можно переходить к движению
      } 
      else 
      {
        string_convert += incomingChar;  // записываем  принятые данные в переменную
      }
    }

    if (incomingChar == '$')  // начало парсинга
    {
      startParsing = true;  // начать принятие пакетов
    }
    i++;
  }

  if (dutyX <= 5 && dutyX >= -5 && dutyY <= 5 && dutyY >= -5)
  {
    speedR = 0;
    speedL = 0;

    return;
  }

  double speedR_as_speed = ((double)((double)dutyX * 0.75 + dutyY * L / 2.0) / 100.0);   // / 100.0;     //100 - константа, на которую умножалось значение скорости на Jetson
  double speedL_as_speed = ((double)((double)dutyX * 0.75 - dutyY * L / 2.0) / 100.0);   // / 100.0;

  speedR = speed_to_pwm(speedR_as_speed);
  speedL = speed_to_pwm(speedL_as_speed);

/*
  speedR_before_conv = (dutyX + dutyY * L / 2) * 35 / 100 + 160;
  speedL_before_conv = (dutyX - dutyY * L / 2) * 35 / 100 + 160;
*/

/*
  speedR = dutyX + ((float)dutyY * L) / 2;
  speedL = dutyY - ((float)dutyY * L) / 2;
*/
  //scaleSpeeds(speedR, speedL);
}

int speed_to_pwm(double &speed)
{
  int pwm_max, pwm_min;
  double vel_max = 0.26, vel_min = 0.0;   //vel_max = 0.5

  if (speed > 0)
  {
    pwm_min = 145;
    pwm_max = 155;
  }
  else if (speed < 0)
  {
    pwm_min = -165;
    pwm_max = -175;
  }

  int pwm_speed = ((max(pwm_max, pwm_min) - min(pwm_max, pwm_min)) / (vel_max - vel_min)) * speed + pwm_min;

  return pwm_speed;
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

  if (radio.available(&pipeNo))          // слушаем эфир со всех труб
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

    //speed_conversion();

    //rotation(speedR, speedL);       //опционально
  
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
  if (maxSpeed > MAX_SPEED_FORW) {
    // Коэффициент масштабирования
    float scaleFactor = MAX_SPEED_FORW / (float)maxSpeed;

    // Масштабируем оба значения
    speedR = speedR * scaleFactor;
    speedL = speedL * scaleFactor;
  }
  
  if (minSpeed < MAX_SPEED_BACK) {
    // Коэффициент масштабирования
    float scaleFactor = MAX_SPEED_BACK / (float)minSpeed;

    // Масштабируем оба значения
    speedR = speedR * scaleFactor;
    speedL = speedL * scaleFactor;
  }
}

void radioSetup()
{
  radio.begin();              // активировать модуль
  radio.setAutoAck(1);        // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);    // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload();   // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);   // размер пакета, в байтах

  radio.openReadingPipe(1, address[0]);   // хотим слушать трубу 0
  radio.setChannel(0x60);     // выбираем канал (в котором нет шумов!)

  radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_2MBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!

  radio.powerUp();        // начать работу
  radio.startListening(); // начинаем слушать эфир, мы приёмный модуль
}
