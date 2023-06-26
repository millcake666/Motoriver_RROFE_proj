// подключаем библиотеки
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>


// false - нормальная работа программы, true - просто вывод нданных с датчиков
#define JustPrintData false

// директивы для мотора
#define motorPinA 3
#define motorPinB 5
#define FOLLOW false
#define BREAK true
#define FORWARD true
#define BACKWARD false

// пин светодиода
#define RedLedPin A2
#define GreenLedPin A3
#define BuzzPin 13

#define RED 0
#define GREEN 1
#define OFF 2

// servo pin
#define ServoPin 11

// пин кнопки
#define ButtonPin 2

// указываем пины ультразвуков
#define trigPinL 10
#define echoPinL 8
#define trigPinR 9
#define echoPinR 7

// указываем пины общения с разбери
#define RPiPinReserved 6
#define RPiPinReady 4
#define RPiPinRed A1
#define RPiPinGreen A0

// разброс значений датчика цвета при определении цвета линии
#define COLOR_THRESHOLD 100

// коэффициенты регуляторов
#define kP 0.8
#define kD 0.3
#define kPCenter 1.
#define kDCenter 0.4

// уменьшение угла поворота сервы при езде
#define lessTurn 10

// экзменпляр класса для управления сервоприводом
Servo servo;

// минимальный и максимальный угол поворота колес
const int ServoMinAngle = 48;
const int ServoMaxAngle = 120;

// экземпляр класса для датчика цвета
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_60X);

// цвета линий на поле
const int lineBlue[4] = {490, 760, 971, 1024};
const int lineSemiBlue[4] = {524, 833, 1024, 1024};

const int lineOrange[4] = {836, 861, 747, 1024};
const int lineSemiOrange[4] = {870, 980, 900, 1024};

const int lineWhite[4] = {1024, 1024, 1024, 1024};

// направление движения в этом раунде true - против часовой / false - по часовой
bool roundDirection = false;

// счетчик поворотов
byte turnCounter = 0;

// предыдущая ошибка ПД регулятора
float oldError = 0;
float oldErrorCenter = 0;

// храним время со старта
long startTime = millis();

// скорость мотора
int targetSpeed = 244;

#define targetDistCenter 500
#define targetDistDetour 200

// последний найденный маркер (-1 - нет маркера; 0 - красный; 1 - зеленый)
byte lastMarker = -1;

// флаг того, что сменили направлени (развернулись после второго круга)
bool isChangeDirection = false;


// получить значение с правого датчика (в мм)
int getDistanceR() {
  //  Clears the trigPin
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long durationR = pulseIn(echoPinR, HIGH, 12000);
  int distanceR = durationR * 0.034 / 0.2;

  // Return the distance
  return (distanceR == 0) ? 1000 : distanceR;
}


// получить значение с левого датчика (в мм)
int getDistanceL() {
  // Clears the trigPin
  digitalWrite(trigPinL, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long durationL = pulseIn(echoPinL, HIGH, 12000);
  int distanceL = durationL * 0.034 / 0.2;

  // Return the distance
  return (distanceL == 0) ? 1000 : distanceL;
}


// настройка пинов ультразвуков
void USPinsBegin() {
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
}


// настройка пинов общения с камерой
void RPiPinBegin() {
  pinMode(RPiPinReserved, OUTPUT);
  pinMode(RPiPinReady, INPUT);
  pinMode(RPiPinRed, INPUT);
  pinMode(RPiPinGreen, INPUT);
}


// определяем какой линии соответствует полученный RGB спектр (0 - синий, 1 - оранжевый, 2 - белое поле)
int getLineColor() {
  // переменные для хранения считанного цветa
  int red = 0;
  int green = 0;
  int blue = 0;
  int contrast = 0;

  tcs.getRawData(&red, &green, &blue, &contrast);  // получаем данные с датчика цвета

  if ((abs(lineBlue[0] - red) <= COLOR_THRESHOLD &&
       abs(lineBlue[1] - green) <= COLOR_THRESHOLD &&
       abs(lineBlue[2] - blue) <= COLOR_THRESHOLD &&
       abs(lineBlue[3] - contrast) <= COLOR_THRESHOLD) ||
      (abs(lineSemiBlue[0] - red) <= COLOR_THRESHOLD &&
       abs(lineSemiBlue[1] - green) <= COLOR_THRESHOLD &&
       abs(lineSemiBlue[2] - blue) <= COLOR_THRESHOLD &&
       abs(lineSemiBlue[3] - contrast) <= COLOR_THRESHOLD)) {
    return 0;
  }

  if ((abs(lineOrange[0] - red) <= COLOR_THRESHOLD &&
       abs(lineOrange[1] - green) <= COLOR_THRESHOLD &&
       abs(lineOrange[2] - blue) <= COLOR_THRESHOLD &&
       abs(lineOrange[3] - contrast) <= COLOR_THRESHOLD) ||
      (abs(lineSemiOrange[0] - red) <= COLOR_THRESHOLD &&
       abs(lineSemiOrange[1] - green) <= COLOR_THRESHOLD &&
       abs(lineSemiOrange[2] - blue) <= COLOR_THRESHOLD &&
       abs(lineSemiOrange[3] - contrast) <= COLOR_THRESHOLD)) {
    return 1;
  }

  if (abs(lineWhite[0] - red) <= COLOR_THRESHOLD &&
      abs(lineWhite[1] - green) <= COLOR_THRESHOLD &&
      abs(lineWhite[2] - blue) <= COLOR_THRESHOLD &&
      abs(lineWhite[3] - contrast) <= COLOR_THRESHOLD) {
    return 2;
  }
}


// настройка пинов мотора
void motorPinBegin() {
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
}


// включение моторов на скорость (от 0 до 255), в направлении (true - вперед, false - назад)
void go(int speed, bool direction) {
  if (direction) {
    analogWrite(motorPinA, speed);
    digitalWrite(motorPinB, LOW);
  } else {
    digitalWrite(motorPinA, LOW);
    analogWrite(motorPinB, speed);
  }
}


// остановка моторов (true - резкая, false - накатом)
void stop(bool var) {
  if (var) {
    digitalWrite(motorPinA, HIGH);
    digitalWrite(motorPinB, HIGH);
  } else {
    digitalWrite(motorPinA, LOW);
    digitalWrite(motorPinB, LOW);
  }
}


// устанавливаем угол поворота колес (угол от -90 до +90 | уменьшение максимального угла на ... значение)
void setServoAngle(int angle, int less = 0) {
  // если значение выходит за пределы, то ограничиваем его
  if (angle > 90) angle = 90;
  else if (angle < -90) angle = -90;

  // преобразуем значение с учетом ограничения и подаем на серву
  servo.write(map(angle, -90, 90, ServoMinAngle + less, ServoMaxAngle - less));
}


// функция езды между стенками
void rideCenter(int speed) {
  go(speed, FORWARD);  // запускаем мотор
  // получаем значения с датчиков дистанции
  int distR = getDistanceR();
  int distL = getDistanceL();

  int error = distR - distL;  // ошибка Р
  int PD = error * kPCenter + (error - oldErrorCenter) * kDCenter;  // ошибка Р + ошибка D
  oldErrorCenter = error;  // сохраняем ошибку

  // подаем управляющее воздействие на серву
  setServoAngle(PD);
}


// просто вывод значений датчиков
void manyPrints() {
  // переменные для хранения считанного цветa
  int red = 0;
  int green = 0;
  int blue = 0;
  int contrast = 0;

  while (true) {
    Serial.print("Right US: " + String(getDistanceR()) + " | Left US: " + String(getDistanceL()));
    tcs.getRawData(&red, &green, &blue, &contrast);  // получаем данные с датчика цвета
    Serial.print(" || Red: ");
    Serial.print((int) red);
    Serial.print(" | Green: ");
    Serial.print((int) green);
    Serial.print(" | Blue: ");
    Serial.print((int) blue);
    Serial.print(" | Contrast: ");
    Serial.print((int) contrast);
    Serial.print(" || Color: ");
    Serial.print(getLineColor());
    Serial.print(" || ButtonL: ");
    Serial.print(digitalRead(ButtonPin));
    Serial.print(" || RPi_Pin_Red: ");
    Serial.print(digitalRead(RPiPinRed));
    Serial.print(" || RPi_Pin_Green: ");
    Serial.print(digitalRead(RPiPinGreen));
    Serial.print(" || RPi_Pin_Ready: ");
    Serial.println(digitalRead(RPiPinReady));
  }
}


// разворот после двух кругов
void changeDirection() {
  stop(BREAK);
  delay(9999999);
}


// функция езды по одному датчику
void rideOneSens(int speed, int distValue, bool side, byte less = 0) {
  go(speed, FORWARD);  // запускаем мотор
  int dist = 0;  // переменная дистанции

  // в зависимости от направления движения опрашивает датчик и сохраняем дистанцию
  if (side) dist = getDistanceL();
  else dist = getDistanceR();

  int error = (side) ? distValue - dist : -(distValue - dist);  // ошибка Р
  int PD = error * kP + (error - oldError) * kD;  // ошибка Р + ошибка D
  oldError = error;  // сохраняем ошибку

  // подаем управляющее воздействие на серву
  setServoAngle(PD, less);
}


// программный старт до первого повотора. Возвращает направление движения
bool start() {
  // выставляем серву в ноль
  setServoAngle(0);
  // счетчик (первые полторы секунду едем с высокой скоростью)
  long fastStart = millis();
  // выключили светодиод
  led(OFF);

  // ждем сигнала с разбери о готовности
  while (digitalRead(RPiPinReady) == 1) delay(1);

  // пикаем что все готово к запуску
  tone(13, 700, 100);
  // ожидание нажатия кнопки старт
  while (digitalRead(ButtonPin) == 0) delay(1);
  while (digitalRead(ButtonPin) == 1) delay(1);

  // запуск моторов на высокой скорости для резкого старта
  go(250, FORWARD);

  // едем пока не обнаружим линию
  while (true) {
    // получили цвет лини
    int color = getLineColor();
    // если прошло полторы секунды, сбавляем скорость
    if (millis() - fastStart > 1500) go(244, FORWARD);

    // если цвет линии синий, то..
    if (color == 0) {
      turnCounter++;  // проехали один поворот
      tone(BuzzPin, 700, 15);  // пик
      startTime = millis();  // счетчик времни для проезда линии
      return true;  // возвращаем что нашли синюю линию
    }
    // если цвет линии оранжевый то..
    if (color == 1) {
      turnCounter++;  // проехали один поворот (едем против часовой стрелки)
      tone(BuzzPin, 700, 15);  // пик
      startTime = millis();  // счетчик времени для проезда линии
      return false;  // возвращаем что нашли оранжевую линию (едем по часовой стрелке)
    }
  }
}


void led(byte state) {
  if (state == 0) {
    digitalWrite(GreenLedPin, HIGH);
    digitalWrite(RedLedPin, LOW);
  } else if (state == 1) {
    digitalWrite(GreenLedPin, LOW);
    digitalWrite(RedLedPin, HIGH);
  } else if (state == 2) {
    digitalWrite(GreenLedPin, LOW);
    digitalWrite(RedLedPin, LOW);
  }
}


// функция иницализации - выполняется один раз при подаче питания
void setup() {
  Serial.begin(115200);  // включаем консоль
  tcs.begin();  // включаем датчик цвета
  USPinsBegin();  // настраиваем пины для ультразвуков
  motorPinBegin();  // настраиваем пины моторов
  servo.attach(ServoPin);  // указываем на каком пине серва
  pinMode(ButtonPin, INPUT);   // указываем на каком пине кнопка
  RPiPinBegin();

  pinMode(BuzzPin, OUTPUT);
  pinMode(RedLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);

  // бесконечный вывод значений датчиков
  if (JustPrintData) manyPrints();

  // программный старт до первого повотора. Возвращает направление движения
  roundDirection = start();
}


// Основной цикл программы
// функция выполняется после setup и работает бесконечно (типо как while true)
void loop() {
  if ((millis() - startTime) > 600) {
    int color = getLineColor();

    if (color == 0 && roundDirection) {
      startTime = millis();
      tone(BuzzPin, 700, 15);
      turnCounter++;
    }
    else if (color == 1 && !roundDirection) {
      startTime = millis();
      tone(BuzzPin, 700, 15);
      turnCounter++;
    }
  }

  if ((digitalRead(RPiPinRed) == 0 && digitalRead(RPiPinGreen) == 0) || (digitalRead(RPiPinRed) == 1 && digitalRead(RPiPinGreen) == 1)) {
    led(OFF);
    rideOneSens(targetSpeed, targetDistCenter, roundDirection);

  } else if (digitalRead(RPiPinRed) == 1) {
    led(RED);
    lastMarker = 0;
    rideOneSens(targetSpeed, targetDistDetour, (isChangeDirection) ? !roundDirection : roundDirection, lessTurn);

  } else if (digitalRead(RPiPinGreen) == 1) {
    led(GREEN);
    lastMarker = 1;
    rideOneSens(targetSpeed, targetDistDetour, (isChangeDirection) ? roundDirection : !roundDirection, lessTurn);
  }

  if (turnCounter == 8) {
    if (lastMarker == 0) {
      roundDirection = !roundDirection;
      isChangeDirection = true;
      changeDirection();
    }
  }

  // стоп - завершени программы
  if (turnCounter == 12) {
    startTime = millis();
    while (millis() - startTime < 1500) rideOneSens(targetSpeed, 244, roundDirection);

    stop(BREAK);
    for (int i = 200; i < 700; i++) {
      tone(BuzzPin, i, 50);
      delay(5);
    }
    delay(999999);
  }
}
