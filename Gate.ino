/*
  Алгоритм:
  Нажатие кнопки
  1. Включение питания, без подачи на мотор.
  2. После 2 секунд, если питание не было включено до этого, подача питания на мотор.
  3. После достижения концевика или нажатия кнопки, остановка подачи питания на мотор.
  4. По истечению 5 минут выключение питания.
*/
const int version = 1;

// Порты
const int reversRelay = 2;              // Порт для управления направлением
const int motorPowerReley = 3;          // Порт для запуска двигателя pwd
const int startPowerRelay = 7;          // Порт для запуска питания
const int magnetPowerRelay = 8;         // Порт для питания магнита

const int switchButton = 4;             // Порт для кнопки

const int limitSwitchOpen = 5;          // Порт концевика на открытие
const int limitSwitchClose = 6;         // Порт концевика на закрытие

// Настройки
const int delayButton = 300;            // Зажержка между нажатиями кнопки в ms
const int delayPowerOn = 1000;          // Задержка перез одобрением включения питания в ms
const int delayPowerOff = 300000;       // Задержка до выключения питания в ms
const int delayGateTimer = 1000;        // Задержка если не сработает концевик, чтобы остановить ворота(для случаев если что-то случилось с концевиками)

const float startSpeed = 0.1275f;        // Скорость старта двигателя
const unsigned long timeToStart = 2000; // Время для разгона

// Остальные
unsigned long switchTime = 0;           // Таймер для кнопки, чтобы небыло быстрых переключений
unsigned long powerOnTime = 0;          // Таймер для питания
unsigned long startEngineTime = -1;     // Таймер для запуска двигателя
unsigned long moveGateTimer = 0;        // Таймер отсчета времени при открытии и закрытие
unsigned long tempMoveGateTimer = 0;    // Таймер отсчета времени при включеном отсчете открытия/закрытия ворот

int timeToOpen = -1;                    // Время которое нужно для открытия ворот
int timeToClose = -1;                   // Время которое нужно для закрытия ворот

bool limitSwitchIsOpen = false;         // Состояние концевика на открыте
bool limitSwitchIsClose = false;        // Состояние концевика на закрытие
int doorStatus = 1;                     // Отпределяет куда ехать или стоять
bool isPower = false;                   // Включено ли питание
bool isSmoothStartEngine = false;       // Регулятор для включения плавнойго включения двигателя
bool isMoveGateTimer = false;           // Включать ли отсчет времени открытия или закрытия ворот

// true и false для обратных реле
bool t = 0;
bool f = 1;

void setup() {
  Serial.begin(19200);
  Serial.println("version: " + version);
  
  pinMode(reversRelay, OUTPUT);
  pinMode(motorPowerReley, OUTPUT);
  pinMode(startPowerRelay, OUTPUT);
  pinMode(magnetPowerRelay, OUTPUT);

  pinMode(switchButton, INPUT_PULLUP);
  pinMode(limitSwitchOpen, INPUT_PULLUP);
  pinMode(limitSwitchClose, INPUT_PULLUP);

  digitalWrite(startPowerRelay, f);         // Выключаем питание
  /*analog*/digitalWrite(motorPowerReley, 1);          // Выклюючаем двигатель
  digitalWrite(reversRelay, f);             // Выключаем направление
  digitalWrite(magnetPowerRelay, t);        // Вклычаем питание на магните
}

void loop() {
  bool statusButton = !digitalRead(switchButton);
  limitSwitchIsOpen = digitalRead(limitSwitchOpen);
  limitSwitchIsClose = digitalRead(limitSwitchClose);

  if (limitSwitchIsOpen && doorStatus != 2) doorStatus = 1;
  else if (limitSwitchIsClose && doorStatus != 0) doorStatus = 3;
  // switch button
  if (statusButton && millis() - switchTime > delayButton) {
    Serial.println("click");
    switchTime = millis();

    if (!isPower) {
      digitalWrite(startPowerRelay, t);
      delay(delayPowerOn);              // Плохое решение, но должно работать.
      isPower = true;
      powerOnTime = millis();
    }

    // switch status door
    doorStatus++;
    if (doorStatus > 3) doorStatus = 0;
  }

  if (!isPower) {                 // Если питания нет не включать реле управления
    digitalWrite(reversRelay, f);
    digitalWrite(motorPowerReley, 1);
    digitalWrite(magnetPowerRelay, 1);  // Поддерживаем включеным магнит
    startEngineTime = -1;
    tempMoveGateTimer = millis();
    return;
  }else if (millis() - powerOnTime > delayPowerOff) {  // Если прошло время выключения
    isPower = false;
    digitalWrite(startPowerRelay, f);
    digitalWrite(reversRelay, f);
    digitalWrite(motorPowerReley, 1);
  }

  // Проверка по времени закрытия
  if (isMoveGateTimer) {
    moveGateTimer += millis() - tempMoveGateTimer;
    if (limitSwitchIsOpen && !limitSwitchIsClose) {
      
    }
  }
  tempMoveGateTimer = millis();

  // Плавный старт
  if (isSmoothStartEngine) smoothStartEngine();

  switch (doorStatus) {
    case 0:
      // open
      if (limitSwitchIsOpen && !limitSwitchIsClose) doorStatus = 2;
      digitalWrite(reversRelay, f);
      digitalWrite(magnetPowerRelay, 0);
      delay(1000);
      isSmoothStartEngine = true;
        
      Serial.println("door=0");
      break;
    case 1:
      // idle
      idleEngine();
        
      Serial.println("door=1");
      break;
    case 2:
      // close
      if (limitSwitchIsClose && !limitSwitchIsOpen) doorStatus = 0;
      digitalWrite(reversRelay, t);
      digitalWrite(magnetPowerRelay, 1);
      isSmoothStartEngine = true;
        
      Serial.println("door=2");
      break;
    case 3:
      // idle
      idleEngine();
        
      Serial.println("door=3");
      break;
  }
}

void smoothStartEngine() {
  Serial.println("smooth");
  if (startEngineTime == -1) startEngineTime = millis();
  unsigned long timeLeft = millis() - startEngineTime;

  int x = 0; //255;
  /*if (timeLeft <= timeToStart) x = 0 + startSpeed * timeLeft;
  else*/ isSmoothStartEngine = false;

  /*analog*/digitalWrite(motorPowerReley, x);
}

void idleEngine() {
  Serial.println("idle");
  /*analog*/digitalWrite(motorPowerReley, 1);
  digitalWrite(reversRelay, f);
  startEngineTime = -1;
}
