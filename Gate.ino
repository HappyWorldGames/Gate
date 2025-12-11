// VERSION 3.5.0 (MODIFIED FOR RPM CONTROL)

// Для легкой настройки обратный реле
const bool powerRelayLOW = LOW;    // LOW or HIGH
const bool magnetRelayLOW = HIGH;    // LOW or HIGH
const int maxSpeedConst = 153; //178; //204;
const int minSpeedConst = 0;

// Пины
const int buttonPin = 4;          // Кнопка на пине 4
const int openLimitSwitch = 2;    // Концевик открытия на пине 2
const int closeLimitSwitch = 3;   // Концевик закрытия на пине 3
const int magnetLimitSwitch = 11;  // Магнитный концевик для подсчёта оборотов

const int motorEN1 = 7;           // Пин питания направления
const int motorEN2 = 8;           // Пин питания направления
const int motorPWD1 = 6;          // ШИМ мотор на пине 5
const int motorPWD2 = 5;          // ШИМ мотор на пине 6

const int powerPin = 10;          // Реле питания двигателя
const int unlockMagnetPin = 9;            // Реле электромагнита который открывает
const int ledPin = 13;

// ========== НОВЫЕ ПАРАМЕТРЫ ДЛЯ RPM ==========
// Параметры мотора
const int MIN_RPM_POWER = 15;     // Минимальная мощность для запуска (%)
const int MAX_RPM_POWER = 100;    // Максимальная мощность (%)

// Параметры измерения
const int MAGNETS_COUNT = 1;  // Количество магнитов на диске
const unsigned long DEBOUNCE_TIME = 1000; // Защита от дребезга (мкс)

// Настройки дистанционного управления
const float SLOWDOWN_START = 0.2;      // Начинать замедление при 20% до конца
const float MIN_DISTANCE_POWER = 10.0; // Минимальная мощность у концевика (%)

// ПИД коэффициенты (настраиваются)
double Kp = 0.8, Ki = 0.2, Kd = 0.05;

// ========== ПЕРЕМЕННЫЕ ДЛЯ RPM ==========
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool newPulse = false;

// Счётчики
volatile unsigned long totalPulses = 0;    // Всего импульсов
volatile unsigned long sessionPulses = 0;  // Импульсы в сессии
volatile unsigned long totalRevolutions = 0; // Полные обороты

float currentRPM = 0;      // Текущие обороты
float targetRPM = 0.0;     // Целевые обороты (0 = остановка)
float motorPowerPercent = 0.0;   // Текущая мощность (0-100%)

// ПИД переменные
float error = 0, lastError = 0, integral = 0, derivative = 0;

// Дистанция до концевика
unsigned long pulsesToEndStop = 0;  // Импульсы до концевика
bool endStopKnown = false;          // Флаг калибровки

// Фильтр RPM
#define RPM_FILTER_SAMPLES 5
float rpmBuffer[RPM_FILTER_SAMPLES];
int rpmIndex = 0;

// ========== ФУНКЦИИ ПОДСЧЁТА ОБОРОТОВ ==========

// Прерывание для магнитного датчика
void rpmSensorInterrupt() {
  unsigned long currentTime = micros();
  unsigned long interval = currentTime - lastPulseTime;
  
  if (interval > DEBOUNCE_TIME) {
    pulseInterval = interval;
    lastPulseTime = currentTime;
    newPulse = true;
    
    // Счётчики
    totalPulses++;
    sessionPulses++;
    
    // Полные обороты
    if (totalPulses % MAGNETS_COUNT == 0) {
      totalRevolutions++;
    }
  }
}

// Расчёт текущих RPM
float calculateRPM() {
  if (newPulse) {
    newPulse = false;
    
    if (pulseInterval > 0) {
      float instantRPM = 60000000.0 / (pulseInterval * MAGNETS_COUNT);
      
      // Сглаживание
      rpmBuffer[rpmIndex] = instantRPM;
      rpmIndex = (rpmIndex + 1) % RPM_FILTER_SAMPLES;
      
      float sum = 0;
      int count = 0;
      for (int i = 0; i < RPM_FILTER_SAMPLES; i++) {
        if (rpmBuffer[i] > 0) {
          sum += rpmBuffer[i];
          count++;
        }
      }
      
      currentRPM = (count > 0) ? (sum / count) : 0;
    }
  }
  
  // Если импульсов нет 2 секунды - обнуляем RPM
  if (micros() - lastPulseTime > 2000000) {
    currentRPM = 0;
  }
  
  return currentRPM;
}

// Получить количество импульсов в сессии
unsigned long getSessionPulses() {
  noInterrupts();
  unsigned long pulses = sessionPulses;
  interrupts();
  return pulses;
}

// Сброс счётчика сессии
void resetSessionCounter() {
  noInterrupts();
  sessionPulses = 0;
  interrupts();
}

// Расчёт дистанции до концевика (0.0-1.0)
float calculateDistanceToEnd() {
  if (!endStopKnown || pulsesToEndStop == 0) return 1.0;
  
  unsigned long currentPulses = getSessionPulses();
  if (currentPulses >= pulsesToEndStop) return 0.0;
  
  return 1.0 - ((float)currentPulses / pulsesToEndStop);
}

// Коррекция мощности по дистанции
float applyDistanceCorrection(float basePower) {
  if (!endStopKnown) return basePower;
  
  float distance = calculateDistanceToEnd();
  
  if (distance < SLOWDOWN_START) {
    float slowdownFactor = distance / SLOWDOWN_START;
    float correctedPower = MIN_DISTANCE_POWER + 
                          (basePower - MIN_DISTANCE_POWER) * slowdownFactor;
    return max(correctedPower, MIN_DISTANCE_POWER);
  }
  
  return basePower;
}

// Вычисление ПИД
float computePID(float setpoint, float input, float dt) {
  error = setpoint - input;
  integral += error * dt;
  
  // Антивинднап
  integral = constrain(integral, -50.0, 50.0);
  
  derivative = (error - lastError) / dt;
  lastError = error;
  
  return Kp * error + Ki * integral + Kd * derivative;
}

// Обновление ПИД регулятора
float updatePID() {
  static unsigned long lastPIDTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastPIDTime >= 100) { // Интервал обновления ПИД
    float dt = (currentTime - lastPIDTime) / 1000.0;
    
    float rpm = calculateRPM();
    float pidAdjustment = computePID(targetRPM, rpm, dt);
    
    motorPowerPercent += pidAdjustment;
    motorPowerPercent = constrain(motorPowerPercent, MIN_RPM_POWER, MAX_RPM_POWER);
    
    // Коррекция по дистанции
    motorPowerPercent = applyDistanceCorrection(motorPowerPercent);
    
    lastPIDTime = currentTime;
  }
  
  return motorPowerPercent;
}

// Установка мощности мотора (0-100% -> 0-255 PWM)
void setMotorPowerPercent(float powerPercent) {
  powerPercent = constrain(powerPercent, 0, 100);
  int pwmValue = map(powerPercent, 0, 100, 0, 255);
  // Используем текущее состояние currentState для определения направления
  analogWrite(motorPWD1, currentState == OPENING ? pwmValue : 0);
  analogWrite(motorPWD2, currentState == CLOSING ? pwmValue : 0);
}

// Установка целевых RPM
void setTargetRPM(float rpm) {
  targetRPM = rpm;
  integral = 0; // Сброс интеграла для плавности
}

// ========== СТАРЫЕ ПАРАМЕТРЫ (МОГУТ НЕ ИСПОЛЬЗОВАТЬСЯ) ==========
// Настройки параметры
const unsigned long 
  MOTOR_DELAY = 2000,           // 2 секунды задержки двигателя
  MAGNET_DELAY = 500,          // 0.5 секунда задержки после магнита
  INACTIVITY_TIMEOUT = 17000,   // 30 секунд неактивности
  accelerationInterval = 50;    // Каждые 50 мс, обновлять скорость
// int maxSpeed = 255; // Не используется
// int minSpeed = 0;   // Не используется
// const int accelerationStep = 5; // Шаг разгона - не используется
// int currentSpeed = 0; // Не используется

// Состояние управления
enum State { STOP, OPENING, CLOSING };
State currentState = STOP;
State previewState = OPENING;
// bool isStopping = false; // Не используется

// Таймеры и флаги
bool powerState = false;
bool startPowerMake = false;
unsigned long powerStartTime = 0;
unsigned long magnetDelayStart = 0;
unsigned long lastActivityTime = 0;

unsigned long notSleepTime = 0;
bool ledState = false;

// Для остановки по времени
bool isTimeStopping = false;
bool isStartFromLimitSwitch = false;
bool isNotLimitSwitch = false;
long fullOpenTime = 8000;   // Время нужное для открытия ворот
// unsigned long fullCloseTime = 0;  //  Время нужное для закрытия ворот
long moveTime = 0;       // Время в пути
long tempMoveTime = 0;   // Для прибовления времени
State previewStateTime = previewState;

// Антидребезг кнопки
int buttonState;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// unsigned long lastAccelTime = 0; // Не используется

void setup() {
  Serial.begin(9600);
  Serial.println("Setup start");

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(openLimitSwitch, INPUT_PULLUP);
  pinMode(closeLimitSwitch, INPUT_PULLUP);
  pinMode(magnetLimitSwitch, INPUT_PULLUP);
  
  pinMode(motorEN1, OUTPUT);
  pinMode(motorEN2, OUTPUT);
  pinMode(motorPWD1, OUTPUT);
  pinMode(motorPWD2, OUTPUT);
  pinMode(powerPin, OUTPUT);
  pinMode(unlockMagnetPin, OUTPUT);
  
  pinMode(ledPin, OUTPUT);
  
  // Инициализация состояний
  digitalWrite(motorEN1, LOW);
  digitalWrite(motorEN2, LOW);
  analogWrite(motorPWD1, 0);
  analogWrite(motorPWD2, 0);
  digitalWrite(powerPin, powerRelayLOW);
  
  digitalWrite(unlockMagnetPin, magnetRelayLOW);

  // ========== ИНИЦИАЛИЗАЦИЯ RPM СИСТЕМЫ ==========
  attachInterrupt(digitalPinToInterrupt(magnetLimitSwitch), rpmSensorInterrupt, FALLING);
  for (int i = 0; i < RPM_FILTER_SAMPLES; i++) rpmBuffer[i] = 0;
  Serial.println("RPM system initialized");

  // Проверка начального положения ворот
  if(digitalRead(closeLimitSwitch) == HIGH) {
    isStartFromLimitSwitch = true;
    previewState = CLOSING;
  }else if(digitalRead(openLimitSwitch) == HIGH) {
    moveTime = fullOpenTime;
    isStartFromLimitSwitch = true;
  }

  load();

  Serial.println("Setup end");
}

void loop() {
  handleButton();
  checkAfterClick();
  checkLimitSwitches(); // Основная остановка по концевикам
  checkMagnetDelay(); // Для логики магнита
  //checkMovementTime(); // Не используется
  // updateMotorSpeed(); // Больше не используется
  checkInactivity();
  
  // ========== ОБНОВЛЕНИЕ RPM СИСТЕМЫ ==========
  if (currentState != STOP) {
    float power = updatePID();
    setMotorPowerPercent(power);
  } else {
      // Если состояние STOP, останавливаем двигатель
      setMotorPowerPercent(0);
  }

  if(millis() - notSleepTime > 5000) {
    Serial.println("Not Sleep!");
    Serial.print("Open limit: ");
    Serial.println(digitalRead(openLimitSwitch));
    Serial.print("Close limit: ");
    Serial.println(digitalRead(closeLimitSwitch));
    // ========== ДОБАВЛЕНО ДЛЯ RPM ==========
    float distance = calculateDistanceToEnd() * 100;
    Serial.print("RPM:");
    Serial.print(currentRPM, 1);
    Serial.print(" Target:");
    Serial.print(targetRPM);
    Serial.print(" Power:");
    Serial.print(motorPowerPercent, 1);
    Serial.print("% Distance:");
    Serial.print(distance, 1);
    Serial.println("%");
    // ========== КОНЕЦ ДОБАВЛЕНИЯ ==========
    digitalWrite(ledPin, ledState);
    ledState = !ledState;
    Serial.flush();
    notSleepTime = millis();
  }
}

void checkMovementTime()  {
  // Не используется в новой системе
}

void handleButton() {
  int reading = digitalRead(buttonPin);
  
  if(reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if((millis() - lastDebounceTime) > debounceDelay) {
    if(reading != buttonState) {
      buttonState = reading;
      
      if(buttonState == LOW) {
        Serial.println("Click");
        lastActivityTime = millis();
        
        if(currentState == STOP) {
          if(!powerState) {
            digitalWrite(powerPin, !powerRelayLOW);
            powerState = true;
            powerStartTime = millis();
          }
          
          if(millis() - powerStartTime >= MOTOR_DELAY) {
            if (previewState == OPENING) {
              startClosing();
            }
            else if(previewState == CLOSING) {
              // Логика для открытия с магнитом (оригинальная)
              digitalWrite(motorEN1, HIGH);
      digitalWrite(motorEN2, HIGH);
      analogWrite(motorPWD2, maxSpeedConst);
      delay(25);
      digitalWrite(motorEN1, LOW);
      digitalWrite(motorEN2, LOW);
      analogWrite(motorPWD2, 0);
            
      digitalWrite(unlockMagnetPin, !magnetRelayLOW);
      magnetDelayStart = millis();
      delay(500);
      digitalWrite(unlockMagnetPin, magnetRelayLOW);
      
      digitalWrite(motorEN1, HIGH);
      digitalWrite(motorEN2, HIGH);
      analogWrite(motorPWD1, maxSpeedConst);
      delay(25);
      digitalWrite(motorEN1, LOW);
      digitalWrite(motorEN2, LOW);
      analogWrite(motorPWD1, 0);
            }
            startPowerMake = false;
          }else{
            startPowerMake = true;
          }
        } else {
          // Остановка при повторном нажатии
          startStopping();
        }
      }
    }
  }
  lastButtonState = reading;
}

void checkAfterClick() {
  if (!startPowerMake) return;
  if(millis() - powerStartTime >= MOTOR_DELAY) {
    if (previewState == OPENING) {
      startClosing();
    }
    else if(previewState == CLOSING) {
      // Логика для открытия с магнитом (оригинальная)
      digitalWrite(motorEN1, HIGH);
      digitalWrite(motorEN2, HIGH);
      analogWrite(motorPWD2, maxSpeedConst);
      delay(25);
      digitalWrite(motorEN1, LOW);
      digitalWrite(motorEN2, LOW);
      analogWrite(motorPWD2, 0);
            
      digitalWrite(unlockMagnetPin, !magnetRelayLOW);
      magnetDelayStart = millis();
      delay(500);
      digitalWrite(unlockMagnetPin, magnetRelayLOW);
      
      digitalWrite(motorEN1, HIGH);
      digitalWrite(motorEN2, HIGH);
      analogWrite(motorPWD1, maxSpeedConst);
      delay(25);
      digitalWrite(motorEN1, LOW);
      digitalWrite(motorEN2, LOW);
      analogWrite(motorPWD1, 0);
    }
    startPowerMake = false;
  }
}

void checkMagnetDelay() {
  if(digitalRead(closeLimitSwitch) == LOW && magnetDelayStart > 0 && (millis() - magnetDelayStart >= MAGNET_DELAY)) {
    magnetDelayStart = 0;
    startOpening();
  }
}

void checkLimitSwitches() {
  // УБРАНА оригинальная логика замедления по magnetLimitSwitch
  // if (digitalRead(magnetLimitSwitch) == LOW && currentState == OPENING) {
  //   Serial.println('Smooth');
  //   minSpeed = 51; //76;
  //   startStopping();
  // }
  
  // Обработка концевика открытия
  if(digitalRead(openLimitSwitch) == HIGH && currentState == OPENING) {
    Serial.println("OpenLimitSwitch");
    lastActivityTime = millis();
    startStopping(); // Останавливаем двигатель
  }
  
  // Обработка концевика закрытия
  if(digitalRead(closeLimitSwitch) == HIGH && currentState == CLOSING) {
    Serial.println("CloseLimitSwitch");
    lastActivityTime = millis();
    startStopping(); // Останавливаем двигатель
  }
}

// Обновленная функция управления двигателем - не использует устаревшие переменные
void updateMotorSpeed() {
  // Эта функция больше не используется, её логика перенесена в RPM-систему
}

void checkInactivity() {
  if(powerState && (millis() - lastActivityTime > INACTIVITY_TIMEOUT)) {
    Serial.println("Inactivity");
    digitalWrite(powerPin, powerRelayLOW);
    powerState = false;
    startStopping(); // Останавливаем двигатель
    digitalWrite(motorEN1, LOW);
    digitalWrite(motorEN2, LOW);
  }
}

void startOpening() {
  Serial.println("Try startOpening");
  if(currentState != OPENING && digitalRead(openLimitSwitch) == LOW) {
    Serial.println("Start Opening");
    digitalWrite(motorEN1, HIGH);
    digitalWrite(motorEN2, HIGH);
    currentState = OPENING;
    resetSessionCounter(); // Сбрасываем счётчик для нового движения
    setTargetRPM(60.0); // Устанавливаем целевую скорость (настраиваемо)
  }
}

void startClosing() {
  Serial.println("Try startClosing");
  if(currentState != CLOSING && digitalRead(closeLimitSwitch) == LOW) {
    Serial.println("Start Closing");
    digitalWrite(motorEN1, HIGH);
    digitalWrite(motorEN2, HIGH);
    currentState = CLOSING;
    resetSessionCounter(); // Сбрасываем счётчик для нового движения
    setTargetRPM(60.0); // Устанавливаем целевую скорость (настраиваемо)
  }
}

void startStopping() {
  Serial.println("Try startStopping");
  if(currentState != STOP) {
    Serial.println("Start Stopping");
    setTargetRPM(0.0); // Плавно останавливаем двигатель через ПИД
    // После остановки устанавливаем состояние STOP
    if (abs(currentRPM) < 1.0) { // Если обороты близки к нулю
        previewState = currentState; // Сохраняем предыдущее направление
        currentState = STOP;
        setMotorPowerPercent(0); // Убеждаемся, что ШИМ выключен
        // Сброс интеграла ПИД на всякий случай
        integral = 0;
    }
  }
}

void emergencyStop() {
  Serial.println("Emergency Stop");
  setTargetRPM(0.0); // Останавливаем через ПИД
  digitalWrite(powerPin, LOW);
  digitalWrite(motorEN1, LOW);
  digitalWrite(motorEN2, LOW);
  previewState = currentState;
  currentState = STOP;
  setMotorPowerPercent(0);
  integral = 0;
}

void currectTimeStopping() {
  Serial.println("currectTimeStopping");
  if (!isStartFromLimitSwitch) return;
  if (/*previewState == OPENING &&*/ digitalRead(openLimitSwitch) == HIGH) {
    //fullOpenTime = moveTime;
    save();
    isTimeStopping = false;
    isNotLimitSwitch = false;
    Serial.print("new fullOpenTime = ");
    Serial.println(fullOpenTime);
  } else if (/*previewState == CLOSING &&*/ digitalRead(closeLimitSwitch) == HIGH) {
       moveTime = 0;
  //   fullCloseTime = moveTime;
  //   save();
  //   moveTime = 0;
  //   isTimeStopping = false;
  //   Serial.print("new fullCloseTime = ");
  //   Serial.println(fullCloseTime);
  } else {
    Serial.println("not contact with limit switch");
    if (previewState == OPENING) {
            isNotLimitSwitch = true;
            // maxSpeed = 127; // Не используется
            startOpening();
    }
    // else if (previewState == CLOSING) startClosing();
  }
}

void load() {
  Serial.println("load");
  // TODO load time
}

void save() {
  Serial.println("save");
  // TODO save time
}

unsigned long moveTimeRevers(unsigned long time, unsigned long fullNowTime, unsigned long fullReversTime) {
  return (fullReversTime * ((time * 100) / fullNowTime)) / 100;
}