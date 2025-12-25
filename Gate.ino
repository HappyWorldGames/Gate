// VERSION 3.5.3

// Для легкой настройки реле (HIGH/LOW для включения)
const bool powerRelayLOW = LOW;    // Состояние реле питания двигателя для ВЫКЛ
const bool latchRelayLOW = HIGH;   // Состояние реле защёлки для ЗАКРЫТИЯ (открытие защёлки - противоположное)

// Пины
const int buttonPin = 4;          // Кнопка на пине 4
const int openLimitSwitch = 2;    // Концевик открытия на пине 2
const int closeLimitSwitch = 11;  // Концевик закрытия на пине 11 (встроен в защёлку)
const int magnetLimitSwitch = 3;  // Магнитный датчик для подсчёта оборотов

const int motorEN1 = 7;           // Пин питания направления 1 (открытие)
const int motorEN2 = 8;           // Пин питания направления 2 (закрытие)
const int motorPWD1 = 6;          // ШИМ мотор на пине 6 (открытие)
const int motorPWD2 = 5;          // ШИМ мотор на пине 5 (закрытие)

const int powerPin = 10;          // Реле питания двигателя
const int latchPin = 9;           // Реле защёлки
const int ledPin = 13;

// ========== ПАРАМЕТРЫ ДВИЖЕНИЯ И RPM ==========
// Параметры мотора
const int MIN_RPM_POWER = 40;        // Минимальная мощность ПИД-регулятора (ШИМ 0-255)
const int MAX_RPM_POWER = 100;      // Максимальная мощность ПИД-регулятора (%)
const float MIN_RPM_SPEED = 1000;   // Целевая скорость "въезда" в концевик (обороты в минуту)
const float TARGET_RPM = 6000.0;      // Базовая целевая скорость движения (обороты в минуту)

// Параметры датчика оборотов
const int MAGNETS_COUNT = 1;        // Количество магнитов на валу
const unsigned long DEBOUNCE_TIME = 500; // Защита от дребезга датчика (мкс)

// Параметры расстояния (импульсы от одного концевика до другого)
const unsigned long PULSES_CLOSE_TO_OPEN = 19; // Примерное значение, нужно установить реальное
const unsigned long PULSES_OPEN_TO_CLOSE = 20; // Примерное значение, нужно установить реальное

// Настройки замедления
const float MIN_DISTANCE_TO_SLOWDOWN = 0.3; // Начинать замедление, когда осталось 20% пути

// ПИД коэффициенты (настраиваются)
double Kp = 0.0005, Ki = 0.00001, Kd = 0.0000010; // 0.8, 0.2, 0.05

// Параметры времени
const unsigned long LATCH_DELAY = 1000;        // Время удержания сигнала на защёлке (мс)
const unsigned long PUSH_DELAY = 100;          // Время краткого толчка (мс)
const unsigned long RAMP_UP_TIME = 500;      // Время плавного разгона (мс)
const unsigned long LATCH_WAIT_TIMEOUT = 2000; // Таймаут ожидания отхода концевика (мс)

// ========== ПЕРЕМЕННЫЕ ДЛЯ RPM И ПИД ==========0
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool newPulse = false;

// Счётчики
volatile unsigned long totalPulses = 0;    // Всего импульсов
volatile unsigned long sessionPulses = 0;  // Импульсы в сессии (от концевика до концевика)
volatile unsigned long totalRevolutions = 0; // Полные обороты

float currentRPM = 0;      // Текущие обороты
float targetRPM = 0.0;     // Целевые обороты (0 = остановка)
float motorPowerPercent = 0.0;   // Текущая мощность (0-100%)

// ПИД переменные
float error = 0, lastError = 0, integral = 0, derivative = 0;

// Фильтр RPM
#define RPM_FILTER_SAMPLES 5
float rpmBuffer[RPM_FILTER_SAMPLES];
int rpmIndex = 0;

// Плавный старт
unsigned long rampStartTime = 0;
float rampStartRPM = 0.0;
float rampTargetRPM = 0.0;
bool isRamping = false;

// ========== ПЕРЕМЕННЫЕ СТАРОЙ СИСТЕМЫ (для совместимости, могут быть не нужны) ==========
const unsigned long 
  MOTOR_DELAY = 2000,           // 2 секунды задержки перед основным движением
  MAGNET_DELAY = 500,           // 0.5 секунды задержки (не используется в новой логике)
  INACTIVITY_TIMEOUT = 20000;   // 17 секунд неактивности

// Состояние управления
enum State { STOP, OPENING, CLOSING };
State currentState = STOP;
State previewState = OPENING; // Направление, в которое будет происходить следующее движение при нажатии

// Таймеры и флаги
bool powerState = false;
bool startPowerMake = false; // Флаг для старта движения после задержки
unsigned long powerStartTime = 0;
unsigned long lastActivityTime = 0;

unsigned long notSleepTime = 0;
bool ledState = false;

// Для остановки по времени (не используется)
bool isTimeStopping = false;
bool isStartFromLimitSwitch = false;
bool isNotLimitSwitch = false;
long fullOpenTime = 8000;   // Не используется
long moveTime = 0;          // Не используется
long tempMoveTime = 0;      // Не используется
State previewStateTime = previewState; // Не используется

// Антидребезг кнопки
int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// ========== НОВЫЕ ПЕРЕМЕННЫЕ ДЛЯ НЕБЛОКИРУЮЩЕЙ ЛОГИКИ ОТКРЫТИЯ ==========
bool waitingForLatch = false; // Флаг ожидания отхода концевика после открытия защёлки
unsigned long latchWaitStart = 0; // Время начала ожидания

// ========== ФУНКЦИИ ПОДСЧЁТА ОБОРОТОВ ==========
void rpmSensorInterrupt() {
  unsigned long currentTime = micros();
  unsigned long interval = currentTime - lastPulseTime;

  if (interval > DEBOUNCE_TIME) {
    pulseInterval = interval;
    lastPulseTime = currentTime;
    newPulse = true;

    totalPulses++;
    sessionPulses++;

    if (totalPulses % MAGNETS_COUNT == 0) {
      totalRevolutions++;
    }
  }
}

float calculateRPM() {
  if (newPulse) {
    newPulse = false;

    if (pulseInterval > 0) {
      float instantRPM = 60000000.0 / (pulseInterval * MAGNETS_COUNT);

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

  if (micros() - lastPulseTime > 2000000) {
    currentRPM = 0;
  }

  return currentRPM;
}

unsigned long getSessionPulses() {
  noInterrupts();
  unsigned long pulses = sessionPulses;
  interrupts();
  return pulses;
}

void resetSessionCounter() {
  noInterrupts();
  sessionPulses = 0;
  interrupts();
}

// ========== ФУНКЦИИ УПРАВЛЕНИЯ RPM И ПИД ==========
float calculateDistanceToEnd() {
  unsigned long totalPulsesForDirection = (previewState == OPENING) ? PULSES_CLOSE_TO_OPEN : PULSES_OPEN_TO_CLOSE;
  if (totalPulsesForDirection == 0) return 1.0; // Защита от деления на 0

  unsigned long currentPulses = getSessionPulses();
  if (currentPulses >= totalPulsesForDirection) return 0.0;

  return 1.0 - ((float)currentPulses / totalPulsesForDirection);
}

float applyDistanceCorrection(float basePower) {
  float distance = calculateDistanceToEnd();

  if (distance < MIN_DISTANCE_TO_SLOWDOWN) {
    // Плавное замедление до MIN_RPM_SPEED
    float slowdownFactor = distance / MIN_DISTANCE_TO_SLOWDOWN;
    float targetRPMForSlowdown = MIN_RPM_SPEED;
    // Здесь можно корректировать power, но лучше корректировать targetRPM
    // или в updatePID уменьшать targetRPM, когда distance < MIN_DISTANCE_TO_SLOWDOWN
    return basePower; // Возвращаем базовую мощность, коррекция в updatePID
  }

  return basePower;
}

float computePID(float setpoint, float input, float dt) {
  error = setpoint - input;
  integral += error * dt;
  integral = constrain(integral, -50.0, 50.0);
  derivative = (error - lastError) / dt;
  lastError = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

// Обновление ПИД с учётом замедления
float updatePID() {
  static unsigned long lastPIDTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastPIDTime >= 100) { // Интервал обновления ПИД
    float dt = (currentTime - lastPIDTime) / 1000.0;

    float currentTargetRPM = targetRPM;

    // Применяем замедление, если близко к концевику
    float distance = calculateDistanceToEnd();
    if (distance < MIN_DISTANCE_TO_SLOWDOWN) {
        currentTargetRPM = MIN_RPM_SPEED + (targetRPM - MIN_RPM_SPEED) * (distance / MIN_DISTANCE_TO_SLOWDOWN);
        currentTargetRPM = max(currentTargetRPM, MIN_RPM_SPEED); // Не ниже минимальной скорости
    }

    float rpm = calculateRPM();
    float pidAdjustment = computePID(currentTargetRPM, rpm, dt);

    motorPowerPercent += pidAdjustment;
    motorPowerPercent = constrain(motorPowerPercent, MIN_RPM_POWER, MAX_RPM_POWER);

    // Коррекция по ди��танции (уже ��чтена в currentTargetRPM, но можно оставить как резерв)
    // motorPowerPercent = applyDistanceCorrection(motorPowerPercent);

    lastPIDTime = currentTime;
  }

  return motorPowerPercent;
}

void setMotorPowerPercent(float powerPercent) {
  powerPercent = constrain(powerPercent, 0, 100);
  int pwmValue = map(powerPercent, 0, 100, 0, 255);

  analogWrite(motorPWD1, currentState == OPENING ? pwmValue : 0);
  analogWrite(motorPWD2, currentState == CLOSING ? pwmValue : 0);
}

void setTargetRPM(float rpm) {
  if (rpm == targetRPM) return; // Не меняем, если значение не изменилось

  if (targetRPM == 0 && rpm != 0) {
    // Начало движения - плавный старт
    rampStartRPM = 0;
    rampTargetRPM = rpm;
    rampStartTime = millis();
    isRamping = true;
  } else {
    // Просто устанавливаем новую цель
    targetRPM = rpm;
    isRamping = false; // Отменяем плавный старт, если был
  }
}

// ========== ФУНКЦИИ ПЛАВНОГО СТАРТА ==========
void updateRamp() {
    if (!isRamping) return;

    unsigned long currentTime = millis();
    float elapsed = currentTime - rampStartTime;
    float rampDuration = RAMP_UP_TIME;

    if (elapsed >= rampDuration) {
        targetRPM = rampTargetRPM;
        isRamping = false;
    } else {
        targetRPM = rampStartRPM + (rampTargetRPM - rampStartRPM) * (elapsed / rampDuration);
    }
}

// ========== ФУНКЦИИ УПРАВЛЕНИЯ СОСТОЯНИЕМ ==========
void handleButton() {
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        Serial.println("Click");
        lastActivityTime = millis();
        waitingForLatch = false;

        if (currentState == STOP) {
          if (!powerState) {
            digitalWrite(powerPin, !powerRelayLOW);
            powerState = true;
            powerStartTime = millis();
          }

          if (millis() - powerStartTime >= MOTOR_DELAY) {
            if (previewState == OPENING) {
              startClosing();
            }
            else if (previewState == CLOSING) {
              // Логика открытия с защёлкой
              startOpeningWithLatch();
            }
            startPowerMake = false;
          } else {
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
  if (millis() - powerStartTime >= MOTOR_DELAY) {
    if (previewState == OPENING) {
      startClosing();
    }
    else if (previewState == CLOSING) {
      startOpeningWithLatch();
    }
    startPowerMake = false;
  }
}

void checkLimitSwitches() {
  // Обработка концевика открытия
  if (digitalRead(openLimitSwitch) == HIGH && currentState == OPENING) {
    Serial.println("OpenLimitSwitch");
    lastActivityTime = millis();
    startStopping(); // Останавливаем двигатель
    // Сброс сессии и обновление previewState происходят в startStopping
  }

  // Обработка концевика закрытия
  if (digitalRead(closeLimitSwitch) == HIGH && currentState == CLOSING) {
    Serial.println("CloseLimitSwitch");
    lastActivityTime = millis();
    startStopping(); // Останавливаем двигатель
    // Сброс сессии и обновление previewState происходят в startStopping
  }
}

void checkInactivity() {
  if (powerState && (millis() - lastActivityTime > INACTIVITY_TIMEOUT)) {
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
  // Проверяем, можно ли начать движение (не в нужном ли уже состоянии, не сработал ли концевик)
  if (currentState != OPENING && digitalRead(openLimitSwitch) == LOW) {
    Serial.println("Start Opening");
    digitalWrite(motorEN1, HIGH);
    digitalWrite(motorEN2, HIGH);
    currentState = OPENING;
    resetSessionCounter(); // Сбрасываем счётчик для нового движения
    setTargetRPM(TARGET_RPM); // Устанавливаем целевую скорость с плавным стартом
  }
}

void startOpeningWithLatch() {
    Serial.println("Try startOpeningWithLatch");
    if (currentState != OPENING && digitalRead(openLimitSwitch) == LOW) {
        Serial.println("Starting latch sequence");
        // 1. Толчок в сторону закрытия
        digitalWrite(motorEN1, HIGH);
        digitalWrite(motorEN2, HIGH);
        /*analogWrite(motorPWD2, 153); // maxSpeedConst из старого кода
        delay(PUSH_DELAY);
        analogWrite(motorPWD2, 0);*/

        // 2. Активация защёлки
        digitalWrite(latchPin, !latchRelayLOW); // Открытие защёлки
        delay(LATCH_DELAY);

        // 3. Деактивация защёлки
        digitalWrite(latchPin, latchRelayLOW); // Закрытие защёлки (возврат)

        // 4. Толчок в сторону открытия
        analogWrite(motorPWD1, 153); // maxSpeedConst из старого кода
        delay(PUSH_DELAY);
        analogWrite(motorPWD1, 0);

        // 5. Установка флага ожидания
        waitingForLatch = true;
        latchWaitStart = millis();
        Serial.println("Latch sequence done, waiting for switch to release.");
    }
}

void startClosing() {
  Serial.println("Try startClosing");
  if (currentState != CLOSING && digitalRead(closeLimitSwitch) == LOW) {
    Serial.println("Start Closing");
    digitalWrite(motorEN1, HIGH);
    digitalWrite(motorEN2, HIGH);
    currentState = CLOSING;
    resetSessionCounter(); // Сбрасываем счётчик для нового движения
    setTargetRPM(TARGET_RPM); // Устанавливаем целевую скорость с плавным стартом
    //targetRPM = TARGET_RPM;
  }
}

void startStopping() {
  Serial.println("Try startStopping");
  if (currentState != STOP) {
    Serial.println("Start Stopping");
    setTargetRPM(0.0); // Плавно останавливаем двигатель через ПИД
    Serial.println("Motor stopped.");
    previewState = currentState; // Сохраняем направление, в котором остановились
    currentState = STOP;
    setMotorPowerPercent(0);
    integral = 0;
    isRamping = false; // Сброс плавного старта
    resetSessionCounter(); // Сброс сессии при остановке у концевика
    // После остановки устанавливаем состояние STOP в updateMotorSpeed
  }
}

void emergencyStop() {
  Serial.println("Emergency Stop");
  setTargetRPM(0.0); // Останавливаем через ПИД
  Serial.println("Motor stopped.");

  previewState = currentState; // Сохраняем направление, в котором остановились
  currentState = STOP;
  setMotorPowerPercent(0);
  integral = 0;
  isRamping = false; // Сброс плавного старта
  resetSessionCounter(); // Сброс сессии при остановке у концевика
   
  digitalWrite(powerPin, LOW);
  digitalWrite(motorEN1, LOW);
  digitalWrite(motorEN2, LOW);
  previewState = currentState;
  currentState = STOP;
  setMotorPowerPercent(0);
  integral = 0;
  isRamping = false; // Сброс плавного старта
  waitingForLatch = false; // Сброс ожидания защёлки
}

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
  pinMode(latchPin, OUTPUT);

  pinMode(ledPin, OUTPUT);

  // Инициализация состояний
  digitalWrite(motorEN1, LOW);
  digitalWrite(motorEN2, LOW);
  analogWrite(motorPWD1, 0);
  analogWrite(motorPWD2, 0);
  digitalWrite(powerPin, powerRelayLOW);
  digitalWrite(latchPin, latchRelayLOW); // Защёлка в закрытом состоянии

  // ========== ИНИЦИАЛИЗАЦИЯ RPM СИСТЕМЫ ==========
  attachInterrupt(digitalPinToInterrupt(magnetLimitSwitch), rpmSensorInterrupt, FALLING);
  for (int i = 0; i < RPM_FILTER_SAMPLES; i++) rpmBuffer[i] = 0;
  Serial.println("RPM system initialized");

  // Проверка начального положения ворот
  if (digitalRead(closeLimitSwitch) == HIGH) {
    isStartFromLimitSwitch = true;
    previewState = CLOSING; // Ворота закрыты, следующее движение - открытие
  } else if (digitalRead(openLimitSwitch) == HIGH) {
    isStartFromLimitSwitch = true;
    previewState = OPENING; // Ворота открыты, следующее движение - закрытие
  }

  Serial.println("Setup end");
}

void loop() {
  handleButton();
  checkAfterClick();
  checkLimitSwitches();
  checkInactivity();

  // ========== ОБНОВЛЕНИЕ СОСТОЯНИЯ ОЖИДАНИЯ ЗАЩЁЛКИ ==========
  if (waitingForLatch) {
    if (digitalRead(closeLimitSwitch) == LOW) {
        Serial.println("Latch released, starting main opening.");
        waitingForLatch = false;
        startOpening(); // Начинаем основное движение
    } else if (millis() - latchWaitStart > LATCH_WAIT_TIMEOUT) {
        Serial.println("ERROR: Latch wait timeout, close limit switch still active. Stopping.");
        waitingForLatch = false;
        emergencyStop(); // просто вручную в STOP
    }
    // Если нажата кнопка, waitingForLatch сбросится в handleButton/startStopping
  }

  // ========== ОБНОВЛЕНИЕ RPM СИСТЕМЫ ==========
  if (currentState != STOP) {
    updateRamp(); // Обновляем плавный старт
    float power = updatePID();
    setMotorPowerPercent(power);
  } else {
    setMotorPowerPercent(0); // Убеждаемся, что двигатель остановлен
  }

  // ========== ПРОВЕРКА ПОЛНОЙ ОСТАНОВКИ ==========
  /*if (currentState != STOP && targetRPM == 0.0 && abs(currentRPM) < 1.0) {
    Serial.println("Motor stopped.");
    previewState = currentState; // Сохраняем направление, в котором остановились
    currentState = STOP;
    setMotorPowerPercent(0);
    integral = 0;
    isRamping = false; // Сброс плавного старта
    resetSessionCounter(); // Сброс сессии при остановке у концевика
  }*/

  if (millis() - notSleepTime > 5000) {
    Serial.println("Not Sleep!");
    Serial.print("Open limit: ");
    Serial.println(digitalRead(openLimitSwitch));
    Serial.print("Close limit: ");
    Serial.println(digitalRead(closeLimitSwitch));
    Serial.print("RPM:");
    Serial.print(currentRPM, 1);
    Serial.print(" Target:");
    Serial.print(targetRPM, 1);
    Serial.print(" Power:");
    Serial.print(motorPowerPercent, 1);
    Serial.print("% Distance:");
    Serial.print(calculateDistanceToEnd() * 100, 1);
    Serial.println("%");
    
    digitalWrite(ledPin, ledState);
    ledState = !ledState;
    Serial.flush();
    notSleepTime = millis();
  }
}

unsigned long moveTimeRevers(unsigned long time, unsigned long fullNowTime, unsigned long fullReversTime) {
  return (fullReversTime * ((time * 100) / fullNowTime)) / 100;
}