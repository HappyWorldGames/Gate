// VERSION 4.2
// Перед запуском в работы выставить значкения для PULSES_CLOSE и PULSES_OPEN, нужные цифры будут в логах.

const bool powerRelayLOW = LOW;
const bool latchRelayLOW = HIGH;

const int buttonPin        = 4;
const int openLimitSwitch  = 2;
const int closeLimitSwitch = 11;
const int magnetLimitSwitch = 3;
const int motorEN1 = 7, motorEN2 = 8;
const int motorPWD1 = 6, motorPWD2 = 5;
const int powerPin = 10, latchPin = 9, ledPin = 13;

// ========== ПАРАМЕТРЫ ==========
const unsigned long TARGET_TIME_MS = 15000;

unsigned long PULSES_OPEN  = 0;
unsigned long PULSES_CLOSE = 0;

// Профиль мощности ОТКРЫТИЕ
const int OPEN_POWER_KICK   = 40;  // стартовый пинок
const int OPEN_POWER_CRUISE = 38;  // крейсер (рассчитано из логов)
const int OPEN_POWER_SLOW   = 35;  // замедление

// Профиль мощности ЗАКРЫТИЕ
const int CLOSE_POWER_KICK   = 55;  // стартовый пинок
const int CLOSE_POWER_CRUISE = 40;  // крейсер (попал точно в 15с по логам)
const int CLOSE_POWER_SLOW   = 48;  // замедление + усилие для замка

// Тайминги профиля
const unsigned long KICK_MS       = 300;
const unsigned long CLOSE_KICK_MS = 500;
const unsigned long RAMP_UP_MS    = 2000;
const float         SLOWDOWN_AT   = 0.22;

// Коррекция
const int   POWER_STEP           = 2;
const int   MAX_CORRECTION       = 8;   // было 15 — слишком много, глушило мотор
const float CORRECTION_DEADBAND  = 0.06;
const unsigned long CORRECTION_INTERVAL = 600;

// Защита
const unsigned long DEBOUNCE_TIME_US = 8000;
const unsigned long STALL_TIMEOUT    = 2500;

// Прочее
const unsigned long LATCH_DELAY        = 1000;
const unsigned long PUSH_DELAY         = 120;
const unsigned long MOTOR_DELAY        = 2000;
const unsigned long INACTIVITY_TIMEOUT = 30000;
const unsigned long LATCH_WAIT_TIMEOUT = 4000;

// ========== СОСТОЯНИЯ ==========
enum State { STOP, OPENING, CLOSING };
State currentState = STOP;
State previewState = OPENING;

// ========== ПЕРЕМЕННЫЕ ==========
volatile unsigned long lastPulseTime_us = 0;
volatile unsigned long sessionPulses    = 0;
unsigned long lastPulseWallTime = 0;

bool powerState     = false;
bool startPowerMake = false;
unsigned long powerStartTime     = 0;
unsigned long lastActivityTime   = 0;
unsigned long moveStartTime      = 0;
unsigned long lastCorrectionTime = 0;

float currentPower = 0;
float powerOffset  = 0;

bool waitingForLatch     = false;
unsigned long latchWaitStart = 0;

bool isCalibrating = false;

int buttonState = HIGH, lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

bool diagActive = false;
unsigned long sessionStartTime = 0;
unsigned long diagTimer        = 0;
String pendingEvent = "";

// ========== ПРЕРЫВАНИЕ ==========
void rpmSensorInterrupt() {
  unsigned long now = micros();
  if (now - lastPulseTime_us > DEBOUNCE_TIME_US) {
    lastPulseTime_us = now;
    sessionPulses++;
    lastPulseWallTime = millis();
  }
}

unsigned long getSessionPulses() {
  noInterrupts();
  unsigned long p = sessionPulses;
  interrupts();
  return p;
}

void resetSession() {
  noInterrupts();
  sessionPulses = 0;
  interrupts();
  lastPulseWallTime = millis();
}

// ========== МОЩНОСТЬ ==========
void setMotorPower(float pct) {
  pct = constrain(pct, 0, 100);
  int pwm = map((int)pct, 0, 100, 0, 255);
  if (currentState == OPENING) {
    analogWrite(motorPWD1, pwm);
    analogWrite(motorPWD2, 0);
  } else if (currentState == CLOSING) {
    analogWrite(motorPWD1, 0);
    analogWrite(motorPWD2, pwm);
  }
}

float getProfilePower() {
  unsigned long elapsed  = millis() - moveStartTime;
  int kick_ms    = (currentState == OPENING) ? KICK_MS         : CLOSE_KICK_MS;
  int pwr_kick   = (currentState == OPENING) ? OPEN_POWER_KICK   : CLOSE_POWER_KICK;
  int pwr_cruise = (currentState == OPENING) ? OPEN_POWER_CRUISE : CLOSE_POWER_CRUISE;
  int pwr_slow   = (currentState == OPENING) ? OPEN_POWER_SLOW   : CLOSE_POWER_SLOW;

  // Фаза 1: стартовый пинок
  if (elapsed < (unsigned long)kick_ms)
    return pwr_kick;

  // Фаза 2: плавный разгон
  unsigned long ramp_elapsed = elapsed - kick_ms;
  if (ramp_elapsed < RAMP_UP_MS) {
    float t = (float)ramp_elapsed / RAMP_UP_MS;
    return pwr_kick + (pwr_cruise - pwr_kick) * (t * t);
  }

  // Фаза 3/4: нужна позиция
  unsigned long totalPulsesDir = (currentState == OPENING) ? PULSES_OPEN : PULSES_CLOSE;
  if (totalPulsesDir == 0) return pwr_cruise;

  float distLeft = 1.0f - (float)getSessionPulses() / totalPulsesDir;
  distLeft = constrain(distLeft, 0.0f, 1.0f);

  // Фаза 4: замедление
  if (distLeft < SLOWDOWN_AT) {
    float t = distLeft / SLOWDOWN_AT;
    return pwr_slow + (pwr_cruise - pwr_slow) * (t * t);
  }

  // Фаза 3: крейсер
  return pwr_cruise;
}

void updatePower() {
  if (currentState != OPENING && currentState != CLOSING) return;

  float profile = getProfilePower();

  unsigned long elapsed  = millis() - moveStartTime;
  int kick_ms = (currentState == OPENING) ? KICK_MS : CLOSE_KICK_MS;
  bool inRamp = elapsed < (unsigned long)(kick_ms + RAMP_UP_MS);

  unsigned long totalPulsesDir = (currentState == OPENING) ? PULSES_OPEN : PULSES_CLOSE;
  float distLeft = (totalPulsesDir > 0)
    ? 1.0f - (float)getSessionPulses() / totalPulsesDir : 1.0f;
  bool inSlow = distLeft < SLOWDOWN_AT;

  if (!inRamp && !inSlow && totalPulsesDir > 0
      && millis() - lastCorrectionTime >= CORRECTION_INTERVAL) {
    lastCorrectionTime = millis();
    float expectedProgress = constrain((float)elapsed / TARGET_TIME_MS, 0, 1);
    float actualProgress   = constrain(1.0f - distLeft, 0, 1);
    float diff = actualProgress - expectedProgress;
    if      (diff < -CORRECTION_DEADBAND) powerOffset += POWER_STEP;
    else if (diff >  CORRECTION_DEADBAND) powerOffset -= POWER_STEP;
    powerOffset = constrain(powerOffset, -MAX_CORRECTION, MAX_CORRECTION);
  }

  // Абсолютный минимум — в замедлении не ниже 30%
  int absoluteMin = inSlow ? 30 : 15;
  currentPower = constrain(profile + powerOffset, absoluteMin, 100);
  setMotorPower(currentPower);
}

// ========== ЗАСТРЕВАНИЕ ==========
void checkStall() {
  if (currentState != OPENING && currentState != CLOSING) return;
  unsigned long elapsed = millis() - moveStartTime;
  int kick_ms = (currentState == OPENING) ? KICK_MS : CLOSE_KICK_MS;
  if (elapsed < (unsigned long)(kick_ms + 1000)) return;
  if (millis() - lastPulseWallTime > STALL_TIMEOUT) {
    logEvent("STALL");
    startStopping();
  }
}

// ========== ЛОГИРОВАНИЕ ==========
void logEvent(const String& evt) {
  pendingEvent = evt;
  Serial.print("EVENT,"); Serial.print(millis());
  Serial.print(","); Serial.println(evt);
}

void printCSVRow() {
  unsigned long now     = millis();
  unsigned long elapsed = now - moveStartTime;
  unsigned long totalPulsesDir = (currentState == OPENING) ? PULSES_OPEN : PULSES_CLOSE;
  float expPct = (totalPulsesDir > 0)
    ? constrain((float)elapsed / TARGET_TIME_MS * 100, 0, 100) : 0;
  float actPct = (totalPulsesDir > 0)
    ? constrain((float)getSessionPulses() / totalPulsesDir * 100, 0, 100) : 0;

  Serial.print(now);                Serial.print(",");
  Serial.print(now-sessionStartTime); Serial.print(",");
  Serial.print(currentState == OPENING ? "O" : "C"); Serial.print(",");
  Serial.print(currentPower, 1);    Serial.print(",");
  Serial.print(powerOffset, 1);     Serial.print(",");
  Serial.print(getSessionPulses()); Serial.print(",");
  Serial.print(expPct, 1);          Serial.print(",");
  Serial.print(actPct, 1);          Serial.print(",");
  Serial.println(pendingEvent);
  pendingEvent = "";
}

// ========== ДВИЖЕНИЕ ==========
void startOpening() {
  if (currentState != OPENING && digitalRead(openLimitSwitch) == LOW) {
    logEvent("START_OPENING");
    digitalWrite(motorEN1, HIGH);
    digitalWrite(motorEN2, HIGH);
    currentState       = OPENING;
    powerOffset        = 0;
    moveStartTime      = millis();
    lastCorrectionTime = millis();
    sessionStartTime   = millis();
    diagActive         = true;
    resetSession();
    Serial.println("---CSV_START---");
    Serial.println("time_ms,ses_ms,dir,power,offset,pulses,exp%,act%,event");
    updatePower();
  }
}

void startOpeningWithLatch() {
  if (currentState != OPENING && digitalRead(openLimitSwitch) == LOW
      && digitalRead(closeLimitSwitch) == HIGH) {
    logEvent("LATCH_START");
    digitalWrite(motorEN1, HIGH);
    digitalWrite(motorEN2, HIGH);

    // 1. Активация защёлки
    digitalWrite(latchPin, !latchRelayLOW);
    delay(LATCH_DELAY);                   // 1000мс — держим защёлку
    digitalWrite(latchPin, latchRelayLOW);

    // 2. Пинок в открытие (60% как в оригинале)
    analogWrite(motorPWD1, 153);
    delay(PUSH_DELAY);                    // 120мс
    analogWrite(motorPWD1, 0);

    // 3. Ждём пока концевик закрытия отпустит
    waitingForLatch  = true;
    latchWaitStart   = millis();
    logEvent("LATCH_WAITING");

  } else if (currentState != OPENING && digitalRead(openLimitSwitch) == LOW) {
    startOpening();
  }
}

void startClosing() {
  if (currentState != CLOSING && digitalRead(closeLimitSwitch) == LOW) {
    logEvent("START_CLOSING");
    digitalWrite(motorEN1, HIGH);
    digitalWrite(motorEN2, HIGH);
    currentState       = CLOSING;
    powerOffset        = 0;
    moveStartTime      = millis();
    lastCorrectionTime = millis();
    sessionStartTime   = millis();
    diagActive         = true;
    resetSession();
    Serial.println("---CSV_START---");
    Serial.println("time_ms,ses_ms,dir,power,offset,pulses,exp%,act%,event");
    updatePower();
  }
}

void startStopping() {
  if (currentState == STOP) return;

  unsigned long pulses = getSessionPulses();

  // Автокалибровка
  if (isCalibrating && pulses > 10) {
    if (currentState == OPENING) {
      PULSES_OPEN = pulses;
      Serial.print("CALIB PULSES_OPEN="); Serial.println(PULSES_OPEN);
    } else if (currentState == CLOSING) {
      PULSES_CLOSE = pulses;
      Serial.print("CALIB PULSES_CLOSE="); Serial.println(PULSES_CLOSE);
    }
    if (PULSES_OPEN > 0 && PULSES_CLOSE > 0) {
      isCalibrating = false;
      Serial.print("CALIBRATION DONE: OPEN="); Serial.print(PULSES_OPEN);
      Serial.print(" CLOSE="); Serial.println(PULSES_CLOSE);
    }
  }

  logEvent("STOP");
  Serial.print("Time:"); Serial.print(millis() - moveStartTime);
  Serial.print("ms Pulses:"); Serial.println(pulses);

  previewState = currentState;
  currentState = STOP;
  analogWrite(motorPWD1, 0);
  analogWrite(motorPWD2, 0);
  diagActive = false;
  Serial.println("---CSV_END---");
  resetSession();
}

// ========== КНОПКА И КОНЦЕВИКИ ==========
void handleButton() {
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) lastDebounceTime = millis();
  if (millis() - lastDebounceTime > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        logEvent("BTN");
        lastActivityTime = millis();

        if (waitingForLatch) {
          waitingForLatch = false;
          startStopping();
          return;
        }

        if (currentState == STOP) {
          if (!powerState) {
            digitalWrite(powerPin, !powerRelayLOW);
            powerState     = true;
            powerStartTime = millis();
          }
          if (millis() - powerStartTime >= MOTOR_DELAY) {
            if      (previewState == OPENING) startClosing();
            else if (previewState == CLOSING) startOpeningWithLatch();
            startPowerMake = false;
          } else {
            startPowerMake = true;
          }
        } else {
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
    if      (previewState == OPENING) startClosing();
    else if (previewState == CLOSING) startOpeningWithLatch();
    startPowerMake = false;
  }
}

void checkLimitSwitches() {
  if (digitalRead(openLimitSwitch) == HIGH && currentState == OPENING) {
    logEvent("OPEN_LIMIT");
    lastActivityTime = millis();
    startStopping();
  }
  if (digitalRead(closeLimitSwitch) == HIGH && currentState == CLOSING) {
    logEvent("CLOSE_LIMIT");
    lastActivityTime = millis();
    startStopping();
  }
}

void checkInactivity() {
  if (powerState && millis() - lastActivityTime > INACTIVITY_TIMEOUT) {
    logEvent("INACTIVITY");
    digitalWrite(powerPin, powerRelayLOW);
    powerState = false;
    startStopping();
    digitalWrite(motorEN1, LOW);
    digitalWrite(motorEN2, LOW);
  }
}

// ========== SETUP & LOOP ==========
void setup() {
  Serial.begin(115200);
  Serial.println("=== GATE v4.2 ===");

  pinMode(buttonPin,         INPUT_PULLUP);
  pinMode(openLimitSwitch,   INPUT_PULLUP);
  pinMode(closeLimitSwitch,  INPUT_PULLUP);
  pinMode(magnetLimitSwitch, INPUT_PULLUP);
  pinMode(motorEN1,  OUTPUT); pinMode(motorEN2,  OUTPUT);
  pinMode(motorPWD1, OUTPUT); pinMode(motorPWD2, OUTPUT);
  pinMode(powerPin,  OUTPUT); pinMode(latchPin,  OUTPUT);
  pinMode(ledPin,    OUTPUT);

  digitalWrite(motorEN1, LOW); digitalWrite(motorEN2, LOW);
  analogWrite(motorPWD1, 0);   analogWrite(motorPWD2, 0);
  digitalWrite(powerPin, powerRelayLOW);
  digitalWrite(latchPin, latchRelayLOW);

  attachInterrupt(digitalPinToInterrupt(magnetLimitSwitch),
                  rpmSensorInterrupt, FALLING);

  if      (digitalRead(closeLimitSwitch) == HIGH) { previewState = CLOSING;  Serial.println("Init: CLOSED"); }
  else if (digitalRead(openLimitSwitch)  == HIGH) { previewState = OPENING;  Serial.println("Init: OPEN"); }
  else                                             { Serial.println("Init: UNKNOWN"); }

  if (PULSES_OPEN == 0 || PULSES_CLOSE == 0) {
    isCalibrating = true;
    Serial.println("*** CALIBRATION MODE ***");
  }

  Serial.print("PULSES_OPEN=");   Serial.print(PULSES_OPEN);
  Serial.print(" PULSES_CLOSE="); Serial.println(PULSES_CLOSE);
  Serial.println("Ready.");
}

void loop() {
  handleButton();
  checkAfterClick();
  checkLimitSwitches();
  checkInactivity();
  checkStall();

  // Ожидание после защёлки
  if (waitingForLatch) {
    if (digitalRead(closeLimitSwitch) == LOW) {
      logEvent("LATCH_OK");
      waitingForLatch = false;
      startOpening();
    } else if (millis() - latchWaitStart > LATCH_WAIT_TIMEOUT) {
      logEvent("LATCH_TIMEOUT");
      waitingForLatch = false;
      startStopping();
    }
  }

  if (currentState == OPENING || currentState == CLOSING) {
    updatePower();
  } else if (currentState == STOP) {
    analogWrite(motorPWD1, 0);
    analogWrite(motorPWD2, 0);
  }

  if (diagActive && millis() - diagTimer >= 200) {
    printCSVRow();
    diagTimer = millis();
  }

  static unsigned long hb = 0;
  if (currentState == STOP && millis() - hb > 5000) {
    Serial.print("IDLE OpenLS:"); Serial.print(digitalRead(openLimitSwitch));
    Serial.print(" CloseLS:");    Serial.print(digitalRead(closeLimitSwitch));
    Serial.print(" Open:");       Serial.print(PULSES_OPEN);
    Serial.print(" Close:");      Serial.println(PULSES_CLOSE);
    digitalWrite(ledPin, !digitalRead(ledPin));
    hb = millis();
  }
}
