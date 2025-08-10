// VERSION 3.3.9

// Для легкой настройки обратный реле
const bool powerRelayLOW = LOW;    // LOW or HIGH
const bool magnetRelayLOW = HIGH;    // LOW or HIGH
const int maxSpeedConst = 204;
const int minSpeedConst = 0;

// Пины
const int buttonPin = 4;          // Кнопка на пине 4
const int openLimitSwitch = 2;    // Концевик открытия на пине 2
const int closeLimitSwitch = 3;   // Концевик закрытия на пине 3
const int magnetLimitSwitch = 11;  // Магнитный концевик для начала остановки

const int motorEN1 = 7;           // Пин питания направления
const int motorEN2 = 8;           // Пин питания направления
const int motorPWD1 = 6;          // ШИМ мотор на пине 5
const int motorPWD2 = 5;          // ШИМ мотор на пине 6

const int powerPin = 10;          // Реле питания двигателя
const int unlockMagnetPin = 9;            // Реле электромакнита который открывает
const int ledPin = 13;

// Настройки параметры
const unsigned long 
  MOTOR_DELAY = 2000,           // 2 секунды задержки двигателя
  MAGNET_DELAY = 500,          // 0.5 секунда задержки после магнита
  INACTIVITY_TIMEOUT = 17000,   // 30 секунд неактивности
  accelerationInterval = 50;    // Каждые 50 мс, обновлять скорость
int maxSpeed = 255;
int minSpeed = 0;
const int accelerationStep = 5; // Шаг разгона

// Состояние управления
enum State { STOP, OPENING, CLOSING };
State currentState = STOP;
State previewState = OPENING;
bool isStopping = false;
int currentSpeed = 0;

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

unsigned long lastAccelTime = 0;

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
  checkLimitSwitches();
  checkMagnetDelay();
  //checkMovementTime();
  updateMotorSpeed();
  checkInactivity();
  
  if(millis() - notSleepTime > 5000) {
    Serial.println("Not Sleep!");
    Serial.print("Open limit: ");
    Serial.println(digitalRead(openLimitSwitch));
    Serial.print("Close limit: ");
    Serial.println(digitalRead(closeLimitSwitch));
    digitalWrite(ledPin, ledState);
    ledState = !ledState;
    Serial.flush();
    notSleepTime = millis();
  }
}

void checkMovementTime()  {
  if(millis() - tempMoveTime >= 250) {
    if (isStartFromLimitSwitch && currentState != STOP) {
      Serial.println("Check Movement");
      // Подсчет времени
      // if (currentState != previewStateTime) {
      //   Serial.println("Revers moveTime");
      //   int fullTime = currentState == OPENING ? fullOpenTime : fullCloseTime;
      //   int rfullTime = currentState == OPENING ? fullCloseTime : fullOpenTime;
      //   moveTime = moveTimeRevers(moveTime, fullTime, rfullTime);
      //   previewStateTime = currentState;
      // }
      if (currentState == OPENING) moveTime += millis() - tempMoveTime;
      else if (currentState == CLOSING) moveTime -= millis() - tempMoveTime;
      if (moveTime < 0) moveTime = 0;
      Serial.print("moveTime = ");
      Serial.println(moveTime);
      // Определение когда будем тормозить, если скорость + время на торможение превышают время до концевика, останавливаемся
      if (!isNotLimitSwitch && currentState == OPENING && fullOpenTime > 0 /*&& fullCloseTime > 0*/ && moveTime + (((maxSpeed / accelerationStep) * accelerationInterval) * 0/*2.8*/) > fullOpenTime) { // currentState == OPENING ? fullOpenTime : fullCloseTime) {
        Serial.println("Check Movement stopping");
        isTimeStopping = true;
        startStopping();
        minSpeed = 102;
      }
    }
    tempMoveTime = millis();
  }
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
              digitalWrite(unlockMagnetPin, !magnetRelayLOW);
              magnetDelayStart = millis();
              delay(500);
              digitalWrite(unlockMagnetPin, magnetRelayLOW);

              digitalWrite(motorEN1, HIGH);
              digitalWrite(motorEN2, HIGH);
              analogWrite(motorPWD1, 204);
              delay(50);
              digitalWrite(motorEN1, LOW);
              digitalWrite(motorEN2, LOW);
              analogWrite(motorPWD1, 0);
            }
            startPowerMake = false;
          }else{
            startPowerMake = true;
            // if (digitalRead(closeLimitSwitch) == LOW)
            //   startClosing();
            // else{
            //   if(magnetState) {
            //     digitalWrite(magnetPin, magnetRelayLOW);
            //     magnetState = false;
            //     magnetDelayStart = millis();
            //   } else {
            //     startOpening();
            //   };
            // }
          }
        } else {
          minSpeed = 0;
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
      digitalWrite(unlockMagnetPin, !magnetRelayLOW);
      magnetDelayStart = millis();
      delay(500);
      digitalWrite(unlockMagnetPin, magnetRelayLOW);
      digitalWrite(motorEN1, HIGH);
      digitalWrite(motorEN2, HIGH);
      analogWrite(motorPWD1, 204);
      delay(50);
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
  if (digitalRead(magnetLimitSwitch) == LOW && currentState == OPENING) {
    minSpeed = 76;
    startStopping();
  }
  // Обработка концевика открытия
  if(digitalRead(openLimitSwitch) == HIGH && currentState == OPENING) {
    Serial.println("OpenLimitSwitch");
    lastActivityTime = millis();
    minSpeed = 0;
    startStopping();

    //if (isTimeStopping) currectTimeStopping();
    //moveTime = fullOpenTime;
    //isStartFromLimitSwitch = true;
  }
  
  // Обработка концевика закрытия
  if(digitalRead(closeLimitSwitch) == HIGH && currentState == CLOSING) {
    Serial.println("CloseLimitSwitch");
    lastActivityTime = millis();
        
    moveTime = 0;
    isStartFromLimitSwitch = true;
    
    minSpeed = 0;
    maxSpeed = 20;
    startStopping();
  }
}

void updateMotorSpeed() {
  if(millis() - lastAccelTime >= accelerationInterval) {
    lastAccelTime = millis();
    
    if(maxSpeed < currentSpeed) currentSpeed = maxSpeed;
    else if(minSpeed > currentSpeed) currentSpeed = minSpeed;
    if(isStopping) {
      currentSpeed = max(currentSpeed - accelerationStep, minSpeed);
      analogWrite(motorPWD1, currentState == OPENING ? currentSpeed : 0);
      analogWrite(motorPWD2, currentState == CLOSING ? currentSpeed : 0);
      
      if(currentSpeed == 0) {
        digitalWrite(motorEN1, LOW);
        digitalWrite(motorEN2, LOW);
        previewState = currentState;
        currentState = STOP;
        isStopping = false;
        minSpeed = 0;
        maxSpeed = maxSpeedConst;
        // if (isTimeStopping) currectTimeStopping();
      }
    }
    else if(currentState != STOP) {
      if(millis() - powerStartTime >= MOTOR_DELAY && magnetDelayStart == 0) {
        currentSpeed = min(currentSpeed + accelerationStep, maxSpeed);
        if(currentSpeed < minSpeed) currentSpeed =  minSpeed;
        analogWrite(motorPWD1, currentState == OPENING ? currentSpeed : 0);
        analogWrite(motorPWD2, currentState == CLOSING ? currentSpeed : 0);
      }
    }
  }
}

void checkInactivity() {
  if(powerState && (millis() - lastActivityTime > INACTIVITY_TIMEOUT)) {
    Serial.println("Inactivity");
    digitalWrite(powerPin, powerRelayLOW);
    powerState = false;
    currentState = STOP;
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
    isStopping = false;
    minSpeed = 0;  // old 250
    maxSpeed = maxSpeedConst;
    currentSpeed = 0;
  }
}

void startClosing() {
  Serial.println("Try startClosing");
  if(currentState != CLOSING && digitalRead(closeLimitSwitch) == LOW) {
    Serial.println("Start Closing");
    digitalWrite(motorEN1, HIGH);
    digitalWrite(motorEN2, HIGH);
    currentState = CLOSING;
    isStopping = false;
    minSpeed = 0;
    maxSpeed = maxSpeedConst;
    currentSpeed = 0;
  }
}

void startStopping() {
  Serial.println("Try startStopping");
  if(currentState != STOP) {
    //minSpeed = 0;
    Serial.println("Start Stoping");
    isStopping = true;
  }
}

void emergencyStop() {
  Serial.println("Emergency Stop");
  digitalWrite(powerPin, LOW);
  digitalWrite(motorEN1, LOW);
  digitalWrite(motorEN2, LOW);
  previewState = currentState;
  currentState = STOP;
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
            maxSpeed = 127;
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
