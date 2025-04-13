const bool powerRelayLOW = HIGH;    // LOW or HIGH
const bool magnetRelayLOW = HIGH;   // LOW or HIGH

// Пины
const int buttonPin = 4;          // Кнопка на пине 4
const int openLimitSwitch = 2;    // Концевик открытия на пине 2
const int closeLimitSwitch = 3;   // Концевик закрытия на пине 3

const int motorEN1 = 7;           // Пин питания направления
const int motorEN2 = 8;           // Пин питания направления
const int motorPWD1 = 5;          // ШИМ мотор на пине 5
const int motorPWD2 = 6;          // ШИМ мотор на пине 6

const int relayPin = 9;           // Реле питания двигателя
const int magnetPin = 10;         // Реле магнита на пине 10

// Состояния управления
int currentDirection = 0;        // 0-стоп, 1-открытие, 2-закрытие
int previewDirection  = 0;
bool isStopping = false;
int currentSpeed = 0;
const int maxSpeed = 255;
const int accelerationStep = 5;
const unsigned long accelerationInterval = 50;

// Таймеры и флаги
bool relayState = false;
bool magnetState = false;
unsigned long relayStartTime = 0;
unsigned long magnetDelayStart = 0;
unsigned long lastActivityTime = 0;
const unsigned long 
  MOTOR_DELAY = 2000,          // 2 секунды задержки двигателя
  MAGNET_DELAY = 1000,         // 1 секунда задержки после магнита
  INACTIVITY_TIMEOUT = 300000; // 5 минут неактивности

// Для остановки по времени
unsigned long fullOpenTime = 0;   // Время нужное для открытия ворот
unsigned long fullCloseTime = 0;  //  Время нужное для закрытия ворот
unsigned long moveTime = 0;       // Время в пути
unsigned long tempMoveTime = 0;   // Для прибовления времени
int previewDirectionForTimer = 0;

// Антидребезг
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
  
  pinMode(motorEN1, OUTPUT);
  pinMode(motorEN2, OUTPUT);
  pinMode(motorPWD1, OUTPUT);
  pinMode(motorPWD2, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(magnetPin, OUTPUT);
  
  // Инициализация состояний
  digitalWrite(motorEN1, LOW);
  digitalWrite(motorEN2, LOW);
  analogWrite(motorPWD1, 0);
  analogWrite(motorPWD2, 0);
  digitalWrite(relayPin, powerRelayLOW);
  
  // Проверка начального положения ворот
  if(digitalRead(closeLimitSwitch) == LOW) {
    digitalWrite(magnetPin, !magnetRelayLOW);
    magnetState = true;
  }
  Serial.println("Setup end");
}

void loop() {
  handleButton();
  checkLimitSwitches();
  checkMagnetDelay();
  // checkMovementTime();
  updateMotorSpeed();
  checkInactivity();
}

void checkMovementTime()  {
  if (currentDirection != 0) {
    if (currentDirection != previewDirectionForTimer) {
      int fullTime = currentDirection == 1 ? fullOpenTime : fullCloseTime;
      int rfullTime = currentDirection == 1 ? fullCloseTime : fullOpenTime;
      moveTime = moveTimeRevers(moveTime, fullTime, rfullTime);
      previewDirectionForTimer = currentDirection;
    }
    moveTime += millis() - tempMoveTime;
  }
  tempMoveTime = millis();
}

void checkInactivity() {
  if(relayState && (millis() - lastActivityTime > INACTIVITY_TIMEOUT)) {
    Serial.println("Inactivity");
    digitalWrite(relayPin, powerRelayLOW);
    relayState = false;
    currentDirection = 0;
    digitalWrite(motorEN1, LOW);
    digitalWrite(motorEN2, LOW);
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
        
        if(currentDirection == 0) {
          if(!relayState) {
            digitalWrite(relayPin, !powerRelayLOW);
            relayState = true;
            relayStartTime = millis();
          }
          
          if(millis() - relayStartTime >= MOTOR_DELAY) {
            if (previewDirection == 1 && digitalRead(openLimitSwitch) == LOW) {
              startClosing();
            }
            else if(previewDirection == 2 && digitalRead(closeLimitSwitch) == LOW) {
              if(magnetState) {
                digitalWrite(magnetPin, magnetRelayLOW);
                magnetState = false;
                magnetDelayStart = millis();
              } else {
                startOpening();
              }
            }
          }
        } else {
          startStopping();
        }
      }
    }
  }
  lastButtonState = reading;
}

void checkMagnetDelay() {
  if(magnetDelayStart > 0 && (millis() - magnetDelayStart >= MAGNET_DELAY)) {
    magnetDelayStart = 0;
    startOpening();
  }
}

void checkLimitSwitches() {
  // Обработка концевика открытия
  if(digitalRead(openLimitSwitch) == LOW && currentDirection == 1) {
    Serial.println("OpenLimitSwitch");
    lastActivityTime = millis();
    startStopping();
  }
  
  // Обработка концевика закрытия
  if(digitalRead(closeLimitSwitch) == LOW && currentDirection == 2) {
    Serial.println("CloseLimitSwitch");
    lastActivityTime = millis();
    digitalWrite(magnetPin, !magnetRelayLOW);
    magnetState = true;
    startStopping();
  }
}

void updateMotorSpeed() {
  if(millis() - lastAccelTime >= accelerationInterval) {
    lastAccelTime = millis();
    
    if(isStopping) {
      currentSpeed = max(currentSpeed - accelerationStep, 0);
      analogWrite(motorPWD1, currentSpeed);
      analogWrite(motorPWD2, currentSpeed);
      
      if(currentSpeed == 0) {
        digitalWrite(motorEN1, LOW);
        digitalWrite(motorEN2, LOW);
        currentDirection = 0;
        isStopping = false;
      }
    } 
    else if(currentDirection != 0) {
      if(millis() - relayStartTime >= MOTOR_DELAY && magnetDelayStart == 0) {
        currentSpeed = min(currentSpeed + accelerationStep, maxSpeed);
        analogWrite(motorPWD1, currentSpeed);
        analogWrite(motorPWD2, currentSpeed);
      }
    }
  }
}

void startOpening() {
  if(currentDirection != 1) {
    Serial.println("Start Opening");
    digitalWrite(motorEN1, HIGH);
    digitalWrite(motorEN2, LOW);
    currentDirection = 1;
    isStopping = false;
    currentSpeed = 0;
  }
}

void startClosing() {
  if(currentDirection != 2) {
    Serial.println("Start Closing");
    digitalWrite(motorEN1, LOW);
    digitalWrite(motorEN2, HIGH);
    currentDirection = 2;
    isStopping = false;
    currentSpeed = 0;
    digitalWrite(magnetPin, magnetRelayLOW);
    magnetState = false;
  }
}

void startStopping() {
  if(currentDirection != 0) {
    Serial.println("Start Stoping");
    isStopping = true;
    previewDirection = currentDirection;
  }
}

void emergencyStop() {
  Serial.println("Emergency Stop");
  digitalWrite(relayPin, LOW);
  digitalWrite(motorEN1, LOW);
  digitalWrite(motorEN2, LOW);
  currentDirection = 0;
  previewDirection = currentDirection;
}

unsigned long moveTimeRevers(unsigned long time, unsigned long fullNowTime, unsigned long fullReversTime) {
  return (fullReversTime * ((time * 100) / fullNowTime)) / 100;
}