// Для легкой настройки обратный реле
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

const int powerPin = 9;           // Реле питания двигателя
const int magnetPin = 10;         // Реле магнита на пине 10

// Настройки параметры
const unsigned long 
  MOTOR_DELAY = 2000,           // 2 секунды задержки двигателя
  MAGNET_DELAY = 1000,          // 1 секунда задержки после магнита
  INACTIVITY_TIMEOUT = 300000,  // 5 минут неактивности
  accelerationInterval = 50;    // Каждые 50 мс, обновлять скорость
const int maxSpeed = 255;
const int accelerationStep = 5; // Шаг разгона

// Состояние управления
enum State { STOP, OPENING, CLOSING };
State currentState = STOP;
State previewState = CLOSING;
bool isStopping = false;
int currentSpeed = 0;

// Таймеры и флаги
bool powerState = false;
bool magnetState = false;
unsigned long powerStartTime = 0;
unsigned long magnetDelayStart = 0;
unsigned long lastActivityTime = 0;

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
  
  pinMode(motorEN1, OUTPUT);
  pinMode(motorEN2, OUTPUT);
  pinMode(motorPWD1, OUTPUT);
  pinMode(motorPWD2, OUTPUT);
  pinMode(powerPin, OUTPUT);
  pinMode(magnetPin, OUTPUT);
  
  // Инициализация состояний
  digitalWrite(motorEN1, LOW);
  digitalWrite(motorEN2, LOW);
  analogWrite(motorPWD1, 0);
  analogWrite(motorPWD2, 0);
  digitalWrite(powerPin, powerRelayLOW);
  
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
              if(magnetState) {
                digitalWrite(magnetPin, magnetRelayLOW);
                magnetState = false;
                magnetDelayStart = millis();
              } else {
                startOpening();
              }
            }
          }else{
            if (digitalRead(openLimitSwitch) == HIGH)
              startOpening();
            else startClosing();
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
  if(digitalRead(openLimitSwitch) == LOW && currentState == OPENING) {
    Serial.println("OpenLimitSwitch");
    lastActivityTime = millis();
    startStopping();
  }
  
  // Обработка концевика закрытия
  if(digitalRead(closeLimitSwitch) == LOW && currentState == CLOSING) {
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
      analogWrite(motorPWD1, currentState == OPENING ? currentSpeed : 0);
      analogWrite(motorPWD2, currentState == CLOSING ? currentSpeed : 0);
      
      if(currentSpeed == 0) {
        digitalWrite(motorEN1, LOW);
        digitalWrite(motorEN2, LOW);
        currentState = STOP;
        isStopping = false;
      }
    }
    else if(currentState != STOP) {
      if(millis() - powerStartTime >= MOTOR_DELAY && magnetDelayStart == 0) {
        currentSpeed = min(currentSpeed + accelerationStep, maxSpeed);
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
  if(currentState != OPENING && digitalRead(openLimitSwitch) == HIGH) {
    Serial.println("Start Opening");
    digitalWrite(motorEN1, HIGH);
    digitalWrite(motorEN2, HIGH);
    currentState = OPENING;
    isStopping = false;
    currentSpeed = 0;
  }
}

void startClosing() {
  if(currentState != CLOSING && digitalRead(closeLimitSwitch) == HIGH) {
    Serial.println("Start Closing");
    digitalWrite(motorEN1, HIGH);
    digitalWrite(motorEN2, HIGH);
    currentState = CLOSING;
    isStopping = false;
    currentSpeed = 0;
    digitalWrite(magnetPin, magnetRelayLOW);
    magnetState = false;
  }
}

void startStopping() {
  if(currentState != STOP) {
    Serial.println("Start Stoping");
    isStopping = true;
    previewState = currentState;
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

unsigned long moveTimeRevers(unsigned long time, unsigned long fullNowTime, unsigned long fullReversTime) {
  return (fullReversTime * ((time * 100) / fullNowTime)) / 100;
}
