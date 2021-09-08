#include <AccelStepper.h>

#define Z_STEP_PIN 46
#define Z_DIR_PIN 48
#define Z_ENABLE_PIN 62
#define Z_MIN_PIN 18
#define Z_MAX_PIN 19

#define maxSpeed 100000000

long startTime, finishTime, diffTime;

long currentPosition = 0;
long currentPositionPrint = 0;
long diffPosition = 0;
long speed = 10;

int buttonPin = 16;
int ledPin = 17;
boolean lastButton = LOW;
boolean currentButton = LOW;
boolean ledOn = false;


AccelStepper stepper(1, Z_STEP_PIN, Z_DIR_PIN);

void setup() {
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);

  stepper.setPinsInverted(false, false, true);

  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(Z_ENABLE_PIN, LOW);

  Serial.begin(115200);
  
  changeSpeed();

  stepper.move(0);
  stepper.setMaxSpeed(maxSpeed);
}

void loop() {
  start();
}

void start() {
  stepper.setSpeed(speed); // скорость в секунду
  
  buttonPressed();
  runStepper();
}

void runStepper() {
  if (ledOn) {
    
    startTime = millis();

    while (ledOn) {
      stepper.runSpeed();
      buttonPressed();

      if (currentPosition != stepper.currentPosition()) {
        currentPositionPrint = stepper.currentPosition() - diffPosition;
        //Serial.println(currentPositionPrint);
        currentPosition = stepper.currentPosition();
      }
    }

    finishTime = millis();
    diffTime = finishTime - startTime;
    
    Serial.println((String) "Затрачено времени: " + diffTime + " ms");
    Serial.println((String) "Пройдено шагов: " + currentPositionPrint + " ms");
    changeSpeed();
    
    diffPosition = stepper.currentPosition();
  }
}

// стабилизация нажатия на кнопку
boolean debounce (boolean last) {
  boolean current = digitalRead(buttonPin);
  if (last != current) {
    delay(5);
    current = digitalRead(buttonPin);
  }

  return current;
}

// обработка нажатия на кнопку
void buttonPressed () {
    currentButton = debounce(lastButton);
    if (currentButton == HIGH && lastButton == LOW) {
      ledOn = !ledOn;
    }

    lastButton = currentButton;
    digitalWrite(ledPin, ledOn);
}


long readSpeed () {
  while (!Serial.available());
  String command = Serial.readStringUntil('\n');

  return command.toInt();
}


void changeSpeed () {
    Serial.println("\n=========== Начало тестирования ===========");
    Serial.println("Введите скорость (одно целое число от 1 до 100000): ");
    long command = readSpeed();
    Serial.println((String) "Введена скорость: " + command);

    speed = command;
}
