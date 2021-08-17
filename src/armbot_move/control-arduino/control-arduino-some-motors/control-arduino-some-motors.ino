#include <Servo.h>
#include "AccelStepper.h"

#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38

#define SERVO_DOWN_PIN 7
#define SERVO_TOP_PIN 6

#define BUTTON_PIN 2
#define LED_PIN 8

AccelStepper stepper(1, X_STEP_PIN, X_DIR_PIN);
Servo servoDown;
Servo servoTop;

boolean lastButton = LOW;
boolean currentButton = LOW;
boolean ledOn = false;
boolean servoDownStraight = true;
boolean servoTopStraight = true;

int servoDownMin = 0;
int servoDownMax = 180;
int servoTopMin = 100;
int servoTopMax = 180;

int servoDownPosition = servoDownMin;
int servoTopPosition = servoTopMin;

int stepperMotorStepCount = 0;

void setup() {
  servoDown.attach(SERVO_DOWN_PIN);
  servoTop.attach(SERVO_TOP_PIN);

  servoDown.write(servoDownMin);
  servoTop.write(servoTopMax);

  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  stepper.setMaxSpeed(8000);
  stepper.setAcceleration(2000);

  stepper.setEnablePin(X_ENABLE_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();
}

void loop() {
    currentButton = debounce(lastButton);
    if (currentButton == HIGH && lastButton == LOW) {

      ledOn = !ledOn;

    }

    lastButton = currentButton;

    digitalWrite(LED_PIN, ledOn);

    servoStart(ledOn);
}

// стабилизация нажатия на кнопку
boolean debounce (boolean last) {
  boolean current = digitalRead(BUTTON_PIN);
  if (last != current) {
    delay(5);
    current = digitalRead(BUTTON_PIN);
  }

  return current;
}


void servoStart(boolean start) {
  if (start) {

      servoRun('d');
      if (!servoDownStraight) {
        servoTop.write(servoTopMax);
        runStepper/Motor(1000);
        servoRun('t');
      }
      
      runStepperMot/or(100);
      
      delay(15);
  }
}


void servoRun(char servoName) {
   switch (servoName) {
     case 'd': // servo down
       if (servoDownStraight) {
        servoDownPosition++;
       } else {
          servoDownPosition--;
       }

       changeDirection('d');
       servoDown.write(servoDownPosition);
       
     break;
     case 't': // servo top
       if (servoTopStraight) {
        servoTopPosition++;
       } else {
          servoTopPosition--;
       }
       
       changeDirection('t');
       servoTop.write(servoTopPosition);
       
     break;
  }
}


void changeDirection(char servoName) {
  switch (servoName) {
     case 'd': // servo down
       if (servoDownPosition == servoDownMax) {
          servoDownStraight = false;
       } else if (servoDownPosition == servoDownMin) {
          servoDownStraight = true;
       }
     break;
     case 't': // servo top
       if (servoTopPosition == servoTopMax) {
          servoTopStraight = false;
       } else if (servoTopPosition == servoTopMin) {
          servoTopStraight = true;
       }
     break;
  }
}


void runStepperMotor(int stepCount) {
  stepper.moveTo(stepCount);
  
  while(stepper.distanceToGo()!=0){
    stepper.run();
  }
}
