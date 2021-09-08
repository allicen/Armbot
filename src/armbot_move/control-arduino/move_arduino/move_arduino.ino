#include <AccelStepper.h>
#include <Servo.h>

#include <ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

// 1 joint
#define Z_STEP_PIN 46
#define Z_DIR_PIN 48
#define Z_ENABLE_PIN 62

#define SERVO_COUNT 2
#define JOINT_1_PIN 11
#define JOINT_GRIP_PIN 6

#define speed 12000

std_msgs::String str; 
ros::NodeHandle nodeHandle;

// Кнопка и светодиод
int buttonPin = 16;
int ledPin = 17;
boolean lastButton = LOW;
boolean currentButton = LOW;
boolean ledOn = false;

AccelStepper stepper(1, Z_STEP_PIN, Z_DIR_PIN);
Servo robotServos[SERVO_COUNT];

void motorControlSubscriberCallbackJointState(const sensor_msgs::JointState& msg) {
  float joint_1 = msg.position[0];
  float joint_2 = msg.position[1];
  float joint_3 = msg.position[2];
  float joint_4 = msg.position[3];
  float joint_grip = msg.position[4];

  char joint_1_test[8];
  char joint_2_test[8];
  char joint_3_test[8];
  char joint_4_test[8];
  char joint_grip_test[8];

  dtostrf(joint_1, 6, 2, joint_1_test);
  dtostrf(joint_2, 6, 2, joint_2_test); 
  dtostrf(joint_3, 6, 2, joint_3_test); 
  dtostrf(joint_4, 6, 2, joint_4_test); 
  dtostrf(joint_grip, 6, 2, joint_grip_test); 

  nodeHandle.logwarn(joint_1_test);
}

ros::Subscriber<sensor_msgs::JointState> motorControlSubscriberJointState("joint_states", &motorControlSubscriberCallbackJointState);

void setup() {
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);

  stepper.setPinsInverted(false, false, true);

  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(Z_ENABLE_PIN, LOW);

  Serial.begin(115200);

  stepper.move(0);
  stepper.setMaxSpeed(speed);

  nodeHandle.getHardware()->setBaud(115200);
  nodeHandle.initNode();
  nodeHandle.subscribe(motorControlSubscriberJointState);
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

    while (ledOn) {
      stepper.runSpeed();
      buttonPressed();
    }
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
