#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

#include <ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

// 1 joint
#define Z_STEP_PIN   46
#define Z_DIR_PIN    48
#define Z_ENABLE_PIN 62

// 2 joint
#define X_STEP_PIN   54
#define X_DIR_PIN    55
#define X_ENABLE_PIN 38

// 3 joint
#define Y_STEP_PIN   60
#define Y_DIR_PIN    61
#define Y_ENABLE_PIN 56

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

AccelStepper stepper_z(1, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepper_x(1, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepper_y(1, Y_STEP_PIN, Y_DIR_PIN);

MultiStepper steppers;

Servo robotServos[SERVO_COUNT];


long positions[5]; // 3 ШД и 2 сервопривода

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

// Инициализация двигателей
void initStepper(AccelStepper stepper, int stepPin, int dirPin, int enablePin) {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  stepper.setPinsInverted(false, false, true);
  digitalWrite(enablePin, LOW);

  stepper.move(0);
  stepper.setMaxSpeed(speed);
  stepper.setSpeed(speed);
}

void setup() {
  initStepper(stepper_z, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
  initStepper(stepper_x, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
  initStepper(stepper_y, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);

  steppers.addStepper(stepper_z);
  steppers.addStepper(stepper_x);
  steppers.addStepper(stepper_y);

  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);

  Serial.begin(115200);

  nodeHandle.getHardware()->setBaud(115200);
  nodeHandle.initNode();
  nodeHandle.subscribe(motorControlSubscriberJointState);
}

void loop() {
  start();
}

void start() {
  buttonPressed();
  runStepper();
}

void runStepper() {
  if (ledOn) {

    while (ledOn) {
      positions[0] = -100; // Пример
      positions[1] = 100;
      
      steppers.moveTo(positions);
      steppers.runSpeedToPosition();
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
