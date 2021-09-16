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

#define JOINT_END_PIN 11
#define JOINT_GRIP_PIN 6

#define MOTOR_COUNT 5
#define MOTOR_STEP_COUNT 3
#define SERVO_COUNT 2

#define SPEED 12000
#define M_PI 3.14159265358979323846
#define RADIAN 57.3

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


//////////// для отладки
float joint_1_rem;
float joint_2_rem;
float joint_3_rem;
float joint_4_rem;
float joint_grip_rem;



//////////////// start

int servo_pins[SERVO_COUNT] = {JOINT_END_PIN, JOINT_GRIP_PIN}; // PWM Pins

int motorCurrentPositions[MOTOR_COUNT] = {0, 0, 0, 0, 0};
float targetJointPositions[MOTOR_COUNT] = {0, 0, 0, 0, 0};


/////////// end

// Convert the joint state values to degrees, adjust for the center and write to the servo
void writeMotors() {

    // Записываем шаговые двигатели
    for (int i = 0; i < MOTOR_STEP_COUNT; i++) {
      
      
    }

    // Записываем сервоприводы
    for (int i = 0; i < SERVO_COUNT; i++) {
      int targetAngle;

      if (i == 0) {
        targetAngle = targetJointPositions[3] *(90/3.14);
      } else {
        targetAngle = targetJointPositions[4] *(72/3.14);
      }
      
      robotServos[i].write(targetAngle);
      motorCurrentPositions[i] = targetAngle;
    }
    
  nodeHandle.spinOnce();
}

// Subscriber Callback to store the jointstate position values in the global variables
void motorControlSubscriberCallbackJointState(const sensor_msgs::JointState& msg) {

  float joint_1 = msg.position[0] + 2.25;
  float joint_2 = msg.position[1];
  float joint_3 = msg.position[2];
  float joint_4 = msg.position[3] + M_PI;
  float joint_grip = msg.position[4] + 2.50;

  float joints_correct[MOTOR_COUNT] = {joint_1, joint_2, joint_3, joint_4, joint_grip};

  if (joint_1 != joint_1_rem || 
      joint_2 != joint_2_rem || 
      joint_3 != joint_3_rem || 
      joint_4 != joint_4_rem || 
      joint_grip != joint_grip_rem) {

        joint_1_rem = joint_1;
        joint_2_rem = joint_2;
        joint_3_rem = joint_3;
        joint_4_rem = joint_4;
        joint_grip_rem = joint_grip;

        char joint_1_current[8];
        char joint_2_current[8];
        char joint_3_current[8];
        char joint_4_current[8];
        char joint_grip_current[8];
      
        dtostrf(joint_1, 1, 5, joint_1_current);
        dtostrf(joint_2, 1, 5, joint_2_current); 
        dtostrf(joint_3, 1, 5, joint_3_current); 
        dtostrf(joint_4, 1, 5, joint_4_current); 
        dtostrf(joint_grip, 1, 5, joint_grip_current);
  
        nodeHandle.logerror("==============");
        nodeHandle.logwarn(joint_1_current);
        nodeHandle.logwarn(joint_2_current);
        nodeHandle.logwarn(joint_3_current);
        nodeHandle.logwarn(joint_4_current);
        nodeHandle.logwarn(joint_grip_current);
  }


  for (int i = 0; i < MOTOR_COUNT; i++) {
    targetJointPositions[i] = joints_correct[i];
  }

  writeMotors();

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
  stepper.setMaxSpeed(SPEED);
  stepper.setSpeed(SPEED);
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

  
  // Инициализировать сервоприводы
  for (unsigned int i = 0; i < SERVO_COUNT; i++) {
    robotServos[i].attach(servo_pins[i]);
  }
  
  nodeHandle.getHardware()->setBaud(115200);
  nodeHandle.initNode();
  nodeHandle.subscribe(motorControlSubscriberJointState);
}

void loop() {
  start();
  nodeHandle.spinOnce();
  delay(1);
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
