#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <stdlib.h>
#include <stdio.h>

#include "button.h"
#include "stepper.h"

// 1 joint (шаговый двигатель)
#define Z_STEP_PIN   46
#define Z_DIR_PIN    48
#define Z_ENABLE_PIN 62

// 2 joint (шаговый двигатель)
#define X_STEP_PIN   54
#define X_DIR_PIN    55
#define X_ENABLE_PIN 38

// 3 joint (шаговый двигатель)
#define Y_STEP_PIN   60
#define Y_DIR_PIN    61
#define Y_ENABLE_PIN 56

// Сервоприводы
#define JOINT_GRIP_PIN 11

#define JOINT_COUNT 5
#define JOINT_STEP_COUNT 4
#define SERVO_COUNT 1

#define SPEED 12000
#define ACCELERATION 1000
#define M_PI 3.14159265358979323846
#define RADIAN 57.3

// 1 градус = 46 шагов
// 200 шагов * 5,18 передаточное отношение * 16 микрошаг / 360 градусов
#define STEP_IN_ANGLE 46

std_msgs::String savePointMessage;
ros::NodeHandle nodeHandle;

// Кнопка включения и светодиод
#define BUTTON_ON_PIN 16
#define LED_ON_PIN 17

// Кнопка для записи координат
#define BUTTON_PUBLISHER_PIN 23

// Кнопка для остановки робота
#define BUTTON_OFF_PIN 25

Button buttonOn(BUTTON_ON_PIN);
Button buttonPublisher(BUTTON_PUBLISHER_PIN);
Button buttonOff(BUTTON_OFF_PIN);
boolean buttonOnPressed = false;

Stepper stepper_x(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
Stepper stepper_y(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
Stepper stepper_z(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);

MultiStepper steppers;

Servo robotServos[SERVO_COUNT];


//////////// для отладки
float joint_1_rem;
float joint_2_rem;
float joint_3_rem;
float joint_4_rem;
float joint_grip_rem;

float stepper_1_rem;
float stepper_2_rem;
float stepper_3_rem;



//////////////// start

int servo_pins[SERVO_COUNT] = {JOINT_GRIP_PIN}; // Servo Pins

int motorCurrentPositions[JOINT_COUNT] = {0, 0, 0, 0, 0};
float targetJointPositions[JOINT_COUNT] = {0, 0, 0, 0, 0};
long stepperPositions[4] = {0, 0, 0, 0}; /// 4 - это временно поменять после пересчета


/////////// end


std_msgs::String str_msg;
ros::Publisher chatter("save_position", &str_msg);


// Конвертирует joint state
void writeMotors() {

    // Записываем шаговые двигатели
    for (int i = 0; i < JOINT_STEP_COUNT; i++) {
      long stepCount = targetJointPositions[i] * RADIAN * STEP_IN_ANGLE;

      // 1 двигатель у основания вращается как 1 joint
      if (i == 0) {
        
      } else {
        // Пересчитать шаги для 2 и 3 двигателя (управление 2, 3, 4 joints)  
        
      }

      stepperPositions[i] = stepCount;
    }


      //////////////////// логи
      float stepper_1 = targetJointPositions[0];
      float stepper_2 = targetJointPositions[1];
      float stepper_3 = targetJointPositions[2];

      if (stepper_1 != stepper_1_rem //|| stepper_2 != stepper_2_rem || stepper_3 != stepper_3_rem
      ) {
          stepper_1_rem = stepper_1;
          stepper_2_rem = stepper_2;
          stepper_3_rem = stepper_3;
          
          char motor_1_test[8];
          char motor_2_test[8];
          char motor_3_test[8];
          
          dtostrf(stepper_1, 1, 5, motor_1_test);
          dtostrf(stepper_2, 1, 5, motor_2_test);
          dtostrf(stepper_3, 1, 5, motor_3_test);
          nodeHandle.logerror("==============");
          nodeHandle.logwarn(motor_1_test);
          nodeHandle.logwarn(motor_2_test);
          nodeHandle.logwarn(motor_3_test);
               
      }

      ////////////////////

    

    steppers.moveTo(stepperPositions);
    steppers.runSpeedToPosition();

    // Записываем сервоприводы
    for (int i = 0; i < SERVO_COUNT; i++) {
      int targetAngle = targetJointPositions[JOINT_STEP_COUNT + i] * (90/3.14);
      robotServos[i].write(targetAngle);
      motorCurrentPositions[i] = targetAngle;
    }

  nodeHandle.spinOnce();
}

// Subscriber Callback to store the jointstate position values in the global variables
void motorControlSubscriberCallbackJointState(const sensor_msgs::JointState& msg) {

  float joint_1 = msg.position[0];
  float joint_2 = msg.position[1];
  float joint_3 = msg.position[2];
  float joint_4 = msg.position[3] + M_PI;
  float joint_grip = msg.position[4] + M_PI/2;

  float joints_correct[JOINT_COUNT] = {joint_1, joint_2, joint_3, joint_4, joint_grip};

  if (joint_1 != joint_1_rem //||
//      joint_2 != joint_2_rem ||
//      joint_3 != joint_3_rem ||
//      joint_4 != joint_4_rem ||
//      joint_grip != joint_grip_rem
      ) {

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

        dtostrf(joint_1*RADIAN, 1, 5, joint_1_current);
        dtostrf(joint_2, 1, 5, joint_2_current);
        dtostrf(joint_3, 1, 5, joint_3_current);
        dtostrf(joint_4, 1, 5, joint_4_current);
        dtostrf(joint_grip, 1, 5, joint_grip_current);

        nodeHandle.logerror("==============1");
        nodeHandle.logwarn(joint_1_current);
//        nodeHandle.logwarn(joint_2_current);
//        nodeHandle.logwarn(joint_3_current);
//        nodeHandle.logwarn(joint_4_current);
//        nodeHandle.logwarn(joint_grip_current);
  }


  for (int i = 0; i < JOINT_COUNT; i++) {
    targetJointPositions[i] = joints_correct[i];
  }

  // записываем на моторы только при нажатой кнопке
  if (buttonOn.wasPressed()) {
    buttonOnPressed = !buttonOnPressed;
  }

  if (buttonOnPressed) {
    if (buttonPublisher.wasPressed()) {
       nodeHandle.logwarn("Button pressed save process .......");
       
       char message[13] = "save";
       str_msg.data = message;
       chatter.publish(&str_msg);
    }

    if (buttonOff.wasPressed()) {
       nodeHandle.logwarn("Button pressed STOP process .......");

       char message[13] = "stop";
       str_msg.data = message;
       chatter.publish(&str_msg);
    }
    
    writeMotors();
  }

  digitalWrite(LED_ON_PIN, buttonOnPressed);
}


ros::Subscriber<sensor_msgs::JointState> motorControlSubscriberJointState("joint_states", &motorControlSubscriberCallbackJointState);


void setup() {
    AccelStepper stepperX = stepper_x.getStepper();
    AccelStepper stepperY = stepper_y.getStepper();
    AccelStepper stepperZ = stepper_z.getStepper();

    steppers.addStepper(stepperX);
    steppers.addStepper(stepperY);
    steppers.addStepper(stepperZ);

    pinMode(LED_ON_PIN, OUTPUT);
  
    // Инициализировать сервоприводы
    for (unsigned int i = 0; i < SERVO_COUNT; i++) {
      robotServos[i].attach(servo_pins[i]);
    }
  
    nodeHandle.getHardware()->setBaud(115200);
    nodeHandle.initNode();
    nodeHandle.subscribe(motorControlSubscriberJointState);
  
    nodeHandle.advertise(chatter);
    
    nodeHandle.loginfo("Startup complete");
}

void loop() {  
  nodeHandle.spinOnce();
  delay(1);
}
