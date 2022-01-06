#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

// Используется для получения joints
#include <rosserial_arduino/Test.h>

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include "button.h"
#include "stepper.h"

// 1 joint (шаговый двигатель) - правый
#define Z_STEP_PIN   46
#define Z_DIR_PIN    48
#define Z_ENABLE_PIN 62

// 2 joint (шаговый двигатель) - нижний
#define X_STEP_PIN   54
#define X_DIR_PIN    55
#define X_ENABLE_PIN 38

// 3 joint (шаговый двигатель) - левый
#define Y_STEP_PIN   60
#define Y_DIR_PIN    61
#define Y_ENABLE_PIN 56

// Сервоприводы
#define JOINT_GRIP_PIN 11 // Поворот захвата
#define JOINT_GRIP_END_PIN 6 // Захват
#define JOINT_GRIP_END_INIT_ANG 30 // Корректировка угла сервопривода захвата при старте скетча (сервопривод не может поворачиваться на 180 град.)

// Количество шаговых двигателей
#define JOINT_STEP_COUNT 3

// Количество сервоприводов
#define SERVO_COUNT 2

// Градусов в радиане 
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

// Кнопка для возврата в исходное положение робота (шаговые двигатели возвращаются в 0)
#define BUTTON_RETURN_START 27

// Кнопка для запуска робота
#define BUTTON_ROBOT_START 31

Button buttonOn(BUTTON_ON_PIN);
Button buttonPublisher(BUTTON_PUBLISHER_PIN);
Button buttonOff(BUTTON_OFF_PIN);
Button buttonReturn(BUTTON_RETURN_START);
Button buttonRobotStart(BUTTON_ROBOT_START);
boolean buttonOnPressed = false;


Stepper stepper_x(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN); // Нижний двигатель
Stepper stepper_y(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN); // Левый двигатель
Stepper stepper_z(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN); // Правый двигатель

AccelStepper stepperX = stepper_x.getStepper();
AccelStepper stepperY = stepper_y.getStepper();
AccelStepper stepperZ = stepper_z.getStepper();

MultiStepper steppers;

Servo robotServos[SERVO_COUNT];

int servo_pins[SERVO_COUNT] = {JOINT_GRIP_PIN, JOINT_GRIP_END_PIN}; // Servo Pins
long stepperPositions[JOINT_STEP_COUNT] = {0, 0, 0}; /// 3 двигателя
int servoPositions[SERVO_COUNT] = {0, 0};

std_msgs::String str_msg;
ros::Publisher chatter("execute_command", &str_msg);

void logWrite(String message) {
    message = "log:" + message;
    str_msg.data = message.c_str();
    chatter.publish(&str_msg);
}


// Возврат мотора в исходное положение
void robotReturnStartPosition () {
  
  for (int i = 0; i < JOINT_STEP_COUNT; i++) {
     stepperPositions[i] = 0;
  }
    
  steppers.moveTo(stepperPositions);
  steppers.runSpeedToPosition();
  
  nodeHandle.spinOnce();
  delay(1);  
}


void motorMoveStart(const std_msgs::String& msg) {
  robotReturnStartPosition ();
}


// Управление моторами с клавиатуры
// Формат данных: '0:1' либо 0:1:46
void motorMove(const std_msgs::String& msg){
  int motorNumber; // 1 - снизу, 2 - слева, 3 - справа
  int motorDirection; // 0 - FORWARD или 1 - INVERSE
  int stepCount = 100; // Количество шагов

  char data[strlen(msg.data)];
  strcpy(data, msg.data);

  int dataArr[3] = {0, 0, 0};
  char *rest = NULL;
  char *dataStr;
  
  int index = 0;
  for (dataStr = strtok_r(data, ":", &rest); dataStr != NULL; dataStr = strtok_r(NULL, ":", &rest)) {
     int item = atoi(dataStr);
     dataArr[index] = item;
     index++;
  }

  motorNumber = dataArr[0];
  motorDirection = dataArr[1];

  if (dataArr[2] != 0) {
    stepCount = dataArr[2];
  }

  if ((motorNumber == 1 || motorNumber == 2 || motorNumber == 3) && (motorDirection == 0 || motorDirection == 1)) { // Шаговые двигатели
    int steps = motorDirection == 0 ? stepCount : -stepCount;
    int motorIndex = motorNumber-1;
    stepperPositions[motorIndex] = stepperPositions[motorIndex] + steps;
    
  } else if (motorNumber == 4 || motorNumber == 5) {
    int servoIndex = motorNumber == 4 ? 0 : 1;
    int deg = motorDirection == 0 ? 10 : -10;
    int start = servoPositions[servoIndex];
    int finish = start + deg;

    if (finish > 180) {
        finish = 180;
    }

    if (finish < 0) {
        finish = 0;
    }

    if (start < finish) {
        for (int j = start; j < finish; j++) {
            robotServos[servoIndex].write(j);
            delay(10);
        }
    }

    if (start > finish) {
        for (int j = start; j > finish; j--) {
            robotServos[servoIndex].write(j);
            delay(10);
        }
    }

    servoPositions[servoIndex] = finish;
    nodeHandle.logerror(String(servoPositions[servoIndex]).c_str());

  } else {
    logWrite("An error when starting the engine. Incorrect engine or direction value");
    nodeHandle.logerror("Ошибка при запуске двигателя. Неверное значение двигателя или направления");
  }

  // Не удалять строку лога (иначе всегда заходит в setup)
  logWrite("Motor run: 1 motor: " + String(stepperPositions[0]) + ", 2 motor: " + String(stepperPositions[1]) + ", 3 motor: " + String(stepperPositions[2]));

  steppers.moveTo(stepperPositions);
  steppers.runSpeedToPosition();
  logWrite("Motor run funish");

  float stepper_x_joint = stepperPositions[0] /STEP_IN_ANGLE / RADIAN;
  float stepper_y_joint = (stepperPositions[1] / STEP_IN_ANGLE / RADIAN);
  float stepper_z_joint = (stepperPositions[2] / STEP_IN_ANGLE / RADIAN); // 2 joint (3 и 4)

  char message[50];
  strcpy(message, "joints=");
  strcat(message, String(stepper_x_joint).substring(0, 8).c_str());
  strcat(message, ":");
  strcat(message, String(stepper_y_joint).substring(0, 8).c_str());
  strcat(message, ":");
  strcat(message, String(stepper_z_joint).substring(0, 8).c_str());
  strcat(message, ", steps: ");
  strcat(message, String(stepperPositions[0]).substring(0, 8).c_str());
  strcat(message, ":");
  strcat(message, String(stepperPositions[1]).substring(0, 8).c_str());
  strcat(message, ":");
  strcat(message, String(stepperPositions[2]).substring(0, 8).c_str());
  str_msg.data = message;
  chatter.publish(&str_msg);
  delay(1);
  nodeHandle.spinOnce();
}


ros::Subscriber<std_msgs::String> motorMoveSubscriber("move_motor", &motorMove);
ros::Subscriber<std_msgs::String> robotReturnStartPositionSubscriber("move_motor_start", &motorMoveStart);


void robotMotorMove(const rosserial_arduino::Test::Request & req, rosserial_arduino::Test::Response & res){
   char commands[strlen(req.input)+1];
   strcpy(commands, req.input);

   logWrite("GET DATA: " + String(commands));

   float jointList[5] = {0, 0, 0, 0, 0};
   char *rest = NULL;
   char *jointStr;

   int index = 0;
   for (jointStr = strtok_r(commands, ":", &rest); jointStr != NULL; jointStr = strtok_r(NULL, ":", &rest)) {

     float joint = atof(jointStr);
     jointList[index] = joint;
     index++;
   }

   // Записываем шаговые двигатели

   // 1 двигатель
   stepperPositions[0] = -jointList[0] * RADIAN * STEP_IN_ANGLE; // Едет в другую сторону

   // 2 двигатель
   stepperPositions[1] = jointList[1] * RADIAN * STEP_IN_ANGLE;

   // 3 двигатель
   stepperPositions[2] = -(jointList[2] + jointList[1]) * RADIAN * STEP_IN_ANGLE;

   if (buttonOnPressed) {
      steppers.moveTo(stepperPositions);
      steppers.runSpeedToPosition();
    }
    
   delay(1);
   nodeHandle.spinOnce(); 
     
   res.output = "FINISH"; 
   logWrite("FINISH motor run");
}


ros::ServiceServer<rosserial_arduino::Test::Request, rosserial_arduino::Test::Response> server("set_joints_arduino", &robotMotorMove);


// Записать координаты с робота
void savePositionRobotModel () {
    if (buttonPublisher.wasPressed()) {
         nodeHandle.logwarn("Button pressed save process .......");
         logWrite("Steps 1 motor : " + String(stepperPositions[0]) + ", 2 motor: " + String(stepperPositions[1]) + ", 3 motor: " + String(stepperPositions[2]));
         
         float joint_1 = -(stepperPositions[0] / RADIAN / STEP_IN_ANGLE);
         float joint_2 = stepperPositions[1] / RADIAN / STEP_IN_ANGLE;
         float joint_3 = -(stepperPositions[2] + stepperPositions[1]) / RADIAN / STEP_IN_ANGLE;
         
         char message[50] = "save:";
         strcat(message, String(joint_1).substring(0, 8).c_str());
         strcat(message, ":");
         strcat(message, String(joint_2).substring(0, 8).c_str());
         strcat(message, ":");
         strcat(message, String(joint_3).substring(0, 8).c_str());
         str_msg.data = message;
         chatter.publish(&str_msg);
    }
}


// Запустить робота
void startMoveRobot() {
    if (buttonRobotStart.wasPressed()) {
         nodeHandle.logwarn("Button pressed robot start process .......");
         
         char message[13] = "start";
         str_msg.data = message;
         chatter.publish(&str_msg);
    }
}


// Остановить робота
void stopMoveRobot() {
    if (buttonOff.wasPressed()) {
        nodeHandle.logwarn("Button pressed STOP process .......");
        
        char message[13] = "stop";
        str_msg.data = message;
        chatter.publish(&str_msg);
    }
}


void setup() {

    steppers.addStepper(stepperX);
    steppers.addStepper(stepperY);
    steppers.addStepper(stepperZ);

    pinMode(LED_ON_PIN, OUTPUT);
  
    // Инициализировать сервоприводы
    for (int i = 0; i < SERVO_COUNT; i++) {
      robotServos[i].attach(servo_pins[i]);
    }

    // Установить начальное значение сервопривода сверху
    robotServos[0].write(0);

    // Установить начальное значение угла сервопривода-захвата
    robotServos[1].write(JOINT_GRIP_END_INIT_ANG);
  
    nodeHandle.getHardware()->setBaud(115200);
    nodeHandle.initNode();
    nodeHandle.advertiseService(server);
    nodeHandle.subscribe(motorMoveSubscriber);
    nodeHandle.subscribe(robotReturnStartPositionSubscriber);
    nodeHandle.advertise(chatter);
}


void loop() {
  
    if (buttonReturn.wasPressed()) {
        robotReturnStartPosition();
    }
  
    // кнопка нажата
    if (buttonOn.wasPressed()) {
      buttonOnPressed = !buttonOnPressed;
      logWrite("Robot is ready");
    }
  
    if (buttonOnPressed) {
      savePositionRobotModel();
      stopMoveRobot();
      startMoveRobot();
    }

    digitalWrite(LED_ON_PIN, buttonOnPressed);

    nodeHandle.spinOnce();
    delay(1);
}
