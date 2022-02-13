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

#define JOINT_COUNT 5

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
long stepperTemp[JOINT_STEP_COUNT] = {0, 0, 0}; // Запись двигателей при работе на медленных скоростях
int servoPositions[SERVO_COUNT] = {0, 0};

int motorSpeed = 700; // Скорость двигателя по умолчанию
int minMotorSpeed = 500; // Минимальная нормальная скорость, всё что ниже - низкая скорость
int maxStepCount = 100; // Максимальное количество шагов для прохождения двигателем при работе на низких скоростях

std_msgs::String str_msg;
ros::Publisher chatter("execute_command", &str_msg);

void logWrite(String message) {
    message = "log:" + message;
    str_msg.data = message.c_str();
    chatter.publish(&str_msg);
    
    nodeHandle.spinOnce();
}


// Возврат мотора в исходное положение
void robotReturnStartPosition () {
  
  for (int i = 0; i < JOINT_STEP_COUNT; i++) {
     stepperPositions[i] = 0;
  }

  writeStepperMotors();
  nodeHandle.spinOnce();
}


void motorMoveStart(const std_msgs::String& msg) {
  robotReturnStartPosition();
  nodeHandle.spinOnce();
}


void setMotorSpeed(const std_msgs::String& msg) {
  motorSpeed = atoi(msg.data);

  // Переопределение скорости из конфигов
  stepperX = stepper_x.setSpeedStepper(stepperX, motorSpeed);
  stepperY = stepper_y.setSpeedStepper(stepperY, motorSpeed);
  stepperZ = stepper_z.setSpeedStepper(stepperZ, motorSpeed);

  logWrite("Arduino get speed: " + String(motorSpeed));
  nodeHandle.spinOnce();
}


// Управление моторами с клавиатуры
// Формат данных: '0:1' либо 0:1:46
void motorMove(const std_msgs::String& msg){
  int motorNumber; // 1 - снизу, 2 - слева, 3 - справа
  int motorDirection; // 0 - FORWARD или 1 - INVERSE
  int stepDegreeValue = 100; // Количество шагов

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
    stepDegreeValue = dataArr[2];
  }

  if ((motorNumber == 1 || motorNumber == 2 || motorNumber == 3) && (motorDirection == 0 || motorDirection == 1)) { // Шаговые двигатели
    int steps = motorDirection == 0 ? stepDegreeValue : -stepDegreeValue;
    int motorIndex = motorNumber-1;
    stepperPositions[motorIndex] = stepperPositions[motorIndex] + steps;
    
  } else if (motorNumber == 4 || motorNumber == 5) {
    int servoIndex = motorNumber == 4 ? 0 : 1;
    int deg = motorDirection == 0 ? stepDegreeValue : -stepDegreeValue;
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
        }
    }

    if (start > finish) {
        for (int j = start; j > finish; j--) {
            robotServos[servoIndex].write(j);
        }
    }

    servoPositions[servoIndex] = finish;

  } else {
    logWrite("An error when starting the engine. Incorrect engine or direction value");
    nodeHandle.logerror("Ошибка при запуске двигателя. Неверное значение двигателя или направления");
  }

  // Не удалять строку лога (иначе всегда заходит в setup)
  logWrite("Motor run: 1 motor: " + String(stepperPositions[0]) + ", 2 motor: " + String(stepperPositions[1]) + ", 3 motor: " + String(stepperPositions[2]));

  writeStepperMotors();
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
  
  nodeHandle.spinOnce();
}


ros::Subscriber<std_msgs::String> motorMoveSubscriber("move_motor", &motorMove);
ros::Subscriber<std_msgs::String> robotReturnStartPositionSubscriber("move_motor_start", &motorMoveStart);
ros::Subscriber<std_msgs::String> setMotorSpeedSubscriber("set_motor_speed", &setMotorSpeed);


void writeMotorChunk(int motorNumber) {
  if (stepperTemp[motorNumber] != stepperPositions[motorNumber]) {
    int diffrence = 0; 
    
    if (stepperPositions[motorNumber] >= 0 && stepperPositions[motorNumber] > stepperTemp[motorNumber]) { // Положительное направление, вперед
        diffrence = stepperTemp[motorNumber] + maxStepCount > stepperPositions[motorNumber] ? stepperPositions[motorNumber] : stepperTemp[motorNumber] + maxStepCount;
      
    } else if (stepperPositions[motorNumber] >= 0 && stepperPositions[motorNumber] < stepperTemp[motorNumber]) { // Положительное направление, назад
        diffrence = stepperTemp[motorNumber] - maxStepCount < 0 ? 0 : stepperTemp[motorNumber] - maxStepCount;
      
    } else if (stepperPositions[motorNumber] < 0 && stepperPositions[motorNumber] < stepperTemp[motorNumber]) { // Отрицательное направление, вперед
        diffrence = stepperTemp[motorNumber] - maxStepCount < stepperPositions[motorNumber] ? stepperPositions[motorNumber] : stepperTemp[motorNumber] - maxStepCount;
        
    } else if (stepperPositions[motorNumber] < 0 && stepperPositions[motorNumber] > stepperTemp[motorNumber]) { // Отрицательное направление, назад
        diffrence = stepperTemp[motorNumber] + maxStepCount > 0 ? 0 : stepperTemp[motorNumber] + maxStepCount;
        
    }

    stepperTemp[motorNumber] = diffrence;
   }
}


void writeStepperMotors() {
    if (motorSpeed <= minMotorSpeed) {
          // Костыль для работы на низких скоростях
          // Решает проблему с ошибкой "Lost sync with device, restarting..." (потеря соединения медлу ROS и Arduino, после которой надо перезапускать робота)
          // В кнопку попадает неточно (+- 2 мм), поэтому для выполнения сценария с нажатиями надо ставить нормальную скорость
          // Чтобы попадал точно, надо будет изменять maxStepCount пропорционально для всех моторов (сейчас maxStepCount одинаковое для всех моторов независимо от расстояния)
        while (stepperTemp[0] != stepperPositions[0] || stepperTemp[1] != stepperPositions[1] || stepperTemp[2] != stepperPositions[2]) {
            writeMotorChunk(0);
            writeMotorChunk(1);
            writeMotorChunk(2);
              
            steppers.moveTo(stepperTemp);
            nodeHandle.spinOnce();
            steppers.runSpeedToPosition();
        }
          
    } else {
        steppers.moveTo(stepperPositions);
         nodeHandle.spinOnce();
         steppers.runSpeedToPosition();
    }
}


// Записываем шаговые двигатели
void runStepMotors(float jointList[]) {

   // 1 двигатель
   stepperPositions[0] = -jointList[0] * RADIAN * STEP_IN_ANGLE; // Смена знака jointList[0]

   // 2 двигатель
   stepperPositions[1] = jointList[1] * RADIAN * STEP_IN_ANGLE;

   // 3 двигатель
   stepperPositions[2] = (jointList[2] - jointList[1]) * RADIAN * STEP_IN_ANGLE;

   // Выполнение траектории завершено (координаты точек не переданы)
   bool stopPosition = jointList[0] == 0 && jointList[1] == 0 && jointList[2] == 0 && jointList[3] == 0 && jointList[4] == 0;

   logWrite("Motor step calculate: 1 motor: " + String(stepperPositions[0]) + ", 2 motor: " + String(stepperPositions[1]) + ", 3 motor: " + String(stepperPositions[2])); 
   
   if (buttonOnPressed && !stopPosition) {
      writeStepperMotors();
   }
}


float prevJointList[] = {0, 0, 0, 0, 0}; // Вернуться в эту позицию после нажатия кнопки
bool runStepper(char* rowStr, int rowIndex) {
      
   float jointList[] = {0, 0, 0, 0, 0};
   char *rest = NULL;
   char *jointStr;

   int index = 0;
   for (jointStr = strtok_r(rowStr, ":", &rest); jointStr != NULL; jointStr = strtok_r(NULL, ":", &rest)) {
     float joint = atof(jointStr);
     jointList[index] = joint;

     if (rowIndex == 0) {
         prevJointList[index] = joint;
     } 
     
     index++;
   }

  runStepMotors(jointList);
   
  rowIndex++;
  return true;

  nodeHandle.spinOnce();
}


// Запуск моторов по командам (позиция - нажатие - позиция)
void robotMotorMove(const rosserial_arduino::Test::Request & req, rosserial_arduino::Test::Response & res) {
   char commands[strlen(req.input)+1];
   strcpy(commands, req.input);

   logWrite("GET DATA: " + String(commands));

   long allProcessTime = millis();

   char *restRow = NULL;
   char *rowStr;
   
   int rowIndex = 0;
   
   for (rowStr = strtok_r(commands, ";", &restRow); rowStr != NULL; rowStr = strtok_r(NULL, ";", &restRow)) {
      runStepper(rowStr, rowIndex);
      rowIndex++;
   }

   // Возврат в исходную позицию после нажатия
   if (rowIndex > 1) {
      runStepMotors(prevJointList);
   }

   long commandExecuteTime = millis() - allProcessTime;
   logWrite("Command #" + String(rowIndex) + " was executed for " + String(commandExecuteTime) + "s."); // Время выполнения 1 команды
     
   res.output = "FINISH"; 
   logWrite("FINISH motor run");
   
   nodeHandle.spinOnce();
}


ros::ServiceServer<rosserial_arduino::Test::Request, rosserial_arduino::Test::Response> server("set_joints_arduino", &robotMotorMove);


// Записать координаты с робота
void savePositionRobotModel () {
    if (buttonPublisher.wasPressed()) {
         nodeHandle.logwarn("Button pressed save process .......");
         logWrite("Steps 1 motor : " + String(stepperPositions[0]) + ", 2 motor: " + String(stepperPositions[1]) + ", 3 motor: " + String(stepperPositions[2]));
         
         float joint_1 = -stepperPositions[0] / RADIAN / STEP_IN_ANGLE;
         float joint_2 = stepperPositions[1] / RADIAN / STEP_IN_ANGLE;
         float joint_3 = (stepperPositions[2] + stepperPositions[1]) / RADIAN / STEP_IN_ANGLE;
         float joint_4 = -(abs(joint_3) - abs(joint_2));
         
         char message[50] = "save:";
         strcat(message, String(joint_1).substring(0, 8).c_str());
         strcat(message, ":");
         strcat(message, String(joint_2).substring(0, 8).c_str());
         strcat(message, ":");
         strcat(message, String(joint_3).substring(0, 8).c_str());
         strcat(message, ":");
         strcat(message, String(joint_4).substring(0, 8).c_str());
         str_msg.data = message;
         chatter.publish(&str_msg);
    }

    nodeHandle.spinOnce();
}


// Запустить робота
void startMoveRobot() {
    if (buttonRobotStart.wasPressed()) {
         nodeHandle.logwarn("Button pressed robot start process .......");
         
         char message[13] = "start";
         str_msg.data = message;
         chatter.publish(&str_msg);
    }

    nodeHandle.spinOnce();
}


// Остановить робота
void stopMoveRobot() {
    if (buttonOff.wasPressed()) {
        nodeHandle.logwarn("Button pressed STOP process .......");
        
        char message[13] = "stop";
        str_msg.data = message;
        chatter.publish(&str_msg);
    }

    nodeHandle.spinOnce();
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

    nodeHandle.advertise(chatter);
    nodeHandle.advertiseService(server);
    nodeHandle.subscribe(motorMoveSubscriber);
    nodeHandle.subscribe(robotReturnStartPositionSubscriber);
    nodeHandle.subscribe(setMotorSpeedSubscriber);
}


void loop() {
  
    if (buttonReturn.wasPressed()) {
        robotReturnStartPosition();
    }
  
    // кнопка нажата
    if (buttonOn.wasPressed()) {
      buttonOnPressed = !buttonOnPressed;
      logWrite("Robot is ready");

      // Получить значение скорости при включении робота
      if (buttonOnPressed) {
        char message[13] = "speed";
        str_msg.data = message;
        chatter.publish(&str_msg);
      }
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
