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
#define JOINT_GRIP_PIN 11 // Поворот захвата
#define JOINT_GRIP_END_PIN 6 // Захват
#define JOINT_GRIP_END_INIT_ANG 30 // Корректировка угла сервопривода захвата (сервопривод не может поворачиваться на 180 град.)

#define JOINT_COUNT 5
#define JOINT_STEP_COUNT 3
#define SERVO_COUNT 2

#define SPEED 12000
#define ACCELERATION 1000
#define M_PI 3.14159265358979323846
#define RADIAN 57.3

// 1 градус = 46 шагов
// 200 шагов * 5,18 передаточное отношение * 16 микрошаг / 360 градусов
#define STEP_IN_ANGLE 46

// Поправочные коэффициенты для двигателей при пересчете joints в шаги (подгонка под модель)
#define MOTOR_Y_RATIO -0.548491 //-0.91827253
#define MOTOR_Z_RATIO -0.2661934 //-0.48330122

// Корректировка 2joint
#define JOINT_2_CORRECTION 0.10

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
#define BUTTON_ROBOT_START 29

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

int servo_pins[SERVO_COUNT] = {JOINT_GRIP_PIN, JOINT_GRIP_END_PIN}; // Servo Pins

// int motorCurrentPositions[JOINT_COUNT] = {0, 0, 0, 0, 0};
// float targetJointPositions[JOINT_COUNT] = {0, 0, 0, 0, 0};
long stepperPositions[JOINT_STEP_COUNT] = {0, 0, 0}; /// 3 двигателя
int servoPositions[SERVO_COUNT] = {0, 0};


/////////// end


std_msgs::String str_msg;
ros::Publisher chatter("save_position", &str_msg);


// Возврат мотора в исходное положение
void robotReturnStartPosition () {
  nodeHandle.logwarn("============== Начальные значения для моторов ==============");
  nodeHandle.logwarn(String(stepperPositions[0]).c_str());
  nodeHandle.logwarn(String(stepperPositions[1]).c_str());
  nodeHandle.logwarn(String(stepperPositions[2]).c_str());
  
  for (int i = 0; i < JOINT_STEP_COUNT; i++) {
     stepperPositions[i] = 0;
  }

  nodeHandle.logwarn("============== Конечные значения для моторов ==============");
  nodeHandle.logwarn(String(stepperPositions[0]).c_str());
  nodeHandle.logwarn(String(stepperPositions[1]).c_str());
  nodeHandle.logwarn(String(stepperPositions[2]).c_str());
    
  steppers.moveTo(stepperPositions);
  steppers.runSpeedToPosition();
  
  nodeHandle.spinOnce();
  delay(1);  
}

// Управление моторами с клавиатуры
void motorMove(const std_msgs::String& msg){
  int motorNumber; // 1 - снизу, 2 - слева, 3 - справа
  int motorDirection; // 0 - FORWARD или 1 - INVERSE

  char data[strlen(msg.data)];
  strcpy(data, msg.data);  

  char *token;
  token = strtok(data, ":");

  if (token != NULL && strlen(msg.data) == 3) { // Формат данных: '0:1'
    motorNumber = data[0] - '0';
    motorDirection = data[2] - '0';
  } else {
    nodeHandle.logerror("Ошибка в данных запуска двигателя. Строка не соответствует формату!");
  }
  
//  nodeHandle.logwarn("-----------------");
  //  nodeHandle.logwarn(String(motorNumber).c_str());
  //  nodeHandle.logwarn(String(motorDirection).c_str());
//  nodeHandle.logwarn(String(stepperPositions[0]).c_str());
//  nodeHandle.logwarn(String(stepperPositions[1]).c_str());
//  nodeHandle.logwarn(String(stepperPositions[2]).c_str());

  if ((motorNumber == 1 || motorNumber == 2 || motorNumber == 3) && (motorDirection == 0 || motorDirection == 1)) { // Шаговые двигатели
    int steps = motorDirection == 0 ? 46 : -46; // По умолчанию ставить 100, на меньшем шаге рушится
    int motorIndex = motorNumber-1;
    stepperPositions[motorIndex] = stepperPositions[motorIndex] + steps;
    
  } else if (motorNumber == 4 || motorNumber == 5) {
    nodeHandle.logerror("Servo start!!!!!!!!!");

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
    
    nodeHandle.logerror("зашло --- ");
    nodeHandle.logerror(String(servoPositions[servoIndex]).c_str());
    nodeHandle.logerror(String(start).c_str());
    nodeHandle.logerror(String(finish).c_str());

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
    nodeHandle.logerror("Ошибка при запуске двигателя. Неверное значение двигателя или направления");
  }

  ////////// НЕ УДАЛЯТЬ! Почему-то без этого заходит в setup постоянно!!!!
  nodeHandle.logwarn("**** Движение двигателей ***");
  nodeHandle.logwarn(String(stepperPositions[0]).c_str());
  nodeHandle.logwarn(String(stepperPositions[1]).c_str());
  nodeHandle.logwarn(String(stepperPositions[2]).c_str());
  /////////////////

  steppers.moveTo(stepperPositions);
  steppers.runSpeedToPosition();
  nodeHandle.logwarn("Движение двигателей --- FINISH");
  ////////////////////////////////////////////////////////////////////////////////////////////////

  float stepper_x_joint = stepperPositions[0] /STEP_IN_ANGLE / RADIAN;
  float stepper_y_joint = (stepperPositions[1] / STEP_IN_ANGLE / RADIAN);
  float stepper_z_joint = (stepperPositions[2] / STEP_IN_ANGLE / RADIAN); // 2 joint (3 и 4)

  // корректрировка joints по ограничениям из модели
//  if (stepper_x_joint < -2.25) {
//    stepper_x_joint = 2.25;
//  } else if (stepper_x_joint > 2.25) {
//    stepper_x_joint = 2.25;
//  }
//
//  if (stepper_y_joint < 0) {
//    stepper_y_joint = 0;  
//  } else if (stepper_y_joint > 1.57) {
//    stepper_y_joint = 1.57;
//  }
//
//  if (stepper_z_joint < 0) {
//    stepper_z_joint = 0;  
//  } else if (stepper_z_joint > 1.57) {
//    stepper_z_joint = 1.57;
//  }

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


void robotMotorMove(const rosserial_arduino::Test::Request & req, rosserial_arduino::Test::Response & res){
   char commands[strlen(req.input)+1];
   strcpy(commands, req.input); 
  
   nodeHandle.logwarn("Пришли данные:");
   nodeHandle.logwarn(String(commands).c_str());
  
   float jointList[5] = {0, 0, 0, 0, 0};
   char *rest = NULL;
   char *jointStr;

   int index = 0;
   for (jointStr = strtok_r(commands, ":", &rest); jointStr != NULL; jointStr = strtok_r(NULL, ":", &rest)) {
    
     float joint = atof(jointStr);
     jointList[index] = joint;

//      nodeHandle.logwarn("---------- joint");
//      nodeHandle.logwarn(jointStr);
//      nodeHandle.logwarn(String(joint).c_str());      
     
     index++;
   }

   nodeHandle.logwarn("---------- joints ---------");
   for (int i = 0; i < 4; i++) {
     nodeHandle.logwarn(String(jointList[i]).substring(0, 8).c_str());
   }


//////////////////////////////////////////
///////////////////////////////////////////
   // Записываем шаговые двигатели
    for (int i = 0; i < JOINT_STEP_COUNT; i++) {
      long stepCount = 0;
      long prevStepCount = 0;

      if (i == 0) {
        stepCount = jointList[i] * RADIAN * STEP_IN_ANGLE;
      }

      if (i == 1) {
        // пересчет для 2 двигателя
//        stepCount = (91 - jointList[i]) * StepsPerDegreeTwo;
//        stepCount = round(stepCount);
        stepCount = jointList[i] * RADIAN * STEP_IN_ANGLE * MOTOR_Y_RATIO;
      }
      
      if (i == 2) {
        // пересчет для 3 двигателя
//        stepCount = ((abs(jointList[i]) - 93) * StepsPerDegreeThree) + stepperPositions[i-1];
//        stepCount = round(stepCount);
        stepCount = jointList[i] * RADIAN * STEP_IN_ANGLE * MOTOR_Z_RATIO;
      }
//      
      nodeHandle.logwarn("********************* Motor #");
      nodeHandle.logwarn(String(i).c_str());
      nodeHandle.logwarn(String(jointList[i]).c_str());
      nodeHandle.logwarn("**********************");

      // сохраняем значения для шаговых двигателей
       stepperPositions[i] = stepCount;
    }

   if (buttonOnPressed) {
//      steppers.moveTo(stepperPositions);
//      steppers.runSpeedToPosition();
    }

//   steppers.moveTo(stepperPositions);
//   steppers.runSpeedToPosition();
   delay(1);
   nodeHandle.spinOnce(); 
     
   res.output = "FINISH"; 
   nodeHandle.logwarn(res.output);
}


ros::ServiceServer<rosserial_arduino::Test::Request, rosserial_arduino::Test::Response> server("set_joints_arduino", &robotMotorMove);


// Записать координаты с робота
void savePositionRobotModel () {
    if (buttonPublisher.wasPressed()) {
         nodeHandle.logwarn("Button pressed save process .......");
         
         char message[13] = "save";
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
    nodeHandle.advertise(chatter);
    
    nodeHandle.loginfo("Startup complete");
}


void loop() {
  
    if (buttonReturn.wasPressed()) {
        robotReturnStartPosition();
    }
  
    // кнопка нажата
    if (buttonOn.wasPressed()) {
      buttonOnPressed = !buttonOnPressed;
      nodeHandle.logwarn("Робот включен");
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
