#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <pluginlib/class_loader.h>
#include <armbot_move/SetPosition.h>
#include <armbot_move/SavePosition.h>
#include <armbot_move/RunArmbot.h>
#include <armbot_move/RunMotor.h>
#include <armbot_move/RunMotorStart.h>
#include <rosserial_arduino/Test.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <boost/scoped_ptr.hpp>

#include <iostream>
#include <math.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <cmath>

#include "../lib/easywsclient/easywsclient.hpp"
#include "../lib/easywsclient/easywsclient.cpp"

#include "MoveOperationClass.hpp"
#include "settings.hpp"
#include "LogClass.hpp"

char FILENAME[20] = "move.cpp";
LogClass logs;
SettingsClass settingsConfig;
using easywsclient::WebSocket;
static WebSocket::pointer ws = NULL;

char positionInfo[20000]; // Информация о рассчитанных позициях
bool calculatePositionSuccess = true; // Результат расчета позиций в череде команд
std::vector<std::string> allCommands; // Все команды из файла запуска
std::vector<std::string> commandDelay; // Задержка после выполнения команды
bool pressButtonFinish = false; // Было нажатие


bool isEmpty(std::ifstream& file) {
    return file.peek() == std::ifstream::traits_type::eof();
}


void webSocketMessage(const std::string &message) {
    printf(">>> %s\n", message.c_str());
    logs.logSimple("Websocket. Get message from server:  ", message.c_str(), FILENAME);

    // Не закрывать соединение при статусе CONNECT
    if (message.find("\"status\":\"CONNECT\"") == std::string::npos) {
        ws->close();
        logs.writeLog("Websocket. Connect close.", FILENAME);
    }
}

void webSocket(const std::string &message) {

    ws = WebSocket::from_url(websocketUrl);
    assert(ws);
    logs.logSimple("Websocket. Connect to:  ", websocketUrl, FILENAME);
    ws->send(message.c_str());
    logs.logSimple("Websocket. Send data:  ", message.c_str(), FILENAME);
    while (ws->getReadyState() != WebSocket::CLOSED) {
      ws->poll();
      ws->dispatch(webSocketMessage);
    }
    delete ws;
}


void setJoints(float joint_1, float joint_2, float joint_3, float joint_4, float joint_grip,
               MoveOperationClass *move_group, robot_state::RobotState start_state) {
    std::map<std::string, double> target;
    target["joint_1"] = joint_1;
    target["joint_2"] = joint_2;
    target["joint_3"] = joint_3;
    target["joint_4"] = joint_4;
    target["joint_grip"] = joint_grip;

    move_group->move->setJointValueTarget(target);
    logs.writeLog("Robot started moving", FILENAME);

    move_group->move->move();
    move_group->move->setStartState(start_state);

    logs.writeLog("Robot stopped ", FILENAME);
}


void saveCommand(std::string commandStr, MoveOperationClass *move_group, robot_state::RobotState start_state) {

    std::stringstream ss(commandStr);
    std::vector<float> result;

    int index = 0;
    while(ss.good()) {
        std::string substr;
        getline(ss, substr, ':');
        if (index > 0) {
            result.push_back(std::stof(substr.c_str()));
        }
        index++;
    }

    float joint_1 = result[0];
    float joint_2 = result[1];
    float joint_3 = result[2];
    float joint_4 = result[3];
    float joint_grip = 0;

    setJoints(joint_1, joint_2, joint_3, joint_4, joint_grip, move_group, start_state);

    robot_state::RobotState planned_state = move_group->move->getJointValueTarget();
    planned_state.update();

    const Eigen::Affine3d pos = planned_state.getGlobalLinkTransform("link_grip");
    std::string posInfo = "X=" + boost::lexical_cast<std::string>(pos.translation().x()) +
                          ", Y=" + boost::lexical_cast<std::string>(pos.translation().y()) +
                          ", Z=" + boost::lexical_cast<std::string>(pos.translation().z());

    logs.logSimple("Get new Affine3d position: ", posInfo.c_str(), FILENAME);

    try {

         float x = pos.translation().x(); // Смена знака
         float y = pos.translation().y();
         float z = pos.translation().z();

         if (saveWebSocket) {
            // отправляем вебсокет
            // переводим м в мм (в UI используются мм)
            x *= 1000;
            y *= 1000;
            z *= 1000;

            auto x_str = boost::lexical_cast<std::string>(x);
            auto y_str = boost::lexical_cast<std::string>(y);
            auto z_str = boost::lexical_cast<std::string>(z);

            std::string commandForWebsocket = x_str + " " + y_str + " " + z_str;
            webSocket(commandForWebsocket);
         }

        if (saveToFile) {
             ROS_INFO("Input command name:");
             std::string commandName;
             std::cin >> commandName;
             logs.logSimple("User input command name: ", commandName.c_str(), FILENAME);
             std::cout << "Command saved!" << std::endl;

             std::ifstream file(commandDescriptionFile);
             if (file.bad() == true) {
                logs.writeLog("File is not exist", FILENAME);
             } else {

                auto x_str = boost::lexical_cast<std::string>(x);
                auto y_str = boost::lexical_cast<std::string>(y);
                auto z_str = boost::lexical_cast<std::string>(z);

                std::ofstream out;
                out.open(commandDescriptionFile, std::ios::app);
                std::string command = commandName + ":" + x_str + " " + y_str + " " + z_str;

                // Если файл не пустой, делаем перенос строки
                std::ifstream file(commandDescriptionFile);
                if (!isEmpty(file)) {
                    out << std::endl;
                }

                out << command;
                out.close();

                logs.logSimple("Point coordinates saved:  ", command.c_str(), FILENAME);
             }
        }
    } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}


void stopCommand() {
    system ("$ARMBOT_PATH/scripts/armbot.sh stop false");
}


void startCommand() {
    system ("$ARMBOT_PATH/scripts/armbot.sh start false");
}


bool setSpeedMotor(ros::Publisher motorSpeedPub) {
    std_msgs::String msg;
    msg.data = boost::lexical_cast<std::string>(maxSpeedStepperMotor).c_str();
    motorSpeedPub.publish(msg);

    logs.logSimple("Finish set speed in Arduino: ", boost::lexical_cast<std::string>(maxSpeedStepperMotor).c_str(), FILENAME);
    return true;
}


// Обработка команд из Arduino
void executeCommand(const std_msgs::String::ConstPtr& msg,
                    MoveOperationClass *move_group,
                    robot_state::RobotState start_state,
                    ros::Publisher motorSpeedPub){

    char logFileName[20] = "move_arduino.ino";

    char command[255];
    strcpy (command, msg->data.c_str());

    std::string markerLog = "log:";
    std::string markerSave = "save:";
    std::string commandStr = command;

    bool isLog = commandStr.rfind(markerLog, 0) == 0;
    if (isLog) {
        commandStr = commandStr.substr(markerLog.length());
    } else {
        logs.logSimple("Get command from Arduino: ", command, logFileName);
    }

    bool isSaveCommand = commandStr.rfind(markerSave, 0) == 0;

    if (isSaveCommand) { // сохранить координату по кнопке
        saveCommand(commandStr, move_group, start_state);
        return;
    } else if (strcmp("stop", command) == 0) { // остановить робота
        stopCommand();
        return;
    } else if (strcmp("start", command) == 0) { // запустить робота
        startCommand();
        return;
    } else if (strcmp("speed", command) == 0) { // получить значение скорости
        setSpeedMotor(motorSpeedPub);
        return;
    } else if (isLog) { // записать лог
    logs.logSimple("", commandStr.c_str(), logFileName);
    } else {
        return;
    }
}


bool runArmbot(armbot_move::RunArmbot::Request &req, armbot_move::RunArmbot::Response &res) {
    char command[500];
    strcpy(command, "$ARMBOT_PATH/scripts/armbot.sh start false");
    strcat(command, " description=");
    strcat(command, req.command_description.c_str());
    strcat(command, " commands=");
    strcat(command, req.commands.c_str());

    ROS_INFO("COMM=%s", command);

    system (command);
    res.result = "FINISH in ROS";

    ros::spinOnce();
    return true;
}


bool runMotor(armbot_move::RunMotor::Request &req, armbot_move::RunMotor::Response &res, ros::Publisher &motorMovePub) {
    char command[50];
    strcpy(command, boost::lexical_cast<std::string>(req.motorNumber).c_str());
    strcat(command, ":");
    strcat(command, boost::lexical_cast<std::string>(req.direction).c_str());
    strcat(command, ":");
    strcat(command, boost::lexical_cast<std::string>(req.stepCount).c_str());

    logs.logSimple("Get data for run step motor: ", command, FILENAME);

    std_msgs::String msg;
    msg.data = command;
    motorMovePub.publish(msg);
    res.result = "FINISH in ROS";
    return true;
}


bool runMotorStart(armbot_move::RunMotorStart::Request &req, armbot_move::RunMotorStart::Response &res, ros::Publisher &motorMoveStartPub) {

    logs.logSimple("Return motor to start", "", FILENAME);

    std_msgs::String msg;
    msg.data = "Start";
    motorMoveStartPub.publish(msg);
    res.result = "FINISH in ROS";
    return true;
}


// Пример сообщения:
// 0.14:0.22:1.5:1.88:0.02;0.14:0.22:1.5:1.88:0.02
bool saveCommandForArduino(const std::vector<double> joints) {

    if (!calculatePositionSuccess) {
        return true;
    }

    char joints_str[200];
    strcpy(joints_str, boost::lexical_cast<std::string>(joints.at(0)).c_str());
    strcat(joints_str, ":");
    strcat(joints_str, boost::lexical_cast<std::string>(joints.at(1)).c_str());
    strcat(joints_str, ":");
    strcat(joints_str, boost::lexical_cast<std::string>(joints.at(2)).c_str());
    strcat(joints_str, ":");
    strcat(joints_str, boost::lexical_cast<std::string>(joints.at(3)).c_str());
    strcat(joints_str, ":");
    strcat(joints_str, boost::lexical_cast<std::string>(joints.at(4)).c_str());

    if (strlen(positionInfo) > 0) {
        strcat(positionInfo, ";");
    }

    strcat(positionInfo, joints_str);

    return 0;
}


bool writeJointsToArduino(ros::ServiceClient arduinoClient, rosserial_arduino::Test srv) {

    int index = 0;
    for(std::string command : allCommands) {
        srv.request.input = command;

        if (arduinoClient.call(srv)) {
            logs.logSimple("Result send command to Arduino: SUCCESS", "", FILENAME);
        } else {
            ROS_ERROR("Failed to call service set_joints_arduino");
            logs.logSimple("Failed to call service set_joints_arduino", "", FILENAME);
            return 1;
        }

        logs.logSimple("Delay after set goal position (s) = ", commandDelay[index].c_str(), FILENAME);
        ros::Duration(std::stof(commandDelay[index])).sleep();
        index++;
    }

    return 0;
}


bool setPosition(armbot_move::SetPosition::Request &req, 
                armbot_move::SetPosition::Response &res,
                MoveOperationClass *move_group,
                const robot_state::JointModelGroup *joint_model_group) {

    std::string result = "ERROR";

    geometry_msgs::Pose pose;
    bool success = true;

    bool returnDefaultPosition = strcmp("return_default_position", req.position.c_str()) == 0;

    // Возврат в исходную позицию не расчитываем
    // Предполагается, что исходная позиция всегда достижима
    if (returnDefaultPosition) {
        robot_state::RobotState start_state(*(move_group->move)->getCurrentState());
        setJoints(0, 0, 0, 0, 0, move_group, start_state);

    } else {
        pose.position.x = req.x == coordinateNone ? defaultPosition_x : req.x;
        pose.position.y = req.y == coordinateNone ? defaultPosition_y : req.y;
        pose.position.z = req.x == coordinateNone && req.y == coordinateNone && req.z == coordinateNone ? defaultPosition_z : req.z;

        if (req.position == "button-pressed") { // кнопка нажата
            pose.position.z = zPositionDefaultDown;
        } else if (req.z != coordinateNone && req.z != 0) { // передана координата Z отличная от 0
            pose.position.z = req.z;
        } else { // используется Z по умолчанию
            pose.position.z = zPositionDefault;
        }

        pose.orientation.x = defaultOrientation_x;
        pose.orientation.y = defaultOrientation_y;
        pose.orientation.z = defaultOrientation_z;
        pose.orientation.w = defaultOrientation_w;

        move_group->move->setApproximateJointValueTarget(pose,"link_grip");
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        robot_state::RobotState current_state(*(move_group->move)->getCurrentState());
        move_group->move->setStartState(current_state);
        success = (move_group->move->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO("Visualizing move 1 (pose goal) %s", success ? "" : "FAILED");
    }

    if (success) {
        std::cout<<"Start move"<<std::endl;

        robot_state::RobotState current_state(*(move_group->move)->getCurrentState());
        move_group->move->setStartState(current_state);
        move_group->move->move();

        std::vector<double> joints = move_group->move->getCurrentJointValues();
        logs.logPrintJoints(joints, FILENAME);


        result = "SUCCESS. Position: " + req.position;
        logs.logSimple("Command execution result: ", result.c_str(), FILENAME);

	    // Отправляет значения joints на Arduino
        ros::NodeHandle nh;
        ros::ServiceClient arduinoClient = nh.serviceClient<rosserial_arduino::Test>("set_joints_arduino");
        rosserial_arduino::Test srv;

        if (!pressButtonFinish) {
            saveCommandForArduino(joints);
        }

        bool buttonPressed = strcmp("button-pressed", req.position.c_str()) == 0;
        bool withPressing = req.pressing;

        if (calculatePositionSuccess && buttonPressed) { //  Кнопка нажата
            allCommands.push_back(positionInfo);
            strcpy(positionInfo, "");

            pressButtonFinish = buttonPressed;

            commandDelay.push_back(boost::lexical_cast<std::string>(req.delay));

        } else if (returnDefaultPosition || calculatePositionSuccess && !req.pressing) { // Нажатие не требуется (возврат в исходную позицию или проверка команды)
            allCommands.push_back(positionInfo);
            commandDelay.push_back("0");
            writeJointsToArduino(arduinoClient, srv);
            strcpy(positionInfo, "");

            allCommands.clear();
            commandDelay.clear();

        } else if (buttonPressed) {
            calculatePositionSuccess = true;
            logs.writeLog("The transition to the position 'button-pressed' is not calculated. Trajectory execution is skipped.", FILENAME);
            strcpy(positionInfo, "");
            allCommands.clear();
            commandDelay.clear();

        } else if (pressButtonFinish) {
            pressButtonFinish = false;
        }

    } else {
        calculatePositionSuccess = false;
        logs.writeLog("Trajectory calculation error. A series of commands will not be sent to the robot.", FILENAME);
        ROS_ERROR("Trajectory calculation error. A series of commands will not be sent to the robot.");
    }

    res.result = result;

    ros::spinOnce();

    return true;
}


int main(int argc, char *argv[]) {
    ROS_INFO("start:");
    ros::init(argc, argv, "move");

    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(10);
    spinner.start();

    std::string planner_plugin_name;
    if (!node_handle.getParam("planning_plugin", planner_plugin_name)) {
        ROS_FATAL_STREAM("Could not find planner plugin name");
        logs.writeLog("Could not find planner plugin name", FILENAME);
    }

    std::string PLANNING_GROUP = "arm";
    MoveOperationClass *move_group = new MoveOperationClass(PLANNING_GROUP);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    robot_state::RobotState start_state(*(move_group->move)->getCurrentState());
    const robot_state::JointModelGroup* joint_model_group = move_group->move->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;

    try {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    } catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
        logs.logSimple("Exception while creating planning plugin loader ", boost::lexical_cast<std::string>(ex.what()).c_str(), FILENAME);
    }

    move_group->move->setPlanningTime(60*5);
    move_group->move->setGoalTolerance(.0001);

    logs.writeVersionLog(FILENAME);
    settingsConfig.update();

    ros::NodeHandle n;

    // Передает значение скорости моторов
    ros::Publisher motorSpeedPub = n.advertise<std_msgs::String>("set_motor_speed", 1000);


    // Передает значение joint в Gazebo
    // ros::Publisher gazeboJoints = n.advertise<trajectory_msgs::JointTrajectoryPoint>("/armbot/arm_controller/command", 1000);

    // Получает позицию из position
    ros::ServiceServer setPositionService = n.advertiseService<armbot_move::SetPosition::Request, armbot_move::SetPosition::Response>
                                ("set_position", boost::bind(setPosition, _1, _2, move_group, joint_model_group));

    // Запуск робота из UI
    ros::ServiceServer armbotRunService = n.advertiseService("armbot_run", runArmbot);

    // Запуск моторов из UI
    ros::Publisher motorMovePub = n.advertise<std_msgs::String>("move_motor", 1000);
    ros::ServiceServer motorRunService = n.advertiseService<armbot_move::RunMotor::Request, armbot_move::RunMotor::Response>
                                ("motor_run", boost::bind(runMotor, _1, _2, motorMovePub));

    // Возврат моторов в исходное положение из UI
    ros::Publisher motorMoveStartPub = n.advertise<std_msgs::String>("move_motor_start", 1000);
    ros::ServiceServer motorRunStartService = n.advertiseService<armbot_move::RunMotorStart::Request, armbot_move::RunMotorStart::Response>
                                    ("motor_run_start", boost::bind(runMotorStart, _1, _2, motorMoveStartPub));

    // Сохраняет позицию из Arduino
    boost::function<void (const std_msgs::String::ConstPtr& msg)> f = boost::bind(executeCommand, _1, move_group, start_state, motorSpeedPub);
    ros::Subscriber savePositionSubscriber = n.subscribe("execute_command", 1000, f);

    ros::Duration(1).sleep();
    ros::waitForShutdown();
    return 0;
}
