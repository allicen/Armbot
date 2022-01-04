#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <pluginlib/class_loader.h>
#include <armbot_move/SetPosition.h>
#include <armbot_move/SavePosition.h>
#include <armbot_move/RunArmbot.h>
#include <rosserial_arduino/Test.h>
#include <std_msgs/String.h>

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
#include <string>
#include <fstream>

#include "../lib/easywsclient/easywsclient.hpp"
#include "../lib/easywsclient/easywsclient.cpp"

#include "MoveOperationClass.hpp"
#include "settings.hpp"
#include "LogClass.hpp"

char FILENAME[20] = "move.cpp";
LogClass logs;
using easywsclient::WebSocket;
static WebSocket::pointer ws = NULL;


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

void saveCommand() {
    tf::TransformListener listener;

    tf::StampedTransform transform;
    try{
         listener.waitForTransform("/link_grip","/base_link", ros::Time(), ros::Duration(5.0));
         listener.lookupTransform("/link_grip", "/base_link", ros::Time(), transform);

         float x = transform.getOrigin().x();
         float y = -transform.getOrigin().y(); // Почему-то записывает с другим знаком
         float z = transform.getOrigin().z();

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


bool writeJointsFromArduino(const std::string &command) {
    ROS_ERROR("TEST_CONNECT!!!");
    ROS_INFO("get: %s",  command.c_str());
    return 0;
}


void executeCommand(const std_msgs::String::ConstPtr& msg){

    char command[50];
    strcpy (command, msg->data.c_str());

    logs.logSimple("Get command: ", command, FILENAME);

    if (strcmp("save", command) == 0) {
        saveCommand();
        return;
    } else if (strcmp("stop", command) == 0) {
        stopCommand();
        return;
    } else if (strcmp("start", command) == 0) {
        startCommand();
        return;
    } else {

        writeJointsFromArduino(command);
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
    return true;
}


bool writeJointsToArduino(ros::ServiceClient arduinoClient, rosserial_arduino::Test srv, const std::vector<double> joints) {

    char joints_str[500];
    strcpy(joints_str, boost::lexical_cast<std::string>(joints.at(0)).c_str());
    strcat(joints_str, ":");
    strcat(joints_str, boost::lexical_cast<std::string>(joints.at(1)).c_str());
    strcat(joints_str, ":");
    strcat(joints_str, boost::lexical_cast<std::string>(joints.at(2)).c_str());
    strcat(joints_str, ":");
    strcat(joints_str, boost::lexical_cast<std::string>(joints.at(3)).c_str());
    strcat(joints_str, ":");
    strcat(joints_str, boost::lexical_cast<std::string>(joints.at(4)).c_str());

    srv.request.input = joints_str;

    if (arduinoClient.call(srv)) {
        ROS_INFO("ARDUINO SUCCESS: ");
        logs.logSimple("Result send command to Arduino: ", "", FILENAME);

    } else {
        ROS_ERROR("Failed to call service set_joints_arduino");
        logs.logSimple("Failed to call service set_joints_arduino: ", "", FILENAME);
        return 1;
    }

    return 0;
}


bool setPosition(armbot_move::SetPosition::Request &req, 
                armbot_move::SetPosition::Response &res,
                MoveOperationClass *move_group,
                robot_state::RobotState start_state,
                const robot_state::JointModelGroup *joint_model_group) {

    std::string result = "ERROR";

    Orientation orientation;

    geometry_msgs::Pose pose;
    pose.position.x = req.x;
    pose.position.y = req.y;

    if (req.position == "button-pressed") { // кнопка нажата
        pose.position.z = zPositionDefaultDown;
    } else if (req.z != zPositionNone && req.z != 0) { // передана координата Z отличная от 0
        pose.position.z = req.z;
    } else { // используется Z по умолчанию
        pose.position.z = zPositionDefault;
    }

    pose.orientation.x = orientation.x;
    pose.orientation.y = orientation.y;
    pose.orientation.z = orientation.z;
    pose.orientation.w = orientation.w;

    move_group->move->setApproximateJointValueTarget(pose,"link_grip");
    bool success = (move_group->move->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Visualizing move 1 (pose goal) %s", success ? "" : "FAILED");

    if (success) {
        std::cout<<"Start move"<<std::endl;
        start_state.setFromIK(joint_model_group, pose);
        move_group->move->setStartState(start_state);

        std::vector<double> joints = move_group->move->getCurrentJointValues();
        logs.logPrintJoints(joints, FILENAME);


        result = "SUCCESS. Position: " + req.position;
        logs.logSimple("Command execution result: ", result.c_str(), FILENAME);


	    // Отправляет значения joints на Arduino
        ros::NodeHandle nh;
        ros::ServiceClient arduinoClient = nh.serviceClient<rosserial_arduino::Test>("set_joints_arduino");
        rosserial_arduino::Test srv;
        writeJointsToArduino(arduinoClient, srv, joints);
    }

    res.result = result;

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

    ros::NodeHandle n;

    // Получает позицию из position
    ros::ServiceServer setPositionService = n.advertiseService<armbot_move::SetPosition::Request, armbot_move::SetPosition::Response>
                                ("set_position", boost::bind(setPosition, _1, _2, move_group, start_state, joint_model_group));

    ros::ServiceServer armbotRunService = n.advertiseService("armbot_run", runArmbot);

    // // Получает значение joint из Arduino
    // ros::ServiceServer setJointsService = n.advertiseService<rosserial_arduino::Test::Request, rosserial_arduino::Test::Response>
    //                                       ("set_joints_model", boost::bind(writeJointsFromArduino, _1, _2));

    // Сохраняет позицию из Arduino
    ros::Subscriber savePositionSubscriber = n.subscribe("save_position", 1000, executeCommand);

    ros::Duration(1).sleep();
    ros::waitForShutdown();
    return 0;
}
