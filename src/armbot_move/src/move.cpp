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


/// Logs Start ///
void logSimple(const char* logText, const char* data) {
    char log[400];
    strcpy(log, logText);
    strcat(log, data);
    logs.writeLog(log, FILENAME);
}

void logGetCoordinates(const std::string* x, const std::string* y, const std::string* z) {
    char log[255];
    strcpy(log, "Coordinates obtained for saving: x=");
    strcat(log, x->c_str());
    strcat(log, ", y=");
    strcat(log, y->c_str());
    strcat(log, ", z=");
    strcat(log, z->c_str());
    logs.writeLog(log, FILENAME);
}

void logPrintPose(const geometry_msgs::Pose pose) {
    char log[255];
    strcpy(log, "Get pose: position_X=");
    strcat(log, boost::lexical_cast<std::string>(pose.position.x).c_str());
    strcat(log, ", position_Y=");
    strcat(log, boost::lexical_cast<std::string>(pose.position.y).c_str());
    strcat(log, ", position_Z=");
    strcat(log, boost::lexical_cast<std::string>(pose.position.z).c_str());
    strcat(log, ", orientation_X=");
    strcat(log, boost::lexical_cast<std::string>(pose.orientation.x).c_str());
    strcat(log, ", orientation_Y=");
    strcat(log, boost::lexical_cast<std::string>(pose.orientation.y).c_str());
    strcat(log, ", orientation_Z=");
    strcat(log, boost::lexical_cast<std::string>(pose.orientation.z).c_str());
    logs.writeLog(log, FILENAME);
}

void logPrintJoints(const std::vector<double> joints) {
    char log[255];
    strcpy(log, "Get joints value: 1=");
    strcat(log, boost::lexical_cast<std::string>(joints.at(0)).c_str());
    strcat(log, ", 2=");
    strcat(log, boost::lexical_cast<std::string>(joints.at(1)).c_str());
    strcat(log, ", 3=");
    strcat(log, boost::lexical_cast<std::string>(joints.at(2)).c_str());
    strcat(log, ", 4=");
    strcat(log, boost::lexical_cast<std::string>(joints.at(3)).c_str());
    logs.writeLog(log, FILENAME);
}
/// Logs End ///


void webSocketMessage(const std::string &message) {
    printf(">>> %s\n", message.c_str());
    logSimple("Websocket. Get message from server:  ", message.c_str());

    // Не закрывать соединение при статусе CONNECT
    if (message.find("\"status\":\"CONNECT\"") == std::string::npos) {
        ws->close();
        logs.writeLog("Websocket. Connect close.", FILENAME);
    }
}

void webSocket(const std::string &message) {

    ws = WebSocket::from_url(websocketUrl);
    assert(ws);
    logSimple("Websocket. Connect to:  ", websocketUrl);
    ws->send(message.c_str());
    logSimple("Websocket. Send data:  ", message.c_str());
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
         auto x = boost::lexical_cast<std::string>(transform.getOrigin().x() * 1000);
         auto y = boost::lexical_cast<std::string>(-transform.getOrigin().y() * 1000); // Почему-то записывает с другим знаком
         auto z = boost::lexical_cast<std::string>(transform.getOrigin().z() * 1000);

         if (saveWebSocket) {
            // отправляем вебсокет
            std::string commandForWebsocket = x + " " + y + " " + z;
            webSocket(commandForWebsocket);
         }

        if (saveToFile) {
             ROS_INFO("Input command name:");
             std::string commandName;
             std::cin >> commandName;
             logSimple("User input command name: ", commandName.c_str());
             std::cout << "Command saved!" << std::endl;

             std::ifstream file(commandDescriptionFile);
             if (file.bad() == true) {
                logs.writeLog("File is not exist", FILENAME);
             } else {
                 std::ofstream out;
                 out.open(commandDescriptionFile, std::ios::app);
                 std::string command = commandName + ":" + x + " " + y + " " + z;

                 // Если файл не пустой, делаем перенос строки
                 std::ifstream file(commandDescriptionFile);
                 if (!isEmpty(file)) {
                     out << std::endl;
                 }

                 out << command;
                 out.close();

                 logSimple("Point coordinates saved:  ", command.c_str());
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


void executeCommand(const std_msgs::String::ConstPtr& msg){

    char command[50];
    strcpy (command, msg->data.c_str());

    logSimple("Get command: ", command);

    if (strcmp("save", command) == 0) {
        saveCommand();
        return;
    }

    if (strcmp("stop", command) == 0) {
        stopCommand();
        return;
    }
}


bool runArmbot(armbot_move::RunArmbot::Request  &req, armbot_move::RunArmbot::Response  &res) {
    system ("$ARMBOT_PATH/scripts/armbot.sh start false");
    res.result = "FINISH in ROS";
    return true;
}

void testArduino (ros::ServiceClient client, rosserial_arduino::Test srv) {
    srv.request.input = "INPUT!!!";

    char log[255];
    if (client.call(srv)) {
        ROS_INFO("Result: %s", srv.response.output.c_str());
    } else {
        ROS_ERROR("Failed to call service arduino");
    }
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
    } else if (req.z != zPositionNone) { // передана координата Z
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

        logPrintJoints(move_group->move->getCurrentJointValues());

        result = "SUCCESS. Position: " + req.position;
        logSimple("Command execution result: ", result.c_str());

        ////////////
        ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<rosserial_arduino::Test>("test_arduino");
        rosserial_arduino::Test srv;
        testArduino(client, srv);
        //////////////

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
        logSimple("Exception while creating planning plugin loader ", boost::lexical_cast<std::string>(ex.what()).c_str());
    }

    move_group->move->setPlanningTime(60*5);
    move_group->move->setGoalTolerance(.0001);

    // Получает позицию
    ros::NodeHandle n;

    ros::ServiceServer setPositionService = n.advertiseService<armbot_move::SetPosition::Request, armbot_move::SetPosition::Response>
                                ("set_position", boost::bind(setPosition, _1, _2, move_group, start_state, joint_model_group));

    ros::ServiceServer armbotRunService = n.advertiseService("armbot_run", runArmbot);

    ros::Subscriber savePositionSubscriber = n.subscribe("save_position", 1000, executeCommand);

    ros::Duration(1).sleep();
    ros::waitForShutdown();
    return 0;
}
