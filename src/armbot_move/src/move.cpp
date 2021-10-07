#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <pluginlib/class_loader.h>
#include <armbot_move/SetPosition.h>
#include <armbot_move/SavePosition.h>
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

#include "MoveOperationClass.hpp"
#include "settings.hpp"


void savePosition(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Get command: [%s]", msg->data.c_str());

    tf::TransformListener listener;

    tf::StampedTransform transform;
    try{
         listener.waitForTransform("/link_grip","/base_link", ros::Time(), ros::Duration(5.0));
         listener.lookupTransform("/link_grip", "/base_link", ros::Time(), transform);
         auto x = std::to_string(transform.getOrigin().x());
         auto y = std::to_string(transform.getOrigin().y());
         auto z = std::to_string(transform.getOrigin().z());

         ROS_INFO("Input command name:");
         std::string commandName;
         std::cin >> commandName;

         ROS_INFO("commandName .....%s", commandName.c_str());
         ROS_INFO("X .....%s", x.c_str());
         ROS_INFO("Y .....%s", y.c_str());
         ROS_INFO("Z .....%s", z.c_str());

         std::ifstream iff(commandDescriptionFile);
         if (iff.bad() == true) {
            ROS_ERROR("File is not exist");
         } else {
             std::ofstream out;
             out.open(commandDescriptionFile, std::ios::app);
             std::string command = "keyEnter:" + commandName + " " + x + " " + y + " " + z;

             ROS_INFO("save command .....%s", command.c_str());
             out << command;
             out.close();
         }

    } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    return;
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
    pose.position.z = req.position == "button-pressed" ? zPositionDefaultDown : zPositionDefault;
    pose.orientation.x = orientation.x;
    pose.orientation.y = orientation.y;
    pose.orientation.z = orientation.z;
    pose.orientation.w = orientation.w;

    move_group->move->setApproximateJointValueTarget(pose,"link_grip");
    bool success = (move_group->move->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Visualizing move 1 (pose goal) %s",success?"":"FAILED");

    if (success) {
        std::cout<<"Start move"<<std::endl;
        start_state.setFromIK(joint_model_group, pose);
        move_group->move->setStartState(start_state);

        std::cout<<"Output : "<<pose.position.x<<"\t"<<pose.position.y<<"\t"<<pose.position.z<<"\t"<<pose.orientation.x
                             <<"\t"<<pose.orientation.y<<"\t"<<pose.orientation.z<<"\t"<<pose.orientation.w<<std::endl;

        std::vector<double> joints;
        joints = move_group->move->getCurrentJointValues();
        std::cout<<"Joints : "<<joints.at(0)<<"\t"<<joints.at(1)<<"\t"<<joints.at(2)<<"\t"<<joints.at(3)<<std::endl;

        result = "SUCCESS. Position: " + req.position;
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
    }

    move_group->move->setPlanningTime(60*5);
    move_group->move->setGoalTolerance(.0001);

    // Получает позицию
    ros::NodeHandle n;

    ros::ServiceServer setPositionService = n.advertiseService<armbot_move::SetPosition::Request, armbot_move::SetPosition::Response>
                                ("set_position", boost::bind(setPosition, _1, _2, move_group, start_state, joint_model_group));

    ros::Subscriber savePositionSubscriber = n.subscribe("save_position", 1000, savePosition);

    ros::Duration(1).sleep();
    ros::waitForShutdown();
    return 0;
}
