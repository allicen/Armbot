#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <armbot_move/move_position.h>
#include <std_msgs/String.h>

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

#include "settings.h"


void reader(const armbot_move::move_position & message) {}

int main(int argc, char *argv[]) {
     ROS_INFO("start:");
     ros::init(argc, argv, "move");

     // Подписка на позицию
     ros::NodeHandle n;
     ros::Subscriber sub = n.subscribe("Position", 1000, reader);
     ros::Duration(1).sleep();

     ros::NodeHandle node_handle("~");
     ros::AsyncSpinner spinner(10);
     spinner.start();

     static const std::string PLANNING_GROUP = "arm";
     moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

     robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

     robot_state::RobotState start_state(*move_group.getCurrentState());
     const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

     const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
     moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
     kinematic_state->setToDefaultValues();
     const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

     boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
     planning_interface::PlannerManagerPtr planner_instance;
     std::string planner_plugin_name;

     if (!node_handle.getParam("planning_plugin", planner_plugin_name)) {
         ROS_FATAL_STREAM("Could not find planner plugin name");
     }

     try {
       planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
     } catch (pluginlib::PluginlibException& ex) {
       ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
     }

     move_group.setPlanningTime(60*5);
     move_group.setGoalTolerance(.001);

     while (ros::ok()) {
          boost::shared_ptr<armbot_move::move_position const> positions;
          positions = ros::topic::waitForMessage<armbot_move::move_position>("Position", ros::Duration(0.5));

          // ждем пока придет сообщение
          if (positions.use_count() == 0) {
               continue;
          }

          armbot_move::move_position position = positions.get()[0];
          Orientation orientation;

          geometry_msgs::Pose pose;
          pose.position.x = position.x;
          pose.position.y = position.y;
          pose.position.z = zPositionDefault;
          pose.orientation.x = orientation.x;
          pose.orientation.y = orientation.y;
          pose.orientation.z = orientation.z;
          pose.orientation.w = orientation.w;

          move_group.setApproximateJointValueTarget(pose,"link_grip");
          bool success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

          ROS_INFO("Visualizing move 1 (pose goal) %s",success?"":"FAILED");

          if (success) {
              std::cout<<"Start move"<<std::endl;
              start_state.setFromIK(joint_model_group, pose);
              move_group.setStartState(start_state);

              std::vector<double> joints;
              joints = move_group.getCurrentJointValues();
              std::cout<<"Joints : "<<joints.at(0)<<"\t"<<joints.at(1)<<"\t"<<joints.at(2)<<"\t"<<joints.at(3)<<std::endl;
          }

          ros::Duration(1).sleep();
     }

    ros::spinOnce();
    return 0;
}
