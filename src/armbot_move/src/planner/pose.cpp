#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>


int main(int argc, char *argv[]) {


    ros::init(argc, argv, "pose");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    if (!node_handle.getParam("planning_plugin", planner_plugin_name))
      ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("link_end");
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    visual_tools.loadRemoteControl();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();


    move_group.setPlanningTime(60*5);
    move_group.setGoalTolerance(.001);

    geometry_msgs::Pose pose;
    // pose.header.frame_id = "link_1";
    pose.position.x = 0.142599;
    pose.position.y = -4.18211e-10;
    pose.position.z = 0.19702;
    pose.orientation.x = 1.76763e-09;
    pose.orientation.y = 0.984811;
    pose.orientation.z = 2.07928e-09;
    pose.orientation.w = 0.173631;


    move_group.setPoseTarget(pose,"link_end");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");


    if (success) {
        std::cout<<"start : "<<std::endl;

        move_group.setPoseTarget(pose);
        ros::Duration(0.5).sleep();

        move_group.move();
    }

    geometry_msgs::PoseStamped robot_pose;
    robot_pose = move_group.getCurrentPose();

    geometry_msgs::Pose current_position;
    current_position = robot_pose.pose;

    /*Retrive position and orientation */
    geometry_msgs::Point exact_pose = current_position.position;
    geometry_msgs::Quaternion exact_orientation = current_position.orientation;

    ROS_INFO("Reference frame : %s",move_group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame : %s",move_group.getEndEffectorLink().c_str());

    std::cout<<"Robot position : "<<exact_pose.x<<"\t"<<exact_pose.y<<"\t"<<exact_pose.z<<std::endl;
    std::cout<<"Robot Orientation : "<<exact_orientation.x<<"\t"<<exact_orientation.y<<"\t"<<exact_orientation.z<<"\t"<<exact_orientation.w<<std::endl;

    sleep(5.0);
    ros::shutdown(); 

    return 0;
}
