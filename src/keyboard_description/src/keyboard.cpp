#include <ros/ros.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "keyboard");

  ros::NodeHandle nh;
  ros::Publisher keyboard;


  ROS_INFO("Should have published");
  ros::Duration(1).sleep();
  ros::shutdown();
}