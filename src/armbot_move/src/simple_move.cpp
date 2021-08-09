#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

void callback (const geometry_msgs::Twist& message) {
    ROS_INFO("test **********************");
    ROS_INFO("Got:\n"
            "1) pos.linear: x= %f y= %f z= %f\n"
            "2) pos.angular x= %f y= %f z= %f\n",
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z);
    return;
}


int main(int argc, char **argv) {

    ROS_INFO("**********************");

    ros::init(argc,argv,"simple_move");

    ros::NodeHandle n; // объект управления нодой
    ros::Subscriber sub = n.subscribe("/turtle1/cmd_vel", 1000, &callback); // подписка на клавиатуру

    ros::spin();
    return 0;
}
