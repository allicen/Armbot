#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <armbot_move/move_position.h>

#include <iostream>

std::string position;

// обрабочик сообщения 
void callback (const sensor_msgs::JointState::ConstPtr &msg) {

    std::cout << "----------------------" << std::endl;
    std::cout << msg->name.size() << "\t" << msg->position.size() << "\t";
    std::cout << msg->velocity.size() << "\t" << msg->effort.size() << std::endl;

    int n = msg->name.size();
    for (int i = 0; i < n; i++) {
        std::cout << msg->name[i] << ":" << msg->position[i] << std::endl;
    }
}


void reader (const armbot_move::move_position & message) {
   
    ROS_INFO("Position: %s", message.position.c_str());
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "subscriber");
    ros::NodeHandle n; // создаем топик
    //ros::Subscriber sub = n.subscribe("joint_states", 1000, callback);

    ros::Subscriber sub = n.subscribe("Position", 1000, reader);
    ROS_INFO("fgsgsdg");
    ros::spin(); // не передает управление ноде, пирерывать только из другого потока или из терминала

    return 0;
}