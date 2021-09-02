#include <ros/ros.h>
#include <armbot_move/move_position.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <iterator>
#include <vector>


int main(int argc, char**argv) {
    ros::init(argc, argv, "writer");
    
    ROS_INFO_STREAM("Writer is ready.");
    
    ros::NodeHandle n("~");
    ros::Publisher p = n.advertise<armbot_move::move_position>("GetPosition", 1000);
    // ros::Duration(1).sleep();

    std::vector<std::string> params;
    std::string param;
    n.getParam("param", param);
    ROS_INFO("Got parameter : %s", param.c_str());

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<armbot_move::move_position>("Position", 1000);

    std::stringstream data(param);

    std::string line;
    while(std::getline(data, line,' ')) {
        params.push_back(line); 
    }

    double delay = atof(params[5].c_str());

    sleep(1);

    armbot_move::move_position message;
    message.position = params[0].c_str();
    message.joint_1 = std::stof(params[1].c_str());
    message.joint_2 = std::stof(params[2].c_str());
    message.joint_3 = std::stof(params[3].c_str());
    message.joint_4 = std::stof(params[4].c_str());

    pub.publish(message);
    
    ros::Duration(delay).sleep();
    ros::spinOnce();
    
    ROS_INFO_STREAM("Publishing is finished!\n");
    return 0;
}
