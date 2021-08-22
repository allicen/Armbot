#include <ros/ros.h>
#include <armbot_move/move_position.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>


int main(int argc, char**argv) {
    ros::init(argc, argv, "writer");
    
    ROS_INFO_STREAM("Writer is ready.\n");
    
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<armbot_move::move_position>("Position", 10);
    
    std::vector<std::string> messageList;
    messageList.push_back("first");
    messageList.push_back("second");

    sleep(1);
    
    ros::Rate loop_rate(5);
    for (int i = 0; i < messageList.size(); i++)
    {
        armbot_move::move_position message;
        message.position = messageList[i];

        pub.publish(message);
        
        ROS_INFO("%s", messageList[i].c_str());
        ros::Duration(10.0).sleep();
        ros::spinOnce();
    }
    
    ROS_INFO_STREAM("Publishing is finished!\n");
    return 0;
}
