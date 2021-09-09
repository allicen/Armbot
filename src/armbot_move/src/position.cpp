#include <ros/ros.h>
#include <armbot_move/move_position.h>
#include <armbot_move/SetPosition.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <iterator>
#include <vector>


int startMoveToPosition(ros::ServiceClient client, armbot_move::SetPosition srv, std::string position, std::vector<std::string> params) {
    srv.request.position = position;
    srv.request.x = std::stof(params[1].c_str());
    srv.request.y = std::stof(params[2].c_str());

    if (client.call(srv)) {
        ROS_INFO("Result: %s", srv.response.result.c_str());
    } else {
        ROS_ERROR("Failed to call service set_position: %s", position.c_str());
        return 1;
    }

    return 0;
}


int main(int argc, char**argv) {
    ros::init(argc, argv, "writer");
    
    ROS_INFO_STREAM("Writer is ready.");
    
    ros::NodeHandle n("~");
    ros::Publisher p = n.advertise<armbot_move::move_position>("GetPosition", 1000);
    // ros::Duration(1).sleep();

    std::vector<std::string> params;
    std::string param;
    n.getParam("param", param);
    ROS_INFO("Got parameter: %s", param.c_str());

    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<armbot_move::SetPosition>("set_position");
    armbot_move::SetPosition srv;

    std::stringstream data(param);

    std::string line;
    while(std::getline(data, line,' ')) {
        params.push_back(line); 
    }

    double delay = atof(params[3].c_str());

    sleep(1);

    int result = 0;

    // Goal position
    result = startMoveToPosition(client, srv, params[0].c_str(), params);

    // Button pressed
    result = startMoveToPosition(client, srv, "button-pressed", params);

    // Button up
    result = startMoveToPosition(client, srv, params[0].c_str(), params);

    ros::Duration(delay).sleep();
    ros::spinOnce();
    
    ROS_INFO_STREAM("Publishing is finished!\n");
    return result;
}
