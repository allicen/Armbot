#include <ros/ros.h>
#include <armbot_move/SetPosition.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <iterator>
#include <vector>

#include "LogClass.hpp"
#include "settings.hpp"

char FILENAME[20] = "position.cpp";
LogClass logs;

int startMoveToPosition(ros::ServiceClient client, armbot_move::SetPosition srv, std::string position, std::vector<std::string> params) {
    srv.request.position = position;
    srv.request.x = std::stof(params[1].c_str());
    srv.request.y = std::stof(params[2].c_str());
    srv.request.z = params.size() == 5 ? std::stof(params[3].c_str()) : zPositionNone;

    if (client.call(srv)) {
        ROS_INFO("Result: %s", srv.response.result.c_str());
        logs.logSimple("Result: ", srv.response.result.c_str(), FILENAME);
    } else {
        ROS_ERROR("Failed to call service set_position: %s", position.c_str());
        logs.logSimple("Failed to call service set_position: ", position.c_str(), FILENAME);
        return 1;
    }

    return 0;
}


int main(int argc, char**argv) {
    ros::init(argc, argv, "position");
    
    ROS_INFO_STREAM("Writer is ready.");
    logs.logSimple("Writer is ready.", "", FILENAME);

    std::vector<std::string> params;
    std::string param;

    ros::NodeHandle n("~");
    n.getParam("param", param);
    ROS_INFO("Got parameter: %s", param.c_str());
    logs.logSimple("Got parameter: ", param.c_str(), FILENAME);

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<armbot_move::SetPosition>("set_position");
    armbot_move::SetPosition srv;

    std::stringstream data(param);

    std::string line;
    while(std::getline(data, line,' ')) {
        params.push_back(line); 
    }

    int result = 0;

    if (params.size() == 1) {
        DefaultPosition defaultPosition;

        params.push_back(boost::lexical_cast<std::string>(defaultPosition.x));
        params.push_back(boost::lexical_cast<std::string>(defaultPosition.y));
        params.push_back(boost::lexical_cast<std::string>(defaultPosition.z));

        result = startMoveToPosition(client, srv, params[0].c_str(), params);
        sleep(1);
        return result;
    }

    double delay = atof(params[params.size()-1].c_str());

    sleep(1);

    // Goal position
    result = startMoveToPosition(client, srv, params[0].c_str(), params);

    // // Button pressed
    result = startMoveToPosition(client, srv, "button-pressed", params);

    // // Button up
    result = startMoveToPosition(client, srv, params[0].c_str(), params);

    ros::Duration(delay).sleep();
    ros::spinOnce();
    
    ROS_INFO_STREAM("Publishing is finished!\n");
    logs.logSimple("Publishing is finished!", "", FILENAME);
    return result;
}
