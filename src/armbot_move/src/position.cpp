#include <ros/ros.h>
#include <armbot_move/SetPosition.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <iterator>
#include <vector>


void writeLog(const char* log) {
    char shell[100] = "./scripts/functions_log_cpp.sh 'position.cpp ";
    strcat (shell, log);
    strcat (shell, "'");
    system(shell);
}

int startMoveToPosition(ros::ServiceClient client, armbot_move::SetPosition srv, std::string position, std::vector<std::string> params) {
    srv.request.position = position;
    srv.request.x = std::stof(params[1].c_str());
    srv.request.y = std::stof(params[2].c_str());

    char log[60];

    if (client.call(srv)) {
        ROS_INFO("Result: %s", srv.response.result.c_str());
        strcat (log, "Result: ");
        strcat (log, srv.response.result.c_str());
        writeLog(log);
    } else {
        ROS_ERROR("Failed to call service set_position: %s", position.c_str());
        strcat (log, "Failed to call service set_position: ");
        strcat (log, position.c_str());
        writeLog(log);
        return 1;
    }

    return 0;
}


int main(int argc, char**argv) {
    ros::init(argc, argv, "position");
    
    ROS_INFO_STREAM("Writer is ready.");
    writeLog("Writer is ready.");

    std::vector<std::string> params;
    std::string param;

    ros::NodeHandle n("~");
    n.getParam("param", param);
    ROS_INFO("Got parameter: %s", param.c_str());

    char log[100] = "Got parameter: ";
    strcat (log, param.c_str());
    writeLog(log);

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
    writeLog("Publishing is finished!");
    return result;
}
