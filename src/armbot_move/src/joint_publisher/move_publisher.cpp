#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <armbot_move/move_position.h>
#include <std_msgs/String.h>

#include <iostream>
#include <math.h> 
#include <stdio.h>

#define JOINT_COUNT 4
#define STEP_VALUE 0.01

std::string position;

urdf::JointConstSharedPtr joint_1;
urdf::JointConstSharedPtr joint_2;
urdf::JointConstSharedPtr joint_3;
urdf::JointConstSharedPtr joint_4;
urdf::JointConstSharedPtr joint_grip;

float currentPosition[JOINT_COUNT] = {0, 0, 0, 0};
float receivedPosition[JOINT_COUNT] = {1, 0, 0, 0};

void reader(const armbot_move::move_position & message) {}

void publishMessage (ros::Publisher pub, float step, int jointId) {
     sensor_msgs::JointState msg;
     msg.header.stamp = ros::Time::now();

     switch (jointId) {
          case 0: 
               msg.name.push_back(joint_1->name);
               break;
          case 1: 
               msg.name.push_back(joint_2->name);
               break;
          case 2: 
               msg.name.push_back(joint_3->name);
               break;
          case 3: 
               msg.name.push_back(joint_4->name);
               break;
     }

     msg.position.push_back(currentPosition[jointId]);
     currentPosition[jointId] = round((currentPosition[jointId] + step) *100)/100;

     pub.publish(msg);
     ros::Duration(STEP_VALUE).sleep();
}


void goToPosition (ros::Publisher pub, std::string position) {
     float positions[JOINT_COUNT];

     memcpy(positions, receivedPosition, sizeof receivedPosition);

     for (int jointId = 0; jointId < JOINT_COUNT; jointId++) {

          if (currentPosition[jointId] < positions[jointId]) {
               while (currentPosition[jointId] <= positions[jointId]) {
                    publishMessage (pub, STEP_VALUE, jointId);
               }
          } else {
               while (positions[jointId] <= currentPosition[jointId]) {
                    publishMessage (pub, -STEP_VALUE, jointId);
               }
          }
     } 
}


int main(int argc, char *argv[]) {
     ROS_INFO("start:");
     ros::init(argc, argv, "publisher");
     ros::NodeHandle n;

     ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
     ros::Subscriber sub = n.subscribe("Position", 1000, reader);
     ros::Duration(1).sleep();

     std::string urdf_file_name;
     ros::param::get("/file", urdf_file_name);

     urdf::Model model;
     if (!model.initFile(urdf_file_name)) {
         ROS_ERROR("Failed to parse urdr file");
         return -1;
     }

     joint_1 = model.getJoint("joint_1");
     joint_2 = model.getJoint("joint_2");
     joint_3 = model.getJoint("joint_3");
     joint_4 = model.getJoint("joint_4");
     joint_grip = model.getJoint("joint_grip");

     while (ros::ok()) {

          ROS_INFO("dsvsdgsdfg:");

          boost::shared_ptr<armbot_move::move_position const> positions;
          positions = ros::topic::waitForMessage<armbot_move::move_position>("Position", ros::Duration(0.5));

          std::cout << "----------------------" << positions.use_count()  << std::endl;

          // ждем пока придет сообщение
          if (positions.use_count() == 0) {
               continue;
          }

          position = positions.get()[0].position.c_str();
          receivedPosition[0] = positions.get()[0].joint_1;
          receivedPosition[1] = positions.get()[0].joint_2;
          receivedPosition[2] = positions.get()[0].joint_3;
          receivedPosition[3] = positions.get()[0].joint_4;


          goToPosition(pub, position);

          ros::Duration(1).sleep();
     }

    ros::spinOnce();

    return 0;
}
