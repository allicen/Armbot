// Считывание нажатий клавиш на клавиатуре
//=====rosrun armbot_move moveMotor
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <string>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define FORWARD 65
#define INVERSE 66
#define RESET 48

// Направление движения двигателя, отправляется на плате Arduino
#define FORWARD_DIRECTION 0
#define INVERSE_DIRECTION 1

int getch() {
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "move_motor");

    ros::NodeHandle n;
    ros::Publisher motorMovePub = n.advertise<std_msgs::String>("move_motor", 1000);

    int motorIndex = 0;

    ROS_INFO("Select the motor (click on the number): \n1) Bottom\n2) Left\n3) Right\n4) Servo top\n5) Servo bottom\n0) Reset (if motor has been selected)");

    while (ros::ok()) {
        int c = getch();   // call your non-blocking input function

        std_msgs::String msg;
        std::stringstream msgText;

        if (motorIndex == 0) {
            switch (c) {
                case '1':
                    motorIndex = 1;
                    break;
                case '2':
                    motorIndex = 2;
                    break;
                case '3':
                    motorIndex = 3;
                    break;
                case '4':
                    motorIndex = 4;
                    break;
                case '5':
                    motorIndex = 5;
                    break;    
                default:
                    ROS_ERROR("Invalid button pressed. Please click 1, 2, 3, 4 or 5");
            }

            if (motorIndex == 0) {
                ROS_INFO("No motor selected");
            } else {
                ROS_INFO("Motor selected: %d", motorIndex);
            }
        } else {
            switch (c) {
                case FORWARD:
                    ROS_INFO("%d MOTOR -- FORWARD", motorIndex);
                    msgText << motorIndex << ":" << FORWARD_DIRECTION;
                    msg.data = msgText.str();
                    motorMovePub.publish(msg);
                    break;
                case INVERSE:
                    ROS_INFO("%d MOTOR -- INVERSE", motorIndex);
                    msgText << motorIndex << ":" << INVERSE_DIRECTION;
                    msg.data = msgText.str();
                    motorMovePub.publish(msg);
                    break;
                case RESET:
                   motorIndex = 0;
                   ROS_INFO("Motor was reset.\nSelect the motor (click on the number): \n1) Bottom\n2) Left\n3) Right\n4) Servo top\n5) Servo bottom\n0) Reset (if motor has been selected)");
                   break;
            }
        }
    }

    ros::spin();

    return 0;
}