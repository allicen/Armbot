// Считывание нажатий клавиш на клавиатуре
// rosrun armbot_move moveMotor
#include <ros/ros.h>

#include <iostream>
#include <string>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KB_UP 65
#define KB_DOWN 66
#define KB_LEFT 68
#define KB_RIGHT 67

int getch() {
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "moveMotor");
    int motorIndex = 0;

    ROS_INFO("Select the motor (click on the number): \n1) Bottom\n2) Left\n3) Right\n0) Reset (if motor has been selected)");

    while (ros::ok()) {
        int c = getch();   // call your non-blocking input function

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
                default:
                    ROS_ERROR("Invalid button pressed. Please click 1, 2 or 3");
            }

            if (motorIndex == 0) {
                ROS_INFO("No motor selected");
            } else {
                ROS_INFO("Motor selected: %d", motorIndex);
            }
        } else {
//            std::cout << '\n' << c << '\n';
            switch (c) {
                case KB_LEFT:
                    ROS_INFO("LEFT\t");
                    break;
                case KB_RIGHT:
                    ROS_INFO("RIGHT\t");
                    break;
                case KB_UP:
                    ROS_INFO("UP\t");
                    break;
                case KB_DOWN:
                   ROS_INFO("DOWN\t");
                   break;
            }
        }
    }

    ros::spin();

    return 0;
}