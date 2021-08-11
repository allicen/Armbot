#include <Servo.h>

#include <ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#define servoCount 2
#define stepMotorCount 3

std_msgs::String str; 
ros::NodeHandle nodeHandle;

Servo robotServos[servoCount];

int servo_pins[servoCount] = {7, 6}; // PWM Pins on Arduino Uno
int midPositions[servoCount] = {100, 100};
int SERVO_CURRENT_POSITIONS[servoCount];

float TARGET_JOINT_POSITIONS[servoCount] = {0, 0};

// Convert the joint state values to degrees, adjust for the center and write to the servo
void writeServos() {
  for (int j = 0; j < servoCount; j++) {
    int targetAngle;
    if (j == 2) {
      // Due to difference in mounting directions
      targetAngle = - TARGET_JOINT_POSITIONS[j]*(180/3.14) + midPositions[j];
    } else {
      targetAngle = TARGET_JOINT_POSITIONS[j]*(180/3.14) + midPositions[j];
     
    }
    robotServos[j].write(targetAngle);
    SERVO_CURRENT_POSITIONS[j] = targetAngle;
  }
  nodeHandle.spinOnce();
}

// Subscriber Callback to store the jointstate position values in the global variables
void servoControlSubscriberCallbackJointState(const sensor_msgs::JointState& msg) {
  TARGET_JOINT_POSITIONS[0] = msg.position[0];
  TARGET_JOINT_POSITIONS[1] = msg.position[1];
  // Call the method to write the joint positions to the servo motors
  writeServos();

}

ros::Subscriber<sensor_msgs::JointState> servoControlSubscriberJointState("joint_states", &servoControlSubscriberCallbackJointState);

void setup() {
  // Initial the servo motor connections and initialize them at home position
  for (unsigned int i = 0; i < servoCount; i++) {
    robotServos[i].attach(servo_pins[i]);
    robotServos[i].write(midPositions[i]);
    SERVO_CURRENT_POSITIONS[i] = midPositions[i];
  }

  // Set the communication BaudRate and start the node
  nodeHandle.getHardware()->setBaud(115200);
  nodeHandle.initNode();
  nodeHandle.subscribe(servoControlSubscriberJointState);
}

void loop() {
  // Keep calling the spinOnce() method in this infinite loop to stay tightly coupled with the ROS Serial
  nodeHandle.spinOnce();
  delay(1);
}
