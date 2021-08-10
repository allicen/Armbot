#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle nh;

std_msgs::String str; 
ros::Publisher chatter("chatter", &str);
char message[13] = "start...";

Servo servo_down;

int buttonPin = 2;
int ledPin = 8;
boolean lastButton = LOW;
boolean currentButton = LOW;
boolean ledOn = false;
boolean servoStraight = true;

int servoDownPosition = 0;

void setup() {

  servo_down.attach(7);
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(chatter);


  str.data = message; 
  chatter.publish(&str);
}

void loop() {

  start();
  
  nh.spinOnce();
  delay(1);
}


void start() {
    currentButton = debounce(lastButton);
    if (currentButton == HIGH && lastButton == LOW) {
      ledOn = !ledOn;
    } 
      
    lastButton = currentButton;


    digitalWrite(ledPin, ledOn);

    servoRun(ledOn);
}

// стабилизация нажатия на кнопку
boolean debounce (boolean last) {
  boolean current = digitalRead(buttonPin);
  if (last != current) {
    delay(5);
    current = digitalRead(buttonPin);
  }

  return current;
}


void servoRun(boolean start) {
  
  if (start) {
     if (servoDownPosition == 180) {
        servoStraight = false;
     } else if (servoDownPosition == 0) {
        servoStraight = true;
     }

      servo_down.write(servoDownPosition);
      
      if (servoStraight) {
          servoDownPosition++;
      } else {
          servoDownPosition--;
      }
    
      delay(15);
  }    
}
