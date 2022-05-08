#!/usr/bin/env python

import time
from math import sin, cos
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String

class Camera():

    def __init__(self):
        rospy.init_node('camera', anonymous=True)

        self.cv_bridge = CvBridge()
        self.Image1 = None
        self.Image2 = None

        self.start_position = [600, 500, 0, 0]
        self.line_position_prev = (0, 0)
        self.permissible_error = 0.0005

        rospy.Subscriber("armbot/camera1/image_raw", Image, self.camera_cb)
        rospy.Subscriber("armbot/camera2/image_raw", Image, self.camera_cb2)
        rospy.Subscriber("/return_default_position", String, self.return_default_position)

        self.rate = rospy.Rate(30)

        rospy.on_shutdown(self.shutdown)


    def camera_cb(self, msg):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        image_info = self.get_position_arm_tool(cv_image)
        self.Image1 = image_info[0]

        if self.start_position == [600, 500, 0, 0]:
            self.start_position = image_info[1]



    def camera_cb2(self, msg):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        self.Image2 = cv_image


    def get_position_arm_tool(self, cv_image):

        img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([100,150,0], dtype=np.uint8)
        upper_blue = np.array([140,255,255], dtype=np.uint8)
        mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        _, contours, hierarchy = cv2.findContours(mask.copy(), 1, 2)

        rect_x, rect_y, rect_w, rect_h =  600, 500, 0, 0

        for i in range (len(contours)):
            cnt = contours[i]
            perimeter = cv2.arcLength(cnt, True)
            x,y,w,h = cv2.boundingRect(cnt)

            if x < rect_x:
                rect_x = x

            if y < rect_y:
                rect_y = y

            if x + w > rect_w:
                rect_w = x + w

            if y + h > rect_h:
                rect_h = y + h

        line_position = ((rect_x + rect_w) / 2, (rect_y + rect_h) / 2)

        cv2.rectangle(cv_image,(rect_x, rect_y), (rect_w, rect_h), (0, 255, 0), 2)

        if (self.line_position_prev == (0, 0)):
            self.line_position_prev = line_position
        
        cv2.line(cv_image, self.line_position_prev, line_position, (0, 0, 255), 3)
        

        return  [cv_image, [rect_x, rect_y, rect_w, rect_h]]



    def return_default_position(self, msg):
        print(msg.data)
        pos = self.get_position_arm_tool(self.Image1)[1]
        print(self.start_position)
        print(pos)



    def spin(self):

        start_time = time.time()
        while not rospy.is_shutdown():
            self.rate.sleep()

            if self.Image1 is not None and self.Image2 is not None:
                cv2.imshow("Room camera", self.Image1)
                cv2.imshow("Robot camera", self.Image2)
                cv2.waitKey(3)


    def shutdown(self):
        rospy.sleep(1)


camera = Camera()

camera.spin()
