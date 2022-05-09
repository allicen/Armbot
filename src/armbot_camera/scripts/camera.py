#!/usr/bin/env python

import time
from math import sin, cos
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
import base64
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from armbot_camera.srv import DefaultService, DefaultServiceResponse
from armbot_camera.msg import ImageCamera

class Camera():

    def __init__(self):
        rospy.init_node('camera', anonymous=True)

        self.cv_bridge = CvBridge()
        self.Image1 = None
        self.Image2 = None
        self.msg_img = ImageCamera()

        #### known sizes
        self.servo_width_real = 0
        self.robot_width = 0
        self.image_width = 600
        self.image_height = 600

        self.start_position = [self.image_width, self.image_height, 0, 0]
        self.line_position_prev = (0, 0)
        self.permissible_error = 0.0005

        rospy.Subscriber("armbot/camera1/image_raw", Image, self.camera_cb)
        rospy.Subscriber("armbot/camera2/image_raw", Image, self.camera_cb2)
        rospy.Subscriber("/return_default_position", String, self.return_default_position)

        self.pub = rospy.Publisher('room_camera_one', ImageCamera, queue_size=10)

        rospy.Service('return_default_pos_camera', DefaultService, self.return_default_pos_camera)


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


    def return_default_pos_camera(self, req):
        pos = self.get_position_arm_tool(self.Image1)[1]
        print(self.start_position)
        print(pos)
        return DefaultServiceResponse("RESP")


    def spin(self):

        start_time = time.time()
        while not rospy.is_shutdown():
            self.rate.sleep()

            if self.Image1 is not None and self.Image2 is not None:
                cv2.imshow("Room camera", self.Image1)
                cv2.imshow("Robot camera", self.Image2)
                cv2.waitKey(3)

            if self.Image1 is not None:
                img = self.cv_bridge.cv2_to_imgmsg(self.Image1)

                _, buffer_img= cv2.imencode('.jpg', self.Image1)

                self.msg_img.data = base64.b64encode(buffer_img).decode("utf-8")
                self.msg_img.encoding = 'base64'
                self.msg_img.width = img.width
                self.msg_img.height = img.height

                # print(self.msg_img.data)

                self.pub.publish(self.msg_img)


    def shutdown(self):
        rospy.sleep(1)


camera = Camera()

camera.spin()
