#!/usr/bin/env python

import time
from math import sin, cos

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

class Camera():

    def __init__(self):
        rospy.init_node('camera', anonymous=True)

        self.cv_bridge = CvBridge()
        self.Image1 = None
        self.Image2 = None
        rospy.Subscriber("armbot/camera1/image_raw", Image, self.camera_cb)
        rospy.Subscriber("armbot/camera2/image_raw", Image, self.camera_cb2)
        self.rate = rospy.Rate(30)

        rospy.on_shutdown(self.shutdown)


    def camera_cb(self, msg):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.Image1 = cv_image
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def camera_cb2(self, msg):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.Image2 = cv_image
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))        


    def spin(self):

        start_time = time.time()
        while not rospy.is_shutdown():
            self.rate.sleep()

            if self.Image1 is not None:
                cv2.imshow("Down view camera from Robot", self.Image1)
                cv2.imshow("Down2 view camera from Robot", self.Image2)
                cv2.waitKey(3)


    def shutdown(self):
        rospy.sleep(1)


camera = Camera()

camera.spin()
