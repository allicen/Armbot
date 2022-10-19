#!/usr/bin/env python

import time
from math import sin, cos, sqrt, atan
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
import base64
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from armbot_camera.srv import DefaultService, DefaultServiceResponse
from armbot_camera.srv import RobotInfoService, RobotInfoServiceResponse
from armbot_camera.msg import ImageCamera

class Camera():

    def __init__(self):
        rospy.init_node('camera', anonymous=True)

        self.cv_bridge = CvBridge()
        self.Image1 = None
        self.ImageRobotLeft = None
        self.ImageRobotRight = None
        self.Image3 = None
        self.Image_table = None
        self.msg_img = ImageCamera()

        #### known sizes --- START
        self.room_camera_distanse_table = 0.73
        self.servo_width = 12      #mm
        self.robot_width = 234     #mm
        self.image_width = 600     #px
        self.image_height = 500    #px
        #### known sizes --- END

        self.start_position_marker = [self.image_width, self.image_height, 0, 0]
        self.current_position_marker = [0, 0, 0, 0]
        self.line_position_prev = (0, 0)
        self.line_position = (0, 0)
        self.marker_radius_model_start = 1
        self.marker_radius_model_current = 1

        self.scale = 1.3 # error = +-0.02
        self.z_scale = 0.0156294

        self.permissible_error = 0.0005
        self.joint_1, self.joint_2, self.joint_3, self.joint_4 = 0, 0, 0, 0

        rospy.Subscriber("/armbot/camera1/image_raw", Image, self.camera_cb)
        rospy.Subscriber("/armbot/camera3/image_raw", Image, self.camera_cb_depth)
        rospy.Subscriber("/armbot/camera_robot_left/image_raw", Image, self.camera_robot_left)
        rospy.Subscriber("/armbot/camera_robot_right/image_raw", Image, self.camera_robot_right)
        rospy.Subscriber("/armbot/camera_table/image_raw", Image, self.camera_table)
        rospy.Subscriber("/return_default_position", String, self.return_default_position)

        self.pub = rospy.Publisher('room_camera_one', ImageCamera, queue_size=10)

        rospy.Service('return_default_pos_camera', DefaultService, self.return_default_pos_camera)
        rospy.Service('robot_info_by_camera', RobotInfoService, self.robot_info)


        self.rate = rospy.Rate(30)

        rospy.on_shutdown(self.shutdown)


    def camera_cb(self, msg):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        image_info = self.get_position_arm_tool(cv_image)
        self.Image1 = image_info[0]

        if self.start_position_marker == [self.image_width, self.image_height, 0, 0]:
            self.start_position_marker = image_info[1]
        else:
            self.current_position_marker = image_info[1]


    def camera_cb_depth(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))
    

        if self.Image1 is not None and self.Image3 is not None:
            img1 = cv2.cvtColor(self.Image1, cv2.COLOR_BGR2GRAY)
            img2 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            minDisparity = 0
            numDisparities = 14

            stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
            disparity = stereo.compute(img1, img2)
            disparity = disparity.astype(np.float32)
            disparity = (disparity/16.0 - minDisparity)/numDisparities

            self.Image3 = disparity
        else:
            self.Image3 = cv_image


    def camera_table(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.Image_table = cv_image
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))


    def camera_robot_left(self, msg):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        self.ImageRobotLeft = cv_image


    def camera_robot_right(self, msg):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        self.ImageRobotRight = cv_image


    def get_position_arm_tool(self, cv_image):

        img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([100, 150, 0], dtype=np.uint8)
        upper_blue = np.array([140, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        _, contours, hierarchy = cv2.findContours(mask.copy(), 1, 2)

        rect_x, rect_y, rect_w, rect_h =  self.image_width, self.image_height, 0, 0

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

        self.line_position = ((rect_x + rect_w) / 2, (rect_y + rect_h) / 2)
        self.marker_radius_model_current = max(rect_w - rect_x, rect_h - rect_y)

        cv2.rectangle(cv_image,(rect_x, rect_y), (rect_w, rect_h), (0, 255, 0), 2)

        if (self.line_position_prev == (0, 0)):
            self.line_position_prev = self.line_position
            self.marker_radius_model_start = max(rect_w - rect_x, rect_h - rect_y)
        
        cv2.line(cv_image, self.line_position_prev, self.line_position, (0, 0, 255), 3)
                

        return  [cv_image, [rect_x, rect_y, rect_w, rect_h]]



    def return_default_position(self, msg):
        print(msg.data)
        pos = self.get_position_arm_tool(self.Image1)[1]
        print(self.start_position_marker)
        print(pos)

    def return_default_pos_camera(self, req):
        pos = self.get_position_arm_tool(self.Image1)[1]
        print(self.start_position_marker)
        print(pos)
        return DefaultServiceResponse("RESP")

    def robot_info(self, req):
        w = self.line_position_prev[0] - self.line_position[0]
        h = self.line_position_prev[1] - self.line_position[1]

        x_error = round(abs(h) * self.scale, 2)
        y_error = round(abs(w) * self.scale, 2)

        if self.line_position[1] > self.image_height / 2:
            x_error = -x_error

        if self.line_position[0] > self.image_width / 2:
            y_error = -y_error

        if h != 0:
            h_correct = self.image_height/2 - self.line_position[1]
            self.joint_1 = atan(w / float(h_correct))

        response = RobotInfoServiceResponse()
        response.info = "simple text"
        response.is_start_position = x_error == 0 and y_error == 0
        response.x_error = x_error
        response.y_error = y_error
        response.z_error = 0.5
        response.joint_1 = self.joint_1
        response.joint_2 = self.joint_2
        response.joint_3 = self.joint_3
        response.joint_4 = self.joint_4

        print(response)

        return response


    def spin(self):

        start_time = time.time()
        while not rospy.is_shutdown():
            self.rate.sleep()

            # if self.Image1 is not None and self.Image2 is not None and self.Image3 is not None and self.Image_table is not None:
            #     cv2.imshow("Room camera", self.Image1)
            #     cv2.imshow("Robot camera", self.Image2)
            #     cv2.imshow("Room camera Depth", self.Image3)
            #     cv2.imshow("Table camera", self.Image_table)
            #     cv2.waitKey(3)

            if self.ImageRobotLeft is not None and self.ImageRobotRight is not None:
                cv2.imshow("Robot camera left", self.ImageRobotLeft)
                cv2.imshow("Robot camera right", self.ImageRobotRight)
                cv2.waitKey(3)


            if self.Image1 is not None:
                img = self.cv_bridge.cv2_to_imgmsg(self.Image1)

                _, buffer_img= cv2.imencode('.jpg', self.Image1)

                self.msg_img.data = base64.b64encode(buffer_img).decode("utf-8")
                self.msg_img.encoding = 'base64'
                self.msg_img.width = img.width
                self.msg_img.height = img.height

                self.pub.publish(self.msg_img)


    def shutdown(self):
        rospy.sleep(1)


camera = Camera()

camera.spin()
