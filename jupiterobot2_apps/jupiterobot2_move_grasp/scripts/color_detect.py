#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Jupiter Robot Technology Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from math import *

class image_converter:
    def __init__(self):    
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.target_sub = rospy.Subscriber("/grasp_target", String, self.target_callback)
        self.image_pub = rospy.Publisher("targte_image", Image, queue_size=1)
        self.target_pub = rospy.Publisher("target_pose", Pose,  queue_size=1)
        self.bridge = CvBridge()
        self.BGR_RANGE = ([200, 70, 20], [250, 200, 60])
        self.FLAG_PROCESS = 0

    def target_callback(self, msg):
        if msg.data.find("red") > -1:
            self.BGR_RANGE = ([40, 0, 180], [95, 15, 225])
        elif msg.data.find("green") > -1:
            self.BGR_RANGE = ([100, 235, 120], [220, 255, 200])
        elif msg.data.find("blue") > -1:
            self.BGR_RANGE = ([200, 70, 20], [250, 200, 60])
        self.FLAG_PROCESS = 1
        rospy.loginfo("Starting color_detect.")

    def image_callback(self, data):
        if self.FLAG_PROCESS == 1:
            # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            # define the list of boundaries in BGR
            boundaries = [self.BGR_RANGE]

            # loop over the boundaries
            # print(boundaries)
            for (lower, upper) in boundaries:
                # create NumPy arrays from the boundaries
                lower = np.array(lower, dtype = "uint8")
                upper = np.array(upper, dtype = "uint8")

            # find the colors within the specified boundaries and apply the mask
            mask = cv2.inRange(cv_image, lower, upper)
            output = cv2.bitwise_and(cv_image, cv_image, mask = mask)

            cvImg = cv2.cvtColor(output, 6) #cv2.COLOR_BGR2GRAY
            npImg = np.asarray(cvImg)
            thresh = cv2.threshold(npImg, 1, 255, cv2.THRESH_BINARY)[1]

            # find contours in the thresholded image
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            #contours = contours[0]

            # loop over the contours
            for c in contours:
                # compute the center of the contour
                M = cv2.moments(c)

                if int(M["m00"]) not in range(1000, 5000):
                    continue

                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                cv2.drawContours(cv_image, [c], -1, (0, 255, 255), 2)
                cv2.circle(cv_image, (cX, cY), 1, (0, 255, 255), -1)
                objPose = Pose()
                objPose.position.x = cX
                objPose.position.y = cY
                objPose.position.z = M["m00"]
                self.target_pub.publish(objPose)

            # 显示opencv格式的图像
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)

            # 再将opencv格式额数据转换成ros image格式的数据发布
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("color_detect")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down color_detect node.")
        cv2.destroyAllWindows()
