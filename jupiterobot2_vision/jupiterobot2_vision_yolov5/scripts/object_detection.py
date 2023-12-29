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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from jupiterobot2_msgs.msg import YoloMsg, ObjectMsg

from detector import Detector


class ObjectDetection():

    def __init__(self):
        
        rospy.init_node('object_detection')

        self.detector = Detector()

        self.bridge = CvBridge()

        rospy.Subscriber('camera/color/image_raw', Image, self.image_callback)

        # 定义数据的发布
        self.info_pub = rospy.Publisher('yolo/object_info', YoloMsg, queue_size=10)
        self.yolo_msgs_pub = YoloMsg()


    def image_callback(self, msg):
        try:
            img0 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(str(e))
            return

        img0, result = self.detector.detect(img0)

        cv2.imshow('result', img0)
        cv2.waitKey(1)

        # 临时的数据类型存放在列表
        object_list = []
        for i, r in enumerate(result):
            rospy.loginfo(
                f'{i}: ({r.u1}, {r.v1}) ({r.u2}, {r.v2})' +
                f' {r.name}, {r.conf:.3f}')
            object_msgs_pub = ObjectMsg()
            object_msgs_pub.id = i
            object_msgs_pub.name = r.name
            object_msgs_pub.conf = r.conf
            object_msgs_pub.xmin = r.u1
            object_msgs_pub.ymin = r.v1
            object_msgs_pub.xmax = r.u2
            object_msgs_pub.ymax = r.v2
            object_list.append(object_msgs_pub)

        # 将列表的数据类型赋值给自定义的数据类型
        self.yolo_msgs_pub = object_list
        self.info_pub.publish(self.yolo_msgs_pub)

if __name__ == "__main__":
    ObjectDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

