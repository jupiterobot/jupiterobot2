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

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf2_geometry_msgs import PointStamped

from detector import Detector


class ObjectDetection():

    def __init__(self):

        rospy.init_node('object_detection')

        self.target_name = 'cell phone'
        self.frame_id = 'target'

        self.detector = Detector()

        self.bridge = CvBridge()
        self.pub_point = rospy.Publisher("target_point", PointStamped, queue_size=1)
        self.sub_info = Subscriber('/camera/depth/camera_info', CameraInfo)
        self.sub_color = Subscriber('/camera/color/image_raw', Image)
        self.sub_depth = Subscriber('/camera/depth/image_raw', Image)
        self.ts = ApproximateTimeSynchronizer([self.sub_info, self.sub_color, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.images_callback)
        self.broadcaster = TransformBroadcaster()
        
    def images_callback(self, msg_info, msg_color, msg_depth):
        try:
            img_color = CvBridge().imgmsg_to_cv2(msg_color, 'bgr8')
            img_depth = CvBridge().imgmsg_to_cv2(msg_depth, 'passthrough')
        except CvBridgeError as e:
            rospy.logwarn(str(e))
            return

        if img_color.shape[0:2] != img_depth.shape[0:2]:
            rospy.logwarn('彩色图像和深度图像分辨率不同')
            return

        img_color, result = self.detector.detect(img_color)

        cv2.imshow('color', img_color)

        target = None
        for r in result:
            if r.name == self.target_name:
                target = r
                break

        if target is not None:
            u1 = round(target.u1)
            u2 = round(target.u2)
            v1 = round(target.v1)
            v2 = round(target.v2)
            u = round((target.u1 + target.u2) / 2)
            v = round((target.v1 + target.v2) / 2)
            depth = np.median(img_depth[v1:v2+1, u1:u2+1])
            if depth != 0:
                z = depth * 1e-3
                fx = msg_info.K[0]
                fy = msg_info.K[4]
                cx = msg_info.K[2]
                cy = msg_info.K[5]
                x = z / fx * (u - cx)
                y = z / fy * (v - cy)
                rospy.loginfo(
                    f'{target.name} ({x:.3f}, {y:.3f}, {z:.3f})')
                
                ts = TransformStamped()
                ts.header = msg_depth.header
                ts.child_frame_id = self.frame_id
                ts.transform.translation.x = x
                ts.transform.translation.y = y
                ts.transform.translation.z = z
                ts.transform.rotation.x = 0
                ts.transform.rotation.y = 0
                ts.transform.rotation.z = 0
                ts.transform.rotation.w = 1
                self.broadcaster.sendTransform(ts)

                buffer = Buffer()
                self.listener = TransformListener(buffer)
                point_source = PointStamped()
                # point_source.header = msg_depth.header
                point_source.header.seq = msg_depth.header.seq
                # point_source.header.stamp = rospy.Time.now()
                point_source.header.frame_id = msg_depth.header.frame_id
                # point_source.header.frame_id = "camera_depth_optical_frame"
                point_source.point.x = x
                point_source.point.y = y
                point_source.point.z = z
                
                point_target = buffer.transform(point_source, "base_link", rospy.Duration(1))
                rospy.loginfo("Target: x = %.2f, y = %.2f, z = %.2f",
                                point_target.point.x,
                                point_target.point.y,
                                point_target.point.z)
                self.pub_point.publish(point_target)
        
        img_depth = img_depth*16
        if target is not None:
            pt1 = (int(target.u1), int(target.v1))
            pt2 = (int(target.u2), int(target.v2))
            cv2.rectangle(img_depth, pt1=pt1, pt2=pt2, color=0xffff)

        cv2.imshow('depth', img_depth)
        cv2.waitKey(1)


if __name__ == "__main__":
    ObjectDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

