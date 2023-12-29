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
#
# Authors: Dante Huang

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, Pose
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf2_geometry_msgs import PointStamped

class TargetTF():

    def __init__(self):

        rospy.init_node('target_tf')

        self.frame_id = 'target'
        self.flag = 0
        self.x = 0
        self.y = 0

        self.bridge = CvBridge()

        rospy.Subscriber('/target_pose', Pose, self.pose_callback)
        self.pub_point = rospy.Publisher("target_point", PointStamped, queue_size=1)
        self.sub_info = Subscriber('/camera/depth/camera_info', CameraInfo)
        self.sub_color = Subscriber('/camera/color/image_raw', Image)
        self.sub_depth = Subscriber('/camera/depth/image_raw', Image)
        self.ts = ApproximateTimeSynchronizer([self.sub_info, self.sub_color, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.images_callback)
        self.broadcaster = TransformBroadcaster()

    def pose_callback(self, msg):
        self.x = int(msg.position.x)
        self.y = int(msg.position.y)
        #print(self.x, self.y)
        self.flag = 1

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

        if self.flag == 1:
            self.flag = 0
            depth = img_depth[self.y, self.x]
            if depth != 0:
                z = depth * 1e-3
                fx = msg_info.K[0]
                fy = msg_info.K[4]
                cx = msg_info.K[2]
                cy = msg_info.K[5]
                x = z / fx * (self.x - cx)
                y = z / fy * (self.y - cy)
                #rospy.loginfo("target: ({:.3f}, {:.3f}, {:.3f})".format(x, y, z))
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
                TransformListener(buffer)
                point_source = PointStamped()
                # point_source.header = msg_depth.header
                point_source.header.seq = msg_depth.header.seq
                # point_source.header.stamp = rospy.Time.now()
                point_source.header.frame_id = msg_depth.header.frame_id
                # point_source.header.frame_id = "camera_depth_optical_frame"
                point_source.point.x = x
                point_source.point.y = y
                point_source.point.z = z
                point_target = buffer.transform(point_source, "base_footprint", rospy.Duration(1))
                rospy.loginfo("Target: x = %.2f, y = %.2f, z = %.2f", 
                              point_target.point.x, point_target.point.y, point_target.point.z)
                self.pub_point.publish(point_target)


if __name__ == "__main__":
    TargetTF()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

