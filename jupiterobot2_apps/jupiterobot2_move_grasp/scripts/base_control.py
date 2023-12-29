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
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def main():
    # 初始化 Twist 控制消息
    global twist, enable, pub
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    enable = 0

    # 初始化 ros 节点
    rospy.init_node("base_control", anonymous=False)

    # 初始化控制命令订阅者
    rospy.Subscriber("base_cmd", String, cmd_vel_callback)

    # 初始化控制命令发布者
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    # 初始化 ros主循环
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if enable == 1:
            pub.publish(twist)
        rate.sleep()

def cmd_vel_callback(msg):
    global enable
    if msg.data.find("whirl") > -1:
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0.1
        enable = 1
    elif msg.data.find("forward") > -1:
        twist.linear.x = 0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        enable = 1
    elif msg.data.find("back") > -1:
        twist.linear.x = -0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        enable = 1
    else:
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        enable = 0
        pub.publish(twist)

if __name__ == "__main__":
    main()
