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
from std_msgs.msg import String
from std_msgs.msg import Float64

class Loop:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        rospy.Subscriber('/dance_arm', String, self.callback)

        # publish command message to joints/servos of arm
        self.joint1 = rospy.Publisher('/arm1_joint/command', Float64, queue_size=10)
        self.joint2 = rospy.Publisher('/arm2_joint/command', Float64, queue_size=10)
        self.joint3 = rospy.Publisher('/arm3_joint/command', Float64, queue_size=10)
        self.joint4 = rospy.Publisher('/arm4_joint/command', Float64, queue_size=10)
        self.joint5 = rospy.Publisher('/gripper_joint/command', Float64, queue_size=10)
        self.pos1 = Float64()
        self.pos2 = Float64()
        self.pos3 = Float64()
        self.pos4 = Float64()
        self.pos5 = Float64()

        # Initial gesture of robot arm
        self.pos1 = 0.0
        self.pos2 = -1.86
        self.pos3 = 2.44
        self.pos4 = 1.0
        self.pos5 = 0.0
        self.joint1.publish(self.pos1)
        self.joint2.publish(self.pos2)
        self.joint3.publish(self.pos3)
        self.joint4.publish(self.pos4)
        self.joint5.publish(self.pos5)

    def callback(self, msg):
        print(msg.data)
        if msg.data == "dance arm":
            count = 0
            while (count < 1):

                # gesture 1
                self.pos1 = -1.0
                self.pos2 = -0.215
                self.pos3 = 1.508
                self.pos4 = 0.496
                self.pos5 = 0.0
                self.joint1.publish(self.pos1)
                self.joint2.publish(self.pos2)
                self.joint3.publish(self.pos3)
                self.joint4.publish(self.pos4)
                self.joint5.publish(self.pos5)
                rospy.sleep(2)

                # gesture 2
                self.pos1 = -1.0
                self.pos2 = 0.24
                self.pos3 = 0.639
                self.pos4 = -1.0
                self.pos5 = 0.0
                self.joint1.publish(self.pos1)
                self.joint2.publish(self.pos2)
                self.joint3.publish(self.pos3)
                self.joint4.publish(self.pos4)
                self.joint5.publish(self.pos5)
                rospy.sleep(3)

                # gesture 1
                self.pos1 = -1.0
                self.pos2 = -0.215
                self.pos3 = 1.508
                self.pos4 = 0.496
                self.pos5 = 0.0
                self.joint1.publish(self.pos1)
                self.joint2.publish(self.pos2)
                self.joint3.publish(self.pos3)
                self.joint4.publish(self.pos4)
                self.joint5.publish(self.pos5)
                rospy.sleep(2)

                # initial gesture
                self.pos1 = -0.5
                self.pos2 = -1.86
                self.pos3 = 2.44
                self.pos4 = 1.0
                self.pos5 = 0.0
                self.joint1.publish(self.pos1)
                self.joint2.publish(self.pos2)
                self.joint3.publish(self.pos3)
                self.joint4.publish(self.pos4)
                self.joint5.publish(self.pos5)
                rospy.sleep(3)

                count = count + 1

    def cleanup(self):
        rospy.loginfo("Shutting down dance arm....")

if __name__=="__main__":
    rospy.init_node('dance_arm')
    try:
        Loop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
