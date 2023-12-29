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
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

original = 0
start = 0

class NavToPoint:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server.")

        rospy.Subscriber("nav_cmd", String, self.nav_callback)
        nav_pub = rospy.Publisher("nav_feedback", String, queue_size=1)

        quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
        self.origin = Pose(Point(0, 0, 0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

        # A variable to hold the initial pose of the robot to be set by the user in RViz
        #initial_pose = PoseWithCovarianceStamped()
        #rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

        # Get the initial pose from the user
        #rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        #rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        
        # Make sure we have the initial pose
        #while initial_pose.header.stamp == "":
        #    rospy.sleep(1)

        rospy.loginfo("Starting navigation node...")
        rospy.sleep(1)

        locations = dict()

        # Location A
        A_x = 2.33
        A_y = 1.42
        A_theta = 1.57
        quaternion = quaternion_from_euler(0.0, 0.0, A_theta)
        locations['A'] = Pose(Point(A_x, A_y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

        # Location B
        B_x = 3.8
        B_y = 0.64
        B_theta = 0
        quaternion = quaternion_from_euler(0.0, 0.0, B_theta)
        locations['B'] = Pose(Point(B_x, B_y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

        # Location C
        C_x = 3
        C_y = 0.5
        C_theta = 3.14
        quaternion = quaternion_from_euler(0.0, 0.0, C_theta)
        locations['C'] = Pose(Point(C_x, C_y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

        # Location D
        D_x = 3
        D_y = 0
        D_theta = 3.14
        quaternion = quaternion_from_euler(0.0, 0.0, D_theta)
        locations['D'] = Pose(Point(D_x, D_y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

        # Location E
        E_x = 3
        E_y = -0.5
        E_theta = 3.14
        quaternion = quaternion_from_euler(0.0, 0.0, E_theta)
        locations['E'] = Pose(Point(E_x, E_y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

        # Location F
        F_x = 3
        F_y = -1
        F_theta = 3.14
        quaternion = quaternion_from_euler(0.0, 0.0, F_theta)
        locations['F'] = Pose(Point(F_x, F_y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

        # Location G
        G_x = 0
        G_y = 0
        G_theta = 0
        quaternion = quaternion_from_euler(0.0, 0.0, G_theta)
        locations['G'] = Pose(Point(G_x, G_y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

        self.goal = MoveBaseGoal()
        rospy.loginfo("Ready to go.")
        global start

        while not rospy.is_shutdown():
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            # Robot will go to point A
            if start == 1:
                rospy.loginfo("Going to point A")
                rospy.sleep(2)
                self.goal.target_pose.pose = locations['A']
                self.move_base.send_goal(self.goal)
                waiting = self.move_base.wait_for_result(rospy.Duration(300))
                if waiting == 1:
                    rospy.loginfo("Reached point A")
                    nav_pub.publish("Done")
                    rospy.sleep(2)
                    start = 0

            # Robot will go to point B
            elif start == 2:
                rospy.loginfo("Going to point B")
                rospy.sleep(2)
                self.goal.target_pose.pose = locations['B']
                self.move_base.send_goal(self.goal)
                waiting = self.move_base.wait_for_result(rospy.Duration(300))
                if waiting == 1:
                    rospy.loginfo("Reached point B")
                    nav_pub.publish("Done")
                    rospy.sleep(2)
                    start = 0

            # Robot will go to point C
            elif start == 3:
                rospy.loginfo("Going to point C")
                rospy.sleep(2)
                self.goal.target_pose.pose = locations['C']
                self.move_base.send_goal(self.goal)
                waiting = self.move_base.wait_for_result(rospy.Duration(300))
                if waiting == 1:
                    rospy.loginfo("Reached point C")
                    nav_pub.publish("Done")
                    rospy.sleep(2)
                    start = 0

            # Robot will go to point D
            elif start == 4:
                rospy.loginfo("Going to point D")
                rospy.sleep(2)
                self.goal.target_pose.pose = locations['D']
                self.move_base.send_goal(self.goal)
                waiting = self.move_base.wait_for_result(rospy.Duration(300))
                if waiting == 1:
                    rospy.loginfo("Reached point D")
                    nav_pub.publish("Done")
                    rospy.sleep(2)
                    start = 0

            # Robot will go to point E
            elif start == 5:
                rospy.loginfo("Going to point E")
                rospy.sleep(2)
                self.goal.target_pose.pose = locations['E']
                self.move_base.send_goal(self.goal)
                waiting = self.move_base.wait_for_result(rospy.Duration(300))
                if waiting == 1:
                    rospy.loginfo("Reached point E")
                    nav_pub.publish("Done")
                    rospy.sleep(2)
                    start = 0

            # Robot will go to point F
            elif start == 6:
                rospy.loginfo("Going to point F")
                rospy.sleep(2)
                self.goal.target_pose.pose = locations['F']
                self.move_base.send_goal(self.goal)
                waiting = self.move_base.wait_for_result(rospy.Duration(300))
                if waiting == 1:
                    rospy.loginfo("Reached point F")
                    nav_pub.publish("Done")
                    rospy.sleep(2)
                    start = 0

            # Robot will go back to initial position
            elif start == 7:
                rospy.loginfo("Going back home...")
                rospy.sleep(2)
                self.goal.target_pose.pose = self.origin
                self.move_base.send_goal(self.goal)
                waiting = self.move_base.wait_for_result(rospy.Duration(300))
                if waiting == 1:
                    rospy.loginfo("Reached home")
                    nav_pub.publish("Done")
                    rospy.sleep(2)
                    start = 0

            rospy.Rate(5).sleep()

    def nav_callback(self, msg):
        global start
        if msg.data.find("A") > -1:
            start = 1
        elif msg.data.find("B") > -1:
            start = 2
        elif msg.data.find("C") > -1:
            start = 3
        elif msg.data.find("D") > -1:
            start = 4
        elif msg.data.find("E") > -1:
            start = 5
        elif msg.data.find("F") > -1:
            start = 6
        elif msg.data.find("G") > -1:
            start = 7

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
        global original
        if original == 0:
            self.origin = self.initial_pose.pose.pose
            original = 1

    def cleanup(self):
        rospy.loginfo("Shutting down navigation...")
        self.move_base.cancel_goal()

if __name__=="__main__":
    rospy.init_node('navi_point')
    try:
        NavToPoint()
        rospy.spin()
    except:
        pass

