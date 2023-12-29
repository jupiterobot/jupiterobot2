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
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion
from tf2_geometry_msgs import PointStamped
from math import pi, radians, sqrt, pow
import PyKDL

class BaseAdjust():
    def __init__(self):
        # Give the node a name
        rospy.init_node('base_adjust', anonymous=False)

        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)

        # How fast will we update the robot's movement
        rate = 20

        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(rate)

        # Set the forward linear speed to 0.15 meters per second
        self.linear_speed = 0.15

        # Set the travel distance in meters
        self.goal_distance = 0

        # Set the rotation speed in radians per second
        self.angular_speed = 0.25

        # Set the angular tolerance in degrees converted to radians
        self.angular_tolerance = radians(1.0)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

        # Set a publisher to mark the end
        self.adj_res = rospy.Publisher("/adj_res", String, queue_size=1)

        # Set the odom frame
        self.odom_frame = '/odom'

        # Set a flag to count msgs
        self.cnt = 0
        self.pos_x = [0 for x in range(0,5)]
        self.pos_y = [0 for x in range(0,5)]

        # Set the required position for arm to grasp
        self.required_pos = Point()
        self.required_pos.x = 0.45
        self.required_pos.y = 0.00

        self.enable = 0
        rospy.Subscriber("/adj_cmd", String, self.enable_callback)
        rospy.Subscriber("/target_point", PointStamped, self.control_callback)

    def enable_callback(self, msg):
        if msg.data.find("Execute") > -1:
            self.enable = 1

    def control_callback(self, msg):
        if self.enable == 1:
            # Read target pos from msgs
            if self.cnt < 5:
                self.pos_x[self.cnt] = msg.point.x
                self.pos_y[self.cnt] = msg.point.y
                self.cnt += 1
            elif self.cnt == 5:
                # Find out if the robot uses /base_link or /base_footprint
                try:
                    self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
                    self.base_frame = '/base_footprint'
                except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                    try:
                        self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                        self.base_frame = '/base_link'
                    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                        rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                        rospy.signal_shutdown("tf Exception")
                self.pos_x = self.qsort(self.pos_x)
                self.pos_y = self.qsort(self.pos_y)
                # Take average to reduce the error
                self.cnt = 0
                self.enable = 0
                print("Start adjust")
                target_pos = Point()
                target_pos.x = self.pos_x[2]
                target_pos.y = self.pos_y[2]
                # Compute the move distance
                dis_x = target_pos.x - self.required_pos.x
                dis_y = target_pos.y - self.required_pos.y
                print(dis_x)
                print(dis_y)
                # Initialize the position variable as a Point type
                position = Point()
                # Get the starting position values before each adjustment
                #move on the x axis
                (position, rotation) = self.get_odom()
                self.go_straight(dis_x, position)
                #move on the y axis
                target_radius = pi/2
                (position, rotation) = self.get_odom()
                self.move_around(target_radius, rotation)
                (position, rotation) = self.get_odom()
                self.go_straight(dis_y, position)
                (position, rotation) = self.get_odom()
                self.move_around(-target_radius, rotation)
                # Stop the robot for good
                self.cmd_vel.publish(Twist())
                rospy.sleep(1)
                response = "Success"
                self.adj_res.publish(response)

    def qsort(self, arr):
        if len(arr) <= 1:
            return arr
        else:
            pivot = arr[0]
            less = [x for x in arr[1:] if x < pivot]
            greater = [x for x in arr[1:] if x >= pivot]
            return self.qsort(less) + [pivot] + self.qsort(greater)

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))

    def go_straight(self,goal_distance,start_pos):
        move_cmd = Twist()

        # Set the movement command to forward motion
        move_cmd.linear.x = self.linear_speed

        if goal_distance < 0:
            move_cmd.linear.x = -self.linear_speed
            goal_distance = -goal_distance

        x_start = start_pos.x
        y_start = start_pos.y

        # Keep track of the distance traveled
        traveled_distance = 0

        # Enter the loop to move along a side
        while traveled_distance < goal_distance and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            self.cmd_vel.publish(move_cmd)

            self.r.sleep()

            # Get the current position
            (position, rotation) = self.get_odom()

            # Compute the Euclidean distance from the start
            traveled_distance = sqrt(pow((position.x - x_start), 2) + 
                            pow((position.y - y_start), 2))

        # Stop the robot 
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)
        print("Go {} meters long end!".format(goal_distance))
        return

    def move_around(self,goal_radius,start_rotation):
        move_cmd = Twist()

        # Set the movement command to a rotation
        move_cmd.angular.z = self.angular_speed
        if goal_radius < 0:
            move_cmd.angular.z = -self.angular_speed
            goal_radius = -goal_radius

        # Track the last angle measured
        last_angle = start_rotation

        # Track how far we have turned
        turn_angle = 0

        while abs(turn_angle + self.angular_tolerance) < abs(goal_radius) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

            # Get the current rotation
            (position, rotation) = self.get_odom()

            # Compute the amount of rotation since the last loop
            delta_angle = self.normalize_angle(rotation - last_angle)

            # Add to the running total
            turn_angle += delta_angle
            last_angle = rotation

        # Stop the robot
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)
        print("Turn {} radius end!".format(goal_radius))

    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping adjusting the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    BaseAdjust()
    rospy.spin()
