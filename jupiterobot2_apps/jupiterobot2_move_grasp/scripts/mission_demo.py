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
#
# Basic Modules
import rospy
from std_msgs.msg import String
from tf2_geometry_msgs import PointStamped
from sound_play.libsoundplay import SoundClient

class MissionDemo(object):

    def __init__(self):
        # Flags
        self._FLAG_NAVI = 1
        self._FLAG_EXECUTE = 0
        self._FLAG_CONFIRM = 0
        self._FLAG_LISTEN = 0
        self._FLAG_BREAK = 0
        self._FLAG_FOUND = 0
        self._FLAG_DONE = 0
        # Soundplay parameter
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        self.speaker = SoundClient(blocking=True)
        rospy.sleep(1)
        rospy.init_node("mission_demo", disable_signals=True)

    def main_loop(self):
        # Initial
        self.pub_arm_cmd = rospy.Publisher("arm_cmd", String, queue_size=1)
        self.pub_base_cmd = rospy.Publisher("base_cmd", String, queue_size=1)
        self.pub_nav_cmd = rospy.Publisher("nav_cmd", String, queue_size=1)
        self.pub_adj_cmd = rospy.Publisher("adj_cmd", String, queue_size=1)
        self.pub_target = rospy.Publisher("grasp_target", String, queue_size=1)
        rospy.Subscriber("nav_feedback", String, self._navi_callback)
        rospy.Subscriber("recognizer/output", String, self._voice_callback)
        rospy.Subscriber("target_point", PointStamped, self._vision_callback)
        rospy.Subscriber("adj_res", String, self._action_callback)
        rospy.sleep(1)
        self.target = ""
        flag_cnt = 0
        # Set arm to rest-state
        arm_cmd = "rest"
        self.pub_arm_cmd.publish(arm_cmd)
        rospy.loginfo("Ready to work.")
        self.speaker.say("Ready to work.", self.voice)
        rospy.sleep(3)
        # Navigating to master-point
        self.pub_nav_cmd.publish("A")
        while (True):
            # Reached the point
            while (True):
                if self._FLAG_NAVI == 0:
                    rospy.loginfo("Sir, what can I do for you?")
                    self.speaker.say("Sir, what can I do for you?", self.voice)
                    rospy.sleep(2)
                    self._FLAG_LISTEN = 1
                    break
            while (True):
                if self._FLAG_CONFIRM == 1:
                    self._FLAG_CONFIRM = 0
                    rospy.loginfo("Sir, do you want me to bring you the {} pot?".format(self.target))
                    self.speaker.say("Sir, do you want me to bring you the {} pot?".format(self.target), self.voice)
                    rospy.sleep(2)
                    self._FLAG_LISTEN = 2
                    break
            while (True):
                if self._FLAG_LISTEN != 2:
                    break
            if self._FLAG_LISTEN == 1:
                rospy.loginfo("Well, So I should ask again.")
                self.speaker.say("Well, So I should ask again.", self.voice)
            elif self._FLAG_LISTEN == 0:
                break
        # Navigating to grasp-point
        while (True):
            if self._FLAG_NAVI == 1:
                self.pub_target.publish(self.target)
                rospy.loginfo("OK, I'll go and get it for you.")
                self.speaker.say("OK, I'll go and get it for you.", self.voice)
                rospy.sleep(2)
                self.pub_nav_cmd.publish("B")
                break
        # Reached the point2 then shut naving
        while (True):
            if self._FLAG_NAVI == 0:
                rospy.loginfo("Got the point.")
                self.speaker.say("Got the point.", self.voice)
                arm_cmd = "down"
                self.pub_arm_cmd.publish(arm_cmd)
                rospy.sleep(2)
                self._FLAG_EXECUTE = 1
                break
        while (True):
            flag_cnt += 1
            rospy.sleep(1)
            if self._FLAG_FOUND > 3:
                break
            elif flag_cnt == 3:
                # Detect the target object
                base_cmd = "whirl"
                self.pub_base_cmd.publish(base_cmd)
                rospy.loginfo("Seeking the pot.")
                self.speaker.say("Seeking the pot.", self.voice)
                rospy.sleep(2)
        # Found the pot
        if flag_cnt > 3:
            rospy.sleep(1)
            base_cmd = "stop"
            self.pub_base_cmd.publish(base_cmd)
            rospy.sleep(2)
        # Execute the action
        rospy.loginfo("Found it.")
        self.speaker.say("Found it.", self.voice)
        adj_cmd = "Execute"
        self.pub_adj_cmd.publish(adj_cmd)
        rospy.sleep(2)
        while (True):
            if self._FLAG_DONE == 1:
                break
        rospy.loginfo("Try to grasp the pot.")
        self.speaker.say("Try to grasp the pot.", self.voice)
        rospy.sleep(2)
        arm_cmd = "ready"
        self.pub_arm_cmd.publish(arm_cmd)
        rospy.sleep(3)
        arm_cmd = "catch"
        self.pub_arm_cmd.publish(arm_cmd)
        rospy.sleep(3)
        arm_cmd = "open"
        self.pub_arm_cmd.publish(arm_cmd)
        rospy.sleep(2)
        # Approach to the target object
        base_cmd = "forward"
        self.pub_base_cmd.publish(base_cmd)
        rospy.sleep(2)
        base_cmd = "stop"
        self.pub_base_cmd.publish(base_cmd)
        rospy.sleep(1)
        # Set arm to catch-state to carry
        arm_cmd = "close"
        self.pub_arm_cmd.publish(arm_cmd)
        rospy.sleep(2)
        arm_cmd = "rest"
        self.pub_arm_cmd.publish(arm_cmd)
        rospy.sleep(2)
        self._FLAG_NAVI = 1
        self.pub_nav_cmd.publish("A")
        rospy.sleep(2)
        while (True):
            if self._FLAG_NAVI == 0:
                break
        arm_cmd = "ready"
        self.pub_arm_cmd.publish(arm_cmd)
        rospy.sleep(2)
        rospy.loginfo("Well, sir! Here you are.")
        self.speaker.say("Well, sir! Here you are.", self.voice)
        rospy.sleep(3)
        # Release the pot
        arm_cmd = "open"
        self.pub_arm_cmd.publish(arm_cmd)
        rospy.sleep(2)
        arm_cmd = "rest"
        self.pub_arm_cmd.publish(arm_cmd)
        rospy.sleep(2)
        rospy.loginfo("Alright! I'm done here.")
        self.speaker.say("Alright! I'm done here.", self.voice)
        rospy.sleep(3)

    def _navi_callback(self, msg):
        if msg.data.find("Done") > -1:
            self._FLAG_NAVI = 0

    def _action_callback(self, msg):
        if msg.data.find("Success") > -1:
            self._FLAG_DONE = 1

    def _vision_callback(self, msg):
        if self._FLAG_EXECUTE == 1:
            self._FLAG_FOUND += 1

    def _voice_callback(self, msg):
        if self._FLAG_LISTEN == 1:
            self._FLAG_LISTEN = 0
            if msg.data.find("red-flask") > -1:
                self.target = "red"
                self._FLAG_CONFIRM = 1
            elif msg.data.find("green-flask") > -1:
                self.target = "green"
                self._FLAG_CONFIRM = 1
            elif msg.data.find("blue-flask") > -1:
                self.target = "blue"
                self._FLAG_CONFIRM = 1
            else:
                self._FLAG_LISTEN = 1
        elif self._FLAG_LISTEN == 2:
            if msg.data.find("yes") > -1 and msg.data.find("i-confirm") > -1:
                self._FLAG_NAVI = 1
                self._FLAG_LISTEN = 0
            elif msg.data.find("not-right") > -1 :
                self._FLAG_LISTEN = 1

if __name__ == "__main__":
    controller = MissionDemo()
    controller.main_loop()

