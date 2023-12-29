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
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sound_play.libsoundplay import SoundClient
from math import radians
from jupiterobot2_msgs.srv import Follow


class PartyBot:
    def __init__(self, script_path):
        rospy.init_node('partybot')

        rospy.on_shutdown(self.cleanup)
        
        # Set the default TTS voice to use
        #self.voice = rospy.get_param("~voice", "voice_don_diphone")
        
        # Set the wave file path if used
        self.wavepath = rospy.get_param("~wavepath", script_path + "/../sounds")
        
        # Create the sound client object
        #self.soundhandle = SoundClient()
        self.soundhandle = SoundClient(blocking=True)
        
        # Wait a moment to let the client connect to the sound_play server
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        
        # Announce that we are ready for input
        self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        #rospy.sleep(1)
        #self.soundhandle.say("Ready")
        
        rospy.loginfo("Ready, waiting for commands...")
        self.soundhandle.say('Hello, I am PartyBot. What can I do for you?')
        #rospy.sleep(2)

        # Subscribe to the recognizer output and set the callback function
        rospy.Subscriber('recognizer/output', String, self.talkback)

        self.dance_arm = rospy.Publisher("dance_arm", String, queue_size=10)

        self.take_photo = rospy.Publisher("take_photo", String, queue_size=10)

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def talkback(self, msg):
        # Print the recognized words on the screen
        rospy.loginfo(msg.data)
        if msg.data.find('introduce-yourself')>-1:
            self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            #rospy.sleep(1)
            self.soundhandle.say("I heard you want me to introduce myself. I am PartyBot. I am a party robot to serve you and have fun.")
            #rospy.sleep(10)
        elif msg.data.find('how-old-are-you')>-1:
            self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            #rospy.sleep(1)
            self.soundhandle.say("I heard you ask about my age. I am five years old.")
            #rospy.sleep(5)
        elif msg.data.find('are-you-from')>-1:
            self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            #rospy.sleep(1)
            self.soundhandle.say("I heard you ask about my hometown. I am from China.")
            #rospy.sleep(5)
        elif msg.data.find('can-you-do')>-1:
            self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            #rospy.sleep(1)
            self.soundhandle.say("I heard you ask me what can I do? I am a home robot. I am good at singing and dancing. I tell funny jokes and I take great photos of people")
            #rospy.sleep(5)
        elif msg.data.find('tell-a-funny-joke')>-1:
            self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            #rospy.sleep(1)
            self.soundhandle.say("You want to hear a joke? What is orange and sounds like a parrot? Erm, It is a carrot. Ha ha ha")
            #rospy.sleep(8)
        elif msg.data.find('take-a-photo')>-1:
            self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            self.soundhandle.say("You want to take a photo? Ok, get ready. One, two, three, say cheese")
            self.take_photo.publish('take photo')
        elif msg.data.find('sing-and-dance')>-1:
            self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
            #rospy.sleep(1)
            self.soundhandle.say("You want me to sing and dance? sure. let me show you")
            #rospy.sleep(5)
            self.dance_arm.publish('dance arm')
            self.soundhandle.playWave(self.wavepath + "/swtheme.wav", blocking=False)
            #rospy.sleep(1)
            # Dancing
            # create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.
            # let's go forward at 0.2 m/s
            move_cmd = Twist()
            move_cmd.linear.x = 0.2
            # by default angular.z is 0 so setting this isn't required
            # let's turn at 45 deg/s
            turn_cmd = Twist()
            turn_cmd.linear.x = 0
            turn_cmd.angular.z = radians(90); #45 deg/s in radians/s
            turn_cmd2 = Twist()
            turn_cmd2.linear.x = 0
            turn_cmd2.angular.z = radians(-90); #45 deg/s in radians/s
            # turn 90 degrees
            rospy.loginfo("Turning")
            for x in range(0,5):
                self.cmd_vel.publish(turn_cmd)
                rospy.sleep(1)
                self.cmd_vel.publish(turn_cmd2)
                rospy.sleep(1)
            rospy.sleep(3)
        elif msg.data.find('follow-me')>-1:
           self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
           self.soundhandle.say("OK. I will start following you.")
           self.control_follow(1)
        elif msg.data.find('stop-follow')>-1:
           self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
           self.soundhandle.say("OK. I will stop following you.")
           self.control_follow(0)
        # else: self.soundhandle.say("Sorry, I cannot hear you clearly. Please say again.")
        else: rospy.sleep(3)

    def control_follow(self, msg):
        rospy.wait_for_service('follower_switch')
        change_state = rospy.ServiceProxy('follower_switch', Follow)
        response = change_state(msg)
        return response.result

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down partybot node...")

if __name__=="__main__":
    try:
        PartyBot(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Partybot node terminated.")
