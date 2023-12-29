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
# Authors: Mark Zhang

import rospy
import cv2
import mediapipe as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
from pathlib import Path
# from sound_play.libsoundplay import SoundClient
import pygame
import PIL.Image
import multiprocessing

def yf(point1, point2):
    if point1[1]>point2[1]:
        return True
    else:
        return False

def xf(point1, point2):
    if point1[0]>point2[0]:
        return True
    else:
        return False

class MediaPipe_Pose:
    def __init__(self, pkg_path):
        self.mp3_path = pkg_path + "/resource/Camera_Shutter.mp3"
        pygame.init()
        pygame.mixer.music.load(self.mp3_path)
        self.mpDraw = mp.solutions.drawing_utils
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(max_num_hands=2) # 这里是修改可以手的数量
        # 拍照状态
        self.FLAG_TAKE_PHOTO = False
        self.start_count = 0 # 触发
        self.time_count = 0 # 倒计时
        # 每个手指是否伸直的状态
        self.THUMB = False # 拇指
        self.INDEX_FINGER = False # 食指
        self.MIDDLE_FINGER = False # 中指
        self.RING_FINGER = False # 无名指
        self.PINKY_FINGER  = False # 小指
        # ros节点的初始化
        self.ros_init()

    def ros_init(self):
        # 初始化ROS节点
        rospy.init_node('gesture_photograph', anonymous=True)

        # 订阅图像话题 
        image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)

        # self.soundhandle = SoundClient(blocking=True)
        rospy.sleep(1)

        # self.soundhandle.stopAll()

        # 保持节点运行
        rospy.spin()

    def palm_func(self, points):
        new_points = [] # 数据分离， 将21个的数据作为一组
        self.THUMB = False # 拇指
        self.INDEX_FINGER = False # 食指
        self.MIDDLE_FINGER = False # 中指
        self.RING_FINGER = False # 无名指
        self.PINKY_FINGER  = False # 小指
        for i in range(0,len(points),21):
            new_points.append(points[i:i+21])

        for point_i in new_points:
            # 拇指
            self.THUMB = xf(point_i[4],point_i[3])
            # 食指
            self.INDEX_FINGER = yf(point_i[6], point_i[8])
            # 中指
            self.MIDDLE_FINGER = yf(point_i[10], point_i[12])
            # 无名指
            self.RING_FINGER = yf(point_i[14], point_i[16])
            # 小指
            self.PINKY_FINGER = yf(point_i[18], point_i[20])

    def detect_hand(self, image, draw=True):
        """
        返回值说明
        image-处理后的图片
        point_list-检测到手指的21个关键点的坐标
        """
        h, w, c = image.shape
        results = self.hands.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        point_list = []
        if results.multi_hand_landmarks and draw:
            for hand_landmarks, hand_class in zip(results.multi_hand_landmarks, results.multi_handedness):
                self.mpDraw.draw_landmarks(image,
                                           hand_landmarks,
                                           self.mpHands.HAND_CONNECTIONS,
                                           mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                                           mp.solutions.drawing_styles.get_default_hand_connections_style())
                for id, lm in enumerate(hand_landmarks.landmark):
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    point_list.append([cx, cy])
        self.palm_func(point_list)
        return image

    def open_image(self):
        image_pil = PIL.Image.fromarray(cv2.cvtColor(self.out_image, cv2.COLOR_BGR2RGB))
        image_pil.show()

    def image_callback(self, msg):
        try:
            # 将ROS图像数据转换为OpenCV图像
            cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            # 调整图像大小以加快处理速度
            # cv_image = cv2.resize(cv_image, (1280, 720))
            cv_image = cv2.resize(cv_image, (1024, 768))
            self.out_image = self.detect_hand(cv_image)

            if self.THUMB and self.INDEX_FINGER and self.MIDDLE_FINGER and self.RING_FINGER and self.PINKY_FINGER:
                self.start_count += 1
                if self.start_count > 20:
                    # 触发拍照计时
                    self.FLAG_TAKE_PHOTO = True
                    self.start_count=0
            else:
                cv2.putText(self.out_image, "", (20, 50), cv2.FONT_HERSHEY_PLAIN, 5, (0, 255, 0), 5)
                self.start_count=0

            if self.FLAG_TAKE_PHOTO == True:

                self.time_count += 1

                if self.time_count > 60:
                    self.FLAG_TAKE_PHOTO = False
                    self.start_count=0
                    self.time_count=0
                    cv2.putText(self.out_image, "", (20, 50), cv2.FONT_HERSHEY_PLAIN, 5, (0, 255, 0), 5)
                    pygame.mixer.music.play()
                    # self.soundhandle.playWave(self.mp3_path)
                    now = datetime.now()
                    time_str = now.strftime("%Y%m%d_%H%M%S")
                    img_name = f"/home/mustar/.ros/image_{time_str}.png"
                    cv2.imwrite(img_name, cv_image)
                    p = multiprocessing.Process(target=self.open_image)
                    p.start()

                elif self.time_count > 40:
                    cv2.putText(self.out_image, "", (20, 50), cv2.FONT_HERSHEY_PLAIN, 5, (0, 255, 0), 5)
                    cv2.putText(self.out_image, "1", (20, 50), cv2.FONT_HERSHEY_PLAIN, 5, (0, 255, 0), 5)

                elif self.time_count > 20:
                    cv2.putText(self.out_image, "", (20, 50), cv2.FONT_HERSHEY_PLAIN, 5, (0, 255, 0), 5)
                    cv2.putText(self.out_image, "2", (20, 50), cv2.FONT_HERSHEY_PLAIN, 5, (0, 255, 0), 5)
                
                elif self.time_count > 0:
                    cv2.putText(self.out_image, "", (20, 50), cv2.FONT_HERSHEY_PLAIN, 5, (0, 255, 0), 5)
                    cv2.putText(self.out_image, "3", (20, 50), cv2.FONT_HERSHEY_PLAIN, 5, (0, 255, 0), 5)

            # 显示图像
            cv2.imshow("Image window", self.out_image)

            # 按q退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown('Exit')
                cv2.destroyAllWindows()

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    pkg_path = str(Path(__file__).resolve().parents[1])
    poseEstimator = MediaPipe_Pose(pkg_path)

