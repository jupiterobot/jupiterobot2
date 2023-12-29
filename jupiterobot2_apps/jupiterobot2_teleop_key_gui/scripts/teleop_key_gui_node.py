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

import sys
import threading
from pathlib import Path
# 添加路径 才能找到其他py文件包
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from dashboard import Dashboard
from Ui_kb import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow, QShortcut
from PyQt5.QtGui import QKeySequence
from PyQt5.QtCore import Qt
import rospy
from geometry_msgs.msg import Twist

MAX_LIN_VEL = 0.26
MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

# 发布频率 s
PUB_RATE = 0.01

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
    return vel

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

class MyWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MyWindow, self).__init__(parent)
        self.init_ui()
        self.pushbutton_manager()
        self.init_ros()
        # 创建一个定时器
        timer = threading.Timer(PUB_RATE, self.twist_pub)
        self.color_model = False
        timer.start()

    def init_ros(self):
        # 节点名称
        rospy.init_node('jupiter2_teleop_key')
        # 发布的话题
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        # 状态
        self.target_linear_vel   = 0.0
        self.target_angular_vel  = 0.0
        self.control_linear_vel  = 0.0
        self.control_angular_vel = 0.0

    def init_ui(self):
        self.setupUi(self)
        # 窗口置顶
        self.setWindowFlags(Qt.WindowStaysOnTopHint)
        self.dash1 = Dashboard(self.frame_2)
        self.dash1.setGeometry(-160, 0, 200, 200)
        self.dash1.setValue(5)

        self.dash2 = Dashboard(self.frame_3)
        self.dash2.setGeometry(-160, 0, 200, 200)
        self.dash2.setValue(5)

    def pushbutton_manager(self):
        self.shortcut_w = QShortcut(QKeySequence('W'), self)
        self.shortcut_w.activated.connect(self.pushButton.click) 
        self.pushButton.clicked.connect(self.func_button1)

        self.pushButton_2.clicked.connect(self.func_button2)
        self.shortcut_x = QShortcut(QKeySequence('X'), self)
        self.shortcut_x.activated.connect(self.pushButton_2.click) 

        self.pushButton_3.clicked.connect(self.func_button3)
        self.shortcut_a = QShortcut(QKeySequence('A'), self)
        self.shortcut_a.activated.connect(self.pushButton_3.click) 

        self.pushButton_4.clicked.connect(self.func_button4)
        self.shortcut_d = QShortcut(QKeySequence('D'), self)
        self.shortcut_d.activated.connect(self.pushButton_4.click) 

        self.pushButton_5.clicked.connect(self.func_button5)
        self.shortcut_s = QShortcut(QKeySequence('S'), self)
        self.shortcut_s.activated.connect(self.pushButton_5.click) 

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_W:
            if not event.isAutoRepeat():
                print("按下按钮")

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_W:
            if not event.isAutoRepeat():
                self.pushButton.setStyleSheet("background-color: rgb(170, 170, 255);")
                self.color_model = True
        
        if event.key() == Qt.Key_X:
            if not event.isAutoRepeat():
                self.pushButton_2.setStyleSheet("background-color: rgb(170, 170, 255);")
                self.color_model = True
        
        if event.key() == Qt.Key_A:
            if not event.isAutoRepeat():
                self.pushButton_3.setStyleSheet("background-color: rgb(170, 170, 255);")
                self.color_model = True
        
        if event.key() == Qt.Key_D:
            if not event.isAutoRepeat():
                self.pushButton_4.setStyleSheet("background-color: rgb(170, 170, 255);")
                self.color_model = True
        
        if event.key() == Qt.Key_S:
            if not event.isAutoRepeat():
                self.pushButton_5.setStyleSheet("background-color: rgb(170, 170, 255);")
                self.color_model = True

    def func_button1(self):
        if self.color_model:
            self.pushButton.setStyleSheet("QPushButton{background-color: blue;border-radius: 10px;  border: 2px groove gray;}QPushButton:pressed {background-color: rgb(170, 170, 255);}")
        else:
            self.pushButton.setStyleSheet("QPushButton{background-color: rgb(170, 170, 255);border-radius: 10px;  border: 2px groove gray;}QPushButton:pressed {background-color: blue;}")
        self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel + LIN_VEL_STEP_SIZE)
        self.color_model = False

    def func_button2(self):
        if self.color_model:
            self.pushButton_2.setStyleSheet("QPushButton{background-color: blue;border-radius: 10px;  border: 2px groove gray;}QPushButton:pressed {background-color: rgb(170, 170, 255);}")
        else:
            self.pushButton_2.setStyleSheet("QPushButton{background-color: rgb(170, 170, 255);border-radius: 10px;  border: 2px groove gray;}QPushButton:pressed {background-color: blue;}")
        self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel - LIN_VEL_STEP_SIZE)
        self.color_model = False

    def func_button3(self):
        if self.color_model:
            self.pushButton_3.setStyleSheet("QPushButton{background-color: blue;border-radius: 10px;  border: 2px groove gray;}QPushButton:pressed {background-color: rgb(170, 170, 255);}")
        else:
            self.pushButton_3.setStyleSheet("QPushButton{background-color: rgb(170, 170, 255);border-radius: 10px;  border: 2px groove gray;}QPushButton:pressed {background-color: blue;}")
        self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel + ANG_VEL_STEP_SIZE)
        self.color_model = False

    def func_button4(self):
        if self.color_model:
            self.pushButton_4.setStyleSheet("QPushButton{background-color: blue;border-radius: 10px;  border: 2px groove gray;}QPushButton:pressed {background-color: rgb(170, 170, 255);}")
        else:
            self.pushButton_4.setStyleSheet("QPushButton{background-color: rgb(170, 170, 255);border-radius: 10px;  border: 2px groove gray;}QPushButton:pressed {background-color: blue;}")
        self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel - ANG_VEL_STEP_SIZE)
        self.color_model = False

    def func_button5(self):
        if self.color_model:
            self.pushButton_5.setStyleSheet("QPushButton{background-color: blue;border-radius: 10px;  border: 2px groove gray;}QPushButton:pressed {background-color: rgb(170, 170, 255);}")
        else:
            self.pushButton_5.setStyleSheet("QPushButton{background-color: rgb(170, 170, 255);border-radius: 10px;  border: 2px groove gray;}QPushButton:pressed {background-color: blue;}")
        self.target_linear_vel   = 0.0
        self.control_linear_vel  = 0.0
        self.target_angular_vel  = 0.0
        self.control_angular_vel = 0.0
        self.color_model = False

    def twist_pub(self):
        self.control_linear_vel = makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
        self.twist.linear.x = self.control_linear_vel; self.twist.linear.y = 0.0; self.twist.linear.z = 0.0

        self.control_angular_vel = makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        self.twist.angular.x = 0.0; self.twist.angular.y = 0.0; self.twist.angular.z = self.control_angular_vel
        # 绘制表盘的数据
        self.dash1.setValue(abs(int((self.control_linear_vel/MAX_LIN_VEL)*100)))
        self.dash2.setValue(abs(int((self.control_angular_vel/MAX_ANG_VEL)*100)))
        self.pub.publish(self.twist)
        # 显示当前速度 
        self.label_3.setText("当前线速度 {:.2f}m/s".format(self.control_linear_vel))
        self.label_4.setText("当前角速度 {:.2f}rad/s".format(self.control_angular_vel))
        timer = threading.Timer(PUB_RATE, self.twist_pub)
        timer.start()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dw = MyWindow()
    dw.show()
    sys.exit(app.exec_())
