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
from std_msgs.msg import String
import sys
from pathlib import Path

# 添加路径 才能找到其他py文件包
scripts_path = str(Path(__file__).resolve().parents[1]/'scripts')
print(scripts_path)
sys.path.insert(0, scripts_path)
import time
from SparkApi import spark_class
import SparkApi
from tts_ws_python3_demo import tts_clss


#以下密钥信息从控制台获取
appid = "d6b3a8d9"     #填写控制台中获取的 APPID 信息
api_secret = "MmJiOWZlNzMzOWZhZmIwZjhmZjEwMDI4"   #填写控制台中获取的 APISecret 信息
api_key ="129ee0861c791e78c7ac3f8605019327"    #填写控制台中获取的 APIKey 信息

#用于配置大模型版本，默认“general/generalv2”
# domain = "general"   # v1.5版本
domain = "generalv2"    # v2.0版本
#云端环境的服务地址
# Spark_url = "ws://spark-api.xf-yun.com/v1.1/chat"  # v1.5环境的地址
Spark_url = "ws://spark-api.xf-yun.com/v2.1/chat"  # v2.0环境的地址

text =[]


def getText(role,content):
    jsoncon = {}
    jsoncon["role"] = role
    jsoncon["content"] = content
    text.append(jsoncon)
    return text

def getlength(text):
    length = 0
    for content in text:
        temp = content["content"]
        leng = len(temp)
        length += leng
    return length

def checklen(text):
    while (getlength(text) > 8000):
        del text[0]
    return text

class RosSpart():
    def __init__(self):
        rospy.init_node("spark_node")
        # 是否发布语音识别
        self.FLAG_PUB = True
        self.words_sub_fun()
        self.wakeup_pub_fun()
    
    def words_sub_fun(self):
        # 订阅语音识别的结果
        sub = rospy.Subscriber("/voiceWords",String,self.spark_call,queue_size=10)


    def wakeup_pub_fun(self):
        # 唤醒的发布
        self.wakeup_pub = rospy.Publisher("/voiceWakeup",String,queue_size=10)
        wakeup_msg = String()  #创建 msg 对象
        wakeup_msg.data = "1"
        rate = rospy.Rate(1)
        time.sleep(1)
        self.wakeup_pub.publish(wakeup_msg)
        while not rospy.is_shutdown():
            if self.FLAG_PUB:
                self.wakeup_pub.publish(wakeup_msg)
                self.FLAG_PUB = False
            rate.sleep()

    def spark_call(self, msg):
        # 接收到词后的对话处理
        SparkApi.answer =""
        question = question = checklen(getText("user",msg.data))
        spark_class(appid,api_key,api_secret,Spark_url,domain,question)
        tts_clss(SparkApi.answer)
        self.FLAG_PUB = True


if __name__ == '__main__':
    RosSpart()


