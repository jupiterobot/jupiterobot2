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
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import mediapipe as mp
import time
# 导入自定义的数据类型和处理函数
from jupiterobot2_msgs.msg import Mediapipe_Pose, Mediapipe_Hand, Mediapipe_Face, Mediapipe_FaceBbox, Mediapipe_FaceBboxes

def pose_topic_data(self, output_point):
    '''
    出来数据发布到话题里
    '''
    if len(output_point)==33:
        # 设置时间戳
        self.pose_joints.times.stamp = rospy.Time.now()
        # 0 nose：鼻子
        self.pose_joints.nose.x = output_point[0][0]
        self.pose_joints.nose.y = output_point[0][1]
        # 1 left_eye_inner：左眼内角
        self.pose_joints.left_eye_inner.x = output_point[1][0]
        self.pose_joints.left_eye_inner.y = output_point[1][1]
        # 2 left_eye：左眼中心
        self.pose_joints.left_eye.x = output_point[2][0]
        self.pose_joints.left_eye.y = output_point[2][1]
        # 3 left_eye_outer：左眼外角
        self.pose_joints.left_eye_outer.x = output_point[3][0]
        self.pose_joints.left_eye_outer.y = output_point[3][1]
        # 4 right_eye_inner：右眼内角
        self.pose_joints.right_eye_inner.x = output_point[4][0]
        self.pose_joints.right_eye_inner.y = output_point[4][1]
        # 5 right_eye：右眼中心
        self.pose_joints.right_eye.x = output_point[5][0]
        self.pose_joints.right_eye.y = output_point[5][1]
        # 6 right_eye_outer：右眼外角
        self.pose_joints.left_eye_inner.x = output_point[6][0]
        self.pose_joints.left_eye_inner.y = output_point[6][1]
        # 7 left_ear：左耳朵
        self.pose_joints.left_eye.x = output_point[7][0]
        self.pose_joints.left_eye.y = output_point[7][1]
        # 8 right_ear：右耳朵
        self.pose_joints.right_ear.x = output_point[8][0]
        self.pose_joints.right_ear.y = output_point[8][1]
        # 9 mouth_left 左嘴
        self.pose_joints.mouth_left.x = output_point[9][0]
        self.pose_joints.mouth_left.y = output_point[9][1]
        # 10 mouth_right 右嘴
        self.pose_joints.mouth_right.x = output_point[10][0]
        self.pose_joints.mouth_right.y = output_point[10][1]
        # 11 left_shoulder 左肩
        self.pose_joints.left_shoulder.x = output_point[11][0]
        self.pose_joints.left_shoulder.y = output_point[11][1]
        # 12 right_shoulder 右肩
        self.pose_joints.right_shoulder.x = output_point[12][0]
        self.pose_joints.right_shoulder.y = output_point[12][1]
        # 13 left_elbow 左肘
        self.pose_joints.left_elbow.x = output_point[13][0]
        self.pose_joints.left_elbow.y = output_point[13][1]
        # 14 right_elbow 右肘
        self.pose_joints.right_elbow.x = output_point[14][0]
        self.pose_joints.right_elbow.y = output_point[14][1]
        # 15 left_wrist 左手腕
        self.pose_joints.left_wrist.x = output_point[15][0]
        self.pose_joints.left_wrist.y = output_point[15][1]
        # 16 right_wrist 右手腕
        self.pose_joints.right_wrist.x = output_point[16][0]
        self.pose_joints.right_wrist.y = output_point[16][1]
        # 17 left_pinky 左小指
        self.pose_joints.left_pinky.x = output_point[17][0]
        self.pose_joints.left_pinky.y = output_point[17][1]
        # 18 right_pinky 右小指
        self.pose_joints.right_pinky.x = output_point[18][0]
        self.pose_joints.right_pinky.y = output_point[18][1]
        # 19 left_index 左食指
        self.pose_joints.left_index.x = output_point[19][0]
        self.pose_joints.left_index.y = output_point[19][1]
        # 20 right_index 右食指
        self.pose_joints.right_index.x = output_point[20][0]
        self.pose_joints.right_index.y = output_point[20][1]
        # 21 left_thumb 左拇指
        self.pose_joints.left_thumb.x = output_point[21][0]
        self.pose_joints.left_thumb.y = output_point[21][1]
        # 22 right_thumb 右拇指
        self.pose_joints.right_thumb.x = output_point[22][0]
        self.pose_joints.right_thumb.y = output_point[22][1]
        # 23 left_hip 左臀
        self.pose_joints.left_hip.x = output_point[23][0]
        self.pose_joints.left_hip.y = output_point[23][1]
        # 24 right_hip 右臀
        self.pose_joints.right_hip.x = output_point[24][0]
        self.pose_joints.right_hip.y = output_point[24][1]
        # 25 left_knee 左膝
        self.pose_joints.left_knee.x = output_point[25][0]
        self.pose_joints.left_knee.y = output_point[25][1]
        # 26 right_knee 右膝
        self.pose_joints.right_knee.x = output_point[26][0]
        self.pose_joints.right_knee.y = output_point[26][1]
        # 27 left_ankle 左脚踝
        self.pose_joints.left_ankle.x = output_point[27][0]
        self.pose_joints.left_ankle.y = output_point[27][1]
        # 28 right_ankle 右脚踝
        self.pose_joints.right_ankle.x = output_point[28][0]
        self.pose_joints.right_ankle.y = output_point[28][1]
        # 29 left_heel 左脚底
        self.pose_joints.left_heel.x = output_point[29][0]
        self.pose_joints.left_heel.y = output_point[29][1]
        # 30 right_heel 右脚底
        self.pose_joints.right_heel.x = output_point[30][0]
        self.pose_joints.right_heel.y = output_point[30][1]
        # 31 left_foot_index 左脚指尖
        self.pose_joints.left_foot_index.x = output_point[31][0]
        self.pose_joints.left_foot_index.y = output_point[31][1]
        # 32 right_foot_index 右脚指尖
        self.pose_joints.right_foot_index.x = output_point[32][0]
        self.pose_joints.right_foot_index.y = output_point[32][1]

        # 发布话题
        self.pose_publisher.publish(self.pose_joints)

def hand_topic_data(self, output_point):
    if len(output_point)==21:
        # 设置时间戳
        self.hand_joints.times.stamp = rospy.Time.now()
        # 0 WRIST 手腕中心点
        self.hand_joints.WRIST.x = output_point[0][0]
        self.hand_joints.WRIST.y = output_point[0][1]
        # 1 THUMB_CMC 拇指中心点
        self.hand_joints.THUMB_CMC.x = output_point[1][0]
        self.hand_joints.THUMB_CMC.y = output_point[1][1]
        # 2 THUMB_MCP 拇指第一关节点
        self.hand_joints.THUMB_MCP.x = output_point[2][0]
        self.hand_joints.THUMB_MCP.y = output_point[2][1]
        # 3 THUMB_IP 拇指第二关节点
        self.hand_joints.THUMB_IP.x = output_point[3][0]
        self.hand_joints.THUMB_IP.y = output_point[3][1]
        # 4 THUMB_TIP 拇指第三关节点
        self.hand_joints.THUMB_TIP.x = output_point[4][0]
        self.hand_joints.THUMB_TIP.y = output_point[4][1]
        # 5 INDEX_FINGER_MCP 食指中心点
        self.hand_joints.INDEX_FINGER_MCP.x = output_point[5][0]
        self.hand_joints.INDEX_FINGER_MCP.y = output_point[5][1]
        # 6 INDEX_FINGER_PIP 食指第一关节点
        self.hand_joints.INDEX_FINGER_PIP.x = output_point[6][0]
        self.hand_joints.INDEX_FINGER_PIP.y = output_point[6][1]
        # 7 INDEX_FINGER_DIP 食指第二关节点
        self.hand_joints.INDEX_FINGER_DIP.x = output_point[7][0]
        self.hand_joints.INDEX_FINGER_DIP.y = output_point[7][1]
        # 8 INDEX_FINGER_TIP 食指第三关节点
        self.hand_joints.INDEX_FINGER_TIP.x = output_point[8][0]
        self.hand_joints.INDEX_FINGER_TIP.y = output_point[8][1]
        # 9 MIDDLE_FINGER_MCP 中指中心点
        self.hand_joints.MIDDLE_FINGER_MCP.x = output_point[9][0]
        self.hand_joints.MIDDLE_FINGER_MCP.y = output_point[9][1]
        # 10 MIDDLE_FINGER_PIP 中指第一关节点
        self.hand_joints.MIDDLE_FINGER_PIP.x = output_point[10][0]
        self.hand_joints.MIDDLE_FINGER_PIP.y = output_point[10][1]
        # 11 MIDDLE_FINGER_DIP 中指第二关节点
        self.hand_joints.MIDDLE_FINGER_DIP.x = output_point[11][0]
        self.hand_joints.MIDDLE_FINGER_DIP.y = output_point[11][1]
        # 12 MIDDLE_FINGER_TIP 中指第三关节点
        self.hand_joints.MIDDLE_FINGER_TIP.x = output_point[12][0]
        self.hand_joints.MIDDLE_FINGER_TIP.y = output_point[12][1]
        # 13 RING_FINGER_MCP 无名指中心点
        self.hand_joints.RING_FINGER_MCP.x = output_point[13][0]
        self.hand_joints.RING_FINGER_MCP.y = output_point[13][1]
        # 14 RING_FINGER_PIP 无名指第一关节点
        self.hand_joints.RING_FINGER_PIP.x = output_point[14][0]
        self.hand_joints.RING_FINGER_PIP.y = output_point[14][1]
        # 15 RING_FINGER_DIP 无名指第二关节点
        self.hand_joints.RING_FINGER_DIP.x = output_point[15][0]
        self.hand_joints.RING_FINGER_DIP.y = output_point[15][1]
        # 16 RING_FINGER_TIP 无名指第三关节点
        self.hand_joints.RING_FINGER_TIP.x = output_point[16][0]
        self.hand_joints.RING_FINGER_TIP.y = output_point[16][1]
        # 17 PINCKY_MCP 小指中心点
        self.hand_joints.PINCKY_MCP.x = output_point[17][0]
        self.hand_joints.PINCKY_MCP.y = output_point[17][1]
        # 18 PINCKY_PIP 小指第一关节点
        self.hand_joints.PINCKY_PIP.x = output_point[18][0]
        self.hand_joints.PINCKY_PIP.y = output_point[18][1]
        # 19 PINCKY_DIP 小指第二关节点
        self.hand_joints.PINCKY_DIP.x = output_point[19][0]
        self.hand_joints.PINCKY_DIP.y = output_point[19][1]
        # 20 PINCKY_TIP 小指第三关节点
        self.hand_joints.PINCKY_TIP.x = output_point[20][0]
        self.hand_joints.PINCKY_TIP.y = output_point[20][1]

        # 发布话题
        self.hand_publisher.publish(self.hand_joints)

def face_topic_data(self, output_point):
    if len(output_point)==468:
        for i in range(468):
            self.face_joints.face_point[i].x = output_point[i][0]
            self.face_joints.face_point[i].y = output_point[i][1]
        # 发布话题
        self.face_publisher.publish(self.face_joints)

class MediaPipe_Pose:
    def __init__(self, select_detect=0):
        self.select_detect = select_detect
        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.mpHands = mp.solutions.hands
        self.mpFaceMesh = mp.solutions.face_mesh
        self.mpFace = mp.solutions.face_detection
        self.pose = self.mpPose.Pose()
        self.hands = self.mpHands.Hands(max_num_hands=1)
        self.faceMesh = self.mpFaceMesh.FaceMesh()
        self.drawing_spec = self.mpDraw.DrawingSpec(thickness=1, circle_radius=1, color=(222, 222, 66))
        self.face = self.mpFace.FaceDetection()
        self.pTime = 0
        # ros节点的初始化
        self.ros_init()

    def ros_init(self):
        # 初始化ROS节点
        rospy.init_node('mediapipe_pose', anonymous=True)

        # 订阅图像话题 
        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.select_detect = rospy.get_param('~detect_mode', 0)
        rospy.Subscriber(image_topic, Image, self.image_callback)

        # 发布姿态关键点
        self.pose_publisher = rospy.Publisher("mediapipe/pose", Mediapipe_Pose, queue_size=10)
        self.pose_joints = Mediapipe_Pose()

        # 人手关键点
        self.hand_publisher = rospy.Publisher("mediapipe/hand", Mediapipe_Hand, queue_size=10)
        self.hand_joints = Mediapipe_Hand()

        # 人脸关键点
        self.face_publisher = rospy.Publisher("mediapipe/face", Mediapipe_Face, queue_size=10)
        self.face_joints = Mediapipe_Face()

        # 人脸位置检测
        self.face_pos_publisher = rospy.Publisher("mediapipe/face_pos", Mediapipe_FaceBboxes, queue_size=10)

        # 保持节点运行q
        rospy.spin()

    def detect_pose(self, image, draw=True):
        '''
        返回值说明：
        image-识别处理后的图片，每次只能识别一个人，多个人时，会不知道是哪个人
        point_list-返回的32个点的坐标
        '''
        results = self.pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        h, w, c = image.shape
        point_list = []
        if results.pose_landmarks and draw:
            self.mpDraw.draw_landmarks(image,
                                       results.pose_landmarks,
                                       self.mpPose.POSE_CONNECTIONS,
                                       mp.solutions.drawing_styles.get_default_pose_landmarks_style())
            for id, lm in enumerate(results.pose_landmarks.landmark):
                cx, cy = int(lm.x * w), int(lm.y * h)
                point_list.append([cx, cy])
            # 数据发布处理
            pose_topic_data(self, point_list)
        return image

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
            # 数据发布处理
            hand_topic_data(self, point_list)
        return image

    def detect_face(self, image):
        '''
        返回值：
        image-识别处理后的图片
        face_bboxes-bounding_box，识别到人脸的id，左上角坐标x,y，脸框宽和高w,h，置信度。检测到多少人，该列表便有多少个元素
            [[id, x, y, w, h, 置信度]]  例如：[[0, 264, 184, 163, 163, 0.9228333830833435]]
        '''
        ih, iw, ic = image.shape
        results = self.face.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        if results.detections:
            face_bboxes = Mediapipe_FaceBboxes()
            face_id = 0
            for detection in results.detections:
                # self.mpDraw.draw_detection(image, detection)
                # print(detection.score)
                bboxC = detection.location_data.relative_bounding_box
                bbox = int(bboxC.xmin * iw), int(bboxC.ymin * ih), int(bboxC.width * iw), int(bboxC.height * ih)
                image = self.fancyDraw(image, bbox)
                cv2.putText(image, str(int(detection.score[0]*100))+"%", (bbox[0], bbox[1]-20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)
                face_bbox = Mediapipe_FaceBbox()
                face_bbox.id = face_id
                face_bbox.xmin = bbox[0]
                face_bbox.ymin = bbox[1]
                face_bbox.width = bbox[2]
                face_bbox.height = bbox[3]
                face_bbox.confidence = detection.score[0]
                face_bboxes.bboxes.append(face_bbox)
                face_id += 1
            # 发布话题
            self.face_pos_publisher.publish(face_bboxes)
        return image

    def fancyDraw(self, image, bbox, l=20, t=3):
        x, y, w, h = bbox
        x1, y1 = x + w, y + h
        # cv2.rectangle(image, bbox, (255, 0, 255), 2)
        # Top left
        cv2.line(image, (x, y), (x+l, y), (255, 0, 255), t)
        cv2.line(image, (x, y), (x, y+l), (255, 0, 255), t)

        # Top right
        cv2.line(image, (x1, y), (x1-l, y), (255, 0, 255), t)
        cv2.line(image, (x1, y), (x1, y+l), (255, 0, 255), t)

        # Bottom left
        cv2.line(image, (x, y1), (x+l, y1), (255, 0, 255), t)
        cv2.line(image, (x, y1), (x, y1-l), (255, 0, 255), t)

        # Bottom right
        cv2.line(image, (x1, y1), (x1-l, y1), (255, 0, 255), t)
        cv2.line(image, (x1, y1), (x1, y1-l), (255, 0, 255), t)

        return image

    def face_mesh(self, image):
        """
        返回值说明：
        image-识别处理后的图片
        point_list-468个特制点，具体对应位置，见图片
        """
        point_list = []
        results = self.faceMesh.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                # 绘制网格
                self.mpDraw.draw_landmarks(image=image, 
                                         landmark_list=face_landmarks,
                                         connections=self.mpFaceMesh.FACEMESH_TESSELATION,
                                         landmark_drawing_spec= None,
                                         connection_drawing_spec=mp.solutions.drawing_styles.get_default_face_mesh_tesselation_style())

                # 绘制线条
                self.mpDraw.draw_landmarks(image=image, 
                                         landmark_list=face_landmarks,
                                         connections=self.mpFaceMesh.FACEMESH_CONTOURS,
                                         landmark_drawing_spec= None,
                                         connection_drawing_spec=self.drawing_spec)

                for lm in face_landmarks.landmark:
                    h, w, c = image.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    point_list.append([cx, cy])
            # print(point_list)
            face_topic_data(self, point_list)
        return image

    def image_callback(self, msg):
        try:
            # 将ROS图像数据转换为OpenCV图像
            cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

            # 调整图像大小以加快处理速度
            # cv_image = cv2.resize(cv_image, (640, 480))

            if self.select_detect==0:
                out_image = self.detect_pose(cv_image)
            elif self.select_detect==1:
                out_image = self.detect_hand(cv_image)
            elif self.select_detect==2:
                out_image = self.detect_face(cv_image)
            elif self.select_detect==3:
                out_image = self.face_mesh(cv_image)
            else:
                print("功能序号出错，请选择正确的序号：0-pose, 1-hand, 2-face_detect, 3-face_mesh")
            cTime = time.time()
            fps = 1 / (cTime - self.pTime)
            self.pTime = cTime

            cv2.putText(out_image, f"FPS:{fps:.1f}", (20, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 3)

            # 显示图像
            cv2.imshow("Image window", out_image)

            # 按q退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown('Exit')
                cv2.destroyAllWindows()

        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    '''
        0 - pose_detect
        1 - hand_detect
        2 - face_detect
        3 - face_mesh
    '''
    poseEstimator = MediaPipe_Pose()
