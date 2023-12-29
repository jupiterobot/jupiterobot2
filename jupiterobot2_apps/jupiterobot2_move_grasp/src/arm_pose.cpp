/*******************************************************************************
* Copyright 2023 Jupiter Robot Technology Co., Ltd.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Dante Huang */

#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "dynamixel_msgs/JointState.h"

class ArmPose
{
public:
  ArmPose();

private:
  void cmdCallback(const std_msgs::String::ConstPtr &cmd);
  void arm1Callback(const dynamixel_msgs::JointState::ConstPtr &arm1_state);
  void arm2Callback(const dynamixel_msgs::JointState::ConstPtr &arm2_state);
  void arm3Callback(const dynamixel_msgs::JointState::ConstPtr &arm3_state);
  void arm4Callback(const dynamixel_msgs::JointState::ConstPtr &arm4_state);
  void gripperCallback(const dynamixel_msgs::JointState::ConstPtr &gripper_state);
  void headCallback(const dynamixel_msgs::JointState::ConstPtr &head_state);
  void publish();

  ros::NodeHandle ph_, nh_;

  ros::Publisher arm1_pub_;
  ros::Publisher arm2_pub_;
  ros::Publisher arm3_pub_;
  ros::Publisher arm4_pub_;
  ros::Publisher gripper_pub_;
  ros::Publisher head_pub_;

  ros::Subscriber cmd_sub_;
  ros::Subscriber arm1_sub_;
  ros::Subscriber arm2_sub_;
  ros::Subscriber arm3_sub_;
  ros::Subscriber arm4_sub_;
  ros::Subscriber gripper_sub_;
  ros::Subscriber head_sub_;

  std_msgs::Float64 arm1_published_;
  std_msgs::Float64 arm1;
  std_msgs::Float64 arm2_published_;
  std_msgs::Float64 arm2;
  std_msgs::Float64 arm3_published_;
  std_msgs::Float64 arm3;
  std_msgs::Float64 arm4_published_;
  std_msgs::Float64 arm4;
  std_msgs::Float64 gripper_published_;
  std_msgs::Float64 gripper;
  std_msgs::Float64 head_published_;
  std_msgs::Float64 head;

  ros::Timer timer_;
};

ArmPose::ArmPose()
{
  arm1_pub_ = ph_.advertise<std_msgs::Float64>("arm1_joint/command", 1, true);
  arm2_pub_ = ph_.advertise<std_msgs::Float64>("arm2_joint/command", 1, true);
  arm3_pub_ = ph_.advertise<std_msgs::Float64>("arm3_joint/command", 1, true);
  arm4_pub_ = ph_.advertise<std_msgs::Float64>("arm4_joint/command", 1, true);
  gripper_pub_ = ph_.advertise<std_msgs::Float64>("gripper_joint/command", 1, true);
  head_pub_ = ph_.advertise<std_msgs::Float64>("head_joint/command", 1, true);

  arm1_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("arm1_joint/state", 10, &ArmPose::arm1Callback, this);
  arm2_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("arm2_joint/state", 10, &ArmPose::arm2Callback, this);
  arm3_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("arm3_joint/state", 10, &ArmPose::arm3Callback, this);
  arm4_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("arm4_joint/state", 10, &ArmPose::arm4Callback, this);
  gripper_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("gripper_joint/state", 10, &ArmPose::gripperCallback, this);
  head_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("head_joint/state", 10, &ArmPose::headCallback, this);

  cmd_sub_ = nh_.subscribe<std_msgs::String>("arm_cmd", 10, &ArmPose::cmdCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&ArmPose::publish, this));
}

void ArmPose::arm1Callback(const dynamixel_msgs::JointState::ConstPtr &arm1_state)
{
  arm1.data = arm1_state->goal_pos;
}

void ArmPose::arm2Callback(const dynamixel_msgs::JointState::ConstPtr &arm2_state)
{
  arm2.data = arm2_state->goal_pos;
}

void ArmPose::arm3Callback(const dynamixel_msgs::JointState::ConstPtr &arm3_state)
{
  arm3.data = arm3_state->goal_pos;
}

void ArmPose::arm4Callback(const dynamixel_msgs::JointState::ConstPtr &arm4_state)
{
  arm4.data = arm4_state->goal_pos;
}

void ArmPose::gripperCallback(const dynamixel_msgs::JointState::ConstPtr &gripper_state)
{
  gripper.data = gripper_state->goal_pos;
}

void ArmPose::headCallback(const dynamixel_msgs::JointState::ConstPtr &head_state)
{
  head.data = head_state->goal_pos;
}

void ArmPose::cmdCallback(const std_msgs::String::ConstPtr &cmd)
{
  if (cmd->data == "left")
  {
    arm1.data = 0.4;
    arm2.data = 0.5;
    arm3.data = 0.4;
    arm4.data = 0.5;
    gripper.data = 0.0;
  }
  if (cmd->data == "right")
  {
    arm1.data = -0.4;
    arm2.data = 0.5;
    arm3.data = 0.4;
    arm4.data = 0.5;
    gripper.data = 0.0;
  }
  if (cmd->data == "ready")
  {
    arm1.data = 0.0;
    arm2.data = 0.333;
    arm3.data = 0.333;
    arm4.data = 0.666;
  }
  if (cmd->data == "catch")
  {
    arm1.data = 0.0;
    arm2.data = 0.7;
    arm3.data = 1.4;
    arm4.data = -0.7;
  }
  if (cmd->data == "rest")
  {
    arm1.data = 0.0;
    arm2.data = -1.8;
    arm3.data = 2.4;
    arm4.data = 1.0;
  }
  if (cmd->data == "open")
  {
    gripper.data = -0.3;
  }
  if (cmd->data == "close")
  {
    gripper.data = 0.3;
  }
  if (cmd->data == "up")
  {
    head.data = -0.5;
  }
  if (cmd->data == "down")
  {
    head.data = 0.5;
  }
  arm1_published_ = arm1;
  arm2_published_ = arm2;
  arm4_published_ = arm4;
  arm3_published_ = arm3;
  gripper_published_ = gripper;
  head_published_ = head;
}

void ArmPose::publish()
{
  arm1_pub_.publish(arm1_published_);
  arm2_pub_.publish(arm2_published_);
  arm4_pub_.publish(arm4_published_);
  arm3_pub_.publish(arm3_published_);
  gripper_pub_.publish(gripper_published_);
  head_pub_.publish(head_published_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_pose");
  ArmPose arm_pose;
  ros::spin();
}
