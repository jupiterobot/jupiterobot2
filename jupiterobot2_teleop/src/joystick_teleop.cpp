/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "dynamixel_msgs/JointState.h"

#include <iostream>
#define ARM_SLOW 0.1
#define ARM_FAST 0.3

class Jupiter2Teleop
{
public:
  Jupiter2Teleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void arm1Callback(const dynamixel_msgs::JointState::ConstPtr &arm1_state);
  void arm2Callback(const dynamixel_msgs::JointState::ConstPtr &arm2_state);
  void arm3Callback(const dynamixel_msgs::JointState::ConstPtr &arm3_state);
  void arm4Callback(const dynamixel_msgs::JointState::ConstPtr &arm4_state);
  void gripperCallback(const dynamixel_msgs::JointState::ConstPtr &gripper_state);
  void headCallback(const dynamixel_msgs::JointState::ConstPtr &head_state);

  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, half_linear_, half_angular_, deadman_axis_, accelerate_button_, dir_left_right_, dir_up_down_, x_left_, b_right_, y_up_, a_down_, axis_down_l_, axis_down_r_, left_trigger_, right_trigger_, back_, start_;
  double l_scale_, a_scale_;

  //int count = 0;
  ros::Publisher vel_pub_;
  ros::Publisher arm1_pub_;
  ros::Publisher arm2_pub_;
  ros::Publisher arm3_pub_;
  ros::Publisher arm4_pub_;
  ros::Publisher gripper_pub_;
  ros::Publisher head_pub_;
  ros::Publisher capture_pub_;

  ros::Subscriber joy_sub_;
  ros::Subscriber arm1_sub_;
  ros::Subscriber arm2_sub_;
  ros::Subscriber arm3_sub_;
  ros::Subscriber arm4_sub_;
  ros::Subscriber gripper_sub_;
  ros::Subscriber head_sub_;

  geometry_msgs::Twist last_published_;
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
  std_msgs::Float64 arm_xb;
  std_msgs::Float64 arm_ya;
  std_msgs::String capture_hold;
  std_msgs::String capture_published_;

  boost::mutex publish_mutex_;
  bool deadman_pressed_, accelerate_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;
};

Jupiter2Teleop::Jupiter2Teleop() : 
                                    l_scale_(0.2),
                                    a_scale_(1.0),
                                    linear_(1),
                                    angular_(0),
                                    half_linear_(3),
                                    half_angular_(2),
                                    deadman_axis_(5),
                                    accelerate_button_(4),
                                    dir_left_right_(4),
                                    dir_up_down_(5),
                                    x_left_(0),
                                    b_right_(2),
                                    y_up_(3),
                                    a_down_(1),
                                    axis_down_l_(10),
                                    axis_down_r_(11),
                                    back_(8),
                                    start_(9),
                                    left_trigger_(6),
                                    right_trigger_(7)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_half_linear", half_linear_, half_linear_);
  ph_.param("axis_half_angular", half_angular_, half_angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("button_accelerate", accelerate_button_, accelerate_button_);
  ph_.param("scale_linear", l_scale_, l_scale_);
  ph_.param("scale_angular", a_scale_, a_scale_);

  deadman_pressed_ = false;
  accelerate_pressed_ = false;
  zero_twist_published_ = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  arm1_pub_ = ph_.advertise<std_msgs::Float64>("arm1_joint/command", 1, true);
  arm2_pub_ = ph_.advertise<std_msgs::Float64>("arm2_joint/command", 1, true);
  arm3_pub_ = ph_.advertise<std_msgs::Float64>("arm3_joint/command", 1, true);
  arm4_pub_ = ph_.advertise<std_msgs::Float64>("arm4_joint/command", 1, true);
  gripper_pub_ = ph_.advertise<std_msgs::Float64>("gripper_joint/command", 1, true);
  head_pub_ = ph_.advertise<std_msgs::Float64>("head_joint/command", 1, true);
  capture_pub_ = ph_.advertise<std_msgs::String>("capture_command", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Jupiter2Teleop::joyCallback, this);
  arm1_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("arm1_joint/state", 10, &Jupiter2Teleop::arm1Callback, this);
  arm2_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("arm2_joint/state", 10, &Jupiter2Teleop::arm2Callback, this);
  arm3_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("arm3_joint/state", 10, &Jupiter2Teleop::arm3Callback, this);
  arm4_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("arm4_joint/state", 10, &Jupiter2Teleop::arm4Callback, this);
  gripper_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("gripper_joint/state", 10, &Jupiter2Teleop::gripperCallback, this);
  head_sub_ = nh_.subscribe<dynamixel_msgs::JointState>("head_joint/state", 10, &Jupiter2Teleop::headCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&Jupiter2Teleop::publish, this));
}

void Jupiter2Teleop::arm1Callback(const dynamixel_msgs::JointState::ConstPtr &arm1_state)
{
  arm1.data = arm1_state->goal_pos;
}

void Jupiter2Teleop::arm2Callback(const dynamixel_msgs::JointState::ConstPtr &arm2_state)
{
  arm2.data = arm2_state->goal_pos;
}

void Jupiter2Teleop::arm3Callback(const dynamixel_msgs::JointState::ConstPtr &arm3_state)
{
  arm3.data = arm3_state->goal_pos;
}

void Jupiter2Teleop::arm4Callback(const dynamixel_msgs::JointState::ConstPtr &arm4_state)
{
  arm4.data = arm4_state->goal_pos;
}

void Jupiter2Teleop::gripperCallback(const dynamixel_msgs::JointState::ConstPtr &gripper_state)
{
  gripper.data = gripper_state->goal_pos;
}

void Jupiter2Teleop::headCallback(const dynamixel_msgs::JointState::ConstPtr &head_state)
{
  head.data = head_state->goal_pos;
}

void Jupiter2Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  geometry_msgs::Twist vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];
  accelerate_pressed_ = joy->buttons[accelerate_button_];

  if (joy->axes[half_linear_] || joy->axes[half_angular_])
  {
    vel.linear.x = l_scale_*0.6*joy->axes[half_linear_];
    vel.angular.z = a_scale_*0.6*joy->axes[half_angular_];
  }
  else if (accelerate_pressed_)
  {
    vel.linear.x = l_scale_*1.5*joy->axes[linear_];
    vel.angular.z = a_scale_*2.0*joy->axes[angular_];
  }
  else if (joy->axes[linear_] || joy->axes[angular_])
  {
    vel.linear.x = l_scale_*joy->axes[linear_];
    vel.angular.z = a_scale_*joy->axes[angular_];
  }

  if (joy->buttons[x_left_] && accelerate_pressed_)
  {
    arm_xb.data = ARM_FAST;
  }
  else if (joy->buttons[b_right_] && accelerate_pressed_)
  {
    arm_xb.data = 0.0 - ARM_FAST;
  }
  else if (joy->buttons[x_left_] && !accelerate_pressed_)
  {
    arm_xb.data = ARM_SLOW;
  }
  else if (joy->buttons[b_right_] && !accelerate_pressed_)
  {
    arm_xb.data = 0.0 - ARM_SLOW;
  }
  else
  {
    arm_xb.data = 0.0;
  }

  if (accelerate_pressed_ && joy->buttons[a_down_])
  {
    arm_ya.data = ARM_FAST;
  }
  else if (accelerate_pressed_ && joy->buttons[y_up_])
  {
    arm_ya.data = 0.0 - ARM_FAST;
  }
  else if (!accelerate_pressed_ && joy->buttons[a_down_])
  {
    arm_ya.data = ARM_SLOW;
  }
  else if (!accelerate_pressed_ && joy->buttons[y_up_])
  {
    arm_ya.data = 0.0 - ARM_SLOW;
  }
  else
  {
    arm_ya.data = 0.0;
  }

  if (joy->axes[dir_up_down_] > 0)
  {
    gripper.data = gripper.data + arm_xb.data;
    if (gripper.data > 0.6)
    {
      gripper.data = 0.6;
    }
    if (gripper.data < -0.4)
    {
      gripper.data = -0.4;
    }
    head.data = head.data + arm_ya.data;
    if (head.data < -0.8)
    {
      head.data = -0.8;
    }
    if (head.data > 0.6)
    {
      head.data = 0.6;
    }
  }
  if (joy->axes[dir_up_down_] < 0)
  {
    arm1.data = arm1.data + arm_xb.data;
    if (arm1.data < -2.6)
    {
      arm1.data = -2.6;
    }
    if (arm1.data > 2.6)
    {
      arm1.data = 2.6;
    }
    arm3.data = arm3.data + arm_ya.data;
    if (arm3.data < -2.5)
    {
      arm3.data = -2.5;
    }
    if (arm3.data > 2.6)
    {
      arm3.data = 2.6;
    }
  }
  if (joy->axes[dir_left_right_] > 0)
  {
    arm2.data = arm2.data + arm_ya.data;
    if (arm2.data < -2.1)
    {
      arm2.data = -2.1;
    }
    if (arm2.data > 2.2)
    {
      arm2.data = 2.2;
    }
  }
  if (joy->axes[dir_left_right_] < 0)
  {
    arm4.data = arm4.data + arm_ya.data;
    if (arm4.data < -1.8)
    {
      arm4.data = -1.8;
    }
    if (arm4.data > 1.8)
    {
      arm4.data = 1.8;
    }
  }
  if (joy->buttons[back_])
  {
    head.data = -0.5;
  }
  if (joy->buttons[start_])
  {
    head.data = 0.5;
  }
  if (joy->buttons[axis_down_l_] && joy->buttons[axis_down_r_])
  {
    arm1.data = 0.0;
    arm2.data = 0.0;
    arm3.data = 0.0;
    arm4.data = 0.0;
    gripper.data = 0.0;
    head.data = 0.0;
  }
  if (joy->buttons[left_trigger_])
  {
    arm1.data = 0.0;
    arm2.data = -1.4;
    arm3.data = 2.2;
    arm4.data = 0.6;
  }
  if (joy->buttons[right_trigger_])
  {
    arm1.data = 0.0;
    arm2.data = 1.9;
    arm3.data = 0.8;
    arm4.data = -1.4;
  }
  last_published_ = vel;
  arm1_published_ = arm1;
  arm2_published_ = arm2;
  arm3_published_ = arm3;
  arm4_published_ = arm4;
  gripper_published_ = gripper;
  head_published_ = head;
  capture_published_ = capture_hold;
}

void Jupiter2Teleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
    zero_twist_published_ = false;
  }
  else if (!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    //arm1_pub_.publish(*new std_msgs::Float64());
    //arm2_pub_.publish(*new std_msgs::Float64());
    //arm3_pub_.publish(*new std_msgs::Float64());
    //arm4_pub_.publish(*new std_msgs::Float64());
    //gripper_pub_.publish(*new std_msgs::Float64());
    //head_pub_.publish(*new std_msgs::Float64());
    zero_twist_published_ = true;
  }

  arm1_pub_.publish(arm1_published_);
  arm2_pub_.publish(arm2_published_);
  arm3_pub_.publish(arm3_published_);
  arm4_pub_.publish(arm4_published_);
  gripper_pub_.publish(gripper_published_);
  head_pub_.publish(head_published_);

  /*if (count)
  {
    capture_pub_.publish(capture_published_);
    count = 0;
  }*/
  capture_pub_.publish(capture_published_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jupiter2_teleop_joy");
  Jupiter2Teleop jupiter2_teleop;
  ros::spin();
}
