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

class Jupiter2Teleop
{
public:
  Jupiter2Teleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, half_linear_, half_angular_, deadman_axis_, accelerate_button_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_, accelerate_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;

};

Jupiter2Teleop::Jupiter2Teleop():
  linear_(1),
  angular_(0),
  half_linear_(3),
  half_angular_(2),
  deadman_axis_(5),
  accelerate_button_(4),
  l_scale_(0.2),
  a_scale_(1.0)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_half_linear", half_linear_, half_linear_);
  ph_.param("axis_half_angular", half_angular_, half_angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("button_accelerate", accelerate_button_, accelerate_button_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  deadman_pressed_ = false;
  accelerate_pressed_ = false;
  zero_twist_published_ = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Jupiter2Teleop::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&Jupiter2Teleop::publish, this));
}

void Jupiter2Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
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
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];
}

void Jupiter2Teleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
    zero_twist_published_=false;
  }
  else if(!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    zero_twist_published_=true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop_base");
  Jupiter2Teleop jupiter2_teleop;
  ros::spin();
}
