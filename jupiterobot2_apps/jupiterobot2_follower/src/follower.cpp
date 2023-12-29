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
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <jupiterobot2_msgs/Follow.h>

class Jupiter2Follower
{
public:
  Jupiter2Follower();

private:
  void imagecb(const sensor_msgs::Image::ConstPtr& depth_msg);
  void publishMarker(double x, double y, double z);
  void publishBbox();
  bool FollowSrvCb(jupiterobot2_msgs::Follow::Request& request, jupiterobot2_msgs::Follow::Response& response);

  ros::NodeHandle nh_;

  ros::Publisher cmd_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher bbox_pub_;
  ros::Subscriber img_sub_;
  ros::ServiceServer follow_srv_;
  
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double byd_z_;
  double max_z_;
  double goal_z_;
  double x_scale_;
  double z_scale_;
  bool enabled_;
};

Jupiter2Follower::Jupiter2Follower()
{
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("marker",1);
  bbox_pub_ = nh_.advertise<visualization_msgs::Marker>("bbox",1);
  img_sub_= nh_.subscribe<sensor_msgs::Image>("camera/depth/image_raw", 1, &Jupiter2Follower::imagecb, this);
  follow_srv_ = nh_.advertiseService("follower_switch", &Jupiter2Follower::FollowSrvCb, this);
  nh_.getParam("/jupiterobot2_follower/enabled", enabled_);
  min_x_ = -0.30;
  max_x_ = 0.30;
  min_y_ = 0.12;
  max_y_ = 0.28;
  byd_z_ = 1.50;
  max_z_ = 1.20;
  goal_z_ = 0.70;
  x_scale_ = 7.00;
  z_scale_ = 2.00;
}

void Jupiter2Follower::imagecb(const sensor_msgs::Image::ConstPtr& depth_msg)
{
  // Precompute the sin function for each row and column
  uint32_t image_width = depth_msg->width;
  float x_radians_per_pixel = 60.0/57.0/image_width;
  float sin_pixel_x[image_width];
  for (int x = 0; x < image_width; ++x) {
    sin_pixel_x[x] = sin((x - image_width/ 2.0)  * x_radians_per_pixel);
  }

  uint32_t image_height = depth_msg->height;
  float y_radians_per_pixel = 45.0/57.0/image_width;
  float sin_pixel_y[image_height];
  for (int y = 0; y < image_height; ++y) {
    // Sign opposite x for y up values
    sin_pixel_y[y] = sin((image_height/ 2.0 - y)  * y_radians_per_pixel);
  }

  //X,Y,Z of the centroid
  float x = 0.0;
  float y = 0.0;
  float z = 1e6;
  //Number of points observed
  unsigned int n = 0;

  //Iterate through all the points in the region and find the average of the position
  cv_bridge::CvImagePtr img_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  for (int v = 0; v < (int)depth_msg->height; ++v)
  {
    for (int u = 0; u < (int)depth_msg->width; ++u)
    {
      float depth = img_depth->image.ptr<float>(v)[u] * 1e-3;
      if (!std::isfinite(depth) || depth > byd_z_) continue;
      float y_val = sin_pixel_y[v] * depth;
      float x_val = sin_pixel_x[u] * depth;
      if ( y_val > min_y_ && y_val < max_y_ &&
          x_val > min_x_ && x_val < max_x_)
      {
        x += x_val;
        y += y_val;
        z = std::min(z, depth); //approximate depth as forward.
        // std::cout<<"distance: "<< z <<std::endl;
        n++;
      }
    }
  }

  //If there are points, find the centroid and calculate the command goal.
  //If there are no points, simply publish a stop goal.
  if (n>4000)
  {
    x /= n;
    y /= n;
    if(z > max_z_){
      ROS_INFO_THROTTLE(1, "Centroid too far away %f, stopping the robot", z);
      if (enabled_)
      {
        cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      }
      return;
    }

    ROS_INFO_THROTTLE(1, "Centroid at %f %f %f with %d points", x, y, z, n);
    publishMarker(x, y, z);

    if (enabled_)
    {
      geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
      cmd->linear.x = (z - goal_z_) * z_scale_;
      cmd->angular.z = -x * x_scale_;
      cmd_pub_.publish(cmd);
    }
  }
  else
  {
    ROS_INFO_THROTTLE(1, "Not enough points(%d) detected, stopping the robot", n);
    publishMarker(x, y, z);

    if (enabled_)
    {
      cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
    }
  }
  publishBbox();
}

void Jupiter2Follower::publishMarker(double x, double y, double z)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "camera_depth_optical_frame";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker_pub_.publish(marker);
}

void Jupiter2Follower::publishBbox()
{
  double x = (min_x_ + max_x_)/2;
  double y = (min_y_ + max_y_)/2;
  double z = (0 + max_z_)/2;

  double scale_x = (max_x_ - x)*2;
  double scale_y = (max_y_ - y)*2;
  double scale_z = (max_z_ - z)*2;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "camera_depth_optical_frame";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale_x;
  marker.scale.y = scale_y;
  marker.scale.z = scale_z;
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  bbox_pub_.publish(marker);
}

bool Jupiter2Follower::FollowSrvCb(jupiterobot2_msgs::Follow::Request& request, jupiterobot2_msgs::Follow::Response& response)
{
  if ((enabled_ == true) && (request.state == request.STOP))
  {
    ROS_INFO("State changed: following stop.");
    cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
    enabled_ = false;
  }
  else if ((enabled_ == false) && (request.state == request.START))
  {
    ROS_INFO("State changed: following start.");
    enabled_ = true;
  }
  response.result = response.SUCCESS;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jupiterobot2_follower");
  Jupiter2Follower jupiterobot2_follower;
  ros::spin();
}
