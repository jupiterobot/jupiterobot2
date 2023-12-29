cmake_minimum_required(VERSION 3.0.2)
project(jupiterobot2_vision_yolov5)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  message_generation
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo_talker_listener
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
