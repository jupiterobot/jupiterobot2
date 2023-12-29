cmake_minimum_required(VERSION 3.0.2)
project(jupiterobot2_move_grasp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamixel_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm_pose src/arm_pose.cpp)
target_link_libraries(arm_pose ${catkin_LIBRARIES})
add_dependencies(arm_pose dynamixel_msgs_gencpp)
