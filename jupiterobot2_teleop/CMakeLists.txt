cmake_minimum_required(VERSION 3.0.2)
project(jupiterobot2_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS rospy roscpp geometry_msgs joy dynamixel_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS rospy roscpp geometry_msgs joy dynamixel_msgs
  DEPENDS
)

###########
## Build ##
###########

add_executable(joystick_teleop src/joystick_teleop.cpp)
target_link_libraries(joystick_teleop ${catkin_LIBRARIES})
add_dependencies(joystick_teleop dynamixel_msgs_gencpp)

#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS joystick_teleop
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
