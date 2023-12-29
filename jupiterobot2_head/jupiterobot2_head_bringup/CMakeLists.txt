cmake_minimum_required(VERSION 3.0.2)
project(jupiterobot2_head_bringup)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
