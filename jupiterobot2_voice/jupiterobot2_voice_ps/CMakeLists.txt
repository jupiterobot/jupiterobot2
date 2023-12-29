cmake_minimum_required(VERSION 3.0.2)
project(jupiterobot2_voice_ps)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

install(PROGRAMS nodes/voice_control_example.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
