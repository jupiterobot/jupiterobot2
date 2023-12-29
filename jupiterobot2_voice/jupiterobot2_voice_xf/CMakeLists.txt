cmake_minimum_required(VERSION 3.0.2)
project(jupiterobot2_voice_xf)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(iat_publish
  src/iat_publish.cpp
  src/speech_recognizer.c
  src/linuxrec.c)
target_link_libraries(iat_publish
  ${catkin_LIBRARIES}
  libmsc.so -ldl -lpthread -lm -lrt -lasound)

add_executable(tts_subscribe src/tts_subscribe.cpp)
target_link_libraries(
  tts_subscribe
  ${catkin_LIBRARIES}
  libmsc.so -ldl -pthread)

add_executable(voice_assistant src/voice_assistant.cpp)
target_link_libraries(
  voice_assistant
  ${catkin_LIBRARIES}
  libmsc.so -ldl -pthread)
