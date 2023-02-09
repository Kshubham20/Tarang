cmake_minimum_required(VERSION 3.0.2)
project(colour_calibration_srv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

add_service_files(
   FILES
   Dump.srv
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES colour_calibration_srv
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
