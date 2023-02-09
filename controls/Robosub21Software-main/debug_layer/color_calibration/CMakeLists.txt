cmake_minimum_required(VERSION 2.8.3)
project(color_calibration)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  dynamic_reconfigure
  vision_commons
)


find_package(Boost REQUIRED COMPONENTS system)

add_service_files(
  FILES
  Dump.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/vision.cfg
)

catkin_package(
CATKIN_DEPENDS cv_bridge image_transport sensor_msgs std_msgs dynamic_reconfigure message_runtime vision_commons
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARIES_DIRS}
)

add_executable(color_calibrate src/color_calibrate.cpp)
target_link_libraries(color_calibrate ${catkin_LIBRARIES})

  
