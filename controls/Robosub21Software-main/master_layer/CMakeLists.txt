cmake_minimum_required(VERSION 2.8.3)
project(master_layer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  anahita_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  #darknet_ros_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED NO_MODULE)

add_message_files(
  FILES
  Waypoint.msg
)

add_service_files(
  FILES
  CurrentTask.srv
  ChangeOdom.srv
  GoToIncremental.srv
  GoTo.srv
  GoToPose.srv
  VerifyObject.srv
  InitCircularTrajectory.srv
  Hold.srv
  TrajectoryComplete.srv
  PoseReach.srv
  ChangeTorpedoHole.srv
  RequestMarkerAngle.srv
  PingerBottomTarget.srv
  PingerFrontTarget.srv
  TargetNormal.srv
  GetMaxDepth.srv
  GetBlackoutTime.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs anahita_msgs geometry_msgs 
  #darknet_ros_msgs  
)

catkin_package(
CATKIN_DEPENDS actionlib actionlib_msgs roscpp rospy std_msgs geometry_msgs message_runtime 
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#   ${task_handler_layer_INCLUDE_DIRS}
#   ${motion_layer_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


#add_executable(local_vision_odom src/local_vision_odom.cpp)
#target_link_libraries(local_vision_odom ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

