cmake_minimum_required(VERSION 2.8.3)
project(vision_commons)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  image_transport
  dynamic_reconfigure
  sensor_msgs
  roslint
)

#find_package(PCL 1.2 REQUIRED)

roslint_cpp()

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/blueFilterDemoRange.cfg)

catkin_package(CATKIN_DEPENDS  
  cv_bridge
	INCLUDE_DIRS include helper_nodes
	LIBRARIES ${PROJECT_NAME}
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} src/filter.cpp src/contour.cpp src/geometry.cpp src/morph.cpp src/threshold.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} )

#add_library(vision_fusion src/color_constancy.cpp src/color_constancy_debug.cpp src/#fusion_framework.cpp src/laplacian.cpp)
#target_link_libraries(vision_fusion ${OpenCV_LIBRARIES} )

add_executable(blue_filter_demo helper_nodes/blue_filter_demo.cpp)
add_dependencies(blue_filter_demo ${vision_commons_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(blue_filter_demo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_executable(image_pub helper_nodes/image_pub.cpp)
target_link_libraries(image_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

add_executable(image_save helper_nodes/image_save.cpp)
target_link_libraries(image_save ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} vision_commons )

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
