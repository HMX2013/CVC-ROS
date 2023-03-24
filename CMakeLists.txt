cmake_minimum_required(VERSION 3.0.2)
project(cvc_ros)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
  autoware_msgs
  jsk_recognition_msgs
)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

generate_dynamic_reconfigure_options(
  cfg/cvc_ros.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cvc_ros
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(cvc_ros_node
  src/cvc_ros_node.cpp
)

add_dependencies(cvc_ros_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(cvc_ros_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)