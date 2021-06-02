cmake_minimum_required(VERSION 3.0.2)
project(sclerp_motion_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  std_msgs
  urdf
  kdl_parser
  trajectory_msgs
)

link_directories(../../devel/lib)
find_package(kinlib) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sclerp_motion_planner
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${kinlib_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/sclerp_interface.cpp
  
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  kinlib
)
