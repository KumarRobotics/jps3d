cmake_minimum_required(VERSION 2.8)
project(jps3d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")
set(USE_ROS TRUE)


IF(USE_ROS)
  message(STATUS "Compile ROS utils.")
  # roslib cmake_modules
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp)
  catkin_package(
    DEPENDS Eigen3 
    CATKIN_DEPENDS 
    INCLUDE_DIRS include
    LIBRARIES jps_3d
    )
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
ENDIF()


find_package(Eigen3 REQUIRED)
include_directories(./include ${EIGEN3_INCLUDE_DIR})

add_library(jps_3d src/planner/nx_jps_3d.cpp
  src/planner/jps_3D.cpp
  src/planner/planner_base.cpp)

