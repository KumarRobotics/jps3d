cmake_minimum_required(VERSION 2.8)
project(jps3d)

OPTION(USE_ROS "Set OFF if you want to build this package without Catkin" OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

IF(USE_ROS)
  message(STATUS "Compile with ROS.")
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
ELSE()
  #SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  #SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(EIGEN3 REQUIRED eigen3)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)
FIND_PACKAGE( Boost COMPONENTS filesystem system REQUIRED )

IF(YAMLCPP_FOUND)
  MESSAGE(STATUS "Found yaml-cpp.")
  #MESSAGE("YAMLCPP_INCLUDE_DIRS: ${YAMLCPP_INCLUDE_DIRS}")
  #MESSAGE("YAMLCPP_LIBRARIES: ${YAMLCPP_LIBRARIES}")
  #MESSAGE("YAMLCPP_LIBRARY_DIRS: ${YAMLCPP_LIBRARY_DIRS}")
ELSE()
  MESSAGE(FATAL_ERROR "Could not find yaml-cpp.")
ENDIF()

INCLUDE_DIRECTORIES(
  include
  ${EIGEN3_INCLUDE_DIRS}
  #${YAMLCPP_INCLUDE_DIRS}
  #${JPEG_INCLUDE_DIR}
  #${LIBXML2_INCLUDE_DIR}
  #${FFTW3_INCLUDE_DIR}
  #${Eigen_INCLUDE_DIRS}
  )


#add_library(jps_3d src/planner/nx_jps_3d.cpp
#  src/planner/jps_3D.cpp
#  src/planner/planner_base.cpp)
add_library(jps_lib src/planner/a_star_util.cpp
  src/planner/jps_2d_util.cpp
  src/planner/jps_3d_util.cpp
  src/planner/planner_base.cpp)

add_executable(test_planner test/test_planner.cpp)
target_link_libraries(test_planner jps_lib ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})


