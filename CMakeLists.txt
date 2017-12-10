cmake_minimum_required(VERSION 2.8)
project(jps3d)

OPTION(USE_ROS "Set OFF if you want to build this package without Catkin" ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)

IF(YAMLCPP_FOUND)
  MESSAGE(STATUS "Found yaml-cpp.")
  #MESSAGE("YAMLCPP_INCLUDE_DIRS: ${YAMLCPP_INCLUDE_DIRS}")
  #MESSAGE("YAMLCPP_LIBRARIES: ${YAMLCPP_LIBRARIES}")
  #MESSAGE("YAMLCPP_LIBRARY_DIRS: ${YAMLCPP_LIBRARY_DIRS}")
ELSE()
  MESSAGE(FATAL_ERROR "Could not find yaml-cpp.")
ENDIF()


IF(USE_ROS)
  message("Compile with ROS.")
  # roslib cmake_modules
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp)
  catkin_package(
   DEPENDS EIGEN3
    CATKIN_DEPENDS
    INCLUDE_DIRS include
    LIBRARIES jps_lib nx_jps_lib
    )
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
ELSE()
  #SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  #SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

find_package(VTK REQUIRED)

if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
  message("VTK was not found")
else()
  include(${VTK_USE_FILE})
  message("VTK was found")
endif()


INCLUDE_DIRECTORIES( include ${EIGEN3_INCLUDE_DIRS})


add_library(jps_lib src/graph_search.cpp
  src/graph_search_2d_util.cpp
  src/graph_search_3d_util.cpp
  src/planner_util_base.cpp)

#add_library(yag_jps_lib
#  src/yag_planner/jps_3d_util.cpp
#  src/nx_planner/planner_base.cpp)
#target_link_libraries(yag_jps_lib ${Boost_LIBRARIES})

add_library(nx_jps_lib src/nx_planner/nx_jps_3d.cpp
  src/nx_planner/jps_3D.cpp
  src/nx_planner/planner_base.cpp)
target_link_libraries(nx_jps_lib ${Boost_LIBRARIES})

add_executable(create_map test/create_map.cpp)
target_link_libraries(create_map ${YAMLCPP_LIBRARIES})

add_executable(test_planner_2d test/test_planner_2d.cpp)
target_link_libraries(test_planner_2d jps_lib ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES})

IF(USE_ROS)
  install(TARGETS jps_lib nx_jps_lib test_planner_2d
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(
    DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
ELSE()
  install(TARGETS jps_lib nx_jps_lib test_planner_2d
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
  install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    )
ENDIF()

