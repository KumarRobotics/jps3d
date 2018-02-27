cmake_minimum_required(VERSION 2.8)
project(jps3d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

set(BUILD_SHARED_LIBS ON)

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)

IF(YAMLCPP_FOUND)
  MESSAGE(STATUS "Found yaml-cpp.")
ELSE()
  MESSAGE(FATAL_ERROR "Could not find yaml-cpp.")
ENDIF()


INCLUDE_DIRECTORIES(include ${EIGEN3_INCLUDE_DIRS})


add_library(jps_lib src/graph_search.cpp
  src/planner_util.cpp)

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

include(CTest)

add_executable(test_planner_2d test/test_planner_2d.cpp)
target_link_libraries(test_planner_2d jps_lib ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})
add_test(test_planner_2d test_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)



install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS jps_lib nx_jps_lib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

