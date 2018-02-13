# - Config file for the jps3d package
# It defines the following variables
#  JPS3D_INCLUDE_DIRS - include directories for motion_primitive_library
#  JPS3D_LIBRARIES    - libraries to link against
 
# Compute paths
get_filename_component(JPS3D_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(JPS3D_INCLUDE_DIRS "${JPS3D_CMAKE_DIR}/../../../include")
 
set(JPS3D_LIBRARIES "${JPS3D_CMAKE_DIR}/../../../lib/libjps_lib.so" 
  "${JPS3D_CMAKE_DIR}/../../../lib/libnx_jps_lib.so")
