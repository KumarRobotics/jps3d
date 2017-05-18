# MRSL Jump Point Search Planning Library

## Compilation

A) Simple cmake
mkdir build && cd build && cmake .. && make


B) Using CATKIN
cd mv jps3d ~/catkin_ws/src
cd ~/catkin_ws & catkin_make -DCMAKE_BUILD_TYPE=Release

## Example Usage
./build/test_planner_2d ../data/corridor.yaml

