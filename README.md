# MRSL Jump Point Search Planning Library
Implementation of Jump Point Search in both 2D and 3D environments. Original jump point seach algorithm is proposed in ["D. Harabor and A. Grastien. Online Graph Pruning for Pathfinding on Grid Maps. In National Conference on Artificial Intelligence (AAAI), 2011"](https://www.aaai.org/ocs/index.php/AAAI/AAAI11/paper/download/3761/4007). The 3D version is proposed in ["S. Liu, M. Watterson, K. Mohta, K. Sun, S. Bhattacharya, C.J. Taylor and V. Kumar. Planning Dynamically Feasible Trajectories for Quadrotors using Safe Flight Corridors in 3-D Complex Environments. ICRA 2017"](http://ieeexplore.ieee.org/abstract/document/7839930/). 

## Compilation
Required: 
 - Boost
 - Eigen
 - yaml-cpp (Reading data files)
 - VTK (For visualizing output)

A) Simple cmake
```sh
$ mkdir build && cd build && cmake .. && make
```

B) Using CATKIN with ROS
```sh
$ mv jps3d ~/catkin_ws/src
$ cd ~/catkin_ws & catkin_make -DCMAKE_BUILD_TYPE=Release
```
Note that in other repository, add following command in `CMakeLists.txt` in order to correctly find `jps3d`:
```sh
include_directories(${JPS3D_INCLUDE_DIRS})
``` 

Two libs will be installed in the system: the standard `jps_lib` and a fast implementation `nx_jps_lib`. The latter one only supports 3D and a bit faster than the former standard one.

## Example Usage
The simple API are provided in the base planner class, here are some important functions to set up a planning thread:
```c++
std::unique_ptr<PlannerBase> planner(new XXXUtil(false)); // Declare a XXX planner
planner->setMapUtil(MAP_UTIL_PTR); // Set collision checking function
bool valid = planner->plan(start, goal); // Plan from start to goal
```
In this library, we consider 3D voxel grid but user can write their own 2D map util plugin using the ```MapBaseUtil``` class. Two planners are provided as follows:
 - ```GraphSearch2DUtil```
 - ```GraphSearch3DUtil```

The results from ```A*``` and ```JPS``` are plotted in [corridor.jpg](https://github.com/sikang/jps3d/blob/master/data/corridor.jpg).
Green path is from A*, black path is from JPS.

![Visualization](./data/corridor.jpg)
```sh
$ ./build/test_planner_2d ../data/corridor.yaml
start: 2.5  -2   0
goal:  35 2.5   0
origin:  0 -5  0
dim: 799 199   1
resolution: 0.05
JPS Planner takes: 4.000000 ms
JPS Path Distance: 35.109545
AStar Planner takes: 77.000000 ms
AStar Path Distance: 35.109545
```

To generate map in `yaml` format which can be loaded directly in the test node, a simple executable file `test/create_map.cpp` is used. User can easily change the location of blocks in the source code.

## Doxygen
For more details, please refer to https://sikang.github.io/jps3d/index.html

