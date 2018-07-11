# MRSL Jump Point Search Planning Library v1.0
[![wercker status](https://app.wercker.com/status/880ab5feaff25f0483e5f2c4f834b8c0/s/master "wercker status")](https://app.wercker.com/project/byKey/880ab5feaff25f0483e5f2c4f834b8c0)
- - -
Jump Point Search for path planning in both 2D and 3D environments. Original jump point seach algorithm is proposed in ["D. Harabor and A. Grastien. Online Graph Pruning for Pathfinding on Grid Maps. In National Conference on Artificial Intelligence (AAAI), 2011"](https://www.aaai.org/ocs/index.php/AAAI/AAAI11/paper/download/3761/4007). The 3D version is proposed in ["S. Liu, M. Watterson, K. Mohta, K. Sun, S. Bhattacharya, C.J. Taylor and V. Kumar. Planning Dynamically Feasible Trajectories for Quadrotors using Safe Flight Corridors in 3-D Complex Environments. ICRA 2017"](http://ieeexplore.ieee.org/abstract/document/7839930/).

## Installation
#### Required:
 - Eigen3
 - yaml-cpp

Simply run following commands to install dependancy:
```sh
$ sudo apt update
$ sudo apt install -y libeigen3-dev libyaml-cpp-dev libboost-dev cmake
```

#### A) Simple cmake
```sh
$ mkdir build && cd build && cmake .. && make -j4
```

#### B) Using CATKIN
```sh
$ mv jps3d ~/catkin_ws/src
$ cd ~/catkin_ws & catkin_make_isolated -DCMAKE_BUILD_TYPE=Release
```

#### CTest
Run following command in the `build` folder for testing the executables:
```sh
$ make test
```

If everything works, you should see the results as:
```
Running tests...
Test project /home/sikang/cpp_ws/src/jps3d/build
    Start 1: test_planner_2d
1/2 Test #1: test_planner_2d ..................   Passed    0.98 sec
    Start 2: test_planner_3d
2/2 Test #2: test_planner_3d ..................   Passed    0.00 sec

100% tests passed, 0 tests failed out of 2

Total Test time (real) =   0.98 sec
```

#### Include in other projects
Note that in other repository, add following commands in `CMakeLists.txt` in order to correctly link `jps3d`:
```sh
find_package(jps3d REQUIRED)
include_directories(${JPS3D_INCLUDE_DIRS})
...
add_executable(test_xxx src/test_xxx.cpp)
target_link_libraries(test_xxx ${JPS3D_LIBRARIES})
```

Two libs will be installed in the system: the standard `jps_lib` and a variation `dmp_lib`.

## Usage
There are three steps to start a planning thread:
```c++
std::unique_ptr<JPSPlanner2D> planner(new JPSPlanner2D(false)); // Declare a 2D planner
planner->setMapUtil(MAP_UTIL_PTR); // Set collision checking function
planner->updateMap(); // Set map, must be called before plan
bool valid_jps = planner->plan(start, goal, 1, true); // Plan from start to goal with heuristic weight 1, using JPS
bool valid_astar = planner->plan(start, goal, 1, false); // Plan from start to goal with heuristic weight 1, using A*
```

First, the collision checking util must be loaded as:
```c++
planner->setMapUtil(MAP_UTIL_PTR); // Set collision checking function
```
The `MAP_UTIL_PTR` can be either `OCCMapUtil` for 2D or `VoxelMapUtil` for 3D. It can be confusing to set up this util, see the example code for more details.

Second, call the function `updateMap()` to allocate the internal map:
```
planner->updateMap(); // Set map, must be called before plan
```

Finally, call the function `plan` to plan a path from `start` to `goal`. The third input is the heuristic weight, the forth input indicates whether planning with `JPS` or `A*`.

Four planners are provided:
 - ```JPSPlanner2D```
 - ```JPSPlanner3D```
 - ```DMPlanner2D```
 - ```DMPlanner3D```

## Example
An example in 2D map is given in `test/test_planner_2d.cpp`, in which we plan from start to goal using both ```A*``` and ```JPS```.
The results are plotted in [corridor.png](https://github.com/sikang/jps3d/blob/master/data/corridor.png).
Green path is from ```A*```, red path is from ```JPS```.

![Visualization](./data/corridor.png)
```sh
$ ./build/test_planner_2d ../data/corridor.yaml
start: 2.5  -2
goal:  35 2.5
origin:  0 -5
dim: 799 199
resolution: 0.05
JPS Planner takes: 5.000000 ms
JPS Path Distance: 35.109545
JPS Planner takes: 5.000000 ms
AStar Planner takes: 62.000000 ms
AStar Path Distance: 35.109545
```

An example in 3D map is presented in `test/test_planner_3d` with the yaml `data/simple3d.yaml`.

To generate map in `yaml` format which can be loaded directly in the test node, a simple executable file `test/create_map.cpp` is used. User can easily change the location of blocks in the source code.

## Doxygen
For more details, please refer to [Doxygen](https://sikang.github.io/jps3d).

