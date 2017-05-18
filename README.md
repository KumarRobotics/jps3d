# MRSL Jump Point Search Planning Library
Implementation of Jump Point Search with [YagSBPL](https://www.math.upenn.edu/~subhrabh/html_cache/7f068a4d19ed85a15c9e25ecae8b40c1.html). 

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

B) Using CATKIN
```sh
$ cd mv jps3d ~/catkin_ws/src
$ cd ~/catkin_ws & catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Example Usage
The results are plotted in [corridor.jpg](https://github.com/sikang/jps3d/blob/master/data/corridor.jpg).
![Visualization](./data/corridor.jpg)
```sh
$ ./build/test_planner_2d ../data/corridor.yaml
JPS Planner takes: 73.000000 ms
JPS Path Distance: 35.192388
AStar Planner takes: 317.000000 ms
AStar Path Distance: 35.192388
```
## Doxygen
For more details, please refer to https://sikang.github.io/jps3d/index.html

