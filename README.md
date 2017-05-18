# MRSL Jump Point Search Planning Library
Implementation of Jump Point Search with [YagSBPL](https://www.math.upenn.edu/~subhrabh/html_cache/7f068a4d19ed85a15c9e25ecae8b40c1.html). 

## Compilation
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
```sh
$ ./build/test_planner_2d ../data/corridor.yaml
```
The results are plotted in [corridor.jpg](https://github.com/sikang/jps3d/blob/master/data/corridor.jpg).
![Visualization](./data/corridor.jpg)

## Doxygen
More details in https://sikang.github.io/jps3d/index.html

