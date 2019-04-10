# generalized_lazy_search

##### Under development for improvements.

Framework implements various lazy search algorithms. The planners have been implemented in OMPL.

Dependencies:
- C++11 or higher
- cmake
- OMPL
- Boost Graph Library

The CMakeLists.txt file supports catkin tools. Once you have created and initialized your workspace, 
you should be able to build the package by running `catkin build gls`.

The planner implemented under GLS return the shortest path on the roadmap graph it is planning on.

------

Example:

An example graph and an environment have been provided to run with `examples/test2d_image.cpp`.

```
rosrun gls test2d_image -s <source_x> <source_y> -t <target_x> <target_y> -o <obstacle_file.png> -g <graph.graphml>
```

* The source and target locations are assumed to be in [0, 1]
* The obstacle file is for visualization purposes, and should correspond with the GraphML file.
* A short-cut to the default graph and obstacle files in the `examples` folder can be run with the following launch file:
```
roslaunch gls test.launch
```
Parameters: `graph`, `obstacles`, `start_x`, `start_y`, `target_x`, `target_y`
