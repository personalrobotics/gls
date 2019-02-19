# generalized_lazy_search

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

./test2d_image -s (sourcex, sourcey) -t (targetx, targety) -o (obstacle_location) -g (graph_location)

The source and target locations are assumed to be in [0, 1]