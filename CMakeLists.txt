cmake_minimum_required(VERSION 3.10.2)
project(gls)
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

#================================================================================
# Compiler settings
#================================================================================
# Enforce the use of C++17
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -Wuninitialized -Wpedantic -Werror")

#================================================================================
# Link Libraries
#================================================================================

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem graph program_options system)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

link_libraries(
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/datastructures/PriorityQueue.cpp
)

#================================================================================
# Example Scripts
#================================================================================

# Correctness Test Script
# add_executable(test_ShortestPath_Forward examples/test_ShortestPath_Forward.cpp)
# target_link_libraries(test_ShortestPath_Forward ${PROJECT_NAME} ${OpenCV_LIBS})
# install(TARGETS test_ShortestPath_Forward RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#================================================================================
# Testing
#================================================================================
enable_testing()
add_subdirectory("tests")

#================================================================================
# Installation
#================================================================================

install(TARGETS gls EXPORT glsConfig
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY include/ DESTINATION include)

# This makes the project importable from the install directory
install(EXPORT glsConfig DESTINATION share/gls/cmake)

# This makes the project importable from the build directory
export(TARGETS gls FILE glsConfig.cmake)

# Install the package.xml file (to satisfy REP-136).
install(FILES "package.xml"
  DESTINATION "share/gls")

#================================================================================
# IDE Comfort
#================================================================================

# This custom target doesn't affect building the project but is only for
# displaying the header files in IDEs.
FILE(GLOB_RECURSE LibFiles "include/*.hpp")
add_custom_target(headers SOURCES ${LibFiles})
