cmake_minimum_required(VERSION 2.8.3)
project(generalized_lazy_search)

#================================================================================
# Compiler settings
#================================================================================
# Enforce the use of C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_compile_options(-Wall -Wextra -Wpedantic)
if(GENERALIZED_LAZY_SEARCH_TREAT_WARNINGS_AS_ERRORS)
  add_compile_options(-Werror)
endif()

#================================================================================
# Link Libraries
#================================================================================

find_package(catkin REQUIRED
  COMPONENTS
  cmake_modules)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem graph program_options system)

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
  src/datastructures/Graph.cpp
  src/datastructures/SearchQueue.cpp
  src/datastructures/State.cpp
  src/GLS.cpp
  src/event/Event.cpp
  src/event/ShortestPathEvent.cpp
  src/selector/Selector.cpp
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#================================================================================
# Formatting
#================================================================================

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format-3.8)

if(CLANG_FORMAT_EXECUTABLE)
  message(STATUS "Found clang-format.")

  file(GLOB_RECURSE ALL_SOURCE_FILES
    LIST_DIRECTORIES false
    include/*.h include/*.hpp src/*.c src/*.cpp tests/*.cpp tests/*.hpp)
  list(LENGTH ALL_SOURCE_FILES NUM_SOURCE_FILES)

  add_custom_target(format
    COMMAND ${CMAKE_COMMAND} -E echo "Formatting ${NUM_SOURCE_FILES} files..."
    COMMAND ${CLANG_FORMAT_EXECUTABLE} -style=file -i ${ALL_SOURCE_FILES}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    DEPENDS ${CLANG_FORMAT_EXECUTABLE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  add_custom_target(check-format
    COMMAND ${CMAKE_COMMAND} -E echo "Checking code style of"
            "${NUM_SOURCE_FILES} files... "
    COMMAND ${CMAKE_SOURCE_DIR}/tools/check_format.sh
            ${CLANG_FORMAT_EXECUTABLE} ${ALL_SOURCE_FILES}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    DEPENDS ${CLANG_FORMAT_EXECUTABLE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
else()
  message(STATUS "Looking for clang-format - NOT found, please install "
      "clang-format to enable automatic code formatting.")
endif()

#================================================================================
# IDE Comfort
#================================================================================

# This custom target doesn't affect building the project but is only for
# displaying the header files in IDEs.
FILE(GLOB_RECURSE LibFiles "include/*.hpp")
add_custom_target(headers SOURCES ${LibFiles})
