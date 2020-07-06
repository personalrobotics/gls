function(gls_add_test target_name)
  add_executable("${target_name}" ${ARGN})
  add_test("${target_name}" "${target_name}")
  target_link_libraries("${target_name}" gtest gtest_main pthread ${PROJECT_NAME})
  install(TARGETS "${target_name}" RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endfunction()

# Add helper scripts to the include path.
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory(datastructures)
add_subdirectory(event)
