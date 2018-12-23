set(GENERALIZED_LAZY_SERACH_TEST_RESOURCES_PATH "${PROJECT_SOURCE_DIR}/tests/resources/")
configure_file(
  "${CMAKE_SOURCE_DIR}/tests/config.hpp.in"
  "${CMAKE_BINARY_DIR}/tests/config.hpp"
  @ONLY
)
include_directories("${CMAKE_SOURCE_DIR}/tests/")
include_directories("${CMAKE_BINARY_DIR}/tests/")

generalized_lazy_search_add_test(test_SearchQueue test_SearchQueue.cpp)
target_link_libraries(test_SearchQueue "${PROJECT_NAME}")