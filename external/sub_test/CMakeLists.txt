cmake_minimum_required(VERSION 3.14)
project(sub_test)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)
add_compile_options(-Wall -Wextra -pedantic -Werror)

set(SUB_TEST_LIB_SRC
  src/sub_test.cpp
)

set(SUB_TEST_LIB_HEADERS
  include/sub_test/sub_test.hpp
  include/sub_test/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${SUB_TEST_LIB_SRC}
  ${SUB_TEST_LIB_HEADERS}
)

set(SUB_TEST_NODE_SRC
  src/sub_test_node.cpp
)

set(SUB_TEST_NODE_HEADERS
  include/sub_test/sub_test_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${SUB_TEST_NODE_SRC}
  ${SUB_TEST_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "sub_test::SubTestNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_sub_test.cpp)
  set(TEST_SUB_TEST_EXE test_sub_test)
  ament_add_gtest(${TEST_SUB_TEST_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_SUB_TEST_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
