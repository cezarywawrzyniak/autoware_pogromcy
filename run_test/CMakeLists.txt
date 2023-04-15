cmake_minimum_required(VERSION 3.14)
project(run_test)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)
add_compile_options(-Wall -Wextra -pedantic -Werror)

set(RUN_TEST_LIB_SRC
  src/run_test.cpp
)

set(RUN_TEST_LIB_HEADERS
  include/run_test/run_test.hpp
  include/run_test/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${RUN_TEST_LIB_SRC}
  ${RUN_TEST_LIB_HEADERS}
)

set(RUN_TEST_NODE_SRC
  src/run_test_node.cpp
)

set(RUN_TEST_NODE_HEADERS
  include/run_test/run_test_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${RUN_TEST_NODE_SRC}
  ${RUN_TEST_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "run_test::RunTestNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_run_test.cpp)
  set(TEST_RUN_TEST_EXE test_run_test)
  ament_add_gtest(${TEST_RUN_TEST_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_RUN_TEST_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
