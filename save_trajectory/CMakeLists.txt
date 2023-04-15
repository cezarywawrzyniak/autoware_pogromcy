cmake_minimum_required(VERSION 3.14)
project(save_trajectory)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)
add_compile_options(-Wall -Wextra -pedantic -Werror)

set(SAVE_TRAJECTORY_LIB_SRC
  src/save_trajectory.cpp
)

set(SAVE_TRAJECTORY_LIB_HEADERS
  include/save_trajectory/save_trajectory.hpp
  include/save_trajectory/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${SAVE_TRAJECTORY_LIB_SRC}
  ${SAVE_TRAJECTORY_LIB_HEADERS}
)

set(SAVE_TRAJECTORY_NODE_SRC
  src/save_trajectory_node.cpp
)

set(SAVE_TRAJECTORY_NODE_HEADERS
  include/save_trajectory/save_trajectory_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${SAVE_TRAJECTORY_NODE_SRC}
  ${SAVE_TRAJECTORY_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "save_trajectory::SaveTrajectoryNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_save_trajectory.cpp)
  set(TEST_SAVE_TRAJECTORY_EXE test_save_trajectory)
  ament_add_gtest(${TEST_SAVE_TRAJECTORY_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_SAVE_TRAJECTORY_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
