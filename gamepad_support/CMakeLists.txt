cmake_minimum_required(VERSION 3.14)
project(gamepad_support)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)
add_compile_options(-Wall -Wextra -pedantic -Werror)

set(GAMEPAD_SUPPORT_LIB_SRC
  src/gamepad_support.cpp
)

set(GAMEPAD_SUPPORT_LIB_HEADERS
  include/gamepad_support/gamepad_support.hpp
  include/gamepad_support/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${GAMEPAD_SUPPORT_LIB_SRC}
  ${GAMEPAD_SUPPORT_LIB_HEADERS}
)

set(GAMEPAD_SUPPORT_NODE_SRC
  src/gamepad_support_node.cpp
)

set(GAMEPAD_SUPPORT_NODE_HEADERS
  include/gamepad_support/gamepad_support_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${GAMEPAD_SUPPORT_NODE_SRC}
  ${GAMEPAD_SUPPORT_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "gamepad_support::GamepadSupportNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_gamepad_support.cpp)
  set(TEST_GAMEPAD_SUPPORT_EXE test_gamepad_support)
  ament_add_gtest(${TEST_GAMEPAD_SUPPORT_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_GAMEPAD_SUPPORT_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
