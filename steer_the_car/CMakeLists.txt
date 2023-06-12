cmake_minimum_required(VERSION 3.14)
project(steer_the_car)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)
add_compile_options(-Wall -Wextra -pedantic -Werror)

set(STEER_THE_CAR_LIB_SRC
  src/steer_the_car.cpp
)

set(STEER_THE_CAR_LIB_HEADERS
  include/steer_the_car/steer_the_car.hpp
  include/steer_the_car/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${STEER_THE_CAR_LIB_SRC}
  ${STEER_THE_CAR_LIB_HEADERS}
)

set(STEER_THE_CAR_NODE_SRC
  src/steer_the_car_node.cpp
)

set(STEER_THE_CAR_NODE_HEADERS
  include/steer_the_car/steer_the_car_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${STEER_THE_CAR_NODE_SRC}
  ${STEER_THE_CAR_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "steer_the_car::SteerTheCarNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_steer_the_car.cpp)
  set(TEST_STEER_THE_CAR_EXE test_steer_the_car)
  ament_add_gtest(${TEST_STEER_THE_CAR_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_STEER_THE_CAR_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
