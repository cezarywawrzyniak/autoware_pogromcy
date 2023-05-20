cmake_minimum_required(VERSION 3.14)
project(hello_world)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
