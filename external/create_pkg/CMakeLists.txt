cmake_minimum_required(VERSION 3.14)
project(create_pkg)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
add_compile_options(-Wall -Wextra -pedantic -Werror)

# Testing
if(BUILD_TESTING)
  # Work-around for problem with ament_lint_auto finding template/package.xml
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_cppcheck()
  ament_cpplint()
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_uncrustify()
  ament_xmllint(${CMAKE_CURRENT_SOURCE_DIR}/package.xml)
endif()

# Install python package
ament_python_install_package(${PROJECT_NAME})

# Install main script
install(PROGRAMS ${PROJECT_NAME}/main.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install templates
install(DIRECTORY ${PROJECT_NAME}/template_cpp/
  DESTINATION share/${PROJECT_NAME}/template_cpp/
)

install(DIRECTORY ${PROJECT_NAME}/template_python_cmake/
  DESTINATION share/${PROJECT_NAME}/template_python_cmake/
)

install(DIRECTORY ${PROJECT_NAME}/template_python_setuptools/
  DESTINATION share/${PROJECT_NAME}/template_python_setuptools/
)

install(DIRECTORY ${PROJECT_NAME}/template_launch/
  DESTINATION share/${PROJECT_NAME}/template_launch/
)

ament_auto_package()
