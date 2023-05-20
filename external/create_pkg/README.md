# create_pkg

This is the copy of [autoware_auto_create_pkg](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/tools/autoware_auto_create_pkg) from [Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto) which is a package creation script that provides default build settings and boilerplate code for use when creating a new ROS 2 package. Several changes have been made toward higher generalization.

## Differences from ROS 2 package creation tools

The below sections describe a couple differences of note between this package creation script and [those one provided by ROS 2](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/#create-your-own-package).

### ament_cmake_auto

The `create_pkg` script sets up packages to use [ament_cmake_auto](https://github.com/ament/ament_cmake/tree/master/ament_cmake_auto) rather than [ament_cmake](https://github.com/ament/ament_cmake/tree/master/ament_cmake).
The `ament_cmake_auto` library contains CMake macros like `ament_auto_add_library` which automate many boilerplate tasks like targeting dependencies.
For example, if you use `ament_auto_find_build_dependencies()`, it will automatically `find_package()` all the build-related dependencies in package.xml.
Then, if you use the `ament_auto_` forms of `add_library` or `add_executable`, it will automatically `ament_target_dependencies()` for all of the found ones.

### Visibility control

The `create_pkg` script gives finer grained control over library linkage via the `visibility_control.hpp` header it includes in each package.

The purpose of visibility control is to get library symbol visibility in a consistent state on different platforms (i.e. Windows uses the equivalent of `-fvisibility=hidden`).
As an example, ROS2 code does this.
This means to get good cross-platform support, we need to prepend i.e. functions with:

```
FOO_PUBLIC int my_func();

class FOO_PUBLIC bar  // all symbols visible by default
{
public:
  void bazzle();  // public visibility

private:
  FOO_LOCAL gaddle();  // LOCAL visibility -> can't be linked to
};
```

This also gives us the ability to mark symbols (classes, functions, methods, etc) as local so that external users can't link to them.
There is also [some argument](https://gcc.gnu.org/wiki/Visibility) that it improves security and linking/loading times.

> NOTE: Default visibility is `LOCAL`, so be sure to mark any classes or functions as `PUBLIC` that you want to be able to link against.

## Usage

To use the tool, run the following:

```
user$ source ~/ros2_ws/install/setup.bash
user$ cd ~/ros2_ws/src/<path_to_folder_containing_your_new_package>
user$ ros2 run create_pkg main.py --pkg-name PKG_NAME --maintainer MAINTAINER --email EMAIL --description DESCRIPTION --destination . --type cpp
```

In the above commands, replace `<path_to_folder_containing_your_new_package>` with the path to the folder where you want to create your package.
Alternatively, add the `--destination <path_to_folder_containing_your_new_package>` argument to generate the package in a different location than the current working directory. You can choose package type using `type` argument (cpp or python). Cpp is default.
Also replace the flag values in ALL_CAPS with appropriate values for your new package.
Once your new package has been created, you will need to clean up the specifics (license, dependencies, design document, etc.).

To obtain more details on the command-line usage, call:

```
user$ ros run create_pkg main.py --help
```

Script allows create different package types:
* cpp - ament_cmake_auto package
* python_cmake - ament_cmake_python package
* python_setuptools - setuptools package (symlinks not supported, thus you shouldn't use --symlink-install flag in colcon build command)
* launch - ament_cmake_auto (portable package for launch files and configuration files)

Once the package has been created with for example `--pkg-name foo`, build and test it.

```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-select foo
colcon test --packages-select foo && colcon test-result --verbose
```

Then follow these steps to run the node from a launch file:

```
source install/setup.bash
ros2 launch foo foo.launch.py
```

The output should be similar to:

```
[component_container-1] Hello World, 789
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/foo_node' in container '/foo_container'
```

Alternatively run the node executable directly:

```
source install/setup.bash
ros2 run foo foo_node_exe
```

## References / External links
<!-- Optional -->

- Basic instructions for creating a new ROS 2 package can be found [in this tutorial](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/#create-your-own-package).
- Basic instructions regarding [ROS2 composition](https://index.ros.org/doc/ros2/Tutorials/Composition/)
