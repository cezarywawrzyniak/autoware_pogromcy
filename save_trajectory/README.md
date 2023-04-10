# save_trajectory
<!-- Required -->
<!-- Package description -->

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
colcon build --symlink-install --packages-select save_trajectory
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

 You have to change in src/universe/autoware.universe/control/trajectory_follower_node/launch/simple_trajectory_follower.launch.xml section below:
```
<remap from="input/kinematics" to="/localization/odometry"/>
<remap from="input/trajectory" to="/trajectory"/>
<remap from="input/current_steering" to="/vehicle/status/steering_status"/>
<remap from="output/control_cmd" to="/control/command/control_cmd"/>
```
Then in file src/universe/autoware.universe/control/trajectory_follower_node/src/controller_node.cpp change this section:
```
  sub_ref_path_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "/trajectory", rclcpp::QoS{1}, std::bind(&Controller::onTrajectory, this, _1));
  sub_steering_ = create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1}, std::bind(&Controller::onSteering, this, _1));
  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
    "/localization/odometry", rclcpp::QoS{1}, std::bind(&Controller::onOdometry, this, _1));
  sub_accel_ = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "/input/current_accel", rclcpp::QoS{1}, std::bind(&Controller::onAccel, this, _1));
  sub_operation_mode_ = create_subscription<OperationModeState>(
    "/input/current_operation_mode", rclcpp::QoS{1},
    [this](const OperationModeState::SharedPtr msg) { current_operation_mode_ptr_ = msg; });
  control_cmd_pub_ = create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
```
then build it with command:
```bash
colcon build --symlink-install --packages-select trajectory_follower_node
```


Run in terminal **F1Tenth_v0.5.x86_64**, then in another terminal run 
```bash
ros2 launch f1tenth_launch f1tenth.launch.py map_path:=autoware_map/imola
```
Then in another terminal run:

```bash
ros2 launch save_trajectory save_trajectory.launch.py
```
To record a bag run command:
```bash
ros2 bag record /trajectory
```

### **If you record a bag, try it!**

Run in terminal **F1Tenth_v0.5.x86_64**, then in another terminals run:
```bash
ros2 launch f1tenth_launch f1tenth.launch.py map_path:=autoware_map/imola.
ros2 launch trajectory_follower_node simple_trajectory_follower.launch.xml
ros2 bag play <name_of_your_recorded_package>
```
Check if you car is moving.


## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `/localization/odometry` | nav_msgs::msg::Odometry | Sample desc. |

### Output

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `/trajectory` | autoware_auto_planning_msgs::msg::Trajectory | Sample desc. |

### Services and Actions

| Name           | Type                   | Description  |
| -------------- | ---------------------- | ------------ |
| `service_name` | std_srvs::srv::Trigger | Sample desc. |

### Parameters

| Name         | Type | Description  |
| ------------ | ---- | ------------ |
| `param_name` | int  | Sample desc. |


## References / External links
<!-- Optional -->
