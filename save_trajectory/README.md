# save_trajectory
<!-- Required -->
<!-- Package description -->
Package was created to subscribe topics, and create trajectory msg, and then record it to rosbag. Points are saving every 500ms.

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

### **Record trajectory!**

Run in terminal **F1Tenth_v0.?.x86_64**, then in another terminal run 
```bash
ros2 launch f1tenth_launch f1tenth.launch.py map_path:=autoware_map/imola
```
Then in another terminal run:

```bash
ros2 launch save_trajectory save_trajectory.launch.py
```


### **If you record a bag, try it!**

Build package with controller:
```bash
colcon build --symlink-install --packages-select trajectory_follower_node_2
```

Run in terminal **F1Tenth_v0.?.x86_64**, then in another terminals run:
```bash
ros2 launch f1tenth_launch f1tenth.launch.py map_path:=autoware_map/imola.
ros2 launch trajectory_follower_node_2 simple_trajectory_follower.launch.xml
ros2 bag play my_bag/
```
Check if you car is moving.

If you want to change some parameters of your controller, fell free to make changes in /external/trajectory_follower_node_2/src/controller_node.cpp.

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `/localization/odometry` | nav_msgs::msg::Odometry | Sample desc. |
| `/vehicle/status/velocity_status` | autoware_auto_vehicle_msgs::msg::VelocityReport | Sample desc. |
| `/vehicle/status/steering_status` | autoware_auto_vehicle_msgs::msg::SteeringReport | Sample desc. |

### Output

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `/trajectory` | autoware_auto_planning_msgs::msg::Trajectory | Sample desc. |



## References / External links
<!-- Optional -->
