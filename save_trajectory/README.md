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


Change path in file save_trajectory_node.cpp: file.open("**/home/czarek/autoware/trajectory.txt**"). Before run script, make sure that you have frames: map and base link. You can check it by using command below:
```bash
ros2 run tf2_tools view_frames
```
It will make a .pdf file with all frames. Remember to run simulator before.

### **Record trajectory!**

Run in terminal your simulator,
```bash
./AWSIM_v1.1.F1TENTH_keyboard_control/AWSIM_F1TENTH.x86_64 
```
then in another terminal run 
```bash
ros2 launch f1tenth_launch f1tenth.launch.py map_path:=autoware_map/imola
```
Then in another terminal run:

```bash
ros2 launch save_trajectory save_trajectory.launch.py
```
Use ctr+s when you would like to end recording. Now you can use your save file with trajectory.


# OLD_VERSION START

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

# OLD_VERSION END

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `/tf` | nav_msgs::msg::Odometry | Transform from map to base_link. |
| `/vehicle/status/velocity_status` | autoware_auto_vehicle_msgs::msg::VelocityReport | Sample desc. |
| `/vehicle/status/steering_status` | autoware_auto_vehicle_msgs::msg::SteeringReport | Sample desc. |

### Output

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| Sample | Sample | Sample desc. |



## References / External links
<!-- Optional -->
