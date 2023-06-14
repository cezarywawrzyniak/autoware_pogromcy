# save_trajectory
<!-- Required -->
<!-- Package description -->
Package was created to subscribe topics, and create trajectory msg, and then record it to rosbag. Points are saving every 100ms.

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
Use ctr+s when you would like to end recording. Congrats, you saved your trajectory in trajectory.txt. If you would like to use .json file move you trajectory.txt to script. Then you have to run script that is placed **script/txt_to_json.py**.

Now you can use your save file with trajectory. 

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

| Name           | Type                                 | Description     |
| -------------- | ------------------------------------ | --------------- |
| trajectory.txt | txt file with 3 values in every line | Trajectory file |



## References / External links
<!-- Optional -->
