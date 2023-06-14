# steer_the_car

## Purpose
This node's puropse is to controll our F1Tenth Car using Pure Pursuit for longitudinal steering and PID for lateral.


## API
First you need to generate a trajectory to follow using planning.py script or save your own using save_trajectory package. Then you can inspect it with show_trajectory.py and change the speeds using change_speed.py.

Package is run using:

```bash
ros2 launch steer_the_car steer_the_car.launch.py 
```

### Input

|               Name               |                     Type                        |      Description     |
| -------------------------------- | ----------------------------------------------- | -------------------- |
|`/vehicle/status/velocity_status` | autoware_auto_vehicle_msgs::msg::VelocityReport |  Current car speed   |
|           TF transform           |     geometry_msgs::msg::TransformStamped        | Current car position |
|           trajectory.txt         |     txt file with 3 values in every line        |    Trajectory file   |


### Output

| Name                           | Type                                                     | Description                             |
| ------------------------------ | -------------------------------------------------------- | --------------------------------------- |
| `/control/command/control_cmd` | autoware_auto_control_msgs::msg::AckermannControlCommand | Steering commands                       |
| `/marker`                      | visualization_msgs::msg::Marker                          | Currently followed point of trajectrory |
| `/marker_list`                 | visualization_msgs::msg::Marker                          | Entire trajectory                       |


## References / External links
<!-- Optional -->
