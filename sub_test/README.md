# sub_test

## Installation

```
colcon build --symlink-install --packages-select sub_test
```


## API

Run F1TENTH simulator and Rviz2. 
```
ros2 run sub_test sub_test.launch.py
``` 
Add point by topic in Rviz2 to display.

### Input

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `/sensing/lidar/scan` | sensor_msgs::msg::LaserScan | Lidar |
| `/localization/odometry` | nav_msgs::msg::Odometry | Odometry |

### Output

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `/planning/point` | geometry_msgs::msg::PoseStamped | Local goal point |



