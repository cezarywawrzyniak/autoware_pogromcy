# gamepad_support

## Purpose

This node's puropse is to use controller (Dualshock4) to steer the car inside the simulation.

## API

First ros2 joy node needs to be launched by:

    ros2 run joy joy_node

(it makes it possible to read controller inputs).
Then this package can be launched by:

    ros2 launch gamepad_support gamepad_support.launch.py

### Input

| Name                     | Type                    | Description     |
| ------------------------ | ----------------------- | --------------- |
| `/localization/odometry` | nav_msgs::msg::Odometry | Car's odometry  |
| `/joy`                   | sensor_msgs::msg::Joy   | Joystick inputs |

### Output

| Name                           | Type                                                    | Description       |
| ------------------------------ | ------------------------------------------------------- | ----------------- |
| `/control/command/control_cmd` | autoware_auto_control_msgs::msg::AckermannControlCommand | Steering commands |

### Services

| Name           | Type                   | Description  |
| -------------- | ---------------------- | ------------ |
| `service_name` | std_srvs::srv::Trigger | Sample desc. |

### Parameters

| Name                    | Type | Description  |
| ----------------------- | ---- | ------------ |
| `param_name`            | int  | Sample desc. |

## References / External links
<!-- Optional -->

