# run_test
<!-- Required -->
<!-- Package description -->

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->
```bash
colcon build --symlink-install --packages-select run_test
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->
    
```bash
cd F1Tenth_v0.5
./F1Tenth_v0.5.x86_64
```
New terminal:
```bash
ros2 launch f1tenth_launch f1tenth.launch.py map_path:=autoware_map/imola
```
New terminal:
```bash
ros2 launch run_test run_test.launch.py
```

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Output

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `/control/command/control_cmd` | autoware_auto_control_msgs::msg::AckermannControlCommand | Sample desc. |



## References / External links
<!-- Optional -->
