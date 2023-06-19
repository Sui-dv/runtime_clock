# runtime_clock
Simple ROS2 Foxy system clock node. Publish builtin_interfaces/msg/Time at fixed interval. Set the frame id and frequency in the config .yaml file. 
```
ros2 run runtime_clock runtime_clock --ros-args --params-file <PATH_TO_YAML_FILE> 
```
