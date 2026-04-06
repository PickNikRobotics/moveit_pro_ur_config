# moveit_pro_ur_config

This is a [MoveIt Pro](https://picknik.ai/pro) robot configuration for UR e-series robots.

Refer to the [UR Hardware Setup Guide](https://docs.picknik.ai/hardware_guides/ur5e_hardware_setup_guide) for installation.

### Build without hardware
```
moveit_pro build --colcon-args "--packages-ignore zed_ros2 zed_components zed_wrapper"

# OR

colcon build --packages-ignore zed_ros2 zed_components zed_wrapper
```