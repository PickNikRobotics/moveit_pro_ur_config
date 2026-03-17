# moveit_pro_ur_config

This is a [MoveIt Pro](https://picknik.ai/pro) robot configuration for UR e-series robots.

Refer to the [UR Hardware Setup Guide](https://docs.picknik.ai/hardware_guides/ur5e_hardware_setup_guide) for installation.

### Build
```
moveit_pro build --colcon-args "--packages-up-to picknik_009_ur5e_config"
```

### Package Hierarchy
```
ur_mock_config -> ur_hw_config
picknik_ur_base_config -> picknik_ur_site_config -> pick_009_ur5e_config
                                                 -> qualcomm_config
```