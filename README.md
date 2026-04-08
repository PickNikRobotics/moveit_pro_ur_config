# moveit_pro_ur_config

This is a [MoveIt Pro](https://picknik.ai/pro) robot configuration for UR e-series robots.

Refer to the [UR Hardware Setup Guide](https://docs.picknik.ai/hardware_guides/ur5e_hardware_setup_guide) for installation.

### Build without hardware
```
moveit_pro build --colcon-args "--packages-ignore zed_ros2 zed_components zed_wrapper"

# OR

colcon build --packages-ignore zed_ros2 zed_components zed_wrapper
```

---

## Hardware Launch & Behavior Documentation

### 1. Launching the System

The RTPC (real-time PC) runs **version 9.0**. On the agent PC, MoveIt Pro is currently on the **main** branch.

Because the UR drivers are launched on the RTPC, the agent PC uses the **developer workflow** — launch via `agent_bridge.app` rather than bringing up drivers locally.

### 2. Calibration

Calibration establishes the positions of the shelf, bin, and scene camera in the planning scene. The procedure has three steps:

1. **Detect April Tags** — Teleop the robot to each shelf/bin fiducial and run the *Detect April Tag Here* objective for each of the 6 tags (IDs 0–4 for the shelf and bin).

2. **Calibrate Scene Camera** — Place AprilTag ID 5 in the bin where both the wrist camera and the scene camera can see it. Position the wrist camera so it has a clear view of the tag, then run the *Calibrate Scene Camera* objective. This computes and saves the scene camera pose via cross-camera tag detection.

3. **Calibrate Detected April Tags** — Run the *Calibrate Detected April Tags* objective. It uses the saved tag poses from step 1 to compute shelf, bin, table, and pole positions, and loads the scene camera pose from step 2. Everything is placed into the planning scene.

> **After a restart (nothing moved):** Simply run *Load Calibrated Scene* to reload all poses from the saved YAML files without re-running the full calibration.

### 3. Qualcomm Pick Pringles v3

*Qualcomm Pick Pringles v3* is the objective currently used to pick Pringles cans from the bin and shelve them. The behavior tree runs through the following steps:

1. **Cleanup** — Detach any previously held grocery items and deactivate the vacuum gripper.
2. **Load Placement Zones** — Load the top and bottom shelf target poses from YAML config files.
3. **Move to Bin & Capture** — Move to the "Above Grocery Bin" waypoint and capture an image and point cloud from the scene camera.
4. **Segment Pringles (SAM3)** — Use the SAM3 vision model to segment Pringles cans from the scene camera image using an exemplar image.
5. **Point Cloud Processing** — Transform the point cloud to the world frame and extract the 3D mask for the detected object.
6. **Fit Cylinder** — Fit a cylinder primitive to the segmented point cloud to model the Pringles can.
7. **Compute Pick Pose** — Derive the object pose from the fitted cylinder and compute a top-down pick pose for the vacuum gripper.
8. **Plan & Execute Pick (MTC)** — Use MoveIt Task Constructor to plan and execute the approach and grasp motion.
9. **Activate Vacuum & Nudge** — Activate the vacuum and nudge downward (up to 3 times, 1 cm per nudge) until the object is detected via pressure feedback.
10. **Retract from Bin** — Retract 20 cm upward above the bin.
11. **Attach Collision Object** — Attach the picked item as a collision object on the gripper for planning.
12. **Move to Shelf Staging** — Navigate to the appropriate staging waypoint (top or bottom shelf) based on the target placement zone.
13. **Compute & Execute Place** — Compute the correct upright orientation for placement and plan/execute the place trajectory.
14. **Force-Lower Until Contact** — Switch to the velocity-force controller and lower the object until contact with the shelf surface is detected.
15. **Release & Retract** — Deactivate the vacuum, detach the collision object, and retract upward.
