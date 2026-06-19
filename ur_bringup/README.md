# ur_bringup

One ROS2 (Jazzy) bringup for a **UR16e** that runs against **Isaac Sim 5.1.0**
or the **real robot**, sharing the same ros2_control + MoveIt2 interface.

> **Three independent rigs** share this package (each in its own files, higher
> sets reuse but never edit lower-set files):
> **Set 1** UR16e only (`ur16e*`) — documented below ·
> **Set 2** + Robotiq 2F-85 via the GRP-ES-CPL-077 coupling (`ur16e_2f85*`) ·
> **Set 3** + RealSense D405 on a PickNik bracket (`ur16e_2f85_d405*`).
> Sets 2/3 are covered in the top-level [`../README.md`](../README.md) §8 / §9.

```
            app / MoveIt2  (identical)
                   │  follow_joint_trajectory
            joint_trajectory_controller
                   │ ros2_control
        ┌──────────┴───────────┐
 use_sim:=true            use_sim:=false
 topic_based_ros2_control   ur_robot_driver (RTDE)
        │ topics                  │ URCap
   Isaac Sim 5.1.0            real UR16e
```

## Run

```bash
# Simulation (Isaac Sim must be running its ROS2-bridge graph — see isaac/README.md)
ros2 launch ur_bringup ur16e.launch.py use_sim:=true

# Real robot
ros2 launch ur_bringup ur16e.launch.py use_sim:=false robot_ip:=<ur16e_ip>

# Real-driver DRY RUN (no physical robot): ur_robot_driver with mock hardware
ros2 launch ur_bringup ur16e.launch.py use_sim:=false use_mock_hardware:=true
```

### Real UR16e prerequisites (use_sim:=false, real hardware)
Verified the launch path works end-to-end via `use_mock_hardware:=true`
(ur_robot_driver 3.7.0 → full UR controller set active → MoveIt plan+execute OK).
To connect the physical robot:
1. **Network**: PC and UR on same subnet; `ping <robot_ip>` must succeed. Pass `robot_ip:=<ip>`.
2. **External Control URCap** installed on the UR; either run an External Control
   program on the pendant, or launch with `headless_mode:=true` (robot must allow
   remote control / be in Remote mode).
3. **Calibration** (recommended): extract the robot's kinematics with
   `ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=<ip> target_filename:=...`
   for accurate TCP poses.
4. Real path uses the genuine `ur_controllers/ScaledJointTrajectoryController`
   (same `follow_joint_trajectory` action MoveIt uses in sim → identical app layer).

> **Sim time:** the sim path defaults to `use_sim_time:=true` and expects Isaac
> to publish `/clock`. Without it the controller_manager update loop is frozen
> and controllers never activate (`Switch controller timed out`). To smoke-test
> the control stack *without* Isaac running, override it:
> `ros2 launch ur_bringup ur16e.launch.py use_sim:=true use_sim_time:=false`
> then `ros2 control list_controllers` should show both controllers `active`.

Both expose `joint_trajectory_controller/follow_joint_trajectory`, so MoveIt and
any higher-level node are written once and work in both modes.

## Layout
- `launch/ur16e/ur16e.launch.py` — the `use_sim` dispatcher.
- `config/ur16e/ur16e_controllers.yaml` — controllers for the sim/mock path.
- `urdf/ur16e/ur16e_sim*.xacro` — UR16e description + `topic_based` ros2_control (sim).
- `isaac/README.md` — Isaac Sim OmniGraph (ROS2 bridge) wiring + topics.

The **real** path delegates to the upstream `ur_robot_driver/ur_control.launch.py`,
so it stays maintained by Universal Robots.

## IMPORTANT: topic_based hardware version (sim backend)

The sim backend uses `joint_state_topic_hardware_interface` cloned in
`src/topic_based_hardware_interfaces`. **Use tag `0.2.1`**, not `main`:

```bash
cd src/topic_based_hardware_interfaces && git checkout 0.2.1
cd /isaac-sim/ur_ws && colcon build --symlink-install \
    --packages-select joint_state_topic_hardware_interface --cmake-args -DBUILD_TESTING=OFF
```

Why: `main`/1.0.0/1.1.0 use the new ros2_control `set_state(name,value)` API, which
does **not** update the exported state interfaces under the apt `ros-jazzy`
ros2_control 4.44.0 binary — so `/joint_states` comes out all `NaN`, MoveIt has
no robot state, and RViz segfaults rendering NaN poses. Tag `0.2.1` uses the
classic explicit-vector `export_state_interfaces()` and works correctly.
(`-DBUILD_TESTING=OFF` skips its `ros_testing` build dep.)

## Depends on (apt, ros-jazzy-*)
`ur` (ur_robot_driver, ur_description, ur_moveit_config, ur_controllers),
`moveit`, `topic_based_ros2_control`, `ros2_control`, `ros2_controllers`.
