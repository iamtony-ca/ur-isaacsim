# ur_bringup

One ROS2 (Jazzy) bringup for a **UR16e** that runs against **Isaac Sim 5.1.0**
or the **real robot**, sharing the same ros2_control + MoveIt2 interface. This is
the project's only hand-written package.

> **Three independent rigs** share this package, each in its own files; higher
> sets reuse but never edit lower-set / shared files:
> **Set 1** UR16e only (`ur16e*`) ·
> **Set 2** + Robotiq 2F-85 via the GRP-ES-CPL-077 coupling (`ur16e_2f85*`) ·
> **Set 3** + RealSense D405 on a PickNik bracket (`ur16e_2f85_d405*`).
>
> Current overview + run commands: [`../README.md`](../README.md) ·
> real HW bring-up: [`../HARDWARE.md`](../HARDWARE.md) ·
> background / history (incl. Set 2 §8, Set 3 §9): [`../HISTORY.md`](../HISTORY.md).

```
            app / MoveIt2  (identical)
                   │  follow_joint_trajectory
            scaled_joint_trajectory_controller
                   │ ros2_control
        ┌──────────┴───────────┐
 use_sim:=true            use_sim:=false
 topic_based_ros2_control   ur_robot_driver (RTDE)
        │ topics                  │ URCap
   Isaac Sim 5.1.0            real UR16e
```

The gripper (sim `topic_based` ↔ real `robotiq_driver`) and camera (sim Isaac ↔
real `realsense2_camera`) swap backend the same way, with **identical topics/frames**
so MoveIt and any perception node are written once and work in both modes.

## Folder convention

`launch` / `config` / `urdf` are split into **per-set subfolders**
(`ur16e/`, `ur16e_2f85/`, `ur16e_2f85_d405/`); files shared by more than one set
live in **`common/`** (e.g. `urdf/common/robotiq_2f85_macro.xacro`,
`config/common/ur16e_2f85_controllers.yaml`, `srdf/common/ur16e_2f85.srdf.xacro`).
`isaac/` mirrors this (`common`, `ur16e_2f85`, `ur16e_2f85_d405`; shared USD in
`isaac/assets`). `ros2 launch ur_bringup <file>` finds launch files by name
regardless of subfolder, so run commands need no path. Full tree: [`../README.md`](../README.md) §3.

## Run (quick)

```bash
# sim (Isaac running its ROS2-bridge graph first — see isaac/README.md)
ros2 launch ur_bringup ur16e.launch.py use_sim:=true                 # set 1
ros2 launch ur_bringup ur16e_2f85.launch.py                          # set 2 (+gripper)
ros2 launch ur_bringup ur16e_2f85_d405.launch.py                     # set 3 (+camera)

# real
ros2 launch ur_bringup ur16e.launch.py use_sim:=false robot_ip:=<ip>            # set 1
ros2 launch ur_bringup ur16e_2f85_real.launch.py robot_ip:=<ip>                 # set 2
ros2 launch ur_bringup ur16e_2f85_d405_real.launch.py robot_ip:=<ip>           # set 3
# real-path DRY RUN, no hardware:
ros2 launch ur_bringup ur16e.launch.py use_sim:=false use_mock_hardware:=true
```

Per-set MoveIt: `ur16e_moveit` / `ur16e_2f85_moveit` / `ur16e_2f85_d405_moveit`
(last one adds depth→OctoMap). The **real** path delegates to upstream
`ur_robot_driver/ur_control.launch.py`, so it stays maintained by Universal Robots.
Step-by-step real bring-up (network, URCap, calibration, tool I/O, hand-eye) is in
[`../HARDWARE.md`](../HARDWARE.md).

> **Sim time:** the sim path defaults to `use_sim_time:=true` and expects Isaac to
> publish `/clock`; without it controllers never activate (`Switch controller timed
> out`). To smoke-test the control stack without Isaac: add `use_sim_time:=false`.

## IMPORTANT: topic_based hardware version (sim backend)

The sim backend uses `joint_state_topic_hardware_interface` from
`src/topic_based_hardware_interfaces`, pinned to **tag `0.2.1`** in `../ur16e.repos`
(`vcs import`). Do **not** use `main`/1.x: their `set_state(name,value)` API does not
update the exported state interfaces under apt `ros-jazzy` ros2_control 4.44.0, so
`/joint_states` comes out all `NaN`, MoveIt has no state, and RViz segfaults on NaN
poses. Tag `0.2.1` uses the classic `export_state_interfaces()` and works.
(`-DBUILD_TESTING=OFF` skips its `ros_testing` build dep.)

## Dependencies

- **apt (`ros-jazzy-*`)**: `ur` (ur_robot_driver, ur_description, ur_moveit_config,
  ur_controllers), `moveit`, `moveit-ros-perception` (Set 3 octomap), `ros2-control`,
  `ros2-controllers`, `robotiq-description`; **Set 3 real camera**:
  `realsense2-camera`, `librealsense2` (+ `diagnostic-updater`/`diagnostic-msgs` for
  realsense ABI parity).
- **source (vcstool, `../ur16e.repos`)**: `topic_based_hardware_interfaces` @ 0.2.1
  (sim backend), `ros2_robotiq_gripper` (`robotiq_driver`/`robotiq_controllers`) +
  `serial` (real 2F-85 gripper).
