# Isaac Sim 5.1.0 side setup (ROS2 bridge) for UR16e

The `use_sim:=true` path of `ur16e.launch.py` runs ros2_control with the
`topic_based_ros2_control` hardware. It expects Isaac Sim to be the "robot":

| Direction | Topic (default) | Type | Isaac OmniGraph node |
|---|---|---|---|
| control → Isaac | `/isaac_joint_commands` | `sensor_msgs/JointState` (position) | **ROS2 Subscribe Joint State** → **Articulation Controller** |
| Isaac → control | `/isaac_joint_states` | `sensor_msgs/JointState` | **ROS2 Publish Joint State** |
| Isaac → ROS | `/clock` | `rosgraph_msgs/Clock` | **ROS2 Publish Clock** (run ROS with `use_sim_time:=true`) |

`joint_state_broadcaster` still publishes the canonical `/joint_states` for
RViz/MoveIt, so the Isaac topics are deliberately named differently to avoid a
publisher clash.

## 1. Get a UR16e USD
Isaac ships UR10; for UR16e import the official description:
```
ros2 run ur_description ...   # or:
xacro $(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro \
      ur_type:=ur16e name:=ur16e > /tmp/ur16e.urdf
```
Then in Isaac Sim: **Isaac Utils → Import URDF**, fixed base, joint drive =
Position, import → save as USD.

## 2. Build the OmniGraph (Action Graph)
Add nodes and wire them:
- `On Playback Tick` → drives the graph each physics step.
- `Isaac Read Simulation Time` → feeds Publish Clock + JointState timestamps.
- **ROS2 Context** (domain id must match `ROS_DOMAIN_ID`).
- **ROS2 Publish Clock** ← sim time. Topic `/clock`.
- **ROS2 Publish Joint State**: targetPrim = the UR16e articulation root, topic
  `/isaac_joint_states`.
- **ROS2 Subscribe Joint State**: topic `/isaac_joint_commands` → wire its
  `positionCommand`/`jointNames` into an **Articulation Controller** node whose
  targetPrim is the UR16e articulation root.

The articulation's drive mode must be **Position** with stiffness high enough to
track (e.g. stiffness ~1e7, damping ~1e5 for UR-scale joints — tune).

## 3. Match joint order
`topic_based_ros2_control` matches by joint name, so order is not critical, but
the 6 names must exactly be:
`shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint`.

## 4. Run order
```
# terminal A: Isaac Sim with the graph, press Play (start simulating + publishing)
# terminal B:
ros2 launch ur_bringup ur16e.launch.py use_sim:=true
ros2 launch ur_bringup ur16e_moveit.launch.py use_sim:=true   # optional, for MoveIt+RViz
```
Verify: `ros2 topic echo /isaac_joint_states` shows motion, and
`ros2 control list_controllers` shows both controllers `active`.

## GUI mode (watch the robot in Isaac's viewport)
- Run **without `--headless`** AND **without `--no-env`**:
  `/isaac-sim/python.sh .../ur16e_isaac_ros2.py`  (loads Simple_Room).
  `--no-env` skips the environment, leaving the scene with **no lights** → the
  RTX viewport is **all black** even though physics/ROS work. Always load the
  env (or add a dome light) for GUI.
- **Startup order matters.** Bring the stack up in this order so MoveIt's
  current-state monitor never caches NaN: (1) Isaac, wait until
  `/isaac_joint_states` is steady, (2) `ur16e.launch.py`, wait until
  `/joint_states` is valid, (3) `ur16e_moveit.launch.py` (move_group + RViz).
  If you restart Isaac, also restart move_group/RViz afterward — otherwise they
  hold the NaN state from the gap and planning fails with error_code -4.

## Troubleshooting (verified findings)
- **`Prim /UR16e is not an articulation`** — the Isaac UR16e USD has no default
  prim and puts `ArticulationRootAPI` on the fixed base joint, so the OmniGraph
  ArticulationController/PublishJointState must target **`/UR16e/root_joint`**,
  not `/UR16e`. The standalone script handles this via `--articulation-root`.
- **`The requested state interface not found: '<joint>/effort'`** — Isaac's
  PublishJointState emits position+velocity+effort, so the sim ros2_control
  xacro must declare an `effort` state interface per joint (it does).
- **`Switch controller timed out after 5 seconds`** — the controller_manager
  update loop is driven by sim time. It means `/clock` is absent or advancing
  too slowly. Either Isaac is not running/playing, or the GPU is saturated (e.g.
  by another job) so sim steps slowly. Confirm with `ros2 topic hz /clock`. To
  test the control stack without Isaac, use `use_sim_time:=false`.

## Scripted (recommended): ur16e_isaac_ros2.py
This folder ships `ur16e_isaac_ros2.py`, which loads the UR16e and builds the
whole graph programmatically (no manual GUI wiring):
```
/isaac-sim/python.sh \
    /isaac-sim/ur_ws/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py [--headless] [--no-env]
```
Verified: it publishes `/isaac_joint_states` with the correct UR joint names,
`/clock`, and gravity-loaded efforts (physics running), and accepts commands on
`/isaac_joint_commands`.

Flags for the gripper / camera rigs (always pass an **absolute** `--asset-path` —
a relative path is treated as Isaac-asset-server-relative and the robot fails to
load, leaving only the camera/gripper visible):
- `--asset-path <usd>` — load a composed scene built by `build_ur16e_2f85.py`:
  - **Set 2**: `assets/ur16e_with_2f85.usd` — UR16e + GRP-ES-CPL-077 coupling
    (+11 mm) + 2F-85, single articulation (coupling visual + standoff baked in so
    the Isaac EE matches the URDF/RViz).
  - **Set 3**: `assets/ur16e_2f85_d405.usd` — Set 2 plus the PickNik camera
    bracket (flush on the flange) + coupling on top (+7 mm) + gripper (+18 mm),
    all baked. Use this with `--with-camera`.
- `--with-camera` — **Set 3 only**: parents an eye-in-hand D405 `Camera` (+ a
  42×42×23 mm body box) under `wrist_3_link`, seated on the bracket cradle
  (pitch 8°, flush), and builds a second OmniGraph (`IsaacCreateRenderProduct` →
  `ROS2CameraHelper` rgb/depth/depth_pcl + `ROS2CameraInfoHelper`) publishing on
  realsense2_camera-standard topics: `/camera/color/image_raw` (rgb8),
  `/camera/depth/image_rect_raw` (32FC1), `/camera/depth/color/points`,
  `/camera/{color,depth}/camera_info` (HFOV≈87°, 640×480). frameIds
  `camera_{color,depth}_optical_frame` match the URDF in `ur16e_2f85_d405_sim.urdf.xacro`.
  Pair with `ur16e_2f85_d405.launch.py` + `ur16e_2f85_d405_moveit.launch.py`.
  > **Real counterpart**: `d405_real.launch.py` runs the `realsense2_camera` driver
  > on these SAME topics/frames (so MoveIt/OctoMap/perception is unchanged), plus a
  > camera robot_state_publisher for the TF. See [`../../HARDWARE.md`](../../HARDWARE.md) §3
  > (USB3, topic-namespace check, hand-eye calibration).

## Headless / scripted alternative
`/isaac-sim/standalone_examples/api/isaacsim.ros2.bridge/` has reference Python
scripts for building this graph programmatically if you prefer a repeatable
scene script over manual GUI wiring.
