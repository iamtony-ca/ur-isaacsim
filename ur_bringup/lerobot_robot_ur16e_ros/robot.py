"""LeRobot `Robot` for the UR16e + Robotiq 2F-85, driven through ROS 2.

*** IMPORTED FROM THE ML ENVIRONMENT, WITH THE ROS OVERLAY SOURCED. ***
`deps/.venv-ml` can import rclpy, torch and numpy together (verified), so the
LeRobot side can talk to ROS directly instead of through a socket of our own.

Why this class exists at all: LeRobot ships robots that open serial ports
(`--robot.type=so100_follower --robot.port=/dev/tty...`). Ours is a UR16e behind
ros2_control, MoveIt and cuMotion, and there is no upstream class for that. This
is the ONE adapter needed; with it, upstream's own tools work unmodified --
`lerobot.async_inference.robot_client` for running a policy now, and
`lerobot-record` with a real teleoperator (the OMY leader) later. Writing our own
policy server or robot client instead would be duplicating what upstream already
maintains.

Schema compatibility is not accidental. `hw_to_dataset_features` turns the joint
entries below into `observation.state` / `action` float32[7] and the camera
entries into `observation.images.<name>`, which is exactly the schema the
existing dataset and the trained ACT checkpoint use (plan_il_vla.md 2.6).

The gripper is reported and accepted NORMALISED 0..1 with 1 = closed, matching
il_recorder.py and therefore the training data. Radians never leave this file.

Prerequisites (the class checks what it can and complains clearly otherwise):
  * control stack + cameras running (README.md 5)
  * the STREAMING controller active -- forward_position_controller and
    scaled_joint_trajectory_controller claim the same interfaces and are mutually
    exclusive:  python3 isaac/common/switch_control_mode.py streaming
"""
import threading
import time
from dataclasses import dataclass, field
from functools import cached_property

import numpy as np

from lerobot.cameras import CameraConfig  # noqa: F401  (kept for config parity)
from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def _image_to_rgb(msg) -> np.ndarray:
    """sensor_msgs/Image -> HxWx3 uint8 RGB, using numpy only.

    NOT cv_bridge. cv_bridge is a C extension built against the ROS distro's
    numpy (1.26.4 on Jazzy), and this process runs on the numpy 2.x that lerobot
    and torch require -- importing it here aborts the process with
    "A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.6".
    rclpy itself is fine; it is the numpy-linked extensions that are not. Decoding
    the buffer by hand sidesteps the ABI entirely, and an Image is just rows of
    bytes with a stride.

    RGB, not BGR, because that is what the training data holds: il_recorder.py
    wrote JPEGs through cv2 (BGR in, correct colours out) and raw_to_lerobot.py
    read them back with PIL `.convert("RGB")`. Feeding BGR to a policy trained on
    RGB is a silent, plausible-looking failure.
    """
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        ch = 3
    elif enc in ("rgba8", "bgra8"):
        ch = 4
    else:
        raise ValueError(f"unsupported image encoding {msg.encoding!r}")
    # step is the row stride in bytes and may exceed width*ch (padding).
    img = buf.reshape(msg.height, msg.step)[:, : msg.width * ch]
    img = img.reshape(msg.height, msg.width, ch)[:, :, :3]
    if enc.startswith("bgr"):
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img)


@RobotConfig.register_subclass("ur16e_ros")
@dataclass
class UR16eROSConfig(RobotConfig):
    joint_states_topic: str = "/joint_states"
    command_topic: str = "/forward_position_controller/commands"
    gripper_action: str = "/gripper_controller/gripper_cmd"
    gripper_joint: str = "finger_joint"
    gripper_open_rad: float = 0.0
    gripper_closed_rad: float = 0.8
    gripper_effort: float = 60.0
    gripper_deadband: float = 0.05
    # name -> image topic. The keys become observation.images.<key>, so they must
    # match what the policy was trained with -- ONE ENTRY PER CAMERA THE POLICY
    # SAW, no more and no less. Override for a wrist-only policy with:
    #   --robot.cameras_ros='{"wrist": "/camera/color/image_raw"}'
    # A key the policy does not know is ignored; a key it expects and does not get
    # raises in prepare_raw_observation. Both are configuration, not code.
    cameras_ros: dict[str, str] = field(default_factory=lambda: {
        "exterior": "/static_cam/color/image_raw",
        "wrist": "/camera/color/image_raw",
    })
    # Declared resolution of the incoming ROS images. This is metadata only: the
    # upstream client resizes each image to the POLICY's feature shape
    # (helpers.resize_robot_observation_image reads policy_image_features, not
    # this), so a mismatch does not corrupt the input -- the 320x240 runs worked
    # while this still said 480x640. Kept honest anyway, since a wrong number here
    # is exactly the kind of thing that gets trusted later.
    image_shape: tuple[int, int, int] = (240, 320, 3)
    obs_timeout: float = 2.0
    # Clamp per-command joint motion. There is no planner between a policy and
    # the hardware here, so a bad prediction is a fast move unless it is slewed.
    max_joint_step: float = 0.05
    use_sim_time: bool = True


class UR16eROS(Robot):
    config_class = UR16eROSConfig
    name = "ur16e_ros"

    def __init__(self, config: UR16eROSConfig):
        super().__init__(config)
        self.config = config
        self._node = None
        self._exec = None
        self._thread = None
        self._joints = None          # (arm[6] rad, gripper normalised)
        self._joints_t = 0.0
        self._imgs = {}
        self._imgs_t = {}
        self._last_grip = None
        self._connected = False

    # ---- features ---------------------------------------------------------
    @cached_property
    def _joint_ft(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in ARM_JOINTS + [self.config.gripper_joint]}

    @cached_property
    def observation_features(self) -> dict:
        return {**self._joint_ft,
                **{k: tuple(self.config.image_shape) for k in self.config.cameras_ros}}

    @cached_property
    def action_features(self) -> dict:
        return self._joint_ft

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        # The UR is calibrated by its own driver and the gripper by robotiq_driver.
        # There is nothing for LeRobot to calibrate here, and pretending otherwise
        # would make `lerobot-calibrate` write a file nothing reads.
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    # ---- ROS plumbing -----------------------------------------------------
    def connect(self, calibrate: bool = True) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSPresetProfiles
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Float64MultiArray
        from control_msgs.action import GripperCommand
        from rclpy.action import ActionClient

        if not rclpy.ok():
            rclpy.init(args=[])
        node = Node("lerobot_ur16e")
        node.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, bool(self.config.use_sim_time))])

        def js_cb(msg):
            idx = {n: i for i, n in enumerate(msg.name)}
            if not all(j in idx for j in ARM_JOINTS):
                return
            arm = [float(msg.position[idx[j]]) for j in ARM_JOINTS]
            gj = self.config.gripper_joint
            raw = float(msg.position[idx[gj]]) if gj in idx else 0.0
            span = (self.config.gripper_closed_rad - self.config.gripper_open_rad) or 1.0
            self._joints = (arm, float(np.clip(
                (raw - self.config.gripper_open_rad) / span, 0.0, 1.0)))
            self._joints_t = time.time()

        def img_cb(key):
            def cb(msg):
                try:
                    self._imgs[key] = _image_to_rgb(msg)
                    self._imgs_t[key] = time.time()
                except Exception as e:
                    node.get_logger().warn(f"{key}: image decode failed: {e}",
                                           throttle_duration_sec=5.0)
            return cb

        node.create_subscription(JointState, self.config.joint_states_topic, js_cb, 10)
        for key, topic in self.config.cameras_ros.items():
            node.create_subscription(Image, topic, img_cb(key),
                                     QoSPresetProfiles.SENSOR_DATA.value)
        self._cmd_pub = node.create_publisher(Float64MultiArray, self.config.command_topic, 10)
        self._grip = ActionClient(node, GripperCommand, self.config.gripper_action)

        self._node = node
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(node)
        self._thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._thread.start()

        deadline = time.time() + self.config.obs_timeout * 5
        while time.time() < deadline:
            if self._joints is not None and len(self._imgs) == len(self.config.cameras_ros):
                self._connected = True
                node.get_logger().info(
                    f"connected: joints + {list(self._imgs)} cameras. "
                    f"Commands go to {self.config.command_topic} -- the STREAMING "
                    "controller must be active (switch_control_mode.py streaming).")
                return
            time.sleep(0.1)
        missing = ([] if self._joints is not None else ["joint_states"]) + \
                  [k for k in self.config.cameras_ros if k not in self._imgs]
        raise RuntimeError(
            f"no data on {missing} within {self.config.obs_timeout * 5:.0f}s. "
            "Is the control stack + Isaac (or the real robot) running?")

    def disconnect(self) -> None:
        if self._exec is not None:
            self._exec.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        self._connected = False

    # ---- observation / action --------------------------------------------
    def get_observation(self) -> dict:
        if not self._connected:
            raise RuntimeError("not connected")
        now = time.time()
        if self._joints is None or now - self._joints_t > self.config.obs_timeout:
            raise RuntimeError("joint states are stale")
        stale = [k for k in self.config.cameras_ros
                 if now - self._imgs_t.get(k, 0.0) > self.config.obs_timeout]
        if stale:
            raise RuntimeError(f"no fresh image from {stale}")
        arm, grip = self._joints
        obs = {f"{j}.pos": v for j, v in zip(ARM_JOINTS, arm)}
        obs[f"{self.config.gripper_joint}.pos"] = grip
        for k in self.config.cameras_ros:
            obs[k] = self._imgs[k]
        return obs

    def send_action(self, action: dict) -> dict:
        """Absolute joint targets, slewed. Returns what was actually sent, which
        is what LeRobot records -- so a clamped command is stored as clamped
        rather than as the value the policy wished for."""
        if not self._connected:
            raise RuntimeError("not connected")
        from std_msgs.msg import Float64MultiArray
        from control_msgs.action import GripperCommand

        arm_now, _ = self._joints
        want = np.array([float(action[f"{j}.pos"]) for j in ARM_JOINTS])
        cur = np.asarray(arm_now, dtype=float)
        step = float(self.config.max_joint_step)
        sent = np.clip(want, cur - step, cur + step)

        msg = Float64MultiArray()
        msg.data = [float(v) for v in sent]
        try:
            self._cmd_pub.publish(msg)
        except Exception as e:
            # The client can be stopped (Ctrl-C, a timeout) between rclpy tearing
            # the context down and the control loop noticing, and publishing then
            # raises "publisher's context is invalid". That is shutdown, not a
            # fault, and it must not mask the real reason the run ended.
            if not self._connected:
                raise
            self._connected = False
            raise RuntimeError(f"ROS publish failed, treating as disconnect: {e}") from e

        gj = self.config.gripper_joint
        g = float(np.clip(action[f"{gj}.pos"], 0.0, 1.0))
        if self._last_grip is None or abs(g - self._last_grip) >= self.config.gripper_deadband:
            if self._grip.server_is_ready():
                goal = GripperCommand.Goal()
                goal.command.position = (self.config.gripper_open_rad + g * (
                    self.config.gripper_closed_rad - self.config.gripper_open_rad))
                goal.command.max_effort = float(self.config.gripper_effort)
                self._grip.send_goal_async(goal)
                self._last_grip = g

        out = {f"{j}.pos": float(v) for j, v in zip(ARM_JOINTS, sent)}
        out[f"{gj}.pos"] = g
        return out
