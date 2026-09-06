#!/usr/bin/env python3
"""Record teleop demonstrations as RAW episodes for the IL/VLA pipeline.

    cameras + /joint_states  ->  this node  ->  <out>/episode_XXXXXX/
                                                  frames/<video_key>/000000.jpg
                                                  data.json
                                                  meta.json

Then, IN THE ML ENVIRONMENT (not here), convert to a real LeRobot dataset:

    python3 scripts/raw_to_lerobot.py --raw <out> --repo-id <user>/<name>

Why two stages
--------------
LeRobot's on-disk format needs parquet (pyarrow) and mp4 (ffmpeg/av), and the
conversion conventions are version specific. None of that exists in the ROS 2
Jazzy Python environment, and installing it there would drag the ML stack into
the robot process -- exactly the environment boundary ur_bringup/docs/plan_il_vla.md
2.2 deliberately keeps. So this node writes a small, self-describing raw format
using only what ROS ships (cv2 + json + numpy), and a converter that runs where
torch/lerobot live produces the dataset.

(rosbag2 was the alternative. Rejected: two 640x480 streams at ~54 Hz uncompressed
is ~100 MB/s, and episode boundaries / success labels / resampling all still need
custom handling on top.)

*** Schema is ur_bringup/docs/plan_il_vla.md 2.6 -- that document is the source
of truth. Changing it later is more expensive than re-collecting data. ***

    video.exterior     external camera RGB
    video.wrist        eye-in-hand RGB
    state.single_arm   (6,) arm joint positions [rad]
    state.gripper      (1,) 0.0 open .. 1.0 closed   <- finger_joint ONLY
    action.single_arm  (6,) target arm joint positions [rad]
    action.gripper     (1,)
    task               natural-language instruction

Teleop-device agnostic by construction
--------------------------------------
Observation always comes from /joint_states, whatever moves the arm:
    keyboard / DualSense  -> EE twist -> MoveIt Servo -> forward_position_controller
    OMY leader / GELLO    -> joint positions -> forward_position_controller (no IK)
    UR freedrive (real)   -> nothing commands the arm; you just move it
The ACTION differs per device, hence --action-source:
    next_state  (default) action[t] = state[t+1]. Works for every device including
                freedrive, where no command exists at all.
    topic       action[t] = a JointState topic you name (--action-topic), e.g. the
                leader arm's own joints. This is the ALOHA/GELLO convention and is
                the right choice once the OMY leader is wired up.

Usage
-----
    ros2 run ur_bringup il_recorder.py --ros-args \\
        -p out_dir:=/isaac-sim/volume/ur_ws/datasets/pick_place_raw \\
        -p task:="put the blue block in the green zone"

    ros2 service call /il/start_episode   std_srvs/srv/Trigger   # begin
    ros2 service call /il/stop_episode    std_srvs/srv/Trigger   # save (success)
    ros2 service call /il/discard_episode std_srvs/srv/Trigger   # throw away
    ros2 param set /il_recorder task "put the red block in the bowl"

Collect at least 2-3 task variants: a single-task dataset trains a VLA that
ignores language, i.e. an expensive ACT (plan_il_vla.md 2.8).
"""
import json
import os
import time
from collections import OrderedDict

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

import cv2

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
GRIPPER_JOINT = "finger_joint"
RAW_FORMAT_VERSION = 1


class ILRecorder(Node):
    def __init__(self):
        super().__init__("il_recorder")
        p = self.declare_parameter
        p("out_dir", os.path.expanduser("~/il_raw"))
        p("task", "")
        p("rate", 30.0)
        p("jpeg_quality", 92)
        p("cameras.exterior", "/static_cam/color/image_raw")
        p("cameras.wrist", "/camera/color/image_raw")
        p("joint_states_topic", "/joint_states")
        p("action_source", "next_state")          # next_state | topic
        p("action_topic", "")                      # JointState, e.g. a leader arm
        p("gripper_open_rad", 0.0)
        p("gripper_closed_rad", 0.8)
        p("min_episode_frames", 10)
        p("auto_reset", True)                      # call /scene/reset_episode on stop

        g = lambda n: self.get_parameter(n).value
        self.out_dir = g("out_dir")
        self.rate = float(g("rate"))
        self.jpeg_q = int(g("jpeg_quality"))
        self.cams = OrderedDict()
        for key in ("exterior", "wrist"):
            topic = g(f"cameras.{key}")
            if topic:
                self.cams[f"video.{key}"] = topic
        self.action_source = g("action_source")
        self.action_topic = g("action_topic")
        self.g_open = float(g("gripper_open_rad"))
        self.g_closed = float(g("gripper_closed_rad"))
        self.min_frames = int(g("min_episode_frames"))
        self.auto_reset = bool(g("auto_reset"))

        self.bridge = CvBridge()
        self._imgs = {k: None for k in self.cams}
        self._js = {}
        self._act_js = {}
        self._recording = False
        self._frames = []          # list of dicts (one per tick)
        self._ep_dir = None
        self._ep_index = self._next_episode_index()

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        for key, topic in self.cams.items():
            self.create_subscription(Image, topic, self._img_cb(key), sensor_qos)
        self.create_subscription(JointState, g("joint_states_topic"),
                                 lambda m: self._js.update(dict(zip(m.name, m.position))), 20)
        if self.action_source == "topic":
            if not self.action_topic:
                self.get_logger().error("action_source=topic but action_topic is empty")
            else:
                self.create_subscription(JointState, self.action_topic,
                                         lambda m: self._act_js.update(dict(zip(m.name, m.position))), 20)

        self.create_service(Trigger, "/il/start_episode", self._srv_start)
        self.create_service(Trigger, "/il/stop_episode", self._srv_stop)
        self.create_service(Trigger, "/il/discard_episode", self._srv_discard)
        self.status_pub = self.create_publisher(String, "/il/status", 10)
        self.reset_cli = self.create_client(Trigger, "/scene/reset_episode")

        self.create_timer(1.0 / self.rate, self._tick)
        self.create_timer(2.0, self._heartbeat)
        os.makedirs(self.out_dir, exist_ok=True)
        self.get_logger().info(
            f"il_recorder ready. out_dir={self.out_dir} rate={self.rate}Hz "
            f"cams={list(self.cams)} action_source={self.action_source}"
            + (f" ({self.action_topic})" if self.action_source == "topic" else ""))

    # ------------------------------------------------------------------ utils
    def _img_cb(self, key):
        def cb(msg):
            try:
                self._imgs[key] = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                self.get_logger().warn(f"{key}: cv_bridge failed: {e}", throttle_duration_sec=5.0)
        return cb

    def _next_episode_index(self):
        if not os.path.isdir(self.out_dir):
            return 0
        idx = [int(d.split("_")[-1]) for d in os.listdir(self.out_dir)
               if d.startswith("episode_") and d.split("_")[-1].isdigit()]
        return max(idx) + 1 if idx else 0

    def _norm_gripper(self, rad):
        span = self.g_closed - self.g_open
        return float(np.clip((rad - self.g_open) / span, 0.0, 1.0)) if span else 0.0

    def _ready(self):
        missing = [k for k, v in self._imgs.items() if v is None]
        if missing:
            return False, f"no image yet on {[self.cams[k] for k in missing]}"
        if any(j not in self._js for j in ARM):
            return False, "joint_states missing arm joints"
        if GRIPPER_JOINT not in self._js:
            return False, f"joint_states missing {GRIPPER_JOINT}"
        return True, ""

    def _heartbeat(self):
        ok, why = self._ready()
        m = String()
        m.data = (f"recording ep{self._ep_index} frames={len(self._frames)}"
                  if self._recording else ("idle (ready)" if ok else f"idle (NOT ready: {why})"))
        self.status_pub.publish(m)

    # ------------------------------------------------------------- recording
    def _tick(self):
        if not self._recording:
            return
        ok, why = self._ready()
        if not ok:
            self.get_logger().warn(f"skipping frame: {why}", throttle_duration_sec=2.0)
            return
        state_arm = [float(self._js[j]) for j in ARM]
        state_grip = self._norm_gripper(float(self._js[GRIPPER_JOINT]))

        act_arm, act_grip = None, None
        if self.action_source == "topic" and self._act_js:
            if all(j in self._act_js for j in ARM):
                act_arm = [float(self._act_js[j]) for j in ARM]
            if GRIPPER_JOINT in self._act_js:
                act_grip = self._norm_gripper(float(self._act_js[GRIPPER_JOINT]))

        i = len(self._frames)
        for key, img in self._imgs.items():
            d = os.path.join(self._ep_dir, "frames", key)
            cv2.imwrite(os.path.join(d, f"{i:06d}.jpg"), img,
                        [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_q])
        self._frames.append({
            "t": self.get_clock().now().nanoseconds * 1e-9,
            "state.single_arm": state_arm,
            "state.gripper": [state_grip],
            "action.single_arm": act_arm,     # filled at save time if next_state
            "action.gripper": [act_grip] if act_grip is not None else None,
        })

    def _finalise_actions(self):
        """action[t] = state[t+1] for the next_state source; last frame repeats."""
        if self.action_source != "next_state":
            return
        n = len(self._frames)
        for i, f in enumerate(self._frames):
            nxt = self._frames[min(i + 1, n - 1)]
            f["action.single_arm"] = list(nxt["state.single_arm"])
            f["action.gripper"] = list(nxt["state.gripper"])

    # ----------------------------------------------------------- services
    def _srv_start(self, req, res):
        if self._recording:
            res.success, res.message = False, "already recording"
            return res
        ok, why = self._ready()
        if not ok:
            res.success, res.message = False, f"not ready: {why}"
            self.get_logger().error(res.message)
            return res
        task = self.get_parameter("task").value
        if not task:
            self.get_logger().warn(
                "task string is EMPTY. A dataset without language instructions trains a VLA "
                "that ignores language (plan_il_vla.md 2.8). Set it: "
                "ros2 param set /il_recorder task \"...\"")
        self._ep_index = self._next_episode_index()
        self._ep_dir = os.path.join(self.out_dir, f"episode_{self._ep_index:06d}")
        for key in self.cams:
            os.makedirs(os.path.join(self._ep_dir, "frames", key), exist_ok=True)
        self._frames = []
        self._recording = True
        res.success, res.message = True, f"recording episode_{self._ep_index:06d} task='{task}'"
        self.get_logger().info(res.message)
        return res

    def _srv_stop(self, req, res):
        if not self._recording:
            res.success, res.message = False, "not recording"
            return res
        self._recording = False
        n = len(self._frames)
        if n < self.min_frames:
            res.success, res.message = False, f"only {n} frames (< {self.min_frames}) -- discarded"
            self._rm_ep()
            self.get_logger().warn(res.message)
            return res
        self._finalise_actions()
        task = self.get_parameter("task").value
        meta = {
            "raw_format_version": RAW_FORMAT_VERSION,
            "episode_index": self._ep_index,
            "task": task,
            "num_frames": n,
            "rate_hz": self.rate,
            "video_keys": list(self.cams),
            "camera_topics": dict(self.cams),
            "arm_joints": ARM,
            "gripper_joint": GRIPPER_JOINT,
            "gripper_open_rad": self.g_open,
            "gripper_closed_rad": self.g_closed,
            "action_source": self.action_source,
            "action_topic": self.action_topic,
            "schema": "ur_bringup/docs/plan_il_vla.md 2.6",
            "recorded_unix_time": time.time(),
        }
        with open(os.path.join(self._ep_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        with open(os.path.join(self._ep_dir, "data.json"), "w") as f:
            json.dump({"frames": self._frames}, f)
        res.success = True
        res.message = f"saved {self._ep_dir} ({n} frames)"
        self.get_logger().info(res.message)
        if self.auto_reset and self.reset_cli.service_is_ready():
            self.reset_cli.call_async(Trigger.Request())
        return res

    def _srv_discard(self, req, res):
        if not self._recording:
            res.success, res.message = False, "not recording"
            return res
        self._recording = False
        self._rm_ep()
        res.success, res.message = True, f"discarded episode_{self._ep_index:06d}"
        self.get_logger().info(res.message)
        if self.auto_reset and self.reset_cli.service_is_ready():
            self.reset_cli.call_async(Trigger.Request())
        return res

    def _rm_ep(self):
        import shutil
        if self._ep_dir and os.path.isdir(self._ep_dir):
            shutil.rmtree(self._ep_dir, ignore_errors=True)
        self._frames = []


def main():
    rclpy.init()
    node = ILRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
