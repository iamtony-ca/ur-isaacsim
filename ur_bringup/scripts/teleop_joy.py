#!/usr/bin/env python3
"""Gamepad teleoperation for UR16e + Robotiq 2F-85 via MoveIt Servo.

    /joy  ->  this node  ->  /servo_node/delta_twist_cmds  ->  servo  ->
              /forward_position_controller/commands  ->  ros2_control  ->  Isaac / real UR16e
           \\->  /gripper_controller/gripper_cmd (GripperCommand action)

Step 1 of the teleop -> IL pick&place pipeline (ur_bringup/docs/plan_il_vla.md).

Defaults target a **PS5 DualSense over USB** (kernel `hid-playstation`), but every
axis/button is a ROS parameter, so any pad works -- check yours with
`ros2 run joy joy_node` + `ros2 topic echo /joy` and override the indices.

*** SAFETY: motion requires HOLDING the deadman button (default L1). ***
Release it and the node publishes a zero twist, so Servo halts. This is not
optional on a 16 kg-payload, 900 mm arm.

Controls (defaults)
-------------------
    L1 (hold)          deadman -- nothing moves without it
    left stick  X/Y    translate in the base XY plane
    right stick Y      translate along base Z (up/down)
    right stick X      yaw about the tool axis
    R1 (hold) + right  swap the right stick to roll/pitch instead
    L2 / R2 triggers   gripper open / close (analog)
    Circle             re-assert TWIST command mode on Servo
    Square             start recording an episode   (il_recorder)
    Triangle           stop + SAVE the episode
    Cross              discard the episode

Bring-up: see launch/common/teleop_dualsense.launch.py
"""
import math

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


class TeleopJoy(Node):
    def __init__(self):
        super().__init__("teleop_joy")

        p = self.declare_parameter
        # --- axis / button map (DualSense via hid-playstation) ---------------
        p("axis_lx", 0);  p("axis_ly", 1)
        p("axis_rx", 3);  p("axis_ry", 4)
        p("axis_l2", 2);  p("axis_r2", 5)
        p("button_deadman", 4)      # L1
        p("button_rotate_mode", 5)  # R1: right stick -> roll/pitch
        p("button_reassert", 1)     # Circle
        # Episode control for IL recording, so the operator never has to let go of
        # the pad to run `ros2 service call`. No-ops if il_recorder is not running.
        p("button_ep_start", 3)     # Square  -> /il/start_episode
        p("button_ep_stop", 2)      # Triangle-> /il/stop_episode  (save)
        p("button_ep_discard", 0)   # Cross   -> /il/discard_episode
        # --- shaping ---------------------------------------------------------
        p("deadzone", 0.08)
        p("linear_scale", 1.0)      # multiplies the unitless [-1,1] command
        p("angular_scale", 1.0)     # (absolute speed is set in ur16e_servo.yaml)
        p("publish_rate", 50.0)
        p("twist_frame", "base_link")
        # Triggers rest at +1.0 and go to -1.0 when pressed on DualSense.
        p("trigger_idle_is_positive", True)
        p("gripper_open", 0.0)      # finger_joint [rad]: 0.0 open .. 0.8 closed
        p("gripper_closed", 0.8)
        p("gripper_max_effort", 100.0)
        p("gripper_step_deadband", 0.02)   # don't spam the action server

        g = lambda n: self.get_parameter(n).value
        self.ax = {k: g(f"axis_{k}") for k in ("lx", "ly", "rx", "ry", "l2", "r2")}
        self.btn_deadman = g("button_deadman")
        self.btn_rot = g("button_rotate_mode")
        self.btn_reassert = g("button_reassert")
        self.btn_ep = {"start": g("button_ep_start"), "stop": g("button_ep_stop"),
                       "discard": g("button_ep_discard")}
        self.deadzone = g("deadzone")
        self.lin_scale = g("linear_scale")
        self.ang_scale = g("angular_scale")
        self.frame = g("twist_frame")
        self.trig_idle_pos = g("trigger_idle_is_positive")
        self.grip_open = g("gripper_open")
        self.grip_closed = g("gripper_closed")
        self.grip_effort = g("gripper_max_effort")
        self.grip_deadband = g("gripper_step_deadband")

        self._joy = None
        self._last_grip = None
        self._was_enabled = False
        self._prev_reassert = 0
        self._prev_ep = {k: 0 for k in ("start", "stop", "discard")}

        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.grip_cli = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")
        self.servo_type_cli = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        self.ep_cli = {k: self.create_client(Trigger, f"/il/{k}_episode")
                       for k in ("start", "stop", "discard")}

        self.create_timer(1.0 / max(1.0, g("publish_rate")), self._tick)
        self.create_timer(2.0, self._startup_once)
        self._did_startup = False

        self.get_logger().info(
            f"teleop_joy up. deadman=button[{self.btn_deadman}] (HOLD to move), "
            f"twist frame='{self.frame}'. Waiting for /joy ...")

    # ---------------------------------------------------------------- helpers
    def _startup_once(self):
        """Put Servo in TWIST mode once it is available."""
        if self._did_startup:
            return
        if not self.servo_type_cli.service_is_ready():
            self.get_logger().info("waiting for /servo_node/switch_command_type ...",
                                   throttle_duration_sec=5.0)
            return
        self._did_startup = True
        self._request_twist_mode()

    def _request_twist_mode(self):
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        fut = self.servo_type_cli.call_async(req)
        fut.add_done_callback(
            lambda f: self.get_logger().info(
                f"Servo TWIST mode: {'OK' if (f.result() and f.result().success) else 'FAILED'}"))

    def _dz(self, v):
        return 0.0 if abs(v) < self.deadzone else (v - math.copysign(self.deadzone, v)) / (1.0 - self.deadzone)

    def _axis(self, msg, key):
        i = self.ax[key]
        return msg.axes[i] if 0 <= i < len(msg.axes) else 0.0

    def _button(self, msg, i):
        return msg.buttons[i] if 0 <= i < len(msg.buttons) else 0

    def _trigger(self, msg, key):
        """Analog trigger -> [0,1]. DualSense rests at +1 and goes to -1."""
        v = self._axis(msg, key)
        return (1.0 - v) / 2.0 if self.trig_idle_pos else max(0.0, v)

    # ------------------------------------------------------------------ input
    def _on_joy(self, msg):
        self._joy = msg
        # edge-triggered: re-assert TWIST mode (e.g. after Servo restart)
        cur = self._button(msg, self.btn_reassert)
        if cur and not self._prev_reassert and self.servo_type_cli.service_is_ready():
            self._request_twist_mode()
        self._prev_reassert = cur

        # Edge-triggered episode control (ignored when no recorder is running).
        for name, idx in self.btn_ep.items():
            now = self._button(msg, idx)
            if now and not self._prev_ep[name]:
                cli = self.ep_cli[name]
                if cli.service_is_ready():
                    cli.call_async(Trigger.Request())
                    self.get_logger().info(f"episode: {name}")
                else:
                    self.get_logger().warn(f"/il/{name}_episode not available "
                                           "(il_recorder not running?)")
            self._prev_ep[name] = now

    # ------------------------------------------------------------------- loop
    def _tick(self):
        msg = self._joy
        if msg is None:
            return

        enabled = bool(self._button(msg, self.btn_deadman))
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame

        if enabled:
            lx, ly = self._dz(self._axis(msg, "lx")), self._dz(self._axis(msg, "ly"))
            rx, ry = self._dz(self._axis(msg, "rx")), self._dz(self._axis(msg, "ry"))
            # Sticks report +1 up/left; map so "push up" = +X (forward), "push right" = +Y.
            t.twist.linear.x = self.lin_scale * ly
            t.twist.linear.y = self.lin_scale * -lx
            if self._button(msg, self.btn_rot):
                t.twist.angular.x = self.ang_scale * ry     # roll
                t.twist.angular.y = self.ang_scale * rx     # pitch
            else:
                t.twist.linear.z = self.lin_scale * ry
                t.twist.angular.z = self.ang_scale * -rx    # yaw
            self._gripper(msg)

        # Always publish: on release this sends an all-zero twist so Servo stops
        # immediately instead of coasting until incoming_command_timeout.
        self.twist_pub.publish(t)
        if self._was_enabled and not enabled:
            self.get_logger().info("deadman released -- holding")
        elif enabled and not self._was_enabled:
            self.get_logger().info("deadman held -- moving")
        self._was_enabled = enabled

    # ---------------------------------------------------------------- gripper
    def _gripper(self, msg):
        close = self._trigger(msg, "r2")
        open_ = self._trigger(msg, "l2")
        cmd = close - open_                       # [-1, 1]
        if cmd <= 0.0:
            target = self.grip_open + (-cmd) * 0.0   # any open press -> fully open
            target = self.grip_open
            if open_ < 0.05:
                return                                # neither trigger pressed
        else:
            target = self.grip_open + cmd * (self.grip_closed - self.grip_open)

        if self._last_grip is not None and abs(target - self._last_grip) < self.grip_deadband:
            return
        if not self.grip_cli.server_is_ready():
            self.get_logger().warn("gripper action server not ready", throttle_duration_sec=5.0)
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(target)
        goal.command.max_effort = float(self.grip_effort)
        self.grip_cli.send_goal_async(goal)
        self._last_grip = target


def main():
    rclpy.init()
    node = TeleopJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
