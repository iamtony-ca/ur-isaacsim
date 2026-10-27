#!/usr/bin/env python3
"""Re-publish a camera's CameraInfo with the stamp of every processed depth frame.

Why (HISTORY.md §56): nvblox_ros 4.6 pairs depth with camera_info through a
message_filters ExactTime synchronizer whose queue is fixed at 10
(nvblox_node.cpp kQueueSize). In ur16e_2f85_d405_nvblox.launch.py the depth is
the robot_segmenter OUTPUT (~220 ms behind the camera), while the camera_info
came straight from the camera (~86 distinct stamps per wall second in Isaac).
By the time a masked depth frame arrives its camera_info partner has been pushed
out of the queue, so nvblox integrated nothing in steady state -- an obstacle
was mapped only when a startup stall happened to let one frame through, and the
A/B demo verdict flipped from run to run.

This node caches the latest camera_info and, for each depth frame, publishes it
with that frame's header. Valid for a fixed-intrinsics camera (static camera).

    remaps:  depth (in) / camera_info (in) / camera_info_out (out)

The depth subscription is raw (serialized bytes): only the CDR header is parsed,
so the 1.2 MB image is never deserialized in Python.
"""
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def header_from_cdr(buf):
    """(sec, nanosec, frame_id) of a serialized std_msgs/Header-first message."""
    # 4-byte encapsulation: 0x00 0x00 = CDR big-endian, 0x00 0x01 = little-endian
    # (0x02/0x03 = PL_CDR variants, same byte order rule).
    end = "<" if buf[1] & 1 else ">"
    sec, nsec, n = struct.unpack_from(end + "iII", buf, 4)
    frame_id = bytes(buf[16:16 + max(n - 1, 0)]).decode("utf-8", "replace")
    return sec, nsec, frame_id


class DepthInfoRestamp(Node):
    def __init__(self):
        super().__init__("depth_info_restamp")
        self._info = None
        self._pub = self.create_publisher(CameraInfo, "camera_info_out", QoSProfile(depth=10))
        self.create_subscription(CameraInfo, "camera_info", self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "depth", self._on_depth, qos_profile_sensor_data, raw=True)

    def _on_info(self, msg):
        self._info = msg

    def _on_depth(self, buf):
        if self._info is None:
            return
        sec, nsec, frame_id = header_from_cdr(buf)
        self._info.header.stamp.sec = sec
        self._info.header.stamp.nanosec = nsec
        self._info.header.frame_id = frame_id
        self._pub.publish(self._info)


def main():
    rclpy.init()
    node = DepthInfoRestamp()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
