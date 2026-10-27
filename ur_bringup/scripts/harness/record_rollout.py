#!/usr/bin/env python3
"""Record one rollout trial to an H.264 mp4: the policy's cameras side by side.

Run with the ML venv (it has rclpy + OpenCV + PyAV/libx264); rollout_n.sh and
rollout_wrist1.sh start it per trial when RECORD_DIR is set:
    RECORD_DIR=outputs/rollout_videos/act_100 ./rollout_gui_then_n.sh 10 70

Frames are paced by Isaac's /clock, not the wall clock: headless Isaac runs faster
than real time (3.78x measured, HISTORY.md 49.5), so a wall-clock video would show
the arm moving unnaturally fast. Every 1/--hz s of SIM time one frame is written;
played at --fps, the video runs at fps/hz x sim speed (default 30/10 = 3x, shown in
the overlay). Stops cleanly on SIGINT/SIGTERM. What it shows is exactly what the
policy sees (320x240 each), upscaled for viewing.
"""
import argparse
import signal
import sys

import av
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

PANEL_W, PANEL_H, BAR_H = 480, 360, 44


def to_rgb(msg):
    """sensor_msgs/Image (rgb8 / bgr8 / rgba8 / bgra8) -> HxWx3 RGB uint8, no cv_bridge."""
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
    if ch is None:
        return None
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : msg.width * ch]
    a = a.reshape(msg.height, msg.width, ch)[..., :3]
    return a[..., ::-1] if msg.encoding.startswith("bgr") else a


class Recorder(Node):
    def __init__(self, a):
        super().__init__("rollout_recorder")
        self.a = a
        self.cams = [c.split("=", 1) for c in a.cams.split(",") if c]
        self.last = {n: None for n, _ in self.cams}
        for name, topic in self.cams:
            self.create_subscription(Image, topic, self._img_cb(name), qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)  # not _clock: Node uses that name
        self.box = av.open(a.out, mode="w")
        self.st = self.box.add_stream("libx264", rate=a.fps)
        self.st.width, self.st.height = PANEL_W * len(self.cams), PANEL_H + BAR_H
        self.st.pix_fmt = "yuv420p"
        self.st.options = {"crf": "23", "preset": "veryfast"}
        self.t0 = None
        self.next_t = None
        self.frames = 0

    def _img_cb(self, name):
        # rclpy inspects the callback's arity: a lambda with a default argument
        # counts as two parameters and is rejected, so bind the name in a closure
        def cb(msg):
            self._img(name, msg)
        return cb

    def _img(self, name, msg):
        rgb = to_rgb(msg)
        if rgb is not None:
            self.last[name] = rgb

    def _on_clock(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.t0 is None:
            self.t0, self.next_t = t, t
        while t >= self.next_t:
            self._write(self.next_t - self.t0)
            self.next_t += 1.0 / self.a.hz

    def _write(self, sim_s):
        panels = []
        for name, _ in self.cams:
            img = self.last[name]
            p = np.zeros((PANEL_H, PANEL_W, 3), np.uint8) if img is None else \
                cv2.resize(img, (PANEL_W, PANEL_H), interpolation=cv2.INTER_CUBIC)
            cv2.putText(p, name, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            panels.append(p)
        bar = np.full((BAR_H, PANEL_W * len(self.cams), 3), 24, np.uint8)
        txt = f"{self.a.label}   sim t = {sim_s:6.1f} s   ({self.a.fps / self.a.hz:g}x sim speed)"
        cv2.putText(bar, txt, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (235, 235, 235), 2, cv2.LINE_AA)
        frame = av.VideoFrame.from_ndarray(np.vstack([bar, np.hstack(panels)]), format="rgb24")
        for pkt in self.st.encode(frame):
            self.box.mux(pkt)
        self.frames += 1

    def close(self):
        for pkt in self.st.encode():
            self.box.mux(pkt)
        self.box.close()
        sim = 0.0 if self.t0 is None else self.next_t - self.t0
        print(f"wrote {self.a.out}: {self.frames} frames, {sim:.1f} s sim, "
              f"cameras seen: {[n for n, v in self.last.items() if v is not None]}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--label", default="")
    ap.add_argument("--cams", default="exterior=/static_cam/color/image_raw,wrist=/camera/color/image_raw")
    ap.add_argument("--hz", type=float, default=10.0, help="frames per SIM second")
    ap.add_argument("--fps", type=int, default=30, help="playback frame rate")
    a = ap.parse_args()
    rclpy.init()
    node = Recorder(a)
    stop = {"now": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    try:
        while rclpy.ok() and not stop["now"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
