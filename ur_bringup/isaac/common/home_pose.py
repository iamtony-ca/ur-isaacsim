# SPDX-License-Identifier: Apache-2.0
"""SINGLE SOURCE of the UR16e arm HOME (initial) pose.

The arm's initial/home pose used to be duplicated in three files with slightly
different numbers (-1.5707 vs -1.5708). They now all import from here, so changing
the initial pose = editing the six numbers in HOME_DEG BELOW, once.

Consumers:
  * eoat/build_ur16e_dualtool.py -> bakes HOME_DEG as the USD articulation default
    (so any loader — GUI / verify_articulation / raw stage-open — spawns off the floor)
  * ur16e_isaac_ros2.py          -> teleports the live articulation to HOME_RAD at
    physics start (Isaac sim initial pose)
  * reset_pose.py                -> the "home" named pose it commands to the controller

Why there is no separate "RViz init pose": in sim, RViz/MoveIt only DISPLAY
/joint_states, which comes from Isaac via topic_based hardware. So the initial pose
has ONE source (Isaac, seeded here); RViz follows automatically. (A named MoveIt
"home" *planning target* is a different thing — that lives in the SRDF group_state.)

Pure python (only stdlib) so it imports under BOTH Isaac bundled python and system
python3. Angle units: USD Physics revolute joints want DEGREES; the runtime
articulation API and ROS trajectories want RADIANS — both provided below.
"""
from __future__ import annotations

import math

# Canonical joint order (matches the UR16e arm chain / controllers).
ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

# ── EDIT HERE to change the initial/home pose (degrees) ──────────────────────
# Arm-up: shoulder_lift=-90° lifts the arm off the floor from the all-zeros
# horizontal USD default (which can spawn in floor collision).
HOME_DEG = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -90.0,
    "elbow_joint": 0.0,
    "wrist_1_joint": 0.0,
    "wrist_2_joint": 0.0,
    "wrist_3_joint": 0.0,
}
# ─────────────────────────────────────────────────────────────────────────────

HOME_RAD = {k: math.radians(v) for k, v in HOME_DEG.items()}


def home_list_rad(order=ARM_JOINTS):
    """HOME as a radians list in the given joint order (for ROS trajectories)."""
    return [HOME_RAD[j] for j in order]
