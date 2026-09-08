"""LeRobot robot plugin for the ROS-driven UR16e + Robotiq 2F-85.

Discovered automatically by lerobot: `register_third_party_plugins()` imports every
installed distribution whose name starts with `lerobot_robot_` (also
`lerobot_teleoperator_`, `lerobot_policy_`, ...), which is how the OMY leader
plugin referenced in CLAUDE.md is packaged too. Importing this module registers
`--robot.type=ur16e_ros`, and upstream builds the robot through
`make_device_from_device_class`: config `UR16eROSConfig` -> class `UR16eROS`,
looked up in this package. Both names are therefore part of the contract.
"""
from .robot import UR16eROS, UR16eROSConfig  # noqa: F401

__all__ = ["UR16eROS", "UR16eROSConfig"]
