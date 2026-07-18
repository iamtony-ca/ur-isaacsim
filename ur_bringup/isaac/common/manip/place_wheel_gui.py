# SPDX-License-Identifier: Apache-2.0
"""Interactive GUI helper to TUNE the wheel grasp pose (attach_wheel.py --grasp-xyz/-rpy).

Opens the composed UR16e+EOAT USD at HOME with the wheel spawned as a child of the
gripper link. Grab the `grasped_wheel` prim with the gizmo and move/rotate it until it
sits where the fingers actually hold it; the terminal prints the wheel's LOCAL transform
(relative to the gripper link) LIVE, in the exact form attach_wheel.py wants:

    --grasp-xyz X,Y,Z --grasp-rpy R,P,Y

Read the final numbers off the terminal and pass them to attach_wheel.py (or paste them
as the DEFAULT_GRASP below). The transform is relative to the gripper link, so the arm
pose doesn't matter. CLOSE THE WINDOW to exit.

Run (needs a display; use the ! prefix so it attaches to your session):
    /isaac-sim/python.sh isaac/common/manip/place_wheel_gui.py
"""
from __future__ import annotations

import math
from pathlib import Path

from isaacsim import SimulationApp
kit = SimulationApp({"headless": False, "renderer": "RayTracedLighting"})

from pxr import Gf, Usd, UsdGeom  # noqa: E402
import omni.usd  # noqa: E402

HERE = Path(__file__).resolve().parent
FULL_USD = HERE.parents[1] / "assets" / "ur16e_dualtool_full.usd"
WHEEL_USD = HERE.parents[1] / "assets" / "manip" / "wheel.usd"
GRIPPER = "/UR16e/wrist_3_link/eoat/tool0/damper/dual_quick_changer/hex_qc/gripper_2fg14"
START_XYZ = (0.0, 0.0, 0.12)          # starting guess (m) in the gripper link frame

ctx = omni.usd.get_context()
ctx.open_stage(str(FULL_USD))
stage = ctx.get_stage()

wheel_path = GRIPPER + "/grasped_wheel"
wp = UsdGeom.Xform.Define(stage, wheel_path)
wp.GetPrim().GetReferences().AddReference(str(WHEEL_USD))
wp.AddTranslateOp().Set(Gf.Vec3d(*START_XYZ))
wp.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
set_camera_view(eye=[0.6, -0.6, 1.4], target=[0.0, 0.2, 1.0])

wheel_prim = stage.GetPrimAtPath(wheel_path)


def local_grasp():
    m = UsdGeom.Xformable(wheel_prim).GetLocalTransformation()   # relative to the gripper link
    t = Gf.Transform(m)
    tr = t.GetTranslation()
    az, ay, ax = t.GetRotation().Decompose(Gf.Vec3d.ZAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.XAxis())
    xyz = ",".join(f"{v:.4f}" for v in (tr[0], tr[1], tr[2]))
    rpy = ",".join(f"{v:.1f}" for v in (ax, ay, az))            # ROS rpy = roll(x),pitch(y),yaw(z)
    return xyz, rpy


print(f"[place-wheel] move the 'grasped_wheel' prim ({wheel_path}) with the gizmo.", flush=True)
print("[place-wheel] live grasp readout below; read the final line into attach_wheel.py. "
      "CLOSE THE WINDOW to exit.", flush=True)
_last = None
_n = 0
while kit.is_running():
    kit.update()
    _n += 1
    if _n % 60 == 0:                                            # ~ once per second
        g = local_grasp()
        if g != _last:
            _last = g
            print(f"[place-wheel] --grasp-xyz {g[0]} --grasp-rpy {g[1]}", flush=True)
kit.close()
