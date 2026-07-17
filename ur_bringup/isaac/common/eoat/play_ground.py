# SPDX-License-Identifier: Apache-2.0
"""Play the combined UR16e+EOAT USD ON A GROUND PLANE to check whether the arm/EOAT
sags into the floor under gravity. Commands the arm to the FIXED home target every
step (a passive PD hold — NOT re-commanding the drifting current pose, which would
ratchet the arm down) and cycles the parallel gripper. Reports the lowest world-Z
reached by any wrist/EOAT rigid body vs the ground at z=0.

This is the passive-drive picture: Isaac joint drives are PD (stiffness/damping +
maxForce), so a heavy EOAT settles at a few degrees of static sag. In the real
pipeline ros2_control streams position commands and holds it with no drift.

Run:  /isaac-sim/python.sh play_ground.py [--headless] [--seconds N]
"""
import sys

HEADLESS = "--headless" in sys.argv
SECONDS = 6.0
if "--seconds" in sys.argv:
    SECONDS = float(sys.argv[sys.argv.index("--seconds") + 1])

from isaacsim import SimulationApp
_app = SimulationApp({"headless": HEADLESS})

import numpy as np  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.api.objects import GroundPlane  # noqa: E402
from isaacsim.core.utils import stage as stage_utils  # noqa: E402
from pxr import UsdLux, Sdf, UsdGeom, UsdPhysics  # noqa: E402
import omni.usd  # noqa: E402

USD = "/isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/assets/ur16e_dualtool_full.usd"
ROOT = "/World/eoat/root_joint"
HOME_DEG = {"shoulder_pan_joint": 0.0, "shoulder_lift_joint": -90.0, "elbow_joint": 0.0,
            "wrist_1_joint": 0.0, "wrist_2_joint": 0.0, "wrist_3_joint": 0.0}

sim = SimulationContext(stage_units_in_meters=1.0)
stage_utils.add_reference_to_stage(USD, "/World/eoat")
GroundPlane("/World/groundPlane", z_position=0.0)   # collision ground at z=0
stg = omni.usd.get_context().get_stage()
UsdLux.DomeLight.Define(stg, Sdf.Path("/World/DomeLight")).CreateIntensityAttr(1200.0)

sim.reset()  # PLAY
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
art = SingleArticulation(ROOT); art.initialize()
names = list(art.dof_names)

# fixed home target for the arm (deg->rad); fingers cycled separately
home = np.zeros(len(names))
for n, d in HOME_DEG.items():
    if n in names:
        home[names.index(n)] = np.radians(d)
fl = names.index("gripper_2fg14__finger_left_joint")
fr = names.index("gripper_2fg14__finger_right_joint")

# track the lowest world-Z of the wrist/EOAT bodies (to detect floor contact)
cache = UsdGeom.XformCache()
track = [p for p in stg.Traverse()
         if p.HasAPI(UsdPhysics.RigidBodyAPI)
         and any(k in str(p.GetPath()) for k in
                 ("wrist_3", "eoat/tool0", "damper", "dual_quick_changer", "hex_qc",
                  "2fg14", "adapter", "screwdriver", "camera", "copick"))]

print("=" * 66, flush=True)
print(f"PLAY on ground plane (headless={HEADLESS}, {SECONDS}s). Arm -> HOME, "
      f"gripper cycling. Tracking {len(track)} EOAT/wrist bodies' min world-Z.", flush=True)
print("=" * 66, flush=True)

steps = int(SECONDS * 60)
min_z = 1e9
min_body = ""
shoulder_lift_i = names.index("shoulder_lift_joint")
i = 0
while _app.is_running():
    phase = (i % 240) / 240.0
    tgt = home.copy()
    grip = 0.025 * (1.0 - abs(2.0 * phase - 1.0))
    tgt[fl] = grip; tgt[fr] = grip
    art.apply_action(ArticulationAction(joint_positions=tgt))
    sim.step(render=not HEADLESS)
    cache.Clear()
    for p in track:
        z = float(cache.GetLocalToWorldTransform(p).ExtractTranslation()[2])
        if z < min_z:
            min_z = z; min_body = p.GetName()
    i += 1
    if HEADLESS and i >= steps:
        break

q = np.asarray(art.get_joint_positions())
sl = float(q[shoulder_lift_i])
print(f"RESULT after {i} steps:", flush=True)
print(f"  shoulder_lift = {sl:.4f} rad ({np.degrees(sl):.1f} deg)  "
      f"[home target -90deg; sag = {abs(np.degrees(sl)+90):.1f} deg]", flush=True)
print(f"  lowest EOAT/wrist body-origin world-Z = {min_z:.3f} m  (body: {min_body})", flush=True)
print(f"  FLOOR CONTACT (< 0.0 m): {'YES' if min_z < 0.0 else 'NO'}", flush=True)
if not HEADLESS:
    print("  (close the window to exit)", flush=True)
    while _app.is_running():
        i += 1
        phase = (i % 240) / 240.0
        tgt = home.copy()
        grip = 0.025 * (1.0 - abs(2.0 * phase - 1.0))
        tgt[fl] = grip; tgt[fr] = grip
        art.apply_action(ArticulationAction(joint_positions=tgt))
        sim.step(render=True)
_app.close()
