# SPDX-License-Identifier: Apache-2.0
"""Open an EOAT USD as the EDITABLE root stage in the Isaac GUI so parts can be
dragged/rotated with the gizmo and saved straight back to the file (Ctrl+S), ready
for extract_poses.py. (The ur16e_isaac_ros2.py loader REFERENCES the asset, so drags
become session overrides that need Save As — this opens it in-place instead.)

Move the LINK prims (damper, dual_quick_changer, hex_qc, gripper_2fg14,
qc_tool_side_B, adapter, screwdriver, camera_adapter, copick), not their `geo`
children. Then Ctrl+S, and run:
    /isaac-sim/python.sh extract_poses.py <this.usd> eoat_dualtool.yaml

Run:  /isaac-sim/python.sh edit_asset.py <asset.usd>
"""
import sys

from isaacsim import SimulationApp
_app = SimulationApp({"headless": False})

import omni.usd  # noqa: E402
from pxr import UsdLux, Sdf  # noqa: E402

ASSET = sys.argv[1]
ctx = omni.usd.get_context()
ctx.open_stage(ASSET)
_app.update()

# add a dome light on the SESSION layer only (visibility; not saved into the asset)
stage = ctx.get_stage()
sess = stage.GetSessionLayer()
stage.SetEditTarget(sess)
UsdLux.DomeLight.Define(stage, Sdf.Path("/SessionEditLight")).CreateIntensityAttr(1000.0)
stage.SetEditTarget(stage.GetRootLayer())   # edits (drags) go to the root layer -> saved by Ctrl+S

print("=" * 66, flush=True)
print("EDIT MODE: drag the LINK prims to mate the parts, then Ctrl+S.", flush=True)
print(f"  stage: {ASSET}", flush=True)
print("  then: /isaac-sim/python.sh extract_poses.py <this.usd> eoat_dualtool.yaml", flush=True)
print("=" * 66, flush=True)

while _app.is_running():
    _app.update()
_app.close()
