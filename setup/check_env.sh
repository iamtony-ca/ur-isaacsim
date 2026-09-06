#!/usr/bin/env bash
# Verify the UR16e workspace environment. Read-only — changes nothing.
#
#   ./check_env.sh          # full report, exit 1 if any REQUIRED check fails
#
# Use it on a fresh machine after setup.sh, and as a health check when something
# behaves oddly (it catches the failure modes that cost us the most time:
# GPU-arch mismatch, apt pin not effective, wrong topic_based version).
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib.sh" 2>/dev/null || { echo "lib.sh missing"; exit 1; }
set +e   # keep going and report everything

FAILED=0
req()  { if [ "$1" = 0 ]; then ok "$2"; else err "$2"; FAILED=1; fi; }
opt()  { if [ "$1" = 0 ]; then ok "$2"; else warn "$2"; fi; }

step "platform"
echo "  $(. /etc/os-release && echo "$PRETTY_NAME")  kernel $(uname -r)"
[ -d "/opt/ros/$ROS_DISTRO" ]; req $? "ROS 2 $ROS_DISTRO present"
if command -v nvidia-smi >/dev/null; then
  nvidia-smi --query-gpu=name,driver_version,compute_cap --format=csv,noheader | sed 's/^/  GPU: /'
else warn "nvidia-smi not found"; fi
if [ -x /isaac-sim/python.sh ]; then
  echo "  Isaac Sim: $(cat /isaac-sim/VERSION 2>/dev/null || echo '?')"
else warn "Isaac Sim not at /isaac-sim (sim stages will not run)"; fi

step "isolation (shared machine)"
[ -f /etc/apt/preferences.d/99-nvidia-isolate.pref ]; req $? "NVIDIA apt pin present"
if ls /etc/apt/sources.list.d/nvidia-isaac-ros.list >/dev/null 2>&1; then
  cand="$(apt-cache policy "ros-$ROS_DISTRO-robotiq-description" 2>/dev/null | awk '/Candidate:/{print $2}')"
  case "$cand" in
    9.*) err "pin INEFFECTIVE — robotiq_description candidate $cand comes from NVIDIA"; FAILED=1 ;;
    "")  warn "robotiq_description candidate unknown" ;;
    *)   ok "pin effective — robotiq_description candidate $cand (ROS repo)" ;;
  esac
fi
for meta in isaac-ros-nvblox isaac-ros-cumotion-examples; do
  if apt_installed "ros-$ROS_DISTRO-$meta"; then
    warn "ros-$ROS_DISTRO-$meta is installed — it drags in ~400 debs and forces python3.12 upgrades"
  else ok "metapackage ros-$ROS_DISTRO-$meta not installed (intended)"; fi
done

step "apt packages"
# Check the real component packages, not the `ros-<distro>-ur` metapackage:
# apt does not keep metapackages installed once their deps are satisfied by
# other means, so testing for it gives false failures.
for p in ur-robot-driver ur-description ur-moveit-config ur-controllers \
         moveit ros2-control ros2-controllers robotiq-description moveit-servo joy; do
  apt_installed "ros-$ROS_DISTRO-$p"; req $? "ros-$ROS_DISTRO-$p"
done
for p in realsense2-camera isaac-ros-cumotion isaac-ros-cumotion-moveit \
         isaac-ros-cumotion-robot-segmenter nvblox-ros; do
  apt_installed "ros-$ROS_DISTRO-$p"; opt $? "ros-$ROS_DISTRO-$p (optional: camera / GPU planning)"
done

step "ABI consistency (ros2_control <-> diagnostic_updater)"
# Compare UPSTREAM versions only: the Debian revision carries a per-package build
# timestamp (…-1noble.20260615.164916) that differs even within one release.
cm="$(dpkg-query -W -f='${Version}' "ros-$ROS_DISTRO-controller-manager" 2>/dev/null | cut -d- -f1)"
hi="$(dpkg-query -W -f='${Version}' "ros-$ROS_DISTRO-hardware-interface" 2>/dev/null | cut -d- -f1)"
echo "  controller-manager=$cm  hardware-interface=$hi"
[ -n "$cm" ] && [ "$cm" = "$hi" ]; req $? "controller-manager / hardware-interface upstream versions match"
# The trap this guards: a partial upgrade (e.g. cuMotion pulling a newer
# diagnostic_updater) leaves controller_manager dying with
#   undefined symbol: diagnostic_updater::Updater
echo "  diagnostic-updater=$(dpkg-query -W -f='${Version}' "ros-$ROS_DISTRO-diagnostic-updater" 2>/dev/null)"

step "source dependencies"
tb="$WS/src/topic_based_hardware_interfaces"
if [ -d "$tb" ]; then
  v="$(grep -m1 '<version>' "$tb/joint_state_topic_hardware_interface/package.xml" 2>/dev/null | sed 's/.*<version>\(.*\)<.*/\1/')"
  [ "$v" = "0.2.1" ]; req $? "topic_based_hardware_interfaces = 0.2.1 (got '${v:-none}'; other versions make /joint_states all-NaN)"
else err "topic_based_hardware_interfaces missing — run setup.sh sources"; FAILED=1; fi
[ -d "$WS/src/ros2_robotiq_gripper" ]; opt $? "ros2_robotiq_gripper (real gripper only)"
if [ -d "$WS/src/isaac_ros_nvblox" ]; then
  [ -f "$WS/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/CMakeLists.txt" ]
  opt $? "nvblox_core submodule initialised"
fi

step "GPU architecture vs binaries"
sm="$(gpu_sm)"
if [ -n "$sm" ]; then
  echo "  this GPU: sm_$sm"
  for bin in "/opt/ros/$ROS_DISTRO/lib/nvblox_ros/nvblox_node:apt nvblox_node" \
             "$WS/install/nvblox_ros/lib/libnvblox_lib.so:workspace nvblox (source build)" \
             "/opt/ros/$ROS_DISTRO/lib/libcumotion.so:cuMotion engine"; do
    f="${bin%%:*}"; label="${bin##*:}"
    [ -e "$f" ] || continue
    a="$(cuda_archs_of "$f")"
    if [ -z "$a" ]; then warn "$label: could not read archs (cuobjdump missing?)"
    elif grep -qw "sm_$sm" <<<"$a"; then ok "$label supports sm_$sm  [$a]"
    else warn "$label does NOT support sm_$sm  [$a]"; fi
  done
  if [ -e "/opt/ros/$ROS_DISTRO/lib/nvblox_ros/nvblox_node" ] \
     && ! cuda_archs_of "/opt/ros/$ROS_DISTRO/lib/nvblox_ros/nvblox_node" | grep -qw "sm_$sm" \
     && [ ! -e "$WS/install/nvblox_ros/lib/libnvblox_lib.so" ]; then
    err "apt nvblox cannot run on this GPU and no source build exists -> nvblox_node will abort with"
    err "  'cudaErrorInvalidDevice: invalid device ordinal'.  Run: setup.sh build   (see SETUP.md 2-B-4)"
    FAILED=1
  fi
fi

step "workspace build"
[ -f "$WS/install/setup.bash" ]; req $? "workspace built ($WS/install)"
for p in ur_bringup joint_state_topic_hardware_interface; do
  [ -d "$WS/install/$p" ]; req $? "  package $p installed"
done

step "ML environment (IL/VLA track — optional)"
mlpy="$WS/deps/.venv-ml/bin/python"
if [ -x "$mlpy" ]; then
  ok "ML venv present: $mlpy"
  out="$("$mlpy" -c '
try:
    import torch
    print("torch", torch.__version__, "cuda", torch.version.cuda, "avail", torch.cuda.is_available())
    print("ARCHS " + " ".join(torch.cuda.get_arch_list()))
except Exception as e:
    print("ERR", e)
' 2>&1)"
  echo "$out" | sed 's/^/  /'
  sm="$(gpu_sm)"
  if grep -q "^ERR" <<<"$out"; then
    err "torch unusable in the ML venv"; FAILED=1
  elif [ -n "$sm" ] && ! grep -q "sm_$sm" <<<"$out"; then
    err "ML torch has NO kernels for sm_$sm -> reinstall from a CUDA index that does:"
    err "  TORCH_INDEX=https://download.pytorch.org/whl/cu130 setup/setup.sh ml"
    FAILED=1
  elif [ -n "$sm" ]; then
    ok "ML torch supports sm_$sm"
  fi
  "$mlpy" -c "import lerobot" 2>/dev/null; opt $? "lerobot importable (needed by raw_to_lerobot.py)"
  # The whole point of the venv: system / Isaac python must stay clean.
  if python3 -c "import torch" 2>/dev/null; then
    err "torch leaked into SYSTEM python (isolation broken)"; FAILED=1
  else ok "system python has no torch (isolation intact)"; fi
  if [ -x /isaac-sim/python.sh ]; then
    if /isaac-sim/python.sh -c "import torch" >/dev/null 2>&1; then
      warn "torch importable in Isaac's python — installed there by mistake?"
    else ok "Isaac python has no torch (isolation intact)"; fi
  fi
else
  warn "ML venv not built (run: setup/setup.sh ml). Only needed for IL/VLA training + LeRobot conversion."
fi

step "OMY-L100 teleop leader stack (optional — IL data collection)"
if [ -d "$WS/install/open_manipulator_description" ]; then
  for p in open_manipulator_description open_manipulator_bringup dynamixel_hardware_interface \
           om_gravity_compensation_controller om_spring_actuator_controller \
           om_joint_trajectory_command_broadcaster robotis_interfaces; do
    [ -d "$WS/install/$p" ]; opt $? "built: $p"
  done
  # The leader URDF is only usable through the xacro: the flat omy_l100.urdf carries
  # NO ros2_control block, and the xacro resolves $(find open_manipulator_bringup).
  [ -f "$WS/install/open_manipulator_description/share/open_manipulator_description/urdf/omy_l100/omy_l100.urdf.xacro" ]
  opt $? "omy_l100.urdf.xacro present (flat .urdf has no ros2_control — do not use it)"
  # Gazebo must NOT have been dragged in: bringup declares gz_ros2_control as an
  # exec dep, but it is ament_python so colcon never resolves it. If ros_gz appears
  # here, something ran rosdep and the isolation guarantee is broken.
  if dpkg-query -W -f='${Package}\n' 'ros-jazzy-ros-gz*' 2>/dev/null | grep -q .; then
    warn "ros_gz packages installed — did something run rosdep? (leader stack does NOT need Gazebo)"
  else ok "no Gazebo pulled in (leader stack needs none)"; fi
  for p in ros-jazzy-dynamixel-sdk ros-jazzy-dynamixel-interfaces; do
    dpkg-query -W "$p" >/dev/null 2>&1; opt $? "apt: $p"
  done
else
  warn "leader stack not built (run: setup/setup.sh leader). Only needed for OMY-L100 teleop."
fi

step "OMY-L100 real-hardware readiness (skipped unless a U2D2 is plugged in)"
# All warnings, never failures: the sim half of this workspace does not need any
# of it. These are the things that bite on the REAL machine (HARDWARE.md 4-B).
if [ -e /dev/ttyUSB0 ]; then
  ok "/dev/ttyUSB0 present"
  lt=/sys/bus/usb-serial/devices/ttyUSB0/latency_timer
  if [ -e "$lt" ]; then
    # FTDI defaults to 16 ms, which throttles a 4 Mbps DYNAMIXEL sync-read to
    # ~60 Hz. The udev rule sets it to 1.
    [ "$(cat "$lt")" = "1" ] && ok "latency_timer = 1" \
      || warn "latency_timer = $(cat "$lt"), expected 1 -> run: setup/setup.sh udev, then replug"
  fi
  [ -r /dev/ttyUSB0 ] && [ -w /dev/ttyUSB0 ] && ok "/dev/ttyUSB0 readable+writable" \
    || warn "/dev/ttyUSB0 not accessible -> setup/setup.sh udev (rule sets mode 0666)"
else
  warn "no /dev/ttyUSB0 — U2D2 not plugged in. Sim needs none of this."
fi
[ -f /etc/udev/rules.d/99-open-manipulator-cdc.rules ] \
  && ok "udev rule installed" || warn "udev rule absent -> setup/setup.sh udev (real HW only)"
# The leader's three motors must have model files or the driver cannot start.
dxl="$WS/install/dynamixel_hardware_interface/share/dynamixel_hardware_interface/param/dxl_model"
if [ -d "$dxl" ]; then
  for m in xh540_w150 xc330_t288 xc330_t181; do
    ls "$dxl" 2>/dev/null | grep -q "^${m}" ; opt $? "DYNAMIXEL model: $m"
  done
fi

step "ros2 run executables"
# --symlink-install reuses the SOURCE file's permissions, so install(PROGRAMS)
# alone is not enough: without +x on the source, `ros2 run` says
# "No executable found". Cheap to check, annoying to debug.
for s in "$WS"/src/ur_bringup/scripts/*.py; do
  [ -x "$s" ]; opt $? "executable bit: $(basename "$s")"
done

step "workspace config sanity"
cfg="$WS/install/ur_bringup/share/ur_bringup/config"
[ -f "$cfg/common/ur16e_servo.yaml" ]; opt $? "servo config present (teleop)"
if [ -f "$cfg/ur16e_2f85_d405/nvblox_cumotion.yaml" ]; then
  grep -q 'esdf_and_gradients_unobserved_value' "$cfg/ur16e_2f85_d405/nvblox_cumotion.yaml"
  opt $? "nvblox unobserved-voxel override set (else cuMotion rejects every start pose)"
fi
[ -f "$cfg/ur16e_2f85_d405/vendor/nvblox_base.yaml" ]; opt $? "vendored nvblox_base.yaml (avoids nvblox_examples_bringup)"

echo
if [ "$FAILED" = 0 ]; then ok "environment OK"; else err "environment has REQUIRED failures (see above)"; fi
exit "$FAILED"
