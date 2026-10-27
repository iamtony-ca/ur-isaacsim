#!/usr/bin/env bash
# Verify the UR16e workspace environment. Read-only — changes nothing.
#
#   ./check_env.sh             # full report, exit 1 if any REQUIRED check fails
#   ./check_env.sh --verbose   # same, plus the per-package apt drift list (normally one line)
#   ./check_env.sh --snapshot  # (re)write apt-versions.lock from THIS machine, then exit
#                              # -- the one mode that writes (a file next to this script)
#
# Use it on a fresh machine after setup.sh, and as a health check when something
# behaves oddly (it catches the failure modes that cost us the most time:
# GPU-arch mismatch, apt pin not effective, wrong topic_based version).
set -uo pipefail
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
# The workspace is wherever this file lives (<ws>/src/setup); WS=... overrides.
export WS="${WS:-$(cd "$HERE/../.." && pwd)}"
source "$HERE/lib.sh" 2>/dev/null || { echo "lib.sh missing"; exit 1; }
set +e   # keep going and report everything

# apt packages whose versions define "the verified stack" (apt-versions.lock). Everything
# setup.sh installs from packages.ros.org / NVIDIA, plus what our code imports. Not the
# Ubuntu base libs: those follow noble-updates and are not what breaks this workspace.
APT_LOCK="$HERE/apt-versions.lock"
APT_LOCK_RE="ros-$ROS_DISTRO-.*|ros2-apt-source|ros-dev-tools|python3-(vcstool|colcon-common-extensions|colcon-core|rosdep|scipy|matplotlib)|cuda-(toolkit|nvcc|cudart|libraries)-[0-9]+-[0-9]+|libnvvpi[0-9]+|vpi[0-9]+-dev"
apt_snapshot() {   # installed "pkg=version" lines of that set, sorted
  dpkg-query -W -f='${db:Status-Abbrev}${Package}=${Version}\n' 2>/dev/null \
    | awk '/^ii /{print $2}' | grep -E "^($APT_LOCK_RE)=" | sort
}
if [ "${1:-}" = "--snapshot" ]; then
  {
    echo "# apt-versions.lock -- installed versions of the apt stack this workspace was verified on."
    echo "# Written by: src/setup/check_env.sh --snapshot   ($(date +%F), Isaac $(cut -d+ -f1 /isaac-sim/VERSION 2>/dev/null || echo '?'))"
    echo "# Set: ros-$ROS_DISTRO-*, ROS tooling, CUDA toolkit, VPI, python3-scipy/matplotlib (APT_LOCK_RE in check_env.sh)."
    echo "# check_env.sh WARNs (never fails) on drift. packages.ros.org keeps ONLY the latest build, so"
    echo "# these exact versions are usually not installable later; this file says what changed, and"
    echo "# snapshots.ros.org (dated Jazzy snapshot) is the way to get them back if a drift breaks something."
    apt_snapshot
  } > "$APT_LOCK.tmp" && mv "$APT_LOCK.tmp" "$APT_LOCK" \
    && echo "wrote $APT_LOCK ($(grep -vc '^#' "$APT_LOCK") packages)" \
    || { echo "could not write $APT_LOCK"; rm -f "$APT_LOCK.tmp"; exit 1; }
  exit 0
fi
VERBOSE=0
case "${1:-}" in
  "") ;;
  -v|--verbose) VERBOSE=1; [ $# -eq 1 ] || { echo "usage: $0 [--verbose | --snapshot]"; exit 2; } ;;
  *) echo "usage: $0 [--verbose | --snapshot]"; exit 2 ;;
esac

FAILED=0
req()  { if [ "$1" = 0 ]; then ok "$2"; else err "$2"; FAILED=1; fi; }
opt()  { if [ "$1" = 0 ]; then ok "$2"; else warn "$2"; fi; }

step "platform"
echo "  $(. /etc/os-release && echo "$PRETTY_NAME")  kernel $(uname -r)"
[ -d "/opt/ros/$ROS_DISTRO" ]; req $? "ROS 2 $ROS_DISTRO present"
if command -v nvidia-smi >/dev/null; then
  nvidia-smi --query-gpu=name,driver_version,compute_cap --format=csv,noheader | sed 's/^/  GPU: /'
  # Driver vs the CUDA toolkit the cumotion stage installed (13.2 here). Within one major
  # version CUDA's minor-version compatibility covers an older driver (verified: driver
  # 580 = "CUDA 13.0" runs the 13.2 toolkit); a LOWER major (e.g. an R570 = 12.x driver)
  # cannot run CUDA 13 code at all -> cuMotion / source-built nvblox fail at start-up.
  drv_cuda="$(nvidia-smi 2>/dev/null | grep -oE 'CUDA Version: *[0-9]+\.[0-9]+' | grep -oE '[0-9]+\.[0-9]+' | head -1)"
  tk="$(grep -A2 '"cuda" *:' /usr/local/cuda/version.json 2>/dev/null | grep -oE '"version" *: *"[0-9.]+"' | grep -oE '[0-9]+(\.[0-9]+)+' | head -1)"
  if [ -n "$drv_cuda" ] && [ -n "$tk" ]; then
    if [ "${tk%%.*}" -gt "${drv_cuda%%.*}" ] 2>/dev/null; then
      warn "CUDA toolkit $tk needs a newer driver: this driver supports CUDA $drv_cuda (CUDA 13 needs >= R580) -> cuMotion/nvblox will not start"
    else ok "driver supports CUDA $drv_cuda; toolkit /usr/local/cuda = $tk (same major: minor-version compatible)"; fi
  fi
else warn "nvidia-smi not found"; fi
if [ -x /isaac-sim/python.sh ]; then
  iv="$(cat /isaac-sim/VERSION 2>/dev/null | cut -d+ -f1)"
  case "$iv" in
    6.0.1*|6.1.0-rc.26*) ok "Isaac Sim $iv (verified: 6.0.1, 6.1.0-rc.26)" ;;
    6.1.0*) warn "Isaac Sim $iv: only 6.1.0-rc.26 of the 6.1.0 line was verified (GA/other builds untested) -- run the SETUP.md 5 smoke" ;;
    *) warn "Isaac Sim ${iv:-?} is not a verified version (6.0.1 / 6.1.0) -- run the SETUP.md 5 smoke first" ;;
  esac
else warn "Isaac Sim not at /isaac-sim (sim stages will not run)"; fi
# Fresh-container trap: the ROS stack is unusable until `apt-get update` has run once,
# and python3 itself only arrives with ros-dev-tools. Both are cheap to report.
command -v python3 >/dev/null 2>&1; req $? "python3 present (the Isaac base image ships none; the ros stage installs it)"

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
# base_placement/ runs on system python3 (scipy: ur16e_kin.py, matplotlib: place_base.py --plot).
python3 -c "import scipy, matplotlib" 2>/dev/null; req $? "system python3: scipy + matplotlib importable (base_placement; base stage installs them)"

# Version drift vs the verified apt stack (apt-versions.lock). WARN only: a newer ROS
# build is usually fine, but when something regresses this is the first place to look
# (topic_based 0.2.1 vs ros2_control, the diagnostic_updater ABI trap, ...).
if [ -f "$APT_LOCK" ]; then
  drift="$(apt_snapshot | awk -F= -v lock="$APT_LOCK" '
    BEGIN { while ((getline l < lock) > 0) { if (l ~ /^#/ || l == "") continue
              i = index(l, "="); want[substr(l, 1, i-1)] = substr(l, i+1) } }
    { i = index($0, "="); have[substr($0, 1, i-1)] = substr($0, i+1) }
    END { for (p in want) {
            if (!(p in have)) { print "  missing   " p " (lock " want[p] ")"; continue }
            if (have[p] != want[p]) {
              wu = want[p]; hu = have[p]; sub(/-[^-]*$/, "", wu); sub(/-[^-]*$/, "", hu)
              print (wu == hu ? "  rebuild   " : "  VERSION   ") p " " want[p] " -> " have[p] } } }' | sort)"
  if [ -z "$drift" ]; then ok "apt stack matches apt-versions.lock ($(grep -vc '^#' "$APT_LOCK") packages)"
  else
    nver="$(grep -c '^  VERSION' <<<"$drift")"; nreb="$(grep -c '^  rebuild' <<<"$drift")"; nmis="$(grep -c '^  missing' <<<"$drift")"
    # One line by default: a FRESH install always drifts (packages.ros.org keeps only the
    # latest build -- measured 2026-10-21: 64 lines on a stock-image install), and the real
    # breakage this guards against is tested exactly by the ABI section below. The list is
    # what to read when something regresses: --verbose.
    if [ "$((nver + nmis))" = 0 ]; then
      ok "apt stack = apt-versions.lock up to $nreb same-version rebuild(s)"
    else
      warn "apt drift vs apt-versions.lock: $nver upstream version(s), $nreb rebuild(s), $nmis missing -- expected on a newer install; suspect it on regressions (list: $0 --verbose)"
    fi
    if [ "$VERBOSE" = 1 ]; then sed 's/^/     /' <<<"$drift"; fi
  fi
else warn "apt-versions.lock missing ($APT_LOCK) -- version drift not checked"; fi

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
# The real test of that trap: ask the dynamic linker. Cheap (tens of ms) and exact.
# ROS binaries carry no RUNPATH: without /opt/ros/<distro>/setup.bash in the environment
# ldd cannot find librclcpp & co. and reports ~56 bogus "undefined symbol"s, so resolve
# with ROS sourced in a subshell (this script is meant to run from a plain terminal too).
_ldd_r() { ( set +u; source "/opt/ros/$ROS_DISTRO/setup.bash" >/dev/null 2>&1; ldd -r "$1" 2>&1 ); }
for bin in "/opt/ros/$ROS_DISTRO/lib/controller_manager/ros2_control_node:req" \
           "/opt/ros/$ROS_DISTRO/lib/librealsense2_camera.so:opt"; do
  f="${bin%:*}"; kind="${bin##*:}"
  [ -e "$f" ] || continue
  und="$(_ldd_r "$f" | grep -cE 'undefined symbol|not found')"
  "$kind" "$([ "$und" = 0 ] && echo 0 || echo 1)" "no undefined symbols / missing libs: $(basename "$f") ($und) -- else partial-upgrade ABI mismatch, HARDWARE.md 4"
done

step "source dependencies"
tb="$WS/src/topic_based_hardware_interfaces"
if [ -d "$tb" ]; then
  v="$(grep -m1 '<version>' "$tb/joint_state_topic_hardware_interface/package.xml" 2>/dev/null | sed 's/.*<version>\(.*\)<.*/\1/')"
  [ "$v" = "0.2.1" ]; req $? "topic_based_hardware_interfaces = 0.2.1 (got '${v:-none}'; other versions make /joint_states all-NaN)"
else err "topic_based_hardware_interfaces missing — run setup.sh sources"; FAILED=1; fi
[ -d "$WS/src/ros2_robotiq_gripper" ]; opt $? "ros2_robotiq_gripper (real gripper only)"
# The build stage builds these whenever the source is there, so a missing install dir
# means that build FAILED (it used to be reported as "built"); the real 2F-85 needs them.
if [ -d "$WS/src/ros2_robotiq_gripper" ]; then
  for p in serial robotiq_driver robotiq_controllers; do
    [ -d "$WS/install/$p" ]; req $? "  built: $p (real 2F-85 driver; rebuild: setup/setup.sh build)"
  done
fi
if [ -d "$WS/src/isaac_ros_nvblox" ]; then
  [ -f "$WS/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/CMakeLists.txt" ]
  opt $? "nvblox_core submodule initialised"
fi

step "GPU architecture vs binaries"
sm="$(gpu_sm)"
if [ -n "$sm" ]; then
  echo "  this GPU: sm_$sm"
  # The apt nvblox_node is sm_75-only (no PTX). When the workspace's source-built nvblox_ros
  # covers this GPU it is the one that runs -- the overlay registers the same component
  # (nvblox::NvbloxNode) and shadows /opt/ros once install/setup.bash is sourced -- so the
  # apt binary's arch list is information, not a problem (it used to WARN on every
  # correct sm_120 install).
  ws_nvblox="$WS/install/nvblox_ros/lib/libnvblox_lib.so"
  ws_nvblox_ok=0
  [ -e "$ws_nvblox" ] && cuda_archs_of "$ws_nvblox" | grep -qw "sm_$sm" \
    && [ -f "$WS/install/nvblox_ros/share/ament_index/resource_index/rclcpp_components/nvblox_ros" ] \
    && ws_nvblox_ok=1
  for bin in "/opt/ros/$ROS_DISTRO/lib/nvblox_ros/nvblox_node:apt nvblox_node" \
             "$WS/install/nvblox_ros/lib/libnvblox_lib.so:workspace nvblox (source build)" \
             "/opt/ros/$ROS_DISTRO/lib/libcumotion.so:cuMotion engine"; do
    f="${bin%%:*}"; label="${bin##*:}"
    [ -e "$f" ] || continue
    a="$(cuda_archs_of "$f")"
    if [ -z "$a" ]; then warn "$label: could not read archs (cuobjdump missing?)"
    elif grep -qw "sm_$sm" <<<"$a"; then ok "$label supports sm_$sm  [$a]"
    elif [ "$label" = "apt nvblox_node" ] && [ "$ws_nvblox_ok" = 1 ]; then
      ok "$label lacks sm_$sm [$a] -- expected and unused: the workspace source build below shadows it"
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
    err "  TORCH_INDEX=https://download.pytorch.org/whl/cu130 setup/setup.sh ml   (keeps torch==2.11.0: lerobot 0.6.1 needs <2.12)"
    FAILED=1
  elif [ -n "$sm" ]; then
    ok "ML torch supports sm_$sm"
  fi
  "$mlpy" -c "import lerobot" 2>/dev/null; opt $? "lerobot importable (needed by raw_to_lerobot.py)"
  # [dataset] alone converts recordings fine but `lerobot-train` dies instantly on
  # "'accelerate' is required but not installed" -- so the gap hides until the first
  # real training run (HISTORY.md 24). Check the training half separately.
  "$mlpy" -c "import accelerate" 2>/dev/null
  opt $? "accelerate importable (lerobot[training] extra -- needed by lerobot-train)"
  # The robot adapter must be pip-installed INTO the venv (lerobot scans installed
  # `lerobot_robot_*` distributions); without it every rollout dies with
  # "--robot.type: invalid choice: 'ur16e_ros'" (HISTORY.md 48.8).
  "$mlpy" -c "import importlib.metadata as m; assert any(d.metadata['Name'].startswith('lerobot_robot_ur16e_ros') for d in m.distributions())" 2>/dev/null
  req $? "LeRobot robot plugin lerobot_robot_ur16e_ros installed in the venv (rollouts need --robot.type=ur16e_ros)"
  # Drift vs the verified venv (setup.sh installs with it as pip constraints unless
  # ML_UNLOCKED=1). Local tags (+cu128) and editable installs are ignored. WARN only.
  if [ -f "$HERE/requirements-ml.lock" ]; then
    mdrift="$(env -i HOME="$HOME" PATH=/usr/bin:/bin "$mlpy" -m pip freeze 2>/dev/null \
      | awk -v lock="$HERE/requirements-ml.lock" '
        function key(n) { n = tolower(n); gsub(/_/, "-", n); return n }
        function ver(v) { sub(/\+.*/, "", v); return v }
        BEGIN { while ((getline l < lock) > 0) { if (l !~ /==/ || l ~ /^#/) continue
                  split(l, a, "=="); want[key(a[1])] = ver(a[2]) } }
        /==/ { split($0, a, "=="); k = key(a[1]); if ((k in want) && ver(a[2]) != want[k]) print k " " want[k] " -> " ver(a[2]) }')"
    if [ -z "$mdrift" ]; then ok "ML venv matches requirements-ml.lock (packages present in both)"
    else warn "ML venv differs from requirements-ml.lock in $(wc -l <<<"$mdrift") package(s):"; head -10 <<<"$mdrift" | sed 's/^/       /'; fi
  fi

  # A container started without --shm-size has a 64 MiB /dev/shm, which cannot be
  # enlarged on a shared machine (a dedicated one: recreate with --shm-size=8g+).
  # Without the .pth workaround, any num_workers>0 then kills the DataLoader.
  # Test the real thing rather than the file's presence: the first attempt at this
  # workaround (a venv sitecustomize.py) LOOKED installed but was shadowed by
  # /usr/lib/python3.12/sitecustomize.py and never ran (HISTORY.md 24).
  shm_mb="$(df -m /dev/shm 2>/dev/null | awk 'NR==2{print $2}')"
  if [ -n "$shm_mb" ] && [ "$shm_mb" -lt 1024 ]; then
    strat="$(UR_WS_TORCH_SHM_FIX=1 "$mlpy" -c \
      'import torch.multiprocessing as m; print(m.get_sharing_strategy())' 2>/dev/null)"
    if [ "$strat" = "file_system" ]; then
      ok "/dev/shm is only ${shm_mb}M, but UR_WS_TORCH_SHM_FIX=1 gives file_system sharing"
    else
      err "/dev/shm is ${shm_mb}M and the file_system workaround is NOT working (got '${strat:-none}')."
      err "  lerobot-train with num_workers>0 will intermittently die on"
      err "  'unable to allocate shared memory' (measured: 2 of 3 runs)."
      err "  fix: setup/setup.sh ml   (or train with --num_workers=0)"
      FAILED=1
    fi
  else
    ok "/dev/shm is ${shm_mb:-?}M (large enough for default DataLoader sharing)"
  fi
  # Caches stay in deps/ (setup.sh ml: ur_ws_cache_paths.pth + resnet18 pre-fetch). Without
  # them ACT training/rollouts write ~/.cache/{torch,huggingface} and a rollout DOWNLOADS
  # resnet18 the first time (needs internet). WARN: functionally fine with a network.
  th="$(env -u TORCH_HOME -u HF_HOME -u PIP_CACHE_DIR "$mlpy" -c 'import os; print(os.environ.get("TORCH_HOME", ""))' 2>/dev/null)"
  [ -n "$th" ] && [ "$(readlink -m "$th")" = "$(readlink -m "$WS/deps/torch_home")" ]
  opt $? "venv caches default into deps/ (TORCH_HOME=${th:-~/.cache/torch}; else: setup/setup.sh ml)"
  [ -f "$WS/deps/torch_home/hub/checkpoints/resnet18-f37072fd.pth" ]
  opt $? "ACT backbone resnet18 IMAGENET1K_V1 in deps/torch_home (else every first rollout downloads it: setup/setup.sh ml)"
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

step "GR00T N1.7 (VLA — optional, setup/setup.sh groot)"
# `import lerobot.policies.groot.modeling_groot` alone passes on a venv WITHOUT the groot
# extra (its heavy imports are lazy) -- seen 2026-09-16 when verify ran before `groot`.
# transformers is only installed by the extra, so require it explicitly.
if [ -x "$mlpy" ] && "$mlpy" -c "import transformers, lerobot.policies.groot.modeling_groot" 2>/dev/null; then
  ok "lerobot[groot] extra importable (transformers $("$mlpy" -c 'import transformers;print(transformers.__version__)' 2>/dev/null))"
  # Offline check of the workspace-local HF cache -- the models are usually copied in by
  # hand on a reproduced PC, and training runs with HF_HUB_OFFLINE=1 (setup/ml_env.sh).
  if hfout="$(WS="$WS" "$WS/src/setup/check_hf_cache.sh" 2>&1)"; then ok "deps/hf_cache complete: GR00T-N1.7-3B + Cosmos-Reason2-2B tokenizer (offline)"
  else
    # The models are an HF-gated user step (CHECKLIST B-2), so this is a WARN; show what
    # is missing without check_hf_cache's own FAIL wording in an otherwise passing report.
    warn "deps/hf_cache incomplete -> SETUP.md 2-C-2 (copy by hand or hf download)"
    sed -E 's/^ *FAIL */       missing: /; s/ missing$//' <<<"$hfout" | grep 'missing:' | head -4
  fi
  nth="$(nproc)"
  [ "$nth" -gt 12 ] && warn "nproc=$nth: run GR00T training with OMP_NUM_THREADS=8 (ml_env.sh does) — default threads = 2x slower (HISTORY.md 45.4)"
else
  warn "lerobot[groot] not installed (setup/setup.sh groot). Only needed for the GR00T N1.7 stage."
fi

step "OMY-L100 teleop leader stack (IL data collection; required once its source is imported)"
# The leader stage builds these whenever src/open_manipulator exists (bootstrap runs it by
# default). A missing install then means the build FAILED -- it used to print "built" and
# this section only warned, so a broken leader build still ended in "environment OK".
if [ -d "$WS/src/open_manipulator" ] || [ -d "$WS/install/open_manipulator_description" ]; then
  for p in open_manipulator_description open_manipulator_bringup dynamixel_hardware_interface \
           om_gravity_compensation_controller om_spring_actuator_controller \
           om_joint_trajectory_command_broadcaster robotis_interfaces; do
    [ -d "$WS/install/$p" ]; req $? "built: $p (rebuild: setup/setup.sh leader)"
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
  warn "leader stack source not imported (setup/setup.sh sources leader). Only needed for OMY-L100 teleop."
fi

step "OMY-L100 real-hardware readiness (skipped unless a U2D2 is plugged in)"
# All warnings, never failures: the sim half of this workspace does not need any
# of it. These are the things that bite on the REAL machine (HARDWARE.md 4-B).
# udev is a HOST daemon: inside a container the rule must be installed on the host
# (HARDWARE.md 4-B (6)); the container's own /etc/udev is never read, and the
# /dev/ttyUSB0 node inside the container keeps the MODE it had when the container
# started -- a rule applied on the host afterwards does not change it until the
# container is restarted (seen 2026-09-17: latency_timer 1 but node still 0660).
in_container=0
{ [ -f /.dockerenv ] || grep -qE 'docker|containerd|kubepods' /proc/1/cgroup 2>/dev/null; } && in_container=1
if [ -e /dev/ttyUSB0 ]; then
  ok "/dev/ttyUSB0 present"
  lt=/sys/bus/usb-serial/devices/ttyUSB0/latency_timer
  if [ -e "$lt" ]; then
    # FTDI defaults to 16 ms, which throttles a 4 Mbps DYNAMIXEL sync-read to
    # ~60 Hz. The udev rule sets it to 1.
    if [ "$(cat "$lt")" = "1" ]; then ok "latency_timer = 1"
    elif [ "$in_container" = 1 ]; then warn "latency_timer = $(cat "$lt"), expected 1 -> on the HOST: setup/setup.sh udev prints the commands; then replug"
    else warn "latency_timer = $(cat "$lt"), expected 1 -> run: setup/setup.sh udev, then replug"; fi
  fi
  if [ -r /dev/ttyUSB0 ] && [ -w /dev/ttyUSB0 ]; then ok "/dev/ttyUSB0 readable+writable"
  elif [ "$in_container" = 1 ]; then
    warn "/dev/ttyUSB0 not accessible ($(stat -c '%A %U:%G' /dev/ttyUSB0)) -> now: sudo chmod 666 /dev/ttyUSB0 ; permanently: host udev rule (mode 0666) + restart the container"
  else warn "/dev/ttyUSB0 not accessible -> setup/setup.sh udev (rule sets mode 0666)"; fi
else
  warn "no /dev/ttyUSB0 — U2D2 not plugged in. Sim needs none of this."
fi
if [ "$in_container" = 1 ]; then
  echo "       udev rule: lives on the HOST when running in a container (not checked here; latency_timer above is the proof)"
else
  [ -f /etc/udev/rules.d/99-open-manipulator-cdc.rules ] \
    && ok "udev rule installed" || warn "udev rule absent -> setup/setup.sh udev (real HW only)"
fi
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
