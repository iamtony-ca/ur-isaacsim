#!/usr/bin/env bash
# Reproduce the UR16e ROS 2 + Isaac Sim workspace on a fresh machine.
#
#   ./setup.sh                 # everything (asks nothing, refuses unsafe apt actions)
#   ./setup.sh --dry-run       # print what WOULD happen, change nothing
#   ./setup.sh base sources build       # only these stages
#   ./setup.sh --list          # show stages
#
# Env overrides:
#   WS=/path/to/ur_ws          workspace root       (default /isaac-sim/volume/ur_ws)
#   ALLOW_UPGRADES=1           permit apt to upgrade existing packages (see below)
#
# ---------------------------------------------------------------------------
# DESIGN RULE: this machine is shared with other workspaces.
#   * Every apt install is SIMULATED first; if it would upgrade or remove an
#     already-installed package the script REFUSES (ALLOW_UPGRADES=1 overrides).
#   * NVIDIA apt repos are PINNED BEFORE they are added, so they can only supply
#     packages nobody else provides -- they never shadow ROS/Ubuntu packages.
#   * Everything we build lands in the workspace overlay ($WS/install), never in
#     /opt/ros. Removing the workspace fully undoes the source-built parts.
# Manual/background for every stage: ../SETUP.md
# ---------------------------------------------------------------------------
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib.sh
source "$HERE/lib.sh"

# Every valid stage name...
STAGES=(preflight pin repos base cumotion sources build leader ml verify udev)
# ...and what a bare `./setup.sh` runs. `udev` is EXCLUDED on purpose: it is the
# only stage that writes outside the workspace (/etc/udev) and it is meaningless
# without a U2D2 attached. Ask for it explicitly on the real machine.
DEFAULT_STAGES=(preflight pin repos base cumotion sources build leader ml verify)
declare -A STAGE_DESC=(
  [preflight]="check the machine can host this workspace at all (read-only)"
  [pin]="apt pin so NVIDIA repos cannot shadow ROS/Ubuntu packages (do this FIRST)"
  [repos]="add Isaac ROS / CUDA / VPI apt repos + keys"
  [base]="core ROS packages (UR, MoveIt, ros2_control, robotiq_description, teleop)"
  [cumotion]="cuMotion + nvblox apt packages (individual, never the metapackages)"
  [sources]="vcs import + nvblox_core submodule"
  [build]="colcon build (ur_bringup, gripper driver, nvblox_ros for this GPU)"
  [leader]="OMY-L100 teleop leader stack (gravity-compensated ros2_control, 7 pkgs)"
  [udev]="U2D2 udev rule for the OMY-L100 leader (REAL HW only, writes /etc/udev)"
  [ml]="ML venv for the IL/VLA track (torch with sm_120 + lerobot), workspace-local"
  [verify]="run check_env.sh"
)

usage() { echo "usage: $0 [--dry-run] [--list] [stage ...]"; echo "stages: ${STAGES[*]}"; }

# ---------------------------------------------------------------------------
# Read-only. Fails fast on a fresh machine BEFORE anything is installed, because
# every failure below costs an hour to discover halfway through a build instead.
stage_preflight() {
  step "preflight — can this machine host the workspace? (read-only)"
  local fail=0

  # Isaac Sim. We target 6.0.1; 5.x needs no code change but was never re-verified.
  if [ -x /isaac-sim/python.sh ]; then
    local iv; iv="$(cat /isaac-sim/VERSION 2>/dev/null | cut -d+ -f1)"
    ok "Isaac Sim present (${iv:-version unknown})"
    case "$iv" in 6.*) : ;; "") warn "cannot read /isaac-sim/VERSION" ;;
      *) warn "Isaac $iv — this workspace is verified on 6.0.1" ;; esac
  else
    err "/isaac-sim/python.sh missing. Run inside the Isaac Sim container."; fail=1
  fi

  # ROS. We do NOT install ROS here: it is a base-image concern, and silently
  # installing a desktop distro on someone's machine is exactly the kind of
  # side effect this workspace forbids.
  if [ -d "/opt/ros/$ROS_DISTRO" ]; then ok "ROS 2 $ROS_DISTRO present"
  else
    err "/opt/ros/$ROS_DISTRO missing. Use an image with ROS 2 $ROS_DISTRO, or install it first."
    fail=1
  fi

  # GPU. sm_120 (Blackwell) is why nvblox gets source-built; see stage_build.
  local sm; sm="$(gpu_sm)"
  if [ -n "$sm" ]; then
    ok "GPU detected: $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1) (sm_$sm)"
    [ "$sm" -ge 89 ] 2>/dev/null && echo "     -> apt nvblox is sm_75-only; the build stage will compile nvblox_ros from source."
  else
    warn "no GPU visible (nvidia-smi failed). Isaac and cuMotion will not run."
  fi

  for c in git curl sudo; do need_cmd "$c" || fail=1; done
  command -v vcs >/dev/null 2>&1 && ok "vcstool present" \
    || warn "vcstool missing — the base stage installs python3-vcstool"

  # Disk: Isaac assets + the ML venv (~8 GB) + build trees.
  local free; free="$(df -BG --output=avail "$WS" 2>/dev/null | tail -1 | tr -dc '0-9')"
  if [ -n "$free" ]; then
    [ "$free" -ge 30 ] && ok "disk free: ${free} GiB" || warn "only ${free} GiB free; want >= 30"
  fi

  [ -f "$WS/src/ur16e.repos" ] && ok "workspace source tree looks right ($WS/src)" \
    || { err "$WS/src/ur16e.repos missing — clone the repo into $WS/src first"; fail=1; }

  [ "$fail" = 0 ] && ok "preflight passed" || { err "preflight FAILED — fix the above first"; return 1; }
}

# Set by stage_pin so stage_repos can tell "the pin is missing" from "the pin is being
# applied in this same invocation". Without it, --dry-run on a FRESH machine always dies
# at stage 2: stage_pin only prints "would write", so the file stage_repos looks for does
# not exist, and set -e turns that into "the whole plan stops here" -- which defeats the
# point of a dry run.
PIN_STAGED=0

stage_pin() {
  step "pin — isolate NVIDIA repos (Pin-Priority 100)"
  local f=/etc/apt/preferences.d/99-nvidia-isolate.pref
  if [ -f "$f" ]; then ok "$f already present"; PIN_STAGED=1; return 0; fi
  echo "  NVIDIA repos ship higher-versioned copies of ROS packages"
  echo "  (measured: robotiq_description 0.0.1 -> 9.0.1, moveit_task_constructor_core -> 99.99.0)."
  echo "  Priority 100 = never upgrade an already-installed package."
  PIN_STAGED=1
  if [ "$DRY_RUN" = 1 ]; then echo "  [dry-run] would write $f"; return 0; fi
  sudo tee "$f" >/dev/null <<'EOF'
# UR16e workspace: keep NVIDIA repos from shadowing ROS/Ubuntu packages.
# Priority 100 = "install only if no version of this package is installed".
# New packages (cuMotion/nvblox/CUDA/VPI) still install normally.
Package: *
Pin: origin isaac.download.nvidia.com
Pin-Priority: 100

Package: *
Pin: origin developer.download.nvidia.com
Pin-Priority: 100

Package: *
Pin: origin repo.download.nvidia.com
Pin-Priority: 100
EOF
  ok "wrote $f"
}

stage_repos() {
  step "repos — Isaac ROS / CUDA / VPI"
  [ -f /etc/apt/preferences.d/99-nvidia-isolate.pref ] || [ "$PIN_STAGED" = 1 ] || {
    err "pin missing. Run the 'pin' stage first (order matters)."; return 1; }
  need_cmd curl
  _repo() { # name key_url list_line
    local name="$1" key="$2" line="$3"
    if [ -f "/etc/apt/sources.list.d/$name.list" ]; then ok "$name already added"; return 0; fi
    if [ "$DRY_RUN" = 1 ]; then echo "  [dry-run] would add repo $name"; return 0; fi
    curl -fsSL "$key" | sudo gpg --dearmor --yes -o "/usr/share/keyrings/$name.gpg"
    echo "$line" | sudo tee "/etc/apt/sources.list.d/$name.list" >/dev/null
    ok "added $name"
  }
  _repo nvidia-isaac-ros "https://isaac.download.nvidia.com/isaac-ros/repos.key" \
    "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4 noble main"
  _repo nvidia-cuda "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/3bf863cc.pub" \
    "deb [signed-by=/usr/share/keyrings/nvidia-cuda.gpg] https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/ /"
  _repo nvidia-vpi "https://repo.download.nvidia.com/jetson/jetson-ota-public.asc" \
    "deb [signed-by=/usr/share/keyrings/nvidia-vpi.gpg] https://repo.download.nvidia.com/jetson/x86_64/noble r38.2 main"
  run "sudo apt-get update -qq"
  # Prove the pin works before anything gets installed. Only meaningful once the NVIDIA
  # repos are actually in sources.list -- in a dry run they are not, so the candidate
  # would be the ROS one and "pin verified" would be verifying nothing.
  if [ "$DRY_RUN" = 1 ]; then
    echo "  [dry-run] pin effectiveness is checked here on a real run"
    echo "            (apt-cache policy ros-$ROS_DISTRO-robotiq-description must NOT be 9.x)"
    return 0
  fi
  local cand
  cand="$(apt-cache policy "ros-$ROS_DISTRO-robotiq-description" 2>/dev/null | awk '/Candidate:/{print $2}')"
  case "$cand" in
    9.*) err "PIN NOT EFFECTIVE: robotiq_description candidate is $cand (NVIDIA). Fix the pin before continuing."; return 1 ;;
    "")  warn "could not read robotiq_description candidate (repo not refreshed yet?)" ;;
    *)   ok "pin verified: robotiq_description candidate = $cand (ROS repo)" ;;
  esac
}

stage_base() {
  step "base — core ROS stack"
  apt_guarded_install \
    "ros-$ROS_DISTRO-ur" "ros-$ROS_DISTRO-moveit" \
    "ros-$ROS_DISTRO-ros2-control" "ros-$ROS_DISTRO-ros2-controllers" \
    "ros-$ROS_DISTRO-ros2-control-cmake" \
    "ros-$ROS_DISTRO-robotiq-description" \
    "ros-$ROS_DISTRO-moveit-ros-perception" \
    "ros-$ROS_DISTRO-moveit-servo" \
    "ros-$ROS_DISTRO-joy" \
    python3-vcstool python3-colcon-common-extensions
  echo "  (optional, real D405 camera only)"
  apt_guarded_install "ros-$ROS_DISTRO-realsense2-camera" "ros-$ROS_DISTRO-librealsense2" || \
    warn "realsense install skipped/failed — only needed for the real camera"
}

stage_cumotion() {
  step "cumotion — GPU planning + nvblox (individual packages only)"
  echo "  NOT installing ros-$ROS_DISTRO-isaac-ros-cumotion-examples or -isaac-ros-nvblox:"
  echo "  those metapackages pull triton/tensor_rt/visual_slam (+400 debs) and force"
  echo "  container-wide python3.12 upgrades. We vendor the one yaml we need instead."
  apt_guarded_install \
    "ros-$ROS_DISTRO-isaac-ros-cumotion" \
    "ros-$ROS_DISTRO-isaac-ros-cumotion-moveit" \
    "ros-$ROS_DISTRO-isaac-ros-cumotion-robot-description" \
    "ros-$ROS_DISTRO-isaac-ros-cumotion-robot-segmenter" \
    "ros-$ROS_DISTRO-nvblox-ros" "ros-$ROS_DISTRO-nvblox-msgs" "ros-$ROS_DISTRO-nvblox-rviz-plugin"
}

stage_sources() {
  step "sources — vcs import + submodules"
  need_cmd vcs
  run "cd '$WS' && vcs import src < src/ur16e.repos"
  local nv="$WS/src/isaac_ros_nvblox"
  if [ -d "$nv" ]; then
    # nvblox_core is a git submodule; vcs import does NOT fetch it.
    run "cd '$nv' && git submodule update --init --recursive --depth 1"
    ok "nvblox_core submodule initialised"
  else
    warn "$nv missing — did vcs import run?"
  fi

  # Mask the open_manipulator packages we do not use, RIGHT AFTER import rather
  # than in the leader stage. Otherwise a plain `colcon build` run between the two
  # stages tries to build open_manipulator_gui and drags in Qt. Only the leader's
  # 5 packages plus its 2 sibling repos are ever wanted here.
  local om="$WS/src/open_manipulator" p
  if [ -d "$om" ]; then
    for p in open_manipulator open_manipulator_collision open_manipulator_gui \
             open_manipulator_moveit_config open_manipulator_playground \
             open_manipulator_teleop; do
      [ -d "$om/$p" ] && run "touch '$om/$p/COLCON_IGNORE'"
    done
    ok "COLCON_IGNORE set on 6 unused open_manipulator packages"
  fi
}

stage_build() {
  step "build — colcon"
  # ROS's setup.bash reads AMENT_TRACE_SETUP_FILES et al. without defaulting them,
  # so sourcing it under `set -u` aborts with "unbound variable". Drop -u across the
  # source only. (Invisible in --dry-run, which skips the source entirely -- this
  # cost a full fresh-machine run to find. See HISTORY.md 23.)
  if [ "$DRY_RUN" != 1 ]; then
    set +u; # shellcheck disable=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"; set -u
  fi
  run "cd '$WS' && colcon build --symlink-install --packages-up-to ur_bringup --cmake-args -DBUILD_TESTING=OFF"
  ok "ur_bringup + topic_based built"

  if [ -d "$WS/src/ros2_robotiq_gripper" ]; then
    run "cd '$WS' && colcon build --symlink-install --packages-select serial robotiq_driver robotiq_controllers --cmake-args -DBUILD_TESTING=OFF"
    ok "real 2F-85 gripper driver built"
  fi

  # nvblox: the apt binary is compiled for a fixed GPU arch list with no PTX.
  # Build from source only when this GPU is not covered.
  local sm archs nvbin="/opt/ros/$ROS_DISTRO/lib/nvblox_ros/nvblox_node"
  sm="$(gpu_sm)"; archs="$(cuda_archs_of "$nvbin")"
  echo "  GPU sm_${sm:-?} ; apt nvblox_node archs: ${archs:-<unknown>}"
  if [ -n "$sm" ] && [ -n "$archs" ] && grep -qw "sm_$sm" <<<"$archs"; then
    ok "apt nvblox supports this GPU — no source build needed"
  elif [ -d "$WS/src/isaac_ros_nvblox" ]; then
    warn "apt nvblox does NOT cover sm_${sm:-?} -> building nvblox_ros from source"
    local nvcc; nvcc="$(ls /usr/local/cuda*/bin/nvcc 2>/dev/null | head -1 || true)"
    [ -n "$nvcc" ] || { err "nvcc not found — run the cumotion stage first"; return 1; }
    run "cd '$WS' && CUDACXX='$nvcc' PATH=\"\$(dirname '$nvcc'):\$PATH\" colcon build --symlink-install --packages-select nvblox_ros --cmake-args -DUSE_NATIVE_CUDA_ARCHITECTURE=1 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
    ok "nvblox_ros built for sm_$sm (workspace overlay only)"
  else
    warn "nvblox source not present and apt build may not run on this GPU — see SETUP.md 2-B-4"
  fi
}

# ---------------------------------------------------------------------------
# OMY-L100 teleop leader (IL data collection). See plan_il_vla.md 3.5, SETUP.md 2-D.
#
# Only SEVEN of open_manipulator's 13 packages are built. The rest get COLCON_IGNORE:
# the gui pulls Qt, and moveit_config/collision/playground/teleop are OMY-specific.
# open_manipulator_bringup IS built even though its package.xml declares
# gz_ros2_control / ros_gz_* -- it is ament_python, so colcon never resolves those
# and Gazebo is neither installed nor needed. Its share dir has to exist because
# omy_l100.urdf.xacro does $(find open_manipulator_bringup).
stage_leader() {
  step "leader — OMY-L100 teleop leader stack"
  local om="$WS/src/open_manipulator"
  if [ ! -d "$om" ]; then
    warn "$om missing — run the 'sources' stage first"; return 0
  fi

  # Verified on 2026-09-06: `0 upgraded, 2 newly installed, 0 to remove`.
  # Both come from the OFFICIAL ROS repo, not the NVIDIA one (which the pin stage
  # holds at Priority 100 precisely so it cannot shadow packages like these).
  apt_guarded_install ros-jazzy-dynamixel-sdk ros-jazzy-dynamixel-interfaces || return 1

  # ROS's setup.bash reads AMENT_TRACE_SETUP_FILES et al. without defaulting them,
  # so sourcing it under `set -u` aborts with "unbound variable". Drop -u across the
  # source only. (Invisible in --dry-run, which skips the source entirely -- this
  # cost a full fresh-machine run to find. See HISTORY.md 23.)
  if [ "$DRY_RUN" != 1 ]; then
    set +u; # shellcheck disable=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"; set -u
  fi
  run "cd '$WS' && colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF --packages-select robotis_interfaces dynamixel_hardware_interface open_manipulator_description open_manipulator_bringup om_gravity_compensation_controller om_spring_actuator_controller om_joint_trajectory_command_broadcaster"
  ok "leader stack built (7 packages)"
}

# ---------------------------------------------------------------------------
# udev for the U2D2 (OMY-L100 leader). SEPARATE stage because it is the only
# thing here that writes outside the workspace -- /etc/udev/rules.d is system
# wide and the rule matches EVERY ftdi_sio tty, so on a shared machine it also
# touches other projects' FTDI devices. The effects (mode 0666, latency_timer 1)
# are benign-to-beneficial, but it is opt-in on purpose. Real hardware only.
stage_udev() {
  step "udev — U2D2 (OMY-L100 leader), real hardware only"
  local src="$WS/src/open_manipulator/open_manipulator_bringup/open-manipulator-cdc.rules"
  local dst="/etc/udev/rules.d/99-open-manipulator-cdc.rules"
  [ -f "$src" ] || { warn "$src missing — run the 'sources' stage first"; return 0; }
  echo "  rule contents:"; sed 's/^/    /' "$src"
  echo "  -> mode 0666 (no sudo to open the port) and latency_timer=1."
  echo "     The FTDI default is 16 ms, which throttles a 4 Mbps DYNAMIXEL sync-read"
  echo "     to roughly 60 Hz. This single attribute is the difference between a"
  echo "     300 Hz leader and an unusable one."
  run "sudo cp '$src' '$dst'"
  run "sudo udevadm control --reload-rules"
  run "sudo udevadm trigger"
  if [ "$DRY_RUN" != 1 ]; then
    local lt=/sys/bus/usb-serial/devices/ttyUSB0/latency_timer
    if [ -e "$lt" ]; then
      [ "$(cat "$lt")" = "1" ] && ok "ttyUSB0 latency_timer = 1" \
        || warn "ttyUSB0 latency_timer = $(cat "$lt") (expected 1) — replug the U2D2"
    else
      warn "no /dev/ttyUSB0 yet — plug the U2D2 in, then re-run this stage"
    fi
  fi
  echo "  smoke test WITHOUT the L100 attached:"
  echo "    ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py \\"
  echo "        use_mock_hardware:=true use_self_collision_avoidance:=false"
  echo "    ros2 control list_controllers -c /leader/controller_manager   # 4 active"
}

# ---------------------------------------------------------------------------
# ML environment. Deliberately a workspace-local venv:
#   * system python must not gain torch -- other projects share this container
#   * Isaac's bundled python must not gain torch either
#   * `rm -rf deps/.venv-ml` undoes the whole thing
stage_ml() {
  step "ml — IL/VLA training + LeRobot conversion venv"
  # NOTE: two statements, not `local venv=... py="$venv/..."`. Bash expands all
  # arguments to `local` BEFORE assigning any of them, so the second would see
  # an unset venv and die under `set -u` (it did: bootstrap --dry-run crashed here).
  local venv="$WS/deps/.venv-ml"
  local py="$venv/bin/python"
  local torch_index="${TORCH_INDEX:-https://download.pytorch.org/whl/cu128}"

  if [ ! -x "$py" ]; then
    if [ "$DRY_RUN" = 1 ]; then echo "  [dry-run] would create venv $venv"; return 0; fi
    # --without-pip is REQUIRED, not an optimisation. Plain `python3 -m venv` runs
    # ensurepip, which this image does not ship (python3-venv is not installed),
    # so it fails outright -- and installing python3-venv would drag a SYSTEM-WIDE
    # python3.12 upgrade (measured: 7 packages), which our own isolation rule
    # forbids. Create the venv empty, then bootstrap pip inside it below.
    python3 -m venv --without-pip "$venv" || { err "venv creation failed"; return 1; }
    ok "created $venv (no pip yet -- bootstrapped below)"
  else ok "venv already present: $venv"; fi
  [ "$DRY_RUN" = 1 ] && { echo "  [dry-run] would install torch + $HERE/requirements-ml.txt"; return 0; }

  # `python3 -m venv` here has no ensurepip, and apt-installing python3-venv drags
  # a SYSTEM-WIDE python3.12 upgrade (measured: 7 packages) -- refused by our own
  # isolation rule. Bootstrap pip inside the venv instead.
  if ! "$py" -m pip --version >/dev/null 2>&1; then
    warn "venv has no pip (ensurepip missing); bootstrapping with get-pip.py"
    local gp; gp="$(mktemp /tmp/get-pip-XXXX.py)"
    curl -fsSL https://bootstrap.pypa.io/get-pip.py -o "$gp" || { err "get-pip download failed"; return 1; }
    "$py" "$gp" -q || { err "pip bootstrap failed"; return 1; }
    rm -f "$gp"
  fi
  ok "pip: $("$py" -m pip --version | cut -d" " -f1-2)"

  # torch FIRST and from the CUDA index -- the default PyPI wheel may lack sm_120.
  echo "  installing torch from $torch_index (large: several GB) ..."
  "$py" -m pip install -q --index-url "$torch_index" torch torchvision || {
    err "torch install failed"; return 1; }
  _verify_torch_arch "$py" || return 1

  echo "  installing $HERE/requirements-ml.txt ..."
  "$py" -m pip install -q -r "$HERE/requirements-ml.txt" || {
    err "requirements-ml install failed"; return 1; }
  # Re-check: a later dependency can silently pull a different torch build.
  _verify_torch_arch "$py" || {
    err "torch was replaced by something without sm_120 -- reinstall it from $torch_index"; return 1; }

  _install_shm_workaround "$venv" || return 1

  ok "ML venv ready: $py"
  echo "  use it with:  $py -m ...   (never 'source' it into a ROS shell)"
}

# The Isaac Sim container ships /dev/shm at Docker's 64 MiB default. PyTorch's
# default 'file_descriptor' sharing strategy hands batches between DataLoader
# workers through /dev/shm, so ANY num_workers>0 dies partway into training:
#     RuntimeError: unable to allocate shared memory (shm) for file <...> (11)
#     RuntimeError: DataLoader worker (pid ...) exited unexpectedly
# Raising /dev/shm needs the container recreated, which we cannot do -- this
# container is shared with other projects (HISTORY.md 24).
#
# One ACT batch (8 x 2 cameras x 3x480x640 float32) is ~59 MiB, so /dev/shm holds
# barely one in-flight batch. Measured: 1 of 3 identical unfixed runs died, peak
# shm 43 MiB -- i.e. the failure is REAL but intermittent, which is worse than
# deterministic (a run can survive smoke-testing and then die hours into training).
#
# 'file_system' passes the same tensors as regular files in the temp dir instead.
# It must take effect in the WORKERS, not just the parent. Two traps:
#   - lerobot pins `dataloader_multiprocessing_context = "spawn"` (configs/train.py),
#     deliberately NOT inheriting parent state -- so a parent-side
#     set_sharing_strategy() never reaches the workers (measured: they still died).
#   - a sitecustomize.py in the venv is USELESS here: /usr/lib/python3.12/
#     sitecustomize.py already exists and the stdlib dir precedes site-packages on
#     sys.path, so ours is shadowed and silently never imported (measured).
# A .pth file has neither problem: `site` executes its `import` line in every
# interpreter that uses this venv, spawned workers included, and .pth files do not
# collide by name. It is gated on an env var so ordinary venv python start-up does
# not pay for importing torch; env vars propagate to spawned children for free.
#
# Measured on this box, ACT / batch 8 / 2 cameras / 200 steps:
#     num_workers=0            data_s 0.13   47 smp/s   40 s
#     file_system + 4 workers  data_s 0.001 161 smp/s   16 s   (identical loss)
_install_shm_workaround() {
  local venv="$1" sp
  sp="$("$venv/bin/python" -c 'import site; print(site.getsitepackages()[0])' 2>/dev/null)" \
    || { err "could not locate site-packages in $venv"; return 1; }

  # Remove the earlier, broken attempt if a previous setup run left one behind.
  [ -f "$sp/sitecustomize.py" ] && grep -q "ur_ws setup" "$sp/sitecustomize.py" 2>/dev/null \
    && rm -f "$sp/sitecustomize.py"

  cat > "$sp/ur_ws_shm_fix.py" <<'EOF'
"""Installed by ur_ws setup/setup.sh (stage `ml`) -- see SETUP.md 2-C.

/dev/shm is 64 MiB in this container (Docker default) and cannot be enlarged --
that needs the container recreated, and this one is shared with other projects.
PyTorch's default 'file_descriptor' sharing strategy moves DataLoader batches
through /dev/shm; one ACT batch is ~59 MiB, so workers intermittently die with
"unable to allocate shared memory". 'file_system' uses the temp dir instead.

Loaded via ur_ws_shm_fix.pth when UR_WS_TORCH_SHM_FIX=1, so it reaches spawned
DataLoader workers too. Unset the variable for stock behaviour.
"""
import torch.multiprocessing as _mp

_mp.set_sharing_strategy("file_system")
EOF

  # site executes any .pth line beginning with "import". Keep it on ONE line --
  # that is a hard requirement of the .pth format. Failures must stay silent:
  # a raising .pth breaks every interpreter in the venv.
  cat > "$sp/ur_ws_shm_fix.pth" <<'EOF'
import os; os.environ.get("UR_WS_TORCH_SHM_FIX") == "1" and __import__("importlib").import_module("ur_ws_shm_fix")
EOF

  if "$venv/bin/python" -c 'import sys' 2>&1 | grep -q .; then
    err "the .pth broke interpreter start-up -- removing it"
    rm -f "$sp/ur_ws_shm_fix.pth" "$sp/ur_ws_shm_fix.py"; return 1
  fi
  local got
  got="$(UR_WS_TORCH_SHM_FIX=1 "$venv/bin/python" -c \
        'import torch.multiprocessing as m; print(m.get_sharing_strategy())' 2>/dev/null)"
  if [ "$got" != "file_system" ]; then
    err "shm workaround did not take effect (strategy=$got) -- train with --num_workers=0"
    return 1
  fi
  ok "shm workaround active (/dev/shm is only $(df -h /dev/shm 2>/dev/null | awk 'NR==2{print $2}'):"
  echo "       run training with  UR_WS_TORCH_SHM_FIX=1  (see SETUP.md 2-C)"
}

_verify_torch_arch() {
  local py="$1" sm; sm="$(gpu_sm)"
  local out; out="$("$py" - <<'EOF' 2>&1
import torch
print(torch.__version__, torch.version.cuda, torch.cuda.is_available())
print(" ".join(torch.cuda.get_arch_list()))
EOF
)" || { err "torch import failed:"; echo "$out" | tail -5; return 1; }
  echo "  torch: $(head -1 <<<"$out")"
  echo "  archs: $(tail -1 <<<"$out")"
  if [ -z "$sm" ]; then warn "no GPU detected; skipping arch check"; return 0; fi
  if grep -qw "sm_$sm" <<<"$out"; then ok "torch supports this GPU (sm_$sm)"; return 0; fi
  err "torch has NO kernels for sm_$sm. Pick a CUDA index that does, e.g."
  err "  TORCH_INDEX=https://download.pytorch.org/whl/cu130 $0 ml"
  return 1
}

stage_verify() { step "verify"; "$HERE/check_env.sh"; }

# ---------------------------------------------------------------------------
main() {
  local want=()
  while [ $# -gt 0 ]; do
    case "$1" in
      --dry-run) DRY_RUN=1 ;;
      --list) for s in "${STAGES[@]}"; do printf "  %-10s %s\n" "$s" "${STAGE_DESC[$s]}"; done; return 0 ;;
      -h|--help) usage; return 0 ;;
      -*) usage; return 2 ;;
      *) want+=("$1") ;;
    esac
    shift
  done
  [ ${#want[@]} -eq 0 ] && want=("${DEFAULT_STAGES[@]}")

  echo "${C_H}UR16e workspace setup${C_0}"
  echo "  WS=$WS  ROS_DISTRO=$ROS_DISTRO  DRY_RUN=$DRY_RUN  ALLOW_UPGRADES=$ALLOW_UPGRADES"
  [ -d "$WS/src" ] || { err "workspace src not found at $WS/src (set WS=...)"; return 1; }

  local unsimulated=() rc=0
  for s in "${want[@]}"; do
    [[ " ${STAGES[*]} " == *" $s "* ]] || { err "unknown stage '$s' (see --list)"; return 2; }
    # `set -e` would abort the whole run on a stage that returns non-zero. That is right
    # for a real install and WRONG for a dry run: later stages legitimately cannot be
    # simulated because they depend on side effects earlier stages did not perform (no
    # NVIDIA repos in sources.list -> `apt-get install -s` for cuMotion cannot resolve,
    # no cumotion -> no nvcc for build, ...). Aborting there hides the rest of the plan,
    # which is the only thing a dry run is for.
    "stage_$s" && rc=0 || rc=$?
    [ "$rc" = 0 ] && continue
    if [ "$DRY_RUN" = 1 ]; then
      warn "stage '$s' could not be simulated (rc=$rc) — see above; continuing"
      unsimulated+=("$s")
    else
      err "stage '$s' FAILED (rc=$rc) — stopping."
      return "$rc"
    fi
  done

  if [ ${#unsimulated[@]} -gt 0 ]; then
    echo
    warn "not simulated: ${unsimulated[*]}"
    echo "  These depend on what an earlier stage would have done (NVIDIA repos added,"
    echo "  CUDA installed, sources imported...). A dry run does none of it, so apt/cmake"
    echo "  cannot resolve them yet. This is EXPECTED on a fresh machine and does not"
    echo "  mean the real run will fail. Re-run --dry-run after the real 'pin repos'"
    echo "  stages if you want to preview the rest."
  fi
  echo; ok "done: ${want[*]}"
  echo "  next: source /opt/ros/$ROS_DISTRO/setup.bash && source $WS/install/setup.bash"
}
main "$@"
