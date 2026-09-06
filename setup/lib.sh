#!/usr/bin/env bash
# Shared helpers for the UR16e workspace provisioning scripts.
#
# The one rule these encode: THIS MACHINE IS SHARED WITH OTHER WORKSPACES.
# Every apt action is simulated first and refused if it would upgrade or remove
# anything that is already installed. Adding brand-new packages is safe; changing
# existing ones is how you break somebody else's project.

set -euo pipefail

WS_DEFAULT="/isaac-sim/volume/ur_ws"
: "${WS:=$WS_DEFAULT}"
: "${ROS_DISTRO:=jazzy}"
: "${DRY_RUN:=0}"
: "${ALLOW_UPGRADES:=0}"

C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_H=$'\033[1m'; C_0=$'\033[0m'
ok()   { echo "${C_OK}  ok${C_0}   $*"; }
warn() { echo "${C_WARN}  warn${C_0} $*"; }
err()  { echo "${C_ERR}  FAIL${C_0} $*" >&2; }
step() { echo; echo "${C_H}== $*${C_0}"; }
run()  { if [ "$DRY_RUN" = 1 ]; then echo "  [dry-run] $*"; else eval "$@"; fi; }

need_cmd() { command -v "$1" >/dev/null 2>&1 || { err "missing command: $1"; return 1; }; }

apt_installed() { dpkg -l "$1" 2>/dev/null | grep -q "^ii"; }

# ---------------------------------------------------------------------------
# apt_guarded_install <pkg...>
#
# Simulates the install, prints the impact, and REFUSES if existing packages
# would be upgraded/removed (override with ALLOW_UPGRADES=1). This is the
# habit SETUP.md 2-B-2 describes, enforced in code.
# ---------------------------------------------------------------------------
apt_guarded_install() {
  local pkgs=("$@") missing=() sim summary upgrades removes
  for p in "${pkgs[@]}"; do apt_installed "$p" || missing+=("$p"); done
  if [ ${#missing[@]} -eq 0 ]; then ok "already installed: ${pkgs[*]}"; return 0; fi

  sim="$(apt-get install -s "${missing[@]}" 2>&1)" || { err "apt simulation failed:"; echo "$sim" | tail -20; return 1; }
  summary="$(grep -m1 'upgraded,' <<<"$sim" || true)"
  upgrades="$(grep '^Inst' <<<"$sim" | grep -E '\[[0-9]' | sed 's/ (.*//;s/^Inst //' || true)"
  removes="$(grep '^Remv' <<<"$sim" | sed 's/ (.*//;s/^Remv //' || true)"

  echo "  target : ${missing[*]}"
  echo "  impact : ${summary:-<none>}"
  if [ -n "$upgrades" ] || [ -n "$removes" ]; then
    [ -n "$upgrades" ] && { warn "would UPGRADE existing packages:"; sed 's/^/           /' <<<"$upgrades"; }
    [ -n "$removes" ]  && { warn "would REMOVE packages:";            sed 's/^/           /' <<<"$removes"; }
    if [ "$ALLOW_UPGRADES" != 1 ]; then
      err "refusing: this would change packages other workspaces may depend on."
      err "          Inspect the list above. To proceed anyway: ALLOW_UPGRADES=1 $0 ..."
      return 1
    fi
    warn "ALLOW_UPGRADES=1 -- proceeding despite the above"
  else
    ok "pure addition (0 upgraded, 0 removed)"
  fi
  run "sudo apt-get install -y ${missing[*]}"
}

# GPU compute capability, e.g. "120" for sm_120 (RTX 5090). Empty if no nvidia-smi.
gpu_sm() {
  command -v nvidia-smi >/dev/null 2>&1 || return 0
  nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null \
    | head -1 | tr -d ' .' || true
}

# Architectures baked into a CUDA binary, e.g. "sm_75" or "sm_120 sm_75 sm_86 sm_89".
cuda_archs_of() {
  local f="$1" cuobj
  cuobj="$(ls /usr/local/cuda*/bin/cuobjdump 2>/dev/null | head -1 || true)"
  [ -n "$cuobj" ] && [ -f "$f" ] || return 0
  "$cuobj" --list-elf "$f" 2>/dev/null | grep -oE 'sm_[0-9]+' | sort -u | tr '\n' ' '
}
