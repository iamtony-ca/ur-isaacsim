# Sourced by the collect / convert / train / rollout harnesses: ONE switch for how
# Isaac holds the part, and everything that has to agree with it (HISTORY.md 53).
#
#   GRASP_MODE=physical (default)  contact friction, soft gripper profile. New data
#                                  goes to *_phys paths, starts at the current READY.
#   GRASP_MODE=attach              the D5 weld. Every dataset/checkpoint made before
#                                  2026-10-19 (il_raw_red_left_100, act_red_left_100,
#                                  3task_v3, ...): reproduce or roll those out with this.
#
# Sets: GRASP_SFX       path suffix ("_phys" / "")
#       ISAAC_GRASP     Isaac arguments (array)
#       START_POSE      rollout start pose, unless already set
# and exports GRASP_MODE so collect_il_episodes.sh picks the matching state-machine
# overrides and records the gripper COMMAND as the action.
GRASP_MODE="${GRASP_MODE:-physical}"
case "$GRASP_MODE" in
  physical)
    GRASP_SFX="_phys"
    ISAAC_GRASP=(--grasp-mode physical)
    # Physical data is collected from the current READY (pick_place_demo.py).
    START_POSE="${START_POSE:-ready}" ;;
  attach)
    GRASP_SFX=""
    ISAAC_GRASP=(--grasp-attach)
    # The weld-era checkpoints were collected from the old READY (HISTORY.md 47).
    START_POSE="${START_POSE:-ready_v1}" ;;
  *) echo "GRASP_MODE must be physical or attach, got '$GRASP_MODE'" >&2; exit 2 ;;
esac
export GRASP_MODE START_POSE
# Every harness falls back to domain 0, which other containers on this host use
# (HISTORY.md 47). Say so instead of silently joining them.
[ -n "${ROS_DOMAIN_ID:-}" ] || echo "WARN: ROS_DOMAIN_ID unset -> domain 0 (shared host: export ROS_DOMAIN_ID=42)" >&2
