#!/usr/bin/env bash
# Watchdog for a lerobot-train run. Emits one line per check, and a loud line on
# anything that a log-filter monitor cannot see: a process that died without a
# traceback, or one that is alive but no longer advancing (hung DataLoader, GPU
# stuck). Silence is not success -- this exists so that quiet == "checked, fine".
#   $1 log file   $2 total steps   $3 check interval s   $4 stall threshold s
L=$1; TOTAL=$2; EVERY=${3:-600}; STALL=${4:-900}
last_step=-1; last_change=$(date +%s)
step_of() { tr '\r' '\n' < "$L" 2>/dev/null | grep -oE "\| *[0-9]+/$TOTAL" | grep -oE "^\| *[0-9]+" | grep -oE "[0-9]+" | tail -1; }
while :; do
  sleep "$EVERY"
  now=$(date +%s)
  step=$(step_of); step=${step:-0}
  alive=$(pgrep -f "lerobot-train" | head -1)
  loss=$(tr '\r' '\n' < "$L" | grep -oE "loss:[0-9.eE+-]+" | tail -1)
  eta=$(tr '\r' '\n' < "$L" | grep -oE "$TOTAL \[[0-9:]+<[0-9:]+" | tail -1 | sed 's/.*<//')
  gpu=$(nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader,nounits | tr -d ' ')
  shm=$(df -k /dev/shm | tail -1 | awk '{printf "%dK", $3}')
  if grep -qE "^== rc=" "$L"; then
    echo "DONE $(grep -E '^== rc=' "$L") step=$step $loss"; exit 0
  fi
  if [ -z "$alive" ]; then
    echo "DEAD: lerobot-train process gone without rc line. step=$step $loss"; exit 1
  fi
  if [ "$step" -ne "$last_step" ]; then last_step=$step; last_change=$now; fi
  if [ $((now - last_change)) -ge "$STALL" ]; then
    echo "STALL: step $step unchanged for $((now - last_change))s (pid $alive). gpu=$gpu shm=$shm"
  else
    printf 'ok step %s/%s %s eta %s gpu %s shm %s\n' "$step" "$TOTAL" "$loss" "${eta:-?}" "$gpu" "$shm"
  fi
done
