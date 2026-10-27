#!/usr/bin/env bash
# Stop everything this workspace started: shutdown_rollout.sh (Isaac, launches and all their
# nodes, policy server/client) PLUS the harness/collection loop scripts and every ML-venv
# process (lerobot-train, raw_to_lerobot.py, DataLoader workers). Same path-based matching,
# same verdict (last line CLEAN / LEFT: ..., exit 0 / 1) -- see shutdown_rollout.sh.
# Never kills this shell or its callers (groot_pipeline.sh runs it before training).
# usage: shutdown_all.sh [--dry-run]
exec bash "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/shutdown_rollout.sh" --all "$@"
