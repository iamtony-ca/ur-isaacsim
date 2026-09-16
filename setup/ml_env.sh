# Environment for every ML-venv command of the IL/VLA track (conversion, lerobot-train,
# policy_server / robot_client). `source` it in bash (ROS overlay may be sourced too):
#
#     source src/setup/ml_env.sh
#
# Why each line exists (details: SETUP.md 2-C / 2-C-2, HISTORY.md 40, 43, 45):
WS_ML_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# HuggingFace cache INSIDE the workspace -- never the shared ~/.cache/huggingface.
# GR00T-N1.7-3B (6.5 GB) and the Cosmos-Reason2-2B tokenizer live under deps/hf_cache/hub.
export HF_HOME="$WS_ML_ROOT/deps/hf_cache"

# Offline by default: on a reproduced PC the models are COPIED into deps/hf_cache by
# hand (SETUP.md 2-C-2 "hand-placed cache"), and lerobot loads the Cosmos tokenizer by
# repo id ("nvidia/Cosmos-Reason2-2B") -- without this flag that is a network call and
# a 401 if the token is missing. Set HF_HUB_OFFLINE=0 only for `hf download`.
export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export HF_HUB_ENABLE_HF_TRANSFER=0

export WANDB_DISABLED=true            # GrootConfig.report_to defaults to wandb
export TOKENIZERS_PARALLELISM=false   # tokenizer forks + DataLoader workers = deadlock warning

# GR00T data_s is the MAIN-PROCESS preprocessor; at torch's default thread count
# (= nproc, 20 here) small tensor ops oversubscribe and the step is 2x slower.
# 8 threads: data_s 0.85 -> 0.125 s (HISTORY.md 45.4). Harmless for ACT.
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-8}"

# A container started without --shm-size has a 64 MiB /dev/shm: file_system sharing for
# DataLoader workers (SETUP.md 2-C). Harmless when /dev/shm is large (check_env.sh says).
export UR_WS_TORCH_SHM_FIX=1
