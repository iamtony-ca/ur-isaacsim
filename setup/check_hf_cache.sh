#!/usr/bin/env bash
# Is the workspace-local HuggingFace cache complete for GR00T N1.7 -- OFFLINE?
# Read-only. Exit 0 = training/inference can start without network or token.
#
# Layout it expects (exactly what huggingface_hub writes, minus the optional blobs/):
#   deps/hf_cache/hub/models--nvidia--GR00T-N1.7-3B/refs/main            <- commit hash, no newline
#   deps/hf_cache/hub/models--nvidia--GR00T-N1.7-3B/snapshots/<hash>/     <- the files
#   deps/hf_cache/hub/models--nvidia--Cosmos-Reason2-2B/refs/main
#   deps/hf_cache/hub/models--nvidia--Cosmos-Reason2-2B/snapshots/<hash>/
# Files may be real files or symlinks; snapshot_download() resolves refs/main -> snapshot
# dir when HF_HUB_OFFLINE=1 (verified with a hand-built copy, HISTORY.md 45.8).
set -u
WS="$(cd "$(dirname "$0")/../.." && pwd)"
py="$WS/deps/.venv-ml/bin/python"
export HF_HOME="$WS/deps/hf_cache" HF_HUB_OFFLINE=1
rc=0
[ -x "$py" ] || { echo "  FAIL ML venv missing ($py)"; exit 1; }

check_repo() {   # $1 = repo dir name, $2... = required files
  local name="$1"; local d="$HF_HOME/hub/$1"; shift
  if [ ! -f "$d/refs/main" ]; then echo "  FAIL $d/refs/main missing"; rc=1; return; fi
  local h; h="$(tr -d '[:space:]' < "$d/refs/main")"
  local snap="$d/snapshots/$h"
  [ -d "$snap" ] || { echo "  FAIL snapshot dir missing: $snap"; rc=1; return; }
  local missing=()
  for f in "$@"; do [ -e "$snap/$f" ] && [ -s "$snap/$f" ] || missing+=("$f"); done
  if [ ${#missing[@]} -gt 0 ]; then echo "  FAIL $name @ $h missing/empty: ${missing[*]}"; rc=1
  else echo "  ok   $name @ ${h:0:12}  ($(du -shL "$snap" | cut -f1))"; fi
}
check_repo models--nvidia--GR00T-N1.7-3B config.json processor_config.json statistics.json \
  embodiment_id.json model.safetensors.index.json model-00001-of-00002.safetensors model-00002-of-00002.safetensors
check_repo models--nvidia--Cosmos-Reason2-2B config.json tokenizer.json tokenizer_config.json \
  vocab.json merges.txt preprocessor_config.json video_preprocessor_config.json
[ "$rc" = 0 ] || exit "$rc"

# The real test: resolve both by repo id and load the tokenizer + processors offline,
# exactly as lerobot's groot processor does (processor_groot.py: AutoTokenizer /
# Qwen2VLImageProcessor / Qwen3VLVideoProcessor .from_pretrained("nvidia/Cosmos-Reason2-2B")).
out="$("$py" - 2>&1 <<'PY'
from huggingface_hub import snapshot_download
for r in ("nvidia/GR00T-N1.7-3B", "nvidia/Cosmos-Reason2-2B"):
    snapshot_download(r)
from transformers import AutoTokenizer, Qwen2VLImageProcessor
from transformers.models.qwen3_vl.video_processing_qwen3_vl import Qwen3VLVideoProcessor
m = "nvidia/Cosmos-Reason2-2B"
t = AutoTokenizer.from_pretrained(m, trust_remote_code=True)
Qwen2VLImageProcessor.from_pretrained(m, trust_remote_code=True)
Qwen3VLVideoProcessor.from_pretrained(m, trust_remote_code=True)
assert len(t) == 151669, len(t)
print("OFFLINE_OK")
PY
)"
if grep -q OFFLINE_OK <<<"$out"; then echo "  ok   offline resolve + Cosmos tokenizer/processors load (vocab 151669)"
else echo "  FAIL offline load:"; grep -vE "torchcodec|libav|FFmpeg|^\s*$" <<<"$out" | tail -5 | sed 's/^/       /'; rc=1; fi
[ -f "$HF_HOME/token" ] && echo "  ok   HF token present (only needed to DOWNLOAD; training is offline)" \
                         || echo "  note no HF token (fine if the cache above is complete)"
exit $rc
