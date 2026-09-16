# harness — 수집 → 변환 → 학습 → 롤아웃 자동화 스크립트

`HISTORY.md` §28~§45 의 실측을 만든 스크립트들. 2026-09-13 까지 세션 임시 폴더에 있던 것을 repo 로 옮겼다
(§45.9). **모델·데이터 경로 코드가 아니라 실행 순서·사전검사·채점**이다 — 정책/데이터셋은 전부 upstream lerobot.

## 공통 규칙

- **경로**: 워크스페이스 루트는 스크립트 위치에서 유추(`src/ur_bringup/scripts/harness` → 4단계 위). 다른 곳에
  두었으면 `UR_WS=/path/to/ws`. ROS 는 `/opt/ros/jazzy`, Isaac 은 `/isaac-sim/python.sh` 고정.
- **로그**: `outputs/harness_logs/`(소스 트리에 쓰지 않는다). `HARNESS_LOG=` 로 변경.
- **ML 환경**: GR00T·벤치 스크립트는 `src/setup/ml_env.sh` 를 스스로 source 한다(HF 캐시·오프라인·`OMP_NUM_THREADS=8`).
- **인터프리터**: `.sh` 는 `bash <파일>` 로(모두 실행권한 있음). `.py` 는 두 부류 —
  ROS python(`python3`): `obs_ready.py` `judge_rollout.py` `judge_task.py` `grasp_watch.py` /
  ML venv(`deps/.venv-ml/bin/python`): `bench_*.py` `groot_latency.py` `groot_mem.py` `groot_probe.py`.
- **종료**: `shutdown_rollout.sh`(정책 서버·클라이언트·Isaac·런치·컨트롤러, ros2 daemon) / `shutdown_all.sh`.
  공유 머신이라 광범위 `pkill` 은 없다 — 워크로드 패턴만(CLAUDE.md).
- 기동 순서·컨트롤러 spawn·관측 검사·청크 검사 같은 **사전조건 검사가 하네스의 본체**다. 빠지면 "정책이 아무것도
  안 함" 으로 오진된다(`HISTORY.md` §44.2 결함 5개).

## ACT (카메라 2대 / 손목 1대)

| 단계 | 2대 (`red_left_100`, 9/10) | 손목 1대 (`wrist_only`, 8/9) |
|---|---|---|
| 수집 (Isaac + 상태머신, `grasp_watch.py` 감시) | `collect_100.sh` | `collect_wrist1.sh` |
| 변환 (`raw_to_lerobot.py`) | `convert_100.sh` | `convert_wrist1.sh` |
| 학습 (60k, 워커 4/2/0 사다리, shm 픽스) | `train_100.sh` | `train_wrist1.sh` |
| 롤아웃 (Isaac 재기동 + `bringup.sh` + `policy_inference.launch.py` + N회 채점) | `rollout_gui_then_n.sh [N] [초] [ckpt]` → `rollout_n.sh` | `rollout_gui_then_n_w1.sh [N] [초] [ckpt]` → `rollout_wrist1.sh` |
| 전체 | `pipeline_100.sh` | `pipeline_wrist1.sh` |

채점 `judge_rollout.py`(GT 로 물체–마커 거리 + 그리퍼 개방). **시작 자세 `START_POSE`**(기본 `ready_v1` = 2026-09-16
이전 데이터셋의 자세; 그 뒤 수집한 데이터셋은 `START_POSE=ready`, `groot_pipeline.sh` 는 자동. `HISTORY.md` §47). 3태스크 씬 수집은 `collect240.sh [N]`/`convert240.sh`
(환경변수 `RAW OUT REPO RADIUS SEED TAG` 로 재사용 — 기본값은 `240_v2` 재현).

## GR00T N1.7

| 단계 | 스크립트 |
|---|---|
| V1~V5 스모크 (빌드·VRAM·step/s·shm·상대액션 빌드) | `groot_smoke.sh [batch] [steps] [workers]` |
| 배치/워커 스윕 | `groot_sweep.sh` |
| 학습 (절대 액션, 10k, `OMP_NUM_THREADS=8`) | `groot_train_abs.sh` (`DS OUT REPO TAG` 덮어쓰기 가능; + `train_watchdog.sh <log> <steps>`) |
| 서비스 가능성 40 s 프로브 | `probe_serve.sh <ckpt>` |
| V7 지연 (서버와 같은 호출 경로) | `groot_latency.py` |
| V8 롤아웃 (3태스크 교대, 사전검사·청크검사·`judge_task.py`) | `groot_v8.sh [태스크당 N] [초] [ckpt]` → `groot_rollout.sh` (`RADIUS SEED` 는 수집과 같아야 함; `TAG`/`ROLL_TAG` 로그 접두) |
| **v3 전체** (30 ep/태스크 수집 → h264 변환 → 학습 → 롤아웃 A(0.06)·B(0.025)) | `groot_pipeline.sh [N]` — 6 h 게이트 내장 |
| 파라미터/전처리 확인 | `groot_mem.py` `groot_probe.py [dataset_root]` |

## 벤치 (§45)

`bench_decode.py`(격리 디코딩 — **1 스레드로 잴 것**), `bench_quality.py`(PSNR vs 원본 JPEG), `bench_data_s.py`
(`data_s` 분해: loader 대기 / to_float / 전처리기), `groot_threads_smoke.sh`(`THREADS="1 4 8"`), `groot_codec_smoke.sh`,
`convert_h264.sh`(코덱 비교 변환).
