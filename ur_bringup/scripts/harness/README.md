# harness — 수집 → 변환 → 학습 → 롤아웃 자동화 스크립트

`HISTORY.md` §28~§45 의 실측을 만든 스크립트들. 2026-09-13 까지 세션 임시 폴더에 있던 것을 repo 로 옮겼다
(§45.9). **모델·데이터 경로 코드가 아니라 실행 순서·사전검사·채점**이다 — 정책/데이터셋은 전부 upstream lerobot.

## 공통 규칙

- **경로**: 워크스페이스 루트는 스크립트 위치에서 유추(`src/ur_bringup/scripts/harness` → 4단계 위). 다른 곳에
  두었으면 `UR_WS=/path/to/ws`. 하네스가 부르는 `scripts/collect_il_episodes.sh` 도 같은 규칙(`ros2 run` 으로 불러도
  install 심볼릭링크/복사본에서 역산). ROS 는 `/opt/ros/jazzy`, Isaac 은 `/isaac-sim/python.sh` 고정.
- **ROS 도메인**: 하네스는 환경의 `ROS_DOMAIN_ID` 를 물려받고, 안 정하면 0 으로 돌며 `grasp_mode.sh` 가 경고한다.
  **Isaac 을 띄우기 전에 모든 터미널에서 `export ROS_DOMAIN_ID=42`** — 0 이 아니고 **같은 호스트의 컨테이너마다 다른 값**,
  모든 터미널에서 같은 값. 42 = 이 워크스페이스(호스트의 첫 컨테이너); 같은 호스트에 두 번째 컨테이너면 다른 값으로
  (도메인이 겹치면 다른 컨테이너의 `/robot_description` 을 받는다, `HISTORY.md` §47.4).
- **로그**: `outputs/harness_logs/`(소스 트리에 쓰지 않는다). `HARNESS_LOG=` 로 변경.
- **ML 환경**: GR00T·벤치 스크립트는 `src/setup/ml_env.sh` 를 스스로 source 한다(HF 캐시·오프라인·`OMP_NUM_THREADS=8`).
- **인터프리터**: `.sh` 는 `bash <파일>` 로(모두 실행권한 있음). `.py` 는 두 부류 —
  ROS python(`python3`): `obs_ready.py` `judge_rollout.py` `judge_task.py` `grasp_watch.py` /
  ML venv(`deps/.venv-ml/bin/python`): `bench_*.py` `groot_latency.py` `groot_mem.py` `groot_probe.py` `record_rollout.py`
  (`grasp_log_stats.py` 는 표준 라이브러리만 — 어느 python 이든).
- **종료**: `shutdown_rollout.sh` = Isaac · `ros2 launch|run ur_bringup` 부모와 그 노드 전부(ros2_control_node·spawner·
  robot_state_publisher·move_group·cuMotion/nvblox 컨테이너·nvblox_node·static TF·servo/joy·`teleop_joy.py`·`omy_to_ur16e.py`·
  `virtual_omy_leader.py`·`il_recorder.py`·`depth_info_restamp.py`·`pick_place_demo.py`·ur_robot_driver 보조 노드
  (`trajectory_until_node`·`dashboard_client`·`urscript_interface`·`controller_stopper_node`·`robot_state_helper`·tool 통신 socat)·RViz)
  + 정책 서버/클라이언트 + 우리 도메인의 ros2 daemon. 학습·변환(ML venv)과 하네스 루프는 건드리지 않는다.
  `shutdown_all.sh` = 그것 + 하네스/수집 루프 `.sh`(먼저) + ML venv 프로세스 전부(`lerobot-train`·`raw_to_lerobot.py`·워커).
  **판정이 진짜다**: 끝나고 남은 프로세스와 `ros2 node list`(`ROS_DOMAIN_ID`)를 다시 보고 마지막 줄 `CLEAN`(rc 0) 또는
  `LEFT: …`(rc 1). `--dry-run` 은 죽이지 않고 대상만 보여준다. TERM 후 `TERM_WAIT`(30 s) 기다렸다 KILL.
  공유 머신이라 광범위 `pkill` 은 없다 — **실행 파일 경로**로만 고른다(`$WS/install/**`, `src/ur_bringup/{scripts,isaac}/**`,
  `deps/.venv-ml` python; `/opt/ros/jazzy/lib/<pkg>/<exe>` 와 `ros2 launch ur_bringup` 은 **환경에 이 워크스페이스의
  install 이 source 된 경우만** — 다른 워크스페이스의 같은 바이너리는 "not ours" 로 보고만 한다). 명령줄에 패턴 문자열이
  들어 있을 뿐인 셸, 그리고 호출자(조상 프로세스)는 절대 죽이지 않는다(`README.md` §8).
- 기동 순서·컨트롤러 spawn·관측 검사·청크 검사 같은 **사전조건 검사가 하네스의 본체**다. 빠지면 "정책이 아무것도
  안 함" 으로 오진된다(`HISTORY.md` §44.2 결함 5개).

## 파지 모드 스위치 (2026-10-19, `HISTORY.md` §53.7)

수집·변환·학습·롤아웃 하네스는 전부 `grasp_mode.sh` 를 source 한다. **`GRASP_MODE=physical`(기본)** = Isaac
`--grasp-mode physical`, 경로 접미사 `_phys`(`il_raw_red_left_100_phys`·`act_red_left_100_phys`·GR00T `v4_phys` …),
시작 자세 `ready`. **`GRASP_MODE=attach`** = 용접 시대 그대로(접미사 없음, 시작 자세 `ready_v1`) — 아래 표의 기존 결과(9/10 등)는
전부 이 모드다(3task_v3 포함 — 셋 다 `ready_v1` 에서 시작, `reset_pose.py` 참조).
physical 첫 실행(2026-10-20): ACT 2카메라 수집 100/100 → 롤아웃 8/10(시드 131)·9/10(`SEED=171`, 영상) — `HISTORY.md` §54.5; 200k 이어 학습 19/20(§54.6).
**현재 최종(2026-10-22)**: 100 ep + 바깥 띠 90 ep = 190 ep 200k `outputs/act_red_left_190_phys/checkpoints/200000` — 기존 범위 19/20 · 넓은 범위(x ±100 mm) 20/20
(같은 배치에서 100 ep 200k 는 13/20), `HISTORY.md` §58.7. 명령은 `PIPELINE.md` §3-2.
`rollout_gui_then_n*.sh` 는 `SEED=` 로 롤아웃 시드를 바꿀 수 있다(기본 131 / 151). `rollout_gui_then_n.sh` 는 `RAND=`(Isaac 무작위 인자 추가, 넓은 범위 평가는
`RAND="--randomize-radius-x 0.10 --randomize-radius-y 0.06"`, §58.7)도 받는다.

**실행 예 (ACT 2카메라, physical, 무인 ≈ 3 h)** — 대화형 터미널에서 차례로 치면 된다. 단 **`bash -c '…'`·nohup 래퍼처럼
두 이름을 명령줄에 함께 담은 프로세스 안에서 돌리지 말 것**: 단계 1·2 가 `pgrep -f collect_100.sh` 로 수집 종료를 기다리는데
그 래퍼도 잡혀 영원히 안 끝난다(`HISTORY.md` §54.1). 무인 실행은 아래를 런처 **파일**에 적어 `bash 파일` 로:
```bash
export ROS_DOMAIN_ID=42 GRASP_MODE=physical      # 0 아닌 값, 모든 터미널 동일(위 'ROS 도메인')
nohup bash collect_100.sh 100 > ../../../../outputs/harness_logs/collect_100.log 2>&1 &
sleep 60; bash pipeline_100.sh > ../../../../outputs/harness_logs/pipeline_100_phys.log 2>&1
```

## ACT (카메라 2대 / 손목 1대)

| 단계 | 2대 (`red_left_100`, 9/10) | 손목 1대 (`wrist_only`, 8/9) |
|---|---|---|
| 수집 (Isaac + 상태머신, `grasp_watch.py` 감시) | `collect_100.sh` | `collect_wrist1.sh` |
| 변환 (`raw_to_lerobot.py`) | `convert_100.sh` | `convert_wrist1.sh` |
| 학습 (60k, 워커 4/2/0 사다리, shm 픽스) | `train_100.sh` | `train_wrist1.sh` |
| 롤아웃 (Isaac 재기동 + `bringup.sh` + `policy_inference.launch.py` + N회 채점) | `rollout_gui_then_n.sh [N] [초] [ckpt]` → `rollout_n.sh` | `rollout_gui_then_n_w1.sh [N] [초] [ckpt]` → `rollout_wrist1.sh` |
| 전체 | `pipeline_100.sh` | `pipeline_wrist1.sh` |

채점 `judge_rollout.py`(GT 로 물체–마커 거리 + 그리퍼 개방). **시작 자세 `START_POSE` 는 `GRASP_MODE` 를 따른다**
(`grasp_mode.sh`: physical → `ready`(실측 랑데부, `HISTORY.md` §49.8), attach → `ready_v1` = 2026-09-16 이전 데이터셋의 자세).
덮어쓰는 건 옛 데이터셋을 다른 모드로 돌릴 때뿐(`ready_v2` = 2026-09-16 URDF 가지, sim 스모크만); `groot_pipeline.sh` 는 자동.
`rollout_n.sh` 의 ckpt 기본값은 `rollout_gui_then_n.sh` 와 같은 `outputs/act_red_left_100${GRASP_SFX}/checkpoints/last/pretrained_model`
— 다른 PC 로 옮긴 체크포인트(예: `act_red_left_100_phys_long/checkpoints/200000/pretrained_model`)는 3번째 인자로 준다.
**다른 세트로 재사용(§58, 기본값 = 원본 100 ep 실행)**: `collect_100.sh` `OUT SEED TAG RAND` · `convert_100.sh` `RAW DS REPO TAG`(목적지는 `OUT` 이 아니라 `DS`) ·
`train_100.sh` `DS REPO CK STEPS SAVE_FREQ TAG`. 덮어쓰기를 주면서 `OUT`/`DS`/`CK` 를 빼면 원본을 지우지 않도록 rc 2, 삭제 대상은 절대경로 `…/outputs/{il_raw,lerobot_ds,act}_*` 만.
★ `train_100.sh` 는 시작 시 `rm -rf "$CK"` — 기존 체크포인트 이어 학습은 `lerobot-train --resume=true --config_path=<CK>/checkpoints/last/pretrained_model/train_config.json`
(`PIPELINE.md` §3-1). 실행 중인 하네스 `.sh` 는 고치지 말 것 — bash 가 바뀐 파일을 이어 읽는다(§58.2). `pipeline_100.sh` 는 이 변수들을 `env -u` 로 지우고 하위 스크립트를 불러 항상 원본 100 ep 를 돈다.
3태스크 씬 수집은 `collect240.sh [N]`/`convert240.sh`
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

## 물리 접촉 파지 (§53)

`grasp_physical.sh <tag> <N> [isaac 인자]` — Isaac(`--grasp-mode physical`, 카메라 없음, `--grasp-log`) + `bringup.sh` + 빨강 큐브
N 사이클 → `grasp_log_stats.py` 요약(사이클 결과 · 그리퍼 DOF 범위 · 가드/forced-open 횟수 · **위치 기준** 팔 속도 · 홀드별
미끄럼). `DEMO_PARAMS=$'grip_closed: 0.8\ngrasp_bias_y: 0.010'` 로 남용 시험, `--gripper-profile asset --gripper-guard off` 로 원래
게인 비교, `MODE=attach` 로 용접 경로 회귀. 로그 `outputs/harness_logs/gp_<tag>.*`.

## 벤치 (§45)

`bench_decode.py`(격리 디코딩 — **1 스레드로 잴 것**), `bench_quality.py`(PSNR vs 원본 JPEG), `bench_data_s.py`
(`data_s` 분해: loader 대기 / to_float / 전처리기), `groot_threads_smoke.sh`(`THREADS="1 4 8"`), `groot_codec_smoke.sh`,
`convert_h264.sh`(코덱 비교 변환).

## 디스플레이 없는 컨테이너 (2026-09-16, `HISTORY.md` §48)

`bringup.sh` 의 move_group 대기 문자열 `Ready to take commands for planning group` 은 **RViz 의 MoveGroupInterface** 가
찍는 것이라, X 소켓 없는 컨테이너(RViz 즉사)에서는 영원히 안 나와 `FAIL: move_group not ready` 가 됐다. 이제 move_group
자체의 `You can start planning now!` 도 허용한다 — 실제 준비 판정은 그 뒤의 `ros2 action list` 검사다.
하네스 전체는 Isaac `--headless` 로 돌므로 디스플레이는 롤아웃 GUI 확인(`rollout_gui_then_*`)에만 필요하다.
