# UR16e ACT 실습 매뉴얼

2026-10-27 기준 · 원본(Claude Docs): https://claude.ai/artifact/92EKUyUr83XZUTwm4J27HH

## 개요

이 매뉴얼은 최종 보고의 ACT 결과를 현재 코드로 직접 다시 만들어 보는 순서입니다. 시뮬레이터를 눈으로 보는 것부터 시작해 수집, 변환, 학습, 평가, 결과 확인까지 6개 실습으로 나눴습니다.

- **대상 구성**: UR16e + Robotiq 2F-85 + D405, Isaac Sim, 물리 접촉 파지(physical) 모드
- **과제**: 빨간 큐브를 집어 왼쪽 marker 위에 놓기
- **최종 보고의 결과물**
  - 데이터: 기본 범위 100 episode + 바깥 띠 90 episode = 190 episode
  - 체크포인트: `outputs/act_red_left_190_phys/checkpoints/200000`
  - 비교 기준(baseline): `outputs/act_red_left_100_phys_long/checkpoints/200000`

실습 순서는 다음과 같습니다. 각 실습은 앞 실습의 결과물을 씁니다. 다만 이미 만들어 둔 데이터셋과 체크포인트가 있으므로, 실습 5(평가)부터 바로 해 볼 수도 있습니다.

1. **실습 1** — Isaac Sim 과 제어 스택을 띄우고 scripted pick&place 를 눈으로 보기
2. **실습 2** — scripted expert 로 데모(episode) 수집
3. **실습 3** — raw 데모를 LeRobot 데이터셋으로 변환
4. **실습 4** — ACT 학습, 이어 학습
5. **실습 5** — rollout 평가와 영상 녹화
6. **실습 6** — 결과 읽기(판정 로그, steps.csv, GIF)

모든 명령은 컨테이너 안, 워크스페이스 루트(`/isaac-sim/volume/ur_ws`)를 기준으로 적었습니다. 각 명령의 근거가 되는 스크립트 위치도 함께 적었습니다.

## 준비

새 터미널을 열 때마다 아래 4줄을 먼저 실행합니다. 하네스 스크립트는 ROS 를 스스로 source 하지만, `ros2 launch` 나 `reset_pose.py` 를 직접 실행할 때는 이 설정이 필요합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /isaac-sim/volume/ur_ws/install/setup.bash
export ROS_DOMAIN_ID=42
cd /isaac-sim/volume/ur_ws
```

### 꼭 지킬 규칙

- **`ROS_DOMAIN_ID=42`**: 설정하지 않으면 모든 하네스가 도메인 0 을 씁니다. 도메인 0 에는 같은 호스트의 다른 컨테이너 노드가 있어 엉뚱한 `/robot_description` 을 받고 죽습니다. Isaac, 제어 스택, 종료 스크립트 모두 같은 값이어야 합니다.
- **무거운 작업은 한 번에 하나**: 호스트 CPU 를 다른 컨테이너와 나눠 씁니다. 수집, 변환, 학습, rollout 을 동시에 돌리지 않습니다.
- **긴 작업은 런처 파일로**: 명령을 `.sh` 파일에 적고 `nohup bash 파일.sh > 로그 2>&1 &` 로 실행합니다. 명령 문자열을 직접 넘기면 `pgrep -f collect_100.sh` 가 래퍼 셸까지 잡아 종료 대기가 끝나지 않습니다.
- **실행 중인 `.sh` 는 고치지 않기**: bash 는 실행 중에도 파일을 이어 읽습니다. 실제로 바깥 띠 수집 때 90/90 성공 뒤 `FAIL: isaac never published /clock` 가 찍힌 적이 있습니다.
- **정리는 정리 스크립트로**: `pkill python3` 같은 넓은 종료는 다른 워크스페이스도 죽입니다. "정리·종료" 섹션의 스크립트를 씁니다.
- **`train_100.sh` 는 기존 체크포인트 폴더에 다시 쓰지 않기**: 시작할 때 출력 폴더를 `rm -rf` 합니다. 이어 학습은 실습 4 의 `--resume` 방법을 씁니다.

### 환경 점검

```bash
bash src/setup/check_env.sh            # 읽기 전용 점검
bash src/setup/check_env.sh --verbose  # apt 버전 차이 목록까지
```

- 마지막 줄이 `ok   environment OK` 이면 통과입니다. `FAIL environment has REQUIRED failures` 이면 위쪽의 `FAIL` 줄을 봅니다.
- `warn` 줄(apt 버전 차이, GR00T 미설치 등)은 실패가 아닙니다.
- `--snapshot` 은 `src/setup/apt-versions.lock` 을 다시 쓰는 모드라 실습에서는 쓰지 않습니다.

### 파지 모드

- 모든 하네스가 `scripts/harness/grasp_mode.sh` 를 읽습니다. 기본값은 `GRASP_MODE=physical` 입니다.
- physical 이면 경로에 `_phys` 가 붙고, Isaac 에 `--grasp-mode physical` 이 가고, 시작 자세는 `ready` 입니다.
- `GRASP_MODE=attach` 는 용접 시대 데이터·체크포인트용입니다. 이 실습에서는 쓰지 않습니다.

### ML 명령

- ML 명령은 venv 를 source 하지 않고 `deps/.venv-ml/bin/python` 이나 `deps/.venv-ml/bin/lerobot-train` 을 직접 실행합니다.
- ACT 하네스(`convert_100.sh`, `train_100.sh`)는 필요한 환경변수를 스스로 넣습니다. 직접 `lerobot-train` 을 칠 때는 `source src/setup/ml_env.sh` 를 먼저 해 두면 안전합니다(오프라인 HF 캐시, `OMP_NUM_THREADS=8`).

### 디렉터리 지도 (`outputs/`)

| 경로 | 크기 | 내용 |
| --- | --- | --- |
| `il_raw_red_left_100_phys/` | 1.4 GB | 기본 범위 raw 데모 100개 (수집 seed 91) |
| `il_raw_red_left_band_phys/` | 1.3 GB | 바깥 띠 raw 데모 90개 (수집 seed 301) |
| `il_raw_red_left_190_phys/` | 16 KB | 위 두 폴더를 가리키는 symlink 190개 |
| `lerobot_ds_red_left_190_phys/` | 382 MB | 최종 학습 데이터셋 |
| `act_red_left_190_phys/` | 5.8 GB | 최종 모델, checkpoints 020000\~200000 |
| `act_red_left_100_phys_long/` | 4.7 GB | baseline 모델, checkpoints 060000\~200000 |
| `harness_logs/` | 580 MB | 모든 하네스 로그, rollout 결과 |
| `rollout_videos/` | 239 MB | rollout trial 별 MP4 |
| `report_media/` | 72 MB | 보고서 GIF 와 정지 이미지 |

실습 중 새로 만드는 결과물은 위 폴더와 겹치지 않는 이름(예: `_practice` 접미사)을 쓰면 보고서 결과물을 보존할 수 있습니다.

## 실습 1 — 시뮬레이터 띄우고 pick&place 보기

터미널 3개로 Isaac(GUI) → 제어 스택과 MoveIt → scripted pick&place 한 번 순서로 실행합니다. 이것이 데모 수집 때 로봇이 하는 동작과 같습니다.

### 터미널 1 — Isaac Sim (GUI)

수집·rollout 하네스가 쓰는 장면과 같은 인자입니다. `--headless` 를 빼면 창이 뜨고, 넣으면 화면 없이 돕니다.

```bash
/isaac-sim/python.sh /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
  --asset-path /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd \
  --scene pick_place --table --table-height 0.20 --table-pose 0.72,0.0 --table-size 0.70,0.90 \
  --object-pose 0.60,0.0,0.225 --object-size 0.035,0.035,0.035 \
  --place-pose 0.60,0.315,0.0 --object-names red,blue --place-names left,right \
  --object-spacing 0.20 --randomize-object --randomize-radius 0.06 --seed 131 \
  --camera-res 320x240 --grasp-mode physical --with-camera --with-static-cam
```

- 로그에 `scene topics` 줄이 찍힌 뒤, 다른 터미널에서 `ros2 topic echo --once /clock` 이 값을 돌려주면 준비된 것입니다. `scene topics` 만으로는 `/clock` 이 도는지 알 수 없습니다.
- 셰이더 캐시가 있으면 약 20초, 처음(콜드)이면 `/clock` 까지 약 3.5분 걸립니다.
- 시작 배너의 `grasp mode` 줄이 `physical  (gripper profile soft, physics 120 Hz, guard ON)` 인지 확인합니다. 배너는 공백으로 줄을 맞추므로 `grep 'grasp mode'` 처럼 단어로 찾습니다. Isaac 은 모르는 인자를 조용히 무시하므로(`--grasp_mode` 같은 오타), 배너로 확인하는 것이 유일한 방법입니다.
- Isaac 은 시작할 때 NVIDIA 에셋 서버(네트워크)가 필요합니다. 못 찾으면 `Could not find Isaac Sim assets folder` 를 찍고 종료합니다.
- 처음 화면의 팔은 위로 선 home 자세이고, 큐브는 기본 배치입니다. 무작위 배치는 첫 episode 리셋 때 적용됩니다.
- `--seed 131` 은 rollout 평가 배치입니다. 수집 때와 같은 배치를 보려면 `--seed 91` 을 씁니다.

### 터미널 2 — 제어 스택 + MoveIt(cuMotion)

```bash
bash src/ur_bringup/scripts/harness/bringup.sh
```

- 마지막 줄이 `== ready` 면 성공입니다. Isaac 이 준비된 뒤 약 10\~20초 걸립니다. RViz 도 함께 뜹니다.
- 하는 일: 기존 제어 스택 종료 → `ur16e_2f85_d405.launch.py use_sim:=true` → `ur16e_2f85_d405_cumotion_moveit.launch.py use_sim:=true ur_only:=false` → `/move_action` 과 `/gripper_controller/gripper_cmd` 확인.
- 로그: `outputs/harness_logs/control.log`, `moveit.log`. 실패 문구는 `FAIL: controllers never activated`, `FAIL: move_group not ready`, `FAIL: missing action ...` 입니다.
- 주의: 시작할 때 컨테이너 안의 `ros2_control_node`, `move_group`, `rviz2` 등을 이름 패턴으로 모두 종료합니다. 다른 작업의 스택이 떠 있으면 같이 죽습니다.
- 수동으로 띄울 때는 위 두 launch 를 터미널 2개에서 실행합니다. `ur_only:=false` 를 빠뜨리면 그리퍼 없는 모델로 계획해서, 목표가 SUCCESS 로 보고되는데 팔은 움직이지 않습니다.

### 터미널 3 — scripted pick&place 1회

```bash
ros2 launch ur_bringup pick_place_demo.launch.py use_sim:=true cycles:=1 \
  object_topic:=/scene/objects/red/pose place_topic:=/scene/places/left/pose \
  extra_params:=$(ros2 pkg prefix ur_bringup)/share/ur_bringup/config/common/pick_place_physical.yaml
```

- `extra_params` 는 빼면 안 됩니다. 빼면 노드 기본값인 용접(attach) 모드, 그리퍼 0.52 로 동작해 물리 파지 장면에서 제대로 못 쥐는 것으로 보입니다(코드상 추론, 실행으로 확인하지는 않음).
- 보이는 순서
  1. `READY(start): ok` — 팔이 MoveIt 으로 시작 자세 `ready` 로 갑니다. 베이스가 약 180° 돌아 상완이 뒤쪽 수평이 됩니다.
  2. `reset_episode: red@[...] yaw ...` — 큐브가 무작위 위치로 순간이동합니다.
  3. `DETECT` → `PRE_GRASP`(큐브 15 cm 위) → `GRASP(linear)` 하강 → `CLOSE: commanded 0.80` → `LIFT`(+20 cm) → `TRANSFER` → `PLACE(linear)` → `OPEN` → `RETRACT`.
  4. `[cycle 1] SUCCESS (part 0.001 m from marker)` 후 `=== 1/1 cycles succeeded ===`.
- `table_sink < 0: planning WITHOUT the work surface` 경고는 정상입니다.
- 성공 기준은 큐브가 marker 에서 3.5 cm 이내입니다. headless 기준 한 사이클이 벽시계로 약 18초 걸렸습니다. GUI 속도는 재지 않았습니다.
- `cycles:=3` 처럼 늘리면 매 사이클마다 새 배치로 반복합니다.

### 손으로 해 볼 수 있는 것

```bash
python3 src/ur_bringup/isaac/common/reset_pose.py ready 6          # 시작 자세로 (초는 정수)
python3 src/ur_bringup/isaac/common/switch_control_mode.py --status  # trajectory / streaming 확인
ros2 service call /scene/reset_episode std_srvs/srv/Trigger       # 큐브 다시 배치
```

- `reset_pose.py` 는 기본 자세가 `home`(특이점)이라 항상 `ready` 를 씁니다. 충돌 검사 없이 컨트롤러로 바로 보내고, trajectory 모드에서만 동작합니다.
- 실패해도 종료 코드가 0 이라, `done, error_code: 0` 문구로 성공을 판단합니다.
- rollout 이후에는 streaming 모드로 남아 있으므로, pick&place 를 다시 하려면 `switch_control_mode.py trajectory` 를 먼저 실행합니다.

주의: 수집(`collect_100.sh`)과 평가(`rollout_gui_then_n.sh`) 하네스는 실행 중인 Isaac 을 모두 종료하고 headless 로 다시 띄웁니다. 실습 1 의 GUI 는 실습 2 로 넘어가면 사라집니다.

## 실습 2 — 데모 수집

`collect_100.sh` 하나가 Isaac(headless) → 제어 스택 → 녹화기 → scripted pick&place 를 N 번 돌립니다. 성공한 사이클만 episode 로 저장되고, 한 사이클에 약 23초가 걸립니다.

### 먼저 알아야 할 것

- **출력 폴더는 시작할 때 지워집니다.** 기본 `OUT` 이 보고서 원본 `outputs/il_raw_red_left_100_phys` 입니다. 실습에서는 항상 새 `OUT` 을 **절대경로**로 줍니다.
- **`TAG` 도 새로 줍니다.** 기본 `c100` 이면 원본 수집 로그(`c100_*`)와 파지 사진 폴더를 덮어씁니다.
- **개수 인자를 꼭 줍니다.** 없으면 50 개만 모읍니다.
- `SEED`/`RAND`/`TAG` 를 주면서 `OUT` 을 빼면 `FAIL: SEED/RAND/TAG given but OUT unset` 로 멈춥니다. 이전 작업에서 `SEED` 가 export 된 채 남아 있어도 걸립니다.
- 지울 수 있는 경로는 `*/outputs/il_raw_*`, `*/outputs/lerobot_ds_*`, `*/outputs/act_*` 로 제한되지만, 오타로 체크포인트 폴더를 가리키면 그것도 지워집니다. `OUT` 은 항상 새 `il_raw_` 이름으로 줍니다.
- 실행 중인 Isaac(실습 1 의 GUI 포함)을 모두 종료하고 시작합니다. 끝난 뒤에는 Isaac 과 스택이 떠 있으므로 직접 정리합니다.

### 1단계 — 스모크 2개 (약 2분)

```bash
cd /isaac-sim/volume/ur_ws
export ROS_DOMAIN_ID=42 GRASP_MODE=physical
OUT=$PWD/outputs/il_raw_practice_smoke TAG=psmoke \
  bash src/ur_bringup/scripts/harness/collect_100.sh 2 2>&1 | tee outputs/harness_logs/psmoke.out
bash src/ur_bringup/scripts/harness/shutdown_rollout.sh   # 마지막 줄 CLEAN
```

### 2단계 — 본 수집 (런처 파일로)

기본 범위 100개는 약 40분, 바깥 띠 90개는 약 36분 걸렸습니다. 아래를 파일(예: `outputs/harness_logs/run_practice_collect.sh`)로 저장하고 실행합니다.

```bash
# run_practice_collect.sh — 기본 범위 (보고서와 같은 seed 91 배치)
export ROS_DOMAIN_ID=42 GRASP_MODE=physical
export OUT=/isaac-sim/volume/ur_ws/outputs/il_raw_practice_base SEED=91 TAG=pbase RAND=
bash /isaac-sim/volume/ur_ws/src/ur_bringup/scripts/harness/collect_100.sh 100
echo "== collect rc=$?"
```

```bash
# run_practice_band.sh — 바깥 띠(x 방향 60~100 mm). 별도 런처로 저장 (터미널에 직접 export 하지 않음)
export ROS_DOMAIN_ID=42 GRASP_MODE=physical
export OUT=/isaac-sim/volume/ur_ws/outputs/il_raw_practice_band SEED=301 TAG=pband
export RAND="--randomize-radius-x 0.10 --randomize-radius-y 0.06 --randomize-inner-x 0.06 --randomize-yaw-edge-frac 0.3 --randomize-yaw-edge-deg 10"
bash /isaac-sim/volume/ur_ws/src/ur_bringup/scripts/harness/collect_100.sh 90
```

```bash
nohup bash outputs/harness_logs/run_practice_collect.sh > outputs/harness_logs/practice_collect.out 2>&1 &
tail -F outputs/harness_logs/pbase_logs/collect_red_left_1.log | grep -E "SUCCESS|FAILED|il/stop"
```

- 런처 파일마다 `export ROS_DOMAIN_ID=42 ...` 부터 다시 씁니다. 이 변수들을 터미널에 직접 export 하면 다음 실행(다른 수집, rollout)에 그대로 넘어갑니다. 예를 들어 남아 있는 `RAND` 는 기본 범위 수집을 바깥 띠 설정으로 바꿉니다.
- 두 수집을 한 런처에 이어 넣을 때는 실행 전에 완성해 둡니다. 도는 중에 런처에 덧붙이면 "실행 중인 `.sh` 는 고치지 않기" 규칙을 어깁니다.
- `RAND` 의 의미: 첫 물체(빨강)는 x 를 ±(60\~100 mm) 에서, y 는 ±60 mm 에서 뽑습니다. yaw 는 30% 확률로 45° 접기 경계 ±10° 안에서 뽑습니다.
- 시드가 같고 `RAND` 가 없으면 난수 사용 순서가 예전과 같아 seed 91 의 배치가 그대로 재현됩니다.
- 중간에 멈추려면 먼저 `pgrep -af run_practice` 로 런처 PID 를 찾아 `kill` 하고, 그다음 `shutdown_rollout.sh --all` 을 씁니다. `--all` 은 `src/` 아래 스크립트만 종료해서 내 런처는 남기고, 학습 중인 `lerobot-train` 은 종료합니다.

### 성공 확인

종료 코드만으로는 판단하지 않습니다. Isaac·bringup 단계 실패는 1 로 끝나지만, 수집 자체가 실패해도 마지막 `echo` 때문에 0 으로 끝납니다. 대신 아래를 봅니다.

```bash
grep -a -E "FAIL|cycles succeeded|episodes on disk|ALL TASKS DONE" outputs/harness_logs/practice_collect.out
ls -d outputs/il_raw_practice_base/episode_* | wc -l     # 100 이어야 함
```

- 정상 출력 꼬리: `=== 100/100 cycles succeeded ===` → `episodes on disk: 100` → `== ALL TASKS DONE: 100 episodes, 1.4G` → `== grasp frames: 300`.
- 모자라면 수집기가 태스크당 최대 4회까지 pick&place 를 다시 띄워 채웁니다(한 번 돌았는데 늘지 않으면 중단). 그래도 모자라면 `SHORT: k episode(s) missing` 가 찍힙니다.
- 파지 순간 사진: `outputs/harness_logs/grasp_pbase/{closed,lift,open}_<n>.jpg` (사이클마다 3장).
- 보고서 수집 결과: 기본 범위 100/100, 바깥 띠 90/90, 실패로 버린 사이클 0.

### episode 하나 열어 보기

```bash
E=outputs/il_raw_red_left_band_phys/episode_000000
cat $E/meta.json                                  # num_frames, rate_hz 30, grasp_mode physical ...
ls $E/frames/video.exterior | wc -l               # = num_frames (외부 카메라 JPEG)
ls $E/frames/video.wrist | wc -l                  # = num_frames (손목 D405 JPEG)
python3 -c "import json;f=json.load(open('$E/data.json'))['frames'];print(len(f),'frames',round(f[-1]['t']-f[0]['t'],2),'s');print(f[0])"
```

- 프레임 하나는 `t`(sim 초), `state.single_arm`(관절 6개), `state.gripper`(0\~1), `action.single_arm`(다음 프레임의 state), `action.gripper`(그리퍼 명령) 입니다.
- 30 Hz 타이머가 각 스트림의 최신값을 집는 방식(zero-order hold)입니다.
- 세트 전체 요약: `deps/.venv-ml/bin/python src/ur_bringup/scripts/raw_to_lerobot.py --raw <OUT> --dry-run` (torch 를 불러오지 않아 가벼움).

## 실습 3 — LeRobot 데이터셋으로 변환

raw episode(JPEG + JSON)를 LeRobot 데이터셋(parquet + h264 영상)으로 바꿉니다. ROS 는 필요 없고 ML venv 에서 도는데, 190 episode 에 약 16\~17분, 100 episode 에 약 7분 걸렸습니다.

### 두 세트 합치기 (symlink)

보고서의 190 세트는 복사본이 아니라, 두 raw 폴더를 가리키는 상대 symlink 190개입니다. 변환기는 폴더 이름 순서로 episode 번호를 다시 매깁니다.

```bash
cd /isaac-sim/volume/ur_ws
D=outputs/il_raw_practice_190
mkdir "$D" && ( cd "$D" &&
  for i in $(seq 0 99); do e=$(printf episode_%06d $i); ln -s ../il_raw_practice_base/$e $e; done &&
  for i in $(seq 0 89); do ln -s ../il_raw_practice_band/$(printf episode_%06d $i) $(printf episode_%06d $((100+i))); done )
ls "$D" | wc -l      # 190
```

- 직접 수집하지 않고 보고서 원본으로 해 보려면 링크 대상을 `../il_raw_red_left_100_phys`, `../il_raw_red_left_band_phys` 로 바꿉니다. 이미 있는 `outputs/il_raw_red_left_190_phys` 를 그대로 써도 됩니다.
- 링크만 담긴 폴더라서 원본 두 폴더를 지우면 안 됩니다.

### 변환 명령

```bash
# 1) 내용 확인 (가벼움: episode 수, frame 수, task 문자열)
deps/.venv-ml/bin/python src/ur_bringup/scripts/raw_to_lerobot.py --raw outputs/il_raw_practice_190 --dry-run

# 2) 변환 (하네스, 경로는 절대경로)
export GRASP_MODE=physical ROS_DOMAIN_ID=42
RAW=$PWD/outputs/il_raw_practice_190 DS=$PWD/outputs/lerobot_ds_practice_190 \
REPO=me/ur16e_practice_190 TAG=convert_practice \
  bash src/ur_bringup/scripts/harness/convert_100.sh
```

- 성공하면 `== CONVERSION DONE` 과 `episodes 190  frames 94574  fps 30`(보고서 세트 기준), 카메라 2대 `video (240, 320, 3)` 이 찍힙니다. 자세한 로그는 `outputs/harness_logs/convert_practice_run.log` 입니다.
- 실패하면 `== CONVERSION FAILED` 와 로그 끝 5줄이 찍힙니다. 이때도 종료 코드는 0 이므로 문구로 판단합니다.
- `DS` 폴더는 변환 전에 지워집니다. 덮어쓰기 인자를 하나도 안 주면 기본값 `outputs/lerobot_ds_red_left_100_phys` 를 지우고 다시 만듭니다.
- 수집(`collect_100.sh`)이 도는 동안에는 끝날 때까지 기다렸다가 시작합니다.
- `Could not load libtorchcodec ... Falling back to 'pyav'` 경고는 무해합니다. task 가 하나뿐이라 나오는 `! Fewer than 2 distinct task strings` 도 ACT 에서는 정상입니다.
- 하네스의 dry-run 요약은 들여쓴 줄을 걸러서 task 목록이 안 보입니다. task 를 확인하려면 위 1) 을 직접 실행합니다.
- 하네스 없이 `raw_to_lerobot.py --raw ... --repo-id ... --root <새 폴더>` 로 직접 변환할 수도 있습니다. `--root` 가 이미 있으면 `FileExistsError` 로 멈춥니다.
- 변환기는 파지 모드가 섞인 세트를 `refusing: episodes differ in (grasp_mode, gripper_action_topic)` 로 거부합니다.

### 데이터셋 확인

```bash
deps/.venv-ml/bin/python -I -c "import json;i=json.load(open('outputs/lerobot_ds_practice_190/meta/info.json'));print(i['total_episodes'],i['total_frames'],i['fps'],i['total_tasks'])"
# 보고서 세트와 같다면: 190 94574 30 1
```

- 구성: `data/chunk-000/*.parquet`(state, action), `videos/observation.images.{exterior,wrist}/...mp4`, `meta/{info.json,stats.json,tasks.parquet,episodes/}`.
- 특징: `observation.state` 7개(팔 6 + 그리퍼), `action` 7개, 외부·손목 카메라 320×240.
- 보고서 세트 `outputs/lerobot_ds_red_left_190_phys` 는 382 MB 입니다.

## 실습 4 — ACT 학습과 이어 학습

`train_100.sh` 로 처음부터 학습하고, 더 돌리거나 중단된 학습을 잇을 때는 `lerobot-train --resume` 을 씁니다. 190 episode 데이터로 200k step 에 약 4시간 23분(약 12.7 step/s) 걸렸습니다.

### 처음부터 학습

```bash
cd /isaac-sim/volume/ur_ws
export ROS_DOMAIN_ID=42 GRASP_MODE=physical
H=src/ur_bringup/scripts/harness
bash $H/shutdown_rollout.sh            # Isaac 등을 먼저 내림. 마지막 줄 CLEAN
```

아래를 런처 파일(예: `outputs/harness_logs/run_practice_train.sh`)로 저장하고 `nohup bash ... &` 로 실행합니다.

```bash
# run_practice_train.sh
export ROS_DOMAIN_ID=42 GRASP_MODE=physical
W=/isaac-sim/volume/ur_ws
DS=$W/outputs/lerobot_ds_practice_190 REPO=me/ur16e_practice_190 \
CK=$W/outputs/act_practice_190 STEPS=200000 SAVE_FREQ=20000 TAG=train_practice \
  bash $W/src/ur_bringup/scripts/harness/train_100.sh
echo "== train rc=$?"
```

- **`CK` 는 반드시 새 이름.** 시작할 때 `rm -rf "$CK"` 를 합니다. 보고서에서 실제로 쓴 명령은 `CK=$PWD/outputs/act_red_left_190_phys` 였으므로, 그대로 치면 최종 모델(5.8 GB)이 지워집니다.
- 덮어쓰기 인자 없이 실행해도 기본값 `outputs/act_red_left_100_phys` 를 지웁니다. `DS`/`STEPS` 등만 주고 `CK` 를 빠뜨리면 `FAIL: overrides given but CK unset` 로 멈춥니다.
- 경로는 절대경로여야 합니다. 상대경로는 지울 수 있는 경로 검사에서 거부됩니다.
- 기본 `STEPS` 는 60000 입니다. 짧게 해 보려면 `STEPS=20000`(약 26분)로 시작하고 나중에 이어 학습으로 늘릴 수 있습니다.
- 고정 설정: `--policy.type=act`, `--batch_size=32`, lr 1e-5, chunk\_size 100. `num_workers` 는 4 → 2 → 0 순서로 시도하고, 보고서 학습은 4 로 돌았습니다.
- 변환이 도는 중이면 최대 40분 기다렸다가 시작합니다.

### 진행 보기

로그는 `outputs/harness_logs/<TAG>_w<num_workers>.log` 입니다(예: `train_practice_w4.log`). 진행 막대가 `\r` 로 찍히므로 `tr` 로 풀어 봅니다.

```bash
L=outputs/harness_logs/train_practice_w4.log
tr '\r' '\n' < $L | grep -a ' step:.*epch:' | tail -1     # step:100K ... epch:33.90 loss:0.019 ...
tr '\r' '\n' < $L | grep -a '^Training' | tail -1        # 진행률, 남은 시간, step/s
grep -a "Checkpoint policy after step" $L               # 20k 마다 한 줄
ls outputs/act_practice_190/checkpoints; readlink outputs/act_practice_190/checkpoints/last
nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader
```

- loss 는 `epch:` 바로 뒤의 `loss:` 를 읽습니다. 200 step(약 16초)마다 한 줄이고, 보고서 학습은 100k 에서 0.019, 200k 에서 0.013 이었습니다.
- `train_100.sh` 가 끝에 찍는 요약(`loss:0.000`, `1.04s/step`)은 틀린 값입니다. `kld_loss` 와 초반 속도를 잘못 집습니다. `train_watchdog.sh` 도 같은 문제가 있고 ACT 로그에는 맞지 않습니다.
- 성공 판단: 로그의 `End of training` 과 `checkpoints/<STEPS>/` 폴더. 종료 코드는 실패해도 0 이고, `DONE (workers=4)` 는 첫 체크포인트(20k)만 생겨도 찍힙니다.
- loss 는 정책 품질 지표가 아닙니다. 판단은 실습 5 의 rollout 으로 합니다.

### 체크포인트 구조

```
outputs/act_practice_190/checkpoints/
  020000/ 040000/ ... 200000/   last -> 200000
  <step>/pretrained_model/   config.json, model.safetensors(약 197 MiB), train_config.json, 정규화 파일
  <step>/training_state/     optimizer, rng, training_step.json  (이어 학습에만 필요)
```

- 추론(rollout)에는 `pretrained_model/` 만 있으면 됩니다. 체크포인트 하나가 약 620 MB, 10개면 약 6 GB 입니다.

### 이어 학습 (`--resume`)

**(a) 중단된 학습을 원래 STEPS 까지** — 보고서에서 100k → 200k 를 이렇게 이었습니다.

```bash
UR_WS_TORCH_SHM_FIX=1 deps/.venv-ml/bin/lerobot-train --resume=true \
  --config_path=$PWD/outputs/act_practice_190/checkpoints/last/pretrained_model/train_config.json \
  > outputs/harness_logs/train_practice_resume.log 2>&1
```

**(b) 원래보다 더 많이** — 복사본에서 이어 학습합니다. baseline 100-episode 모델이 60k → 200k 를 이렇게 만들었습니다.

```bash
cp -a outputs/act_practice_190 outputs/act_practice_190_300k     # 디스크 여유 먼저 확인(df -h)
UR_WS_TORCH_SHM_FIX=1 deps/.venv-ml/bin/lerobot-train --resume=true \
  --config_path=$PWD/outputs/act_practice_190_300k/checkpoints/last/pretrained_model/train_config.json \
  --output_dir=$PWD/outputs/act_practice_190_300k --steps=300000 --save_freq=20000 \
  > outputs/harness_logs/train_practice_300k.log 2>&1
```

- (b) 에서 `--output_dir` 를 빼면 새 체크포인트가 복사본이 아니라 **원본 폴더**에 쓰입니다. `train_config.json` 에 원본의 절대경로가 들어 있기 때문입니다.
- `--config_path` 가 로컬에 없으면 lerobot 이 Hub repo id 로 보고 내려받으려 합니다. 경로를 먼저 `ls` 로 확인합니다.
- 이어 학습 로그 시작에 `Resuming data order at epoch 33, sample 78464` 같은 줄이 나오고, 진행 막대는 **남은** step 수(예: `100000/100000`)로 표시됩니다.
- `last` 가 없으면 `checkpoints/200000/pretrained_model/train_config.json` 처럼 숫자 폴더를 직접 가리킵니다.
- 두어 시간 넘게 걸리므로 (a)·(b) 모두 런처 파일에 넣어 `nohup` 으로 실행합니다. 예시는 `outputs/harness_logs/resume_train_190.sh` 입니다. 설정 파일과 `training_state` 가 있는지, 다른 학습이 도는지 먼저 확인합니다.
- ML 환경변수를 맞추려면 앞에 `source src/setup/ml_env.sh` 를 넣습니다. 보고서 학습은 넣지 않고 돌렸습니다.

### 학습 멈추기

1. `checkpoints/<N>/training_state/training_step.json` 이 생기고 `readlink checkpoints/last` 가 N 이 될 때까지 기다립니다.
2. `pgrep -af 'run_practice_train|train_100.sh|lerobot-train'` 으로 PID 를 찾아 런처 → `train_100.sh` → `lerobot-train` 순서로 그 PID 만 `kill` 합니다. `lerobot-train` 만 먼저 죽이면, 첫 체크포인트 전일 때 `train_100.sh` 가 `CK` 를 지우고 다른 `num_workers` 로 다시 시작합니다.
3. `shutdown_all.sh` 는 모든 ML 프로세스를 죽이므로, 다른 학습이 도는 중에는 쓰지 않습니다.

## 실습 5 — rollout 평가와 영상 녹화

`rollout_gui_then_n.sh` 가 Isaac(headless) → 제어 스택 → 스트리밍 컨트롤러 → trial 10번을 돌립니다. 10 trial 에 벽시계로 약 15분 걸렸습니다. 이름에 GUI 가 들어 있지만 Isaac 은 항상 headless 로 뜹니다.

### 최종 모델 평가 (보고서와 같은 조건)

```bash
cd /isaac-sim/volume/ur_ws
export ROS_DOMAIN_ID=42 GRASP_MODE=physical
CK=$PWD/outputs/act_red_left_190_phys/checkpoints/200000/pretrained_model
H=$PWD/src/ur_bringup/scripts/harness; L=$PWD/outputs/harness_logs
ls $CK/model.safetensors                     # 체크포인트 확인

# (a) 기본 범위 ±60 mm, seed 131
SEED=131 RAND= GRASP_LOG=$L/my_s131.steps.csv RECORD_DIR=$PWD/outputs/rollout_videos/my_s131 \
  bash $H/rollout_gui_then_n.sh 10 70 "$CK" > $L/my_s131.run.log 2>&1

# (b) 넓은 범위 x ±100 mm / y ±60 mm, seed 211
SEED=211 RAND="--randomize-radius-x 0.10 --randomize-radius-y 0.06" \
GRASP_LOG=$L/my_w211.steps.csv RECORD_DIR=$PWD/outputs/rollout_videos/my_w211 \
  bash $H/rollout_gui_then_n.sh 10 70 "$CK" > $L/my_w211.run.log 2>&1

grep -E '^== [0-9]+/[0-9]+ rollouts' $L/my_s131.run.log $L/my_w211.run.log
bash $H/shutdown_rollout.sh                  # 끝난 뒤 Isaac · 스택이 남아 있음. 마지막 줄 CLEAN
```

- 인자 순서: `trial 수`, `trial 당 초`, `체크포인트`. **체크포인트를 빼면** 기본값인 60k 100-episode 모델(`act_red_left_100_phys/checkpoints/last`)을 조용히 평가합니다.
- (a) 의 `RAND=` 는 일부러 비운 것입니다. 이전에 export 된 `RAND` 가 남아 있으면 기본 범위 평가가 조용히 넓은 범위로 바뀝니다.
- 한 조건에 약 15분이라, 터미널을 닫아도 계속 돌게 하려면 명령을 런처 파일에 넣어 `nohup` 으로 실행합니다.
- baseline 비교: `CK=$PWD/outputs/act_red_left_100_phys_long/checkpoints/200000/pretrained_model`.
- 실습 4 에서 만든 모델: `CK=$PWD/outputs/act_practice_190/checkpoints/200000/pretrained_model`.
- `my_` 같은 새 태그를 씁니다. 보고서의 `e190_*` 로그와 영상을 덮어쓰지 않기 위해서입니다. 같은 `RECORD_DIR` 를 다시 쓰면 이전 영상과 섞입니다.
- 여러 조건을 연달아 돌릴 때는 보고서에서 쓴 런처 `outputs/harness_logs/eval_190.sh` 를 복사해 씁니다(12 조건에 약 3시간). 복사본에서 `SUM=` 줄(요약 파일은 시작할 때 비워짐)과 `run()` 안의 `e190_` 접두사 3곳을 새 이름으로 바꾸고, 학습 대기 줄(`resume_train_190.out` 을 기다리는 `until grep`)은 지웁니다.
- 같은 seed 는 매번 같은 10개 배치를 줍니다. 하네스가 실행마다 Isaac 을 새로 띄우기 때문입니다.

### trial 한 번에 일어나는 일

1. trajectory 모드로 바꾸고 `/scene/reset_episode` 로 큐브를 새 위치에 놓습니다.
2. `reset_pose.py ready` 로 시작 자세에 갑니다. 3번 실패하면 `SKIP: could not reach READY` 로 실패 처리됩니다.
3. streaming 모드(`forward_position_controller`)로 바꿉니다.
4. `RECORD_DIR` 가 있으면 녹화기(`record_rollout.py`)를 켭니다.
5. LeRobot robot client 가 70초 동안 돕니다. action 은 30 Hz 로 보내고, policy 추론은 action 50개 단위(약 1.7초마다)로 받습니다. 성공해도 70초를 다 채웁니다.
6. `judge_rollout.py` 가 판정하고 영상 파일 이름에 결과를 붙입니다.

판정 기준: 큐브와 왼쪽 marker 의 수평(XY) 거리 `d` 가 35 mm 이하이고, 그리퍼가 열려 있으면(`finger_joint` ≤ 0.4 rad) SUCCESS 입니다. 높이는 보지 않습니다.

### 만들어지는 파일

| 파일 | 내용 |
| --- | --- |
| `outputs/harness_logs/my_s131.run.log` | trial 별 `start=[x, y]  SUCCESS d=2mm grip=0.05` 와 `== 9/10 rollouts succeeded` |
| `outputs/harness_logs/my_s131.steps.csv` | Isaac 의 렌더 step 마다 한 줄: 관절, TCP, 큐브 위치·자세 (약 22 MB) |
| `outputs/rollout_videos/my_s131/trial_NN_SUCCESS.mp4` | trial 영상 (약 1 MB, 3배속) |
| `r100_isaac.log`, `rn_server.log`, `rn_client_<i>.log`, `rec_<i>.log` | 구성요소 로그. **실행마다 덮어씁니다** |

- 클라이언트 로그 끝의 `ROS publish failed ... publisher's context is invalid` 는 70초 타임아웃으로 종료될 때 항상 나오는 정상 문구입니다.
- 영상 이름이 `trial_NN_.mp4` 처럼 비어 있으면 판정 스크립트가 실패한 것이고, 실패로 집계됩니다.
- ACT 는 이 설정에서 CPU 로 추론하며, 한 번에 약 64 ms 걸렸습니다.

### Isaac 화면을 보면서 평가하기

하네스는 Isaac 을 죽이고 headless 로 다시 띄우므로, 화면을 보려면 스택을 직접 올리고 `rollout_n.sh` 를 부릅니다. 두 방법은 동시에 쓸 수 없습니다.

```bash
# 터미널 1 — 실습 1 의 Isaac GUI 명령 끝에 로그 저장을 붙임 (start= 출력용)
#   ... --with-camera --with-static-cam 2>&1 | tee outputs/harness_logs/gui_isaac.log
#   넓은 범위는 --seed 211 --randomize-radius-x 0.10 --randomize-radius-y 0.06

# 터미널 2 — /clock 이 도는 것을 확인한 뒤
export ROS_DOMAIN_ID=42 GRASP_MODE=physical
timeout 5 ros2 topic echo --once /clock
bash src/ur_bringup/scripts/harness/bringup.sh                       # == ready
ros2 launch ur_bringup policy_inference.launch.py use_sim:=true       # Loaded forward_position_controller
RECORD_DIR=$PWD/outputs/rollout_videos/gui_s131 \
  bash src/ur_bringup/scripts/harness/rollout_n.sh 10 70 $PWD/outputs/act_red_left_190_phys/checkpoints/200000/pretrained_model
```

- `policy_inference.launch.py` 를 빠뜨리면 streaming 전환이 조용히 실패해서, 팔이 전혀 움직이지 않고 모든 trial 이 FAIL 로 채점됩니다. `ros2 control list_controllers` 로 `forward_position_controller` 가 로드됐는지 확인합니다.
- `rollout_n.sh` 를 직접 부르면 기본값이 6 trial, 90초입니다. 인자를 항상 줍니다.
- 이 경로에서는 `SEED`/`RAND` 환경변수가 효과가 없습니다. Isaac 인자로 직접 줍니다. 같은 Isaac 에서 `rollout_n.sh` 를 두 번 돌리면 두 번째는 다른 배치가 나옵니다.
- GUI Isaac 의 속도는 재지 않았습니다. trial 이 시간 부족으로 끊기면 초를 120 정도로 늘립니다. 이 초는 벽시계 기준입니다.

## 실습 6 — 결과 읽기

결과는 세 가지로 봅니다. `run.log` 로 성공률을, `steps.csv` 로 trial 안의 움직임을, MP4 로 장면을 확인합니다.

### run.log — 성공률

```
== rollout_gui_then_n: SEED=131 RAND=<none>
== 10 rollouts x 70s  ckpt=/isaac-sim/volume/ur_ws/outputs/act_red_left_190_phys/checkpoints/200000/pretrained_model
---- trial 1/10
     start=[0.618, -0.088]  SUCCESS d=2mm grip=0.05
...
---- trial 9/10
     start=[0.544, -0.092]  FAIL d=182mm grip=0.05 (too far)
...
== 9/10 rollouts succeeded
```

- `start=[x, y]`: 리셋 직후 빨간 큐브의 위치(m). 기본 위치는 \[0.60, −0.10\] 이고, |x − 0.60| 이 60 mm 를 넘으면 기본 범위 밖(바깥 띠)입니다.
- `d`: 큐브와 왼쪽 marker(\[0.60, 0.215\]) 의 수평 거리. `grip`: 끝났을 때 그리퍼 각도(rad).
- `(still holding)` 은 끝났을 때 그리퍼가 닫혀 있었다(`grip` > 0.4)는 뜻입니다. 큐브를 쥐 채 멀리 있는 경우와 빈 그리퍼를 닫은 경우가 모두 들어갑니다. `(too far)` 는 그리퍼는 열렸지만 큐브가 marker 에서 35 mm 넘게 떨어져 있는 경우입니다.
- 한꺼번에 보기: `grep -H '^== [0-9]*/' outputs/harness_logs/*.run.log`

보고서 rollout 결과 (각 10 trial × 70초, 오차는 성공 trial 의 평균 `d`):

| run.log | 체크포인트 | seed / 범위 | 결과 | 평균 오차 |
| --- | --- | --- | --- | --- |
| `e190_new200k_s131` | 최종 200k | 131 / 기본 | 9/10 | 4.0 mm |
| `e190_new200k_s171` | 최종 200k | 171 / 기본 | 10/10 | 6.0 mm |
| `e190_new200k_w211` | 최종 200k | 211 / 넓음 | 10/10 | 4.7 mm |
| `e190_new200k_w251` | 최종 200k | 251 / 넓음 | 10/10 | 6.0 mm |
| `e190_old200k_w211` | baseline 200k | 211 / 넓음 | 7/10 | 11.4 mm |
| `e190_old200k_w251` | baseline 200k | 251 / 넓음 | 6/10 | 10.5 mm |

직접 돌린 결과가 이 표와 비슷하면 재현된 것입니다. 같은 seed 는 같은 배치를 주므로 `start=` 목록도 같아야 합니다.

### steps.csv — trial 안의 움직임

Isaac 이 `--grasp-log` 로 쓰는 파일입니다. sim 1초에 60줄이고, 10 trial 이 한 파일에 이어져 있습니다(약 61,000줄, 22 MB).

- 열(45개): `t`(sim 초), `q_*`/`v_*`(팔 6개 + 그리퍼 6개 관절의 위치·속도), `eff_finger`, `pad_sep`, `tcp_x/y/z`, `grasp_active`, `red_x/y/z/qw/qx/qy/qz`, `blue_...`.
- 접촉 중에는 `v_*` 값을 믿을 수 없습니다(PhysX 가 상한값을 보고). 움직임은 `q_*` 위치로 판단합니다.
- trial 구분: 리셋 때 큐브가 z 0.225 로 순간이동했다가 테이블(0.2175)로 떨어집니다. 거리 기준(예: 4 cm 점프)은 첫 trial 이나 넘어진 큐브 때문에 틀릴 수 있습니다. 아래 명령은 보고서의 모든 run 에서 정확히 10개를 찾았습니다.

```bash
F=outputs/harness_logs/e190_new200k_s131.steps.csv
head -1 $F | tr ',' '\n' | grep -n '^red_'            # red_x/y/z = 32/33/34 열인지 확인
awk -F, 'NR>2{dx=$32-px;dy=$33-py; if($34=="0.22500"&&pz!="0.22500"&&sqrt(dx*dx+dy*dy)>0.003) printf "%.1f [%.3f,%.3f]\n",$1,$32,$33} NR>1{px=$32;py=$33;pz=$34}' $F
```

- 출력의 k 번째 줄이 trial k 의 시작입니다. trial 간격은 sim 약 100초입니다.
- 파지 요약: `python3 src/ur_bringup/scripts/harness/grasp_log_stats.py outputs/harness_logs/e190_new200k_s131` (표준 라이브러리만, 0.5초). 쥐고 있던 구간(hold)마다 길이, 최대 높이, 미끄러짐(mm)을 보여 줍니다. 예를 들어 trial 9 의 떨어뜨림은 hold #9 의 `3.5 s ... slip 9.3 mm` 입니다. hold 번호는 trial 번호와 자동으로 연결되지 않습니다.
- 스택이 떠 있을 때 현재 장면을 손으로 판정: `python3 src/ur_bringup/scripts/harness/judge_rollout.py`

### 영상 — MP4 와 GIF

- `outputs/rollout_videos/<tag>/trial_NN_SUCCESS.mp4`: 왼쪽은 외부 카메라, 오른쪽은 손목 카메라, 위쪽 막대에 체크포인트 이름, trial 번호, sim 시간이 나옵니다(960×404).
- frame i = 녹화 시작 후 sim i/10 초. sim 1초에 10장을 30 fps 로 재생하므로 3배속입니다.
- 첫 몇 초는 팔이 시작 자세에 멈춰 있고, frame 0 은 카메라 이미지가 아직 없어 검을 수 있습니다.
- 녹화기 로그 `rec_<i>.log` 의 `cameras seen: ['exterior', 'wrist']` 에 카메라가 빠지면 그쪽 화면이 검게 녹화된 것입니다.

GIF 만들기 (보고서 GIF 와 같은 방식: 2 frame 마다 1장, 640×269, 96색, 150 ms → 1.33배속):

```bash
deps/.venv-ml/bin/python -I - <<'PY'
import av
from PIL import Image
SRC = "outputs/rollout_videos/e190_new200k_w211/trial_04_SUCCESS.mp4"
OUT = "outputs/report_media/my_success.gif"
A, B = 10, 225                     # 시작·끝 frame (sim 1.0 s ~ 22.5 s)
fr = []
with av.open(SRC) as c:
    for i, f in enumerate(c.decode(video=0)):
        if i > B: break
        if i >= A and (i - A) % 2 == 0:
            fr.append(f.to_image().resize((640, 269), Image.Resampling.LANCZOS)
                      .convert("P", palette=Image.Palette.ADAPTIVE, colors=96))
fr[0].save(OUT, save_all=True, append_images=fr[1:], duration=150, loop=0, optimize=True)
print(len(fr), "frames ->", OUT)
PY
```

- `A, B = 90, 225` 로 바꾸면 보고서의 `success_final_raw.gif`(68장, 약 3.0 MB)가 거의 같은 크기로 나옵니다. 각 GIF 의 원본 trial 과 frame 범위는 `outputs/report_media/README.txt` 에 있습니다.
- 막대 글자를 바꾼 버전(`final (190 demos)`, `(1.33x)`)은 같은 프레임 위 막대(색 (24,24,24))를 PIL `ImageDraw` 와 `DejaVuSans-Bold.ttf` 로 덮어 그린 것입니다.
- 여러 frame 을 한 장에 보려면 같은 방식으로 frame 을 골라 `Image.new` 에 격자로 붙입니다.

## 정리·종료와 자주 만나는 문제

수집과 rollout 은 끝나도 Isaac 과 제어 스택을 남겨 둡니다. 다음 작업(특히 학습) 전에 항상 정리합니다.

```bash
export ROS_DOMAIN_ID=42
bash src/ur_bringup/scripts/harness/shutdown_rollout.sh --dry-run   # 무엇을 종료할지 목록만
bash src/ur_bringup/scripts/harness/shutdown_rollout.sh             # Isaac, 제어 스택, MoveIt, RViz, policy server/client
bash src/ur_bringup/scripts/harness/shutdown_all.sh                 # 위 + 수집 루프 + 모든 ML 프로세스(학습·변환)
```

- 마지막 줄이 `CLEAN` 이면 끝입니다. `LEFT: ...` 이면 남은 프로세스나 노드가 찍힙니다.
- 이 워크스페이스의 `install/` 에서 뜬 프로세스만 골라서 종료합니다. 다른 워크스페이스의 프로세스는 `not ours (left running)` 으로 남깁니다.
- `ROS_DOMAIN_ID` 없이 실행하면 도메인 0 의 다른 컨테이너 노드를 보고 `LEFT` 를 냅니다.
- `outputs/harness_logs/` 나 다른 곳에 둔 내 런처(`run_practice_*.sh`, `eval_190.sh` 복사본)는 `shutdown_all.sh` 도 종료하지 않습니다. 런처가 살아 있으면 다음 단계를 다시 시작하므로, `pgrep -af run_practice` 로 PID 를 찾아 먼저 `kill` 합니다.

### 문제 해결

| 증상 | 원인 | 해결 |
| --- | --- | --- |
| `FAIL: isaac`, Isaac 로그에 `Could not find Isaac Sim assets folder` | NVIDIA 에셋 서버에 접속 불가 | 네트워크 확인 후 재시도 |
| `scene topics` 는 나왔는데 `/clock` 이 수 분간 없음 | 셰이더 캐시가 비어 컬파일 중(약 3.5분) | 기다림. 하네스는 최대 5\~10분 기다림 |
| `no 'ros2_control' tag found in the URDF`, 모르는 노드가 보임 | 도메인 0 의 다른 컨테이너와 섞임 | 모든 터미널에서 `export ROS_DOMAIN_ID=42` 후 Isaac 부터 다시 |
| rollout 에서 팔이 전혀 안 움직이고 모두 FAIL | `forward_position_controller` 미로드 | `ros2 launch ur_bringup policy_inference.launch.py use_sim:=true` |
| pick&place 가 SUCCESS 라는데 팔이 안 움직임 | MoveIt 을 `ur_only:=true`(기본값)로 띄움 | `ur_only:=false` 로 다시, 또는 `bringup.sh` 사용 |
| `reset_pose.py` 후 Servo/계획이 거부됨 | 기본 자세 `home` 은 팔꿈치 특이점 | 항상 `reset_pose.py ready` |
| `reset_pose.py` 가 `goal REJECTED` 또는 무반응 | streaming 모드에 있음 | `switch_control_mode.py trajectory` |
| 수집 후 변환·학습이 계속 "기다리는 중" | 명령줄에 `collect_100.sh` 가 들어 간 래퍼 셸을 `pgrep` 이 잡음 | 명령을 런처 파일로 실행 |
| `FileExistsError: Output directory ... already exists` | `lerobot-train` 의 출력 폴더가 이미 있음 | 새 `--output_dir`, 또는 `--resume` |
| 하네스 종료 코드는 0 인데 결과가 없음 | 수집·변환·학습 하네스는 실패해도 0 을 돌려줌 | `FAIL` 을 grep, 결과물(episode 수, `info.json`, `checkpoints/<STEPS>`) 확인 |
| GUI Isaac 이 갑자기 사라짐 | 수집·rollout 하네스가 모든 Isaac 을 종료하고 headless 로 다시 띄움 | 정상 동작. 화면이 필요하면 실습 5 의 수동 경로 |
| Isaac 인자를 바꿨는데 변화가 없음 | Isaac 은 모르는 인자(오타)를 조용히 무시 | 시작 배너(`grasp mode :`, `work table :`)로 확인 |
| Isaac 종료 때 `RCLError: failed to initialize wait set` | 종료 시 나오는 정상 로그 | 무시 |

## 빠른 참조

### 하네스와 덮어쓰기 변수

모두 `src/ur_bringup/scripts/harness/` 에 있습니다. "지우는 대상"은 덮어쓰기 없이 실행했을 때 지워지는 기본 경로입니다.

| 하네스 | 인자 | 환경변수 (기본값) | 지우는 대상 | 성공 문구 |
| --- | --- | --- | --- | --- |
| `bringup.sh` | 없음 | `HARNESS_LOG` | 없음(기존 제어 스택 종료) | `== ready` |
| `collect_100.sh` | episode 수 (50) | `OUT`, `SEED`(91), `TAG`(c100), `RAND`, `APPEND`(0) | `outputs/il_raw_red_left_100_phys` | `== ALL TASKS DONE: N episodes` |
| `convert_100.sh` | 없음 | `RAW`, `DS`, `REPO`, `TAG`(convert\_100) | `outputs/lerobot_ds_red_left_100_phys` | `== CONVERSION DONE` |
| `train_100.sh` | 없음 | `DS`, `REPO`, `CK`, `STEPS`(60000), `SAVE_FREQ`(20000), `TAG`(train\_100) | `outputs/act_red_left_100_phys` | 로그의 `End of training` |
| `rollout_gui_then_n.sh` | trial 수 (10), 초 (70), 체크포인트 | `SEED`(131), `RAND`, `GRASP_LOG`, `RECORD_DIR` | 없음(구성요소 로그는 덮어씁) | `== k/N rollouts succeeded` |
| `rollout_n.sh` | trial 수 (6), 초 (90), 체크포인트 | `RECORD_DIR`, `START_POSE`(ready) | 없음 | `== k/N rollouts succeeded` |
| `shutdown_rollout.sh` | `--dry-run`, `--all` | `ROS_DOMAIN_ID` | 해당 없음 | `CLEAN` |

공통: `ROS_DOMAIN_ID`(기본 0 → 항상 42), `GRASP_MODE`(physical). 경로 변수(`OUT`, `DS`, `CK`)는 절대경로여야 합니다.

### seed 와 배치 범위

| 용도 | seed | Isaac 추가 인자 | 큐브 범위 |
| --- | --- | --- | --- |
| 기본 범위 수집 (100) | 91 | 없음 | x, y ±60 mm |
| 바깥 띠 수집 (90) | 301 | `--randomize-radius-x 0.10 --randomize-radius-y 0.06 --randomize-inner-x 0.06 --randomize-yaw-edge-frac 0.3 --randomize-yaw-edge-deg 10` | x ±60\~100 mm, y ±60 mm |
| 기본 범위 평가 | 131, 171 | 없음 | x, y ±60 mm |
| 넓은 범위 평가 | 211, 251 | `--randomize-radius-x 0.10 --randomize-radius-y 0.06` | x ±100 mm, y ±60 mm |

### 체크포인트

| 모델 | 경로 (`pretrained_model` 까지) | 학습 데이터 |
| --- | --- | --- |
| 최종 | `outputs/act_red_left_190_phys/checkpoints/200000/pretrained_model` | 190 episode, 200k step |
| baseline | `outputs/act_red_left_100_phys_long/checkpoints/200000/pretrained_model` | 100 episode, 200k step |
| rollout 기본값(주의) | `outputs/act_red_left_100_phys/checkpoints/last` → 060000 | 100 episode, 60k step |

### 걸리는 시간 (보고서 작업 실측, 벽시계)

| 단계 | 시간 |
| --- | --- |
| Isaac + 제어 스택 기동 | 약 45초 (셰이더 캐시 없으면 +3.5분) |
| 수집 100 / 90 episode | 약 40분 / 36분 (사이클당 약 23초) |
| 변환 100 / 190 episode | 약 7분 / 16\~17분 |
| 학습 190 episode, 200k step | 약 4시간 23분 (12.7 step/s, 20k 당 약 26분) |
| rollout 10 trial | 약 15분 |

### 더 자세한 문서

- `src/PIPELINE.md` — 수집→변환→학습→추론 명령과 단계별 합격 기준
- `src/HISTORY.md` §53(물리 파지), §54(physical ACT), §58(190 episode 최종)
- `src/ur_bringup/scripts/harness/README.md` — 하네스 목록
- `outputs/harness_logs/eval_190.sh` — 보고서의 12개 rollout 을 연달아 돌린 런처(git 에는 없음). 다시 쓸 때는 복사해서 태그와 요약 파일 이름을 바꿉니다. 그대로 실행하면 보고서 결과를 덮어씁니다.
