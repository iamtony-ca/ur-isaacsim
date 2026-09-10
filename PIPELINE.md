# PIPELINE — IL 전체 파이프라인 실행 명령

**수집 → 변환 → 학습 → 추론** 을 처음부터 끝까지 재현하는 명령 모음.
아래 명령은 전부 2026-09-09 에 실제로 돌려 **9/10 성공**까지 확인한 것이다.

- 새 PC 설치·하드웨어 연결은 [`CHECKLIST.md`](CHECKLIST.md).
- 왜 이 값들인지(그립 기하, 대기시간, 데이터량)는 [`HISTORY.md`](HISTORY.md) §28~§38.
- 설계 방향은 [`ur_bringup/docs/plan_il_vla.md`](ur_bringup/docs/plan_il_vla.md).

---

## 0. 모든 터미널에서 먼저

```bash
source /opt/ros/jazzy/setup.bash
source /isaac-sim/volume/ur_ws/install/setup.bash
export ROS_DOMAIN_ID=0
cd /isaac-sim/volume/ur_ws
```

> ML 명령(변환·학습·추론)은 **ROS 쉘에 venv 를 source 하지 않고** 인터프리터를 직접 지정한다:
> `deps/.venv-ml/bin/python`. 섞으면 numpy ABI 가 충돌한다(SETUP.md §2-C).

---

## 1. 수집

### 1-1. Isaac (터미널 1)

```bash
/isaac-sim/python.sh src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
  --asset-path /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd \
  --scene pick_place --table --table-height 0.20 --table-pose 0.72,0.0 --table-size 0.70,0.90 \
  --object-pose 0.60,0.0,0.225 --object-size 0.035,0.035,0.035 \
  --place-pose 0.60,0.315,0.0 --object-names red,blue --place-names left,right \
  --object-spacing 0.20 --randomize-object --randomize-radius 0.06 --seed 91 \
  --camera-res 320x240 --grasp-attach --with-camera --with-static-cam --headless
```

`scene topics` 가 찍히면 준비 완료. GUI 로 보려면 `--headless` 를 뺀다.

| 인자 | 왜 이 값인가 |
|---|---|
| `--randomize-radius 0.06` | 물체 랜덤 ±60 mm. **최대치** — 더 키우면 red/blue 큐브가 겹친다(홈 간격 200 mm, 큐브 35 mm) |
| `--camera-res 320x240` | ACT 는 리사이즈하지 않아 **데이터셋 해상도 = 모델 입력**. 480×640 은 `/dev/shm` 64 MiB 에서 DataLoader 가 죽는다 |
| `--grasp-attach` | 접촉 마찰 대신 FixedJoint 로 파지. 없으면 손가락이 물체를 튕겨낸다 |
| `--seed` | 재현용. **롤아웃 때는 다른 시드**를 써야 학습에서 본 적 없는 위치가 된다 |

### 1-2. 제어 스택 + MoveIt (터미널 2)

```bash
ros2 launch ur_bringup ur16e_2f85_d405.launch.py use_sim:=true
# 다른 터미널에서
ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py use_sim:=true ur_only:=false
```

> **`ur_only:=false` 필수.** 기본값 true 는 move_group 에 팔만 있는 모델을 줘서 `gripper_frame`
> 을 모르고, MoveIt 은 모르는 링크 제약을 "이미 만족"으로 처리한다 → **매번 SUCCESS 인데 안 움직인다.**

### 1-3. 수집 실행 (터미널 3)

```bash
# 단일 태스크 N개 (ACT용). ACT 는 지시문을 읽지 않으므로 태스크 1종이 원칙(HISTORY.md §30)
TASKS="red|left|put the red block on the left marker" \
  src/ur_bringup/scripts/collect_il_episodes.sh 100 outputs/il_raw_red_left_100

# 3태스크 (VLA용) — 인자 없이 기본값
src/ur_bringup/scripts/collect_il_episodes.sh 20 outputs/il_raw_3task
```

| 환경변수 | 용도 |
|---|---|
| `TASKS` | `"obj\|place\|instruction"`. 여러 개는 개행/세미콜론 구분 |
| `CAM_EXTERIOR=none` | 손목 카메라만 기록 (빈 문자열은 rcl 이 거부하므로 `none`) |
| `APPEND=1` | 기존 에피소드 유지하고 추가. **파라미터가 같을 때만** 안전 |
| `LOGS` | 로그 디렉터리 |

**수집 후 확인** — 성공 카운트만 보지 말 것:

```bash
python3 - <<'PY'
import json, glob, numpy as np
mv=[]; g=[]; lens=[]
for f in sorted(glob.glob("outputs/il_raw_red_left_100/episode_*/data.json")):
    d=json.load(open(f)); fr=d["frames"] if isinstance(d,dict) and "frames" in d else d
    st=np.array([x["state.single_arm"] for x in fr]); ac=np.array([x["action.single_arm"] for x in fr])
    gs=np.array([x["state.gripper"][0] for x in fr]); ga=np.array([x["action.gripper"][0] for x in fr])
    mv.append(np.maximum(np.abs(ac-st).max(1), np.abs(ga-gs))); g.append(gs); lens.append(len(fr))
m=np.concatenate(mv); G=np.concatenate(g)
print(f"에피소드 {len(lens)} 프레임 {len(m)} 길이중앙값 {np.median(lens)/30:.1f}s")
print(f"정지 {(m<=1e-4).mean()*100:.1f}%  그리퍼max {G.max():.4f}  >0.95 {(G>0.95).mean()*100:.1f}%")
print(f"이상치(>25s) {sum(1 for l in lens if l/30>25)}")
PY
```

**합격 기준** (실측 근거는 HISTORY.md §31~§38):

| 지표 | 기준 | 어긋나면 |
|---|---|---|
| 정지 프레임 | **< 15%** | ACT 가 "움직이지 마"를 배운다 (§31) |
| 그리퍼 최대값 | **0.650** (= `grip_closed` 0.52 / 0.8) | 1.0 이면 손가락이 물체를 뚫고 닫힌 것 (§33) |
| `>0.95` 비율 | **0%** | 실물 2F-85 가 낼 수 없는 값 |
| 길이 이상치(>25 s) | **0** | 그리퍼 액션이 15 s 타임아웃 중 (§34) |

---

## 2. 변환 (LeRobot v3.0)

```bash
deps/.venv-ml/bin/python src/ur_bringup/scripts/raw_to_lerobot.py \
  --raw outputs/il_raw_red_left_100 --dry-run          # 먼저 에피소드/태스크 확인

deps/.venv-ml/bin/python src/ur_bringup/scripts/raw_to_lerobot.py \
  --raw outputs/il_raw_red_left_100 \
  --repo-id tony/ur16e_pick_place_red_left_100 \
  --root outputs/lerobot_ds_red_left_100
```

- 카메라 대수는 `meta.json` 의 `video_keys` 를 따라가므로 **인자 불필요**. 1대면 feature 1개.
- `--max-idle-run N` 은 정지 구간을 N 프레임으로 제한하는 **후처리**. 상태머신이 최적화된
  지금은 필요 없다(정지 8.6%). 옛 데이터를 살릴 때만 쓴다.
- `torchcodec` 로드 실패 트레이스백은 **무해**(pyav 폴백). 성공 판정은 `meta/info.json` 존재로.

```bash
deps/.venv-ml/bin/python -c "
import json; i=json.load(open('outputs/lerobot_ds_red_left_100/meta/info.json'))
print(i['total_episodes'],'ep', i['total_frames'],'frames')
[print(' ',k,v['dtype'],tuple(v['shape'])) for k,v in i['features'].items() if k.startswith(('observation','action'))]"
```

---

## 3. 학습 (ACT)

> **★ 학습 전에 Isaac 을 내린다.** Isaac + ROS 스택이 64 MiB `/dev/shm` 중 ~14 MiB 를
> Fast-DDS 로 점유해서 DataLoader 워커가 죽는다(§32.4).

```bash
pkill -f ur16e_isaac_ros2.py ; pkill -f "ros2 launch ur_bringup" ; pkill -f "lib/rviz2/rviz2"
sleep 10; rm -f /dev/shm/torch_*; df -h /dev/shm     # 60 MiB 가까이 비어야 한다

UR_WS_TORCH_SHM_FIX=1 deps/.venv-ml/bin/lerobot-train \
  --policy.type=act --policy.push_to_hub=false \
  --dataset.repo_id=tony/ur16e_pick_place_red_left_100 \
  --dataset.root=outputs/lerobot_ds_red_left_100 \
  --steps=60000 --batch_size=32 --num_workers=4 \
  --output_dir=outputs/act_red_left_100
```

| 인자 | 이유 |
|---|---|
| `--num_workers=4` | 실측 최적. 8/6 은 `/dev/shm` 부족으로 죽고, 2 는 7.9, 0 은 1.4 step/s |
| `--push_to_hub=false` | `[training]` extra 설치 후 hub 검증이 켜져서 없으면 즉사 |
| `--steps=60000` | 데이터량을 비교할 때는 **스텝을 고정**해야 한다 |

**실측 속도**: 2대 카메라 43,222 프레임 → 14.7 step/s (68분) / 1대 20,996 프레임 → 25.6 step/s (39분)

> **loss 로 정책을 비교하지 말 것.** 데이터가 2배면 같은 스텝에서 epoch 이 절반이라 loss 가
> 높게 나오지만 성능은 더 좋았다(§38.3). 정지 프레임이 많으면 loss 는 낮은데 태스크를 못 한다(§32.5).
> **판정은 롤아웃이다.**

### 3-B. 학습 (GR00T N1.7, VLA) — ⏳ 미검증

**§1·§2 (수집·변환)는 그대로다.** 같은 데이터셋을 쓰되, ACT 와 달리 **태스크를 필터하지 않는다** —
GR00T 는 `task` 문자열을 조건으로 받으므로 3종 혼합 데이터셋이 그대로 맞다(§30 의 ACT 제약과 반대).

설치와 게이트는 [`SETUP.md`](SETUP.md) §2-C-2 를 먼저 볼 것 (`lerobot[groot]`, `HF_HOME`,
**gated `nvidia/Cosmos-Reason2-2B` 라이선스 동의 + `HF_TOKEN`**).

```bash
export HF_HOME=/isaac-sim/volume/ur_ws/deps/hf_cache
export WANDB_DISABLED=true
BASE=$(deps/.venv-ml/bin/python -c \
  "from huggingface_hub import snapshot_download; print(snapshot_download('nvidia/GR00T-N1.7-3B'))")

deps/.venv-ml/bin/lerobot-train \
  --policy.type=groot --policy.push_to_hub=false --wandb.enable=false \
  --policy.base_model_path="$BASE" \
  --policy.model_params_fp32=false \
  --policy.use_relative_actions=true \
  --policy.relative_exclude_joints='["gripper"]' \
  --dataset.repo_id=tony/ur16e_pick_place_240_v2 \
  --dataset.root=outputs/lerobot_ds_240_v2 \
  --batch_size=8 --num_workers=2 \
  --output_dir=outputs/groot_240_v2
```

| 인자 | 이유 |
|---|---|
| `--policy.base_model_path="$BASE"` | **repo id 금지** — `is_dir()` 판정에 걸려 체크포인트 사이드카가 경고 없이 무시되고 전처리가 lerobot 기본값으로 바뀐다. 학습은 그냥 돌아가서 알아챌 수 없다 |
| `--policy.model_params_fp32=false` | 기본 `true` 는 정적 **29.8/31.8 GiB** 로 **배치 크기와 무관하게** 32 GB 초과. `false` 면 ≈17.8 GiB |
| `--policy.use_relative_actions=true` | N1.7 은 상대 액션 청크로 사전학습됐다(`processor_kwargs.use_relative_action: True`) |
| `--policy.relative_exclude_joints='["gripper"]'` | Isaac-GR00T 의 single-arm + absolute-gripper 규약. 그리퍼를 델타로 두면 파지/해제 같은 이산 사건이 누적오차에 녹는다 |
| `--wandb.enable=false` | `GrootConfig.report_to` 기본값이 `wandb` |

**롤아웃(§4)은 세 인자만 바뀐다**: `--policy_type=groot`, `--pretrained_name_or_path=<체크포인트>`,
`--actions_per_chunk=40`(ACT 는 50). 판정기 `judge_rollout.py` 가 같으므로 **ACT 9/10 · 7.6 mm 와 직접 비교 가능**하다.

> **아직 안 잰 것**: step/s(→ 총 학습시간), 추론 지연(청크 40 @30 Hz = **1.33 s 안에** 끝나야 한다),
> 21 에피소드로 충분한지(ACT 는 100 이 필요했다 §38). 설계와 합격 기준은
> [`ur_bringup/docs/plan_groot_n17.md`](ur_bringup/docs/plan_groot_n17.md) §4.

---

## 4. 추론 (롤아웃)

### 4-1. Isaac + 제어 스택 재기동

수집 때와 같되 **다른 시드**로 — 학습에서 본 적 없는 물체 위치여야 한다.

```bash
/isaac-sim/python.sh src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
  --asset-path /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd \
  --scene pick_place --table --table-height 0.20 --table-pose 0.72,0.0 --table-size 0.70,0.90 \
  --object-pose 0.60,0.0,0.225 --object-size 0.035,0.035,0.035 \
  --place-pose 0.60,0.315,0.0 --object-names red,blue --place-names left,right \
  --object-spacing 0.20 --randomize-object --randomize-radius 0.06 --seed 131 \
  --camera-res 320x240 --grasp-attach --with-camera --with-static-cam        # GUI 로 보려면 --headless 제거

ros2 launch ur_bringup ur16e_2f85_d405.launch.py use_sim:=true
ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py use_sim:=true ur_only:=false
```

### 4-2. 스트리밍 컨트롤러 + 초기 상태

```bash
ros2 launch ur_bringup policy_inference.launch.py use_sim:=true   # forward_position_controller 스폰(inactive)

# ★ 학습과 같은 초기 조건을 만든다. 안 하면 정책이 얼어붙는다(§31.1)
python3 src/ur_bringup/isaac/common/switch_control_mode.py trajectory
ros2 service call /scene/reset_episode std_srvs/srv/Trigger
python3 src/ur_bringup/isaac/common/reset_pose.py ready        # error_code: 0 확인
python3 src/ur_bringup/isaac/common/switch_control_mode.py streaming
```

> 시연 100개의 시작 자세 편차는 **0.0002 rad** — 사실상 한 점이다. READY 가 아닌 곳에서
> 시작하면 정책이 본 적 없는 상태이고, 명령은 나가는데 팔이 안 움직인다.

### 4-3. 정책 실행 (upstream LeRobot)

```bash
# 터미널 A — 정책 서버
deps/.venv-ml/bin/python -m lerobot.async_inference.policy_server --host=127.0.0.1 --port=8080

# 터미널 B — 로봇 클라이언트 (우리 코드는 --robot.type 플러그인 하나뿐)
deps/.venv-ml/bin/python -m lerobot.async_inference.robot_client \
  --robot.type=ur16e_ros --robot.id=sim \
  --policy_type=act \
  --pretrained_name_or_path=outputs/act_red_left_100/checkpoints/last/pretrained_model \
  --actions_per_chunk=50 --task="put the red block on the left marker" \
  --server_address=127.0.0.1:8080
```

**카메라 1대로 학습한 정책이면** 어댑터에도 알려야 한다(안 그러면 `/static_cam` 을 기다리다 실패):

```bash
  --robot.cameras_ros='{"wrist": "/camera/color/image_raw"}'
```

> `--task` 는 ACT 가 읽지 않는다(§30). 클라이언트가 요구해서 넣을 뿐이고, 정책이 VLA 가 되면
> 그때는 실제 입력이 된다 — **명령줄은 그대로**.

### 4-4. 판정 — GT 로

"팔이 움직였다"는 판정이 아니다. 상태머신과 같은 기준(마커에서 35 mm 이내 + 그리퍼 개방)을 쓴다.

```bash
python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
TOL = 0.035
rclpy.init(); n = Node("judge"); st = {}
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
n.create_subscription(PoseStamped, "/scene/objects/red/pose",
                      lambda m: st.__setitem__("p", m.pose.position), 10)
for q in (latched, 10):                      # 마커 포즈는 latched 라 일반 구독은 못 받는다
    n.create_subscription(PoseStamped, "/scene/places/left/pose",
                          lambda m: st.__setitem__("L", m.pose.position), q)
n.create_subscription(JointState, "/joint_states",
                      lambda m: st.__setitem__("g", m.position[m.name.index("finger_joint")])
                      if "finger_joint" in m.name else None, 10)
e = time.time() + 15
while time.time() < e and len(st) < 3: rclpy.spin_once(n, timeout_sec=0.1)
p, L, g = st["p"], st["L"], st["g"]
d = ((p.x-L.x)**2 + (p.y-L.y)**2) ** 0.5
print(f"{'SUCCESS' if d <= TOL and g <= 0.4 else 'FAIL'}  d={d*1000:.0f}mm grip={g:.2f}")
PY
```

**1회 성공은 성공률이 아니다.** 시행마다 리셋 + READY 를 거쳐 10회는 돌리고, 물체 시작 위치를
함께 기록한다 — 실패가 특정 영역에 몰리면 그 구역의 시연이 부족하다는 뜻이다(§36).

---

## 5. 실측 기준값

| | 값 | 출처 |
|---|---|---|
| 사이클 시간 | 11 s | §32 (대기 제거 전 41.3 s) |
| 정지 프레임 | 8.6% | §38 (v2 는 65.8%) |
| `grip_closed` | **0.52** | §35 접촉 시작점. 힘 평형점 0.599 가 아니다 |
| `grasp_z_offset` | **0.015** | §34 더 깊으면 핑거가 테이블과 간섭 |
| 에피소드 수 | **100** | §36/§38 50 은 랜덤 범위 가장자리에서 정렬 실패 |
| 롤아웃 | **9/10**, 평균 7.6 mm | §38 |

---

## 6. 자주 걸리는 것

| 증상 | 원인 |
|---|---|
| 명령은 나가는데 팔이 안 움직인다 | 스트리밍/궤적 컨트롤러 상호배타. `ros2 control list_controllers` |
| MoveIt 이 매번 SUCCESS 인데 안 움직인다 | `ur_only:=false` 누락 |
| 정책이 PRE_GRASP 에서 얼어붙는다 | 시작 자세가 READY 가 아니다 (§31.1) |
| DataLoader 워커가 죽는다 | Isaac 이 `/dev/shm` 점유 중 (§32.4) |
| 학습 loss 는 좋은데 태스크를 못 한다 | 정지 프레임 과다 (§31) |
| 수집 성공률은 높은데 사이클이 30 s | 그리퍼 액션 15 s 타임아웃 (§34) |
| 그리퍼가 물체를 뚫고 닫힌다 | `grip_closed` 가 접촉점보다 깊다 (§35) |
