# PIPELINE — IL 전체 파이프라인 실행 명령

**수집 → 변환 → 학습 → 추론** 을 처음부터 끝까지 재현하는 명령 모음.
아래 명령은 2026-09-09 에 실제로 돌려 **9/10 성공**까지 확인한 흐름이다.

> **★ 2026-10-19 부터 수집 기본은 물리 접촉 파지(`--grasp-mode physical`, `HISTORY.md` §53).** 아래 명령·경로는
> physical 기준(`*_phys`)으로 고쳤다. **2026-10-20 physical 전체 실행**: 수집 100/100 → 학습 60k → 롤아웃 **8/10**(시드 131)·
> **9/10**(새 시드 171, 영상) (`HISTORY.md` §54.5); 200k 까지 이어 학습하면 **19/20**·오차 절반(§54.6). **2026-10-22 현재 최종**: 바깥 띠 90 ep 를 더한 190 ep 200k(`outputs/act_red_left_190_phys/checkpoints/200000`, §3-2) — 기존 범위 **19/20**·넓은 범위(x ±100 mm) **20/20**(기존 100 ep 모델 13/20), 놓는 오차 4–6 mm(`HISTORY.md` §58.7). §5 표는 용접(attach) 시대 값이고 physical 값은 §54.5. 용접 시대 데이터(`il_raw_red_left_100` 등)를 재현하거나 그 체크포인트를 롤아웃할 때는
> Isaac 에 `--grasp-attach`, 하네스·수집기에 `GRASP_MODE=attach`. **두 모드의 에피소드를 섞지 말 것** — 변환기가 거부한다.

- 새 PC 설치·하드웨어 연결은 [`CHECKLIST.md`](CHECKLIST.md).
- 왜 이 값들인지(그립 기하, 대기시간, 데이터량)는 [`HISTORY.md`](HISTORY.md) §28~§38.
- 설계 방향은 [`ur_bringup/docs/plan_il_vla.md`](ur_bringup/docs/plan_il_vla.md).

---

## 0. 모든 터미널에서 먼저

```bash
source /opt/ros/jazzy/setup.bash
source /isaac-sim/volume/ur_ws/install/setup.bash
export ROS_DOMAIN_ID=42      # 0 이 아니고 같은 호스트의 컨테이너마다 다른 값, 모든 터미널에서 같게 — 42 = 이 워크스페이스(두 번째 컨테이너면 바꾼다)
cd /isaac-sim/volume/ur_ws
```

> 하네스(`collect_il_episodes.sh` 등)는 환경의 `ROS_DOMAIN_ID` 를 그대로 물려받고, 안 정하면 0 이다.

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
  --camera-res 320x240 --grasp-mode physical --with-camera --with-static-cam --headless
```

`scene topics` 가 찍히면 준비 완료. GUI 로 보려면 `--headless` 를 뺀다.

| 인자 | 왜 이 값인가 |
|---|---|
| `--randomize-radius 0.06` | 물체 랜덤 ±60 mm. **y 는 최대치** — 더 키우면 red/blue 큐브가 겹친다(홈 간격 y 200 mm, 큐브 35 mm). x 는 `--randomize-radius-x 0.10` 으로 넓힐 수 있다(190 ep 데이터셋의 바깥 띠, §3-2·`HISTORY.md` §58) |
| `--camera-res 320x240` | ACT 는 리사이즈하지 않아 **데이터셋 해상도 = 모델 입력**. 480×640 은 `/dev/shm` 64 MiB 에서 DataLoader 가 죽는다 |
| `--grasp-mode physical` | 용접 없이 **실제 접촉 마찰로** 파지 — 힘·속도 제한 soft 프로파일(3 N·m, 1 rad/s, 120 Hz)이 기본으로 붙는다. 상태머신 sim 20/20(`HISTORY.md` §53). Isaac 이 `/scene/grasp_mode` 로 알리고 수집기가 `GRASP_MODE`(기본 physical)와 다르면 거부 |
| (구) `--grasp-attach` | 용접(FixedJoint) 파지. 2026-10-19 이전 데이터셋·체크포인트는 전부 이 모드 — 그것들을 재현·롤아웃할 때만 |
| `--seed` | 재현용. **롤아웃 때는 다른 시드**를 써야 학습에서 본 적 없는 위치가 된다 |

### 1-2. 제어 스택 + MoveIt (터미널 2)

```bash
ros2 launch ur_bringup ur16e_2f85_d405.launch.py use_sim:=true
# 다른 터미널에서 (디스플레이 없는 컨테이너면 끝에 launch_rviz:=false — 기본 true)
ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py use_sim:=true ur_only:=false
```

> **`ur_only:=false` 필수.** 기본값 true 는 move_group 에 팔만 있는 모델을 줘서 `gripper_frame`
> 을 모르고, MoveIt 은 모르는 링크 제약을 "이미 만족"으로 처리한다 → **매번 SUCCESS 인데 안 움직인다.**

### 1-3. 수집 실행 (터미널 3)

```bash
# 단일 태스크 N개 (ACT용). ACT 는 지시문을 읽지 않으므로 태스크 1종이 원칙(HISTORY.md §30)
TASKS="red|left|put the red block on the left marker" \
  src/ur_bringup/scripts/collect_il_episodes.sh 100 outputs/il_raw_red_left_100_phys

# 3태스크 (VLA용) — 인자 없이 기본값. 이름은 groot_pipeline.sh 의 physical 이름과 같게(§3-B)
src/ur_bringup/scripts/collect_il_episodes.sh 30 outputs/il_raw_3task_v4_phys
```

| 환경변수 | 용도 |
|---|---|
| `TASKS` | `"obj\|place\|instruction"`. 여러 개는 개행/세미콜론 구분 |
| `CAM_EXTERIOR=none` | 손목 카메라만 기록 (빈 문자열은 rcl 이 거부하므로 `none`) |
| `APPEND=1` | 기존 에피소드 유지하고 추가. **파라미터가 같을 때만** 안전 |
| `LOGS` | 로그 디렉터리 |
| `GRASP_MODE` | `physical`(기본) — 상태머신에 `config/common/pick_place_physical.yaml`(한 번에 0.8 로 닫기), 기록기는 **`action.gripper` 를 명령(`/isaac_joint_commands`)에서** 가져온다(정지값 0.506 을 배우면 쥐는 힘 0 → 기존 체크포인트 0/10, §53.6). `attach` = 용접 시대 설정 그대로. 둘 다 `meta.json` 에 기록 |

**수집 후 확인** — 성공 카운트만 보지 말 것:

```bash
python3 - <<'PY'
import json, glob, numpy as np
mv=[]; g=[]; lens=[]
for f in sorted(glob.glob("outputs/il_raw_red_left_100_phys/episode_*/data.json")):
    d=json.load(open(f)); fr=d["frames"] if isinstance(d,dict) and "frames" in d else d
    st=np.array([x["state.single_arm"] for x in fr]); ac=np.array([x["action.single_arm"] for x in fr])
    gs=np.array([x["state.gripper"][0] for x in fr]); ga=np.array([x["action.gripper"][0] for x in fr])
    dg=np.abs(np.diff(gs, append=gs[-1]))   # gripper: state change, not command-state (physical: 1.0 vs 0.63)
    mv.append(np.maximum(np.abs(ac-st).max(1), dg)); g.append(gs); lens.append(len(fr))
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
| 그리퍼 최대값 (state) | physical **≈0.63** (정지 0.506 / 0.8; 스모크 0.632) · attach 0.650 | 1.0 이면 손가락이 물체를 뚫고 닫힌 것 (§33). physical 의 **action** 최대 1.0 은 정상(명령 0.8) |
| `>0.95` 비율 | **0%** | 실물 2F-85 가 낼 수 없는 값 |
| 길이 이상치(>25 s) | **0** | 그리퍼 액션이 15 s 타임아웃 중 (§34) |

---

## 2. 변환 (LeRobot v3.0)

```bash
deps/.venv-ml/bin/python src/ur_bringup/scripts/raw_to_lerobot.py \
  --raw outputs/il_raw_red_left_100_phys --dry-run          # 먼저 에피소드/태스크 확인

deps/.venv-ml/bin/python src/ur_bringup/scripts/raw_to_lerobot.py \
  --raw outputs/il_raw_red_left_100_phys \
  --repo-id tony/ur16e_pick_place_red_left_100_phys \
  --root outputs/lerobot_ds_red_left_100_phys
```

- 카메라 대수는 `meta.json` 의 `video_keys` 를 따라가므로 **인자 불필요**. 1대면 feature 1개.
- `--max-idle-run N` 은 정지 구간을 N 프레임으로 제한하는 **후처리**. 상태머신이 최적화된
  지금은 필요 없다(정지 attach 8.6 % · physical 10.0 %, 바깥 띠 11.8 %). 옛 데이터를 살릴 때만 쓴다.
- `torchcodec` 로드 실패 트레이스백은 **무해**(pyav 폴백). 성공 판정은 `meta/info.json` 존재로.
  디코딩은 학습 병목이 아니다(§45.4 — 병목은 메인 프로세스 스레드 과다할당, 학습 명령에 `OMP_NUM_THREADS=8`).
- 코덱: 기본 **h264 crf23**(`--vcodec/--crf/--fast-decode`, lerobot `RGBEncoderConfig` 그대로 통과).
  AV1(lerobot 기본 `libsvtav1`) 과 같은 크기·1 dB 이내이고 어떤 PC 에서도 디코딩된다. 기존 AV1 데이터셋은
  `meta/info.json` 에 코덱이 박혀 있어 그대로 학습된다(재변환 불필요). 실측 [`HISTORY.md`](HISTORY.md) §45.3.

```bash
deps/.venv-ml/bin/python -c "
import json; i=json.load(open('outputs/lerobot_ds_red_left_100_phys/meta/info.json'))
print(i['total_episodes'],'ep', i['total_frames'],'frames')
[print(' ',k,v['dtype'],tuple(v['shape'])) for k,v in i['features'].items() if k.startswith(('observation','action'))]"
```

---

## 3. 학습 (ACT)

> **★ 학습 전에 Isaac 을 내린다.** Isaac + ROS 스택이 64 MiB `/dev/shm` 중 ~14 MiB 를
> Fast-DDS 로 점유해서 DataLoader 워커가 죽는다(§32.4). `--shm-size=8g` 로 만든 새 컨테이너는 용량 문제는 없지만
> GPU 를 나눠 쓰지 않도록 그래도 내린다. `/dev/shm` 크기는 `src/setup/check_env.sh` 가 알려준다.

```bash
bash src/ur_bringup/scripts/harness/shutdown_rollout.sh   # Isaac·런치·노드 전부(cuMotion 컨테이너 포함)·정책 서버 → 마지막 줄 CLEAN 확인(LEFT 면 rc 1)
rm -f /dev/shm/torch_*; df -h /dev/shm     # 64 MiB 컨테이너면 60 MiB 가까이 비어야 한다

source src/setup/ml_env.sh      # HF_HOME · OMP_NUM_THREADS=8 · UR_WS_TORCH_SHM_FIX=1 (SETUP.md §2-C) — 모든 ML 명령 앞에
deps/.venv-ml/bin/lerobot-train \
  --policy.type=act --policy.push_to_hub=false \
  --dataset.repo_id=tony/ur16e_pick_place_red_left_100_phys \
  --dataset.root=outputs/lerobot_ds_red_left_100_phys \
  --steps=60000 --batch_size=32 --num_workers=4 \
  --output_dir=outputs/act_red_left_100_phys
```

| 인자 | 이유 |
|---|---|
| `--num_workers=4` | **320×240 데이터셋(이 파이프라인) 규칙: 4, 워커가 `/dev/shm` 으로 죽으면 2 → 0** — `harness/train_100.sh` 가 이 사다리로 자동 재시도한다. 실측(64 MiB, Isaac 내림) 4 = 12.5, 2 = 7.9, 0 = 1.4 step/s, 8/6 은 사망. **480×640 데이터셋을 64 MiB `/dev/shm` 에서 학습하면 `0`**(배치 하나가 ~59 MiB, SETUP.md §2-C) |
| `--push_to_hub=false` | `[training]` extra 설치 후 hub 검증이 켜져서 없으면 즉사 |
| `--steps=60000` | 데이터량을 비교할 때는 **스텝을 고정**해야 한다 |

**실측 속도**: physical 2대 카메라 49,862 프레임(100 ep) → ≈ 15.8 step/s (60k 1 h 06 m, §54.5) · 94,574 프레임(190 ep) → ≈ 12.7 step/s (200k ≈ 4 h 23 m, §58.6). 용접 시대: 2대 43,222 프레임 → 14.7 step/s (68분) / 1대 20,996 프레임 → 25.6 step/s (39분)

### 3-1. 이어 학습 60k → 200k (100 ep 의 최고 19/20, `HISTORY.md` §54.6 — 현재 최종은 §3-2 의 190 ep)

60k 원본(롤아웃 하네스 기본)은 그대로 두고 복사본에서 이어 간다. 옵티마이저 상태·데이터 순서까지 이어지고, ACT 는
스케줄러가 없어(lr 고정) 스텝만 늘리면 된다. 140k 스텝 ≈ 2 h 38 m(15 step/s).

```bash
source src/setup/ml_env.sh
cp -a outputs/act_red_left_100_phys outputs/act_red_left_100_phys_long
deps/.venv-ml/bin/lerobot-train --resume=true \
  --config_path=outputs/act_red_left_100_phys_long/checkpoints/last/pretrained_model/train_config.json \
  --output_dir=outputs/act_red_left_100_phys_long \
  --steps=200000 --save_freq=20000
```

- `--resume` 은 `checkpoints/last`(심볼릭링크)의 `training_state` 를 읽고, `train_config.json` 의 `dataset.root` 는
  **절대경로**다 — 다른 경로·다른 PC 에서 이어 가면 그 값이 실제 데이터셋을 가리키는지 먼저 확인한다.
- 학습 없이 결과만 쓰려면 이 PC 의 `outputs/act_red_left_190_phys/checkpoints/200000/pretrained_model`(§3-2 현재 최종, ≈198 MB; 100 ep 판은 `act_red_left_100_phys_long/checkpoints/200000/pretrained_model`)만
  옮기면 된다(추론 전용). 옮길 목록은 [`CHECKLIST.md`](CHECKLIST.md) B-3(옛 PC 의 학습 산출물 옮기기).

> **loss 로 정책을 비교하지 말 것.** 데이터가 2배면 같은 스텝에서 epoch 이 절반이라 loss 가
> 높게 나오지만 성능은 더 좋았다(§38.3). 정지 프레임이 많으면 loss 는 낮은데 태스크를 못 한다(§32.5).
> **판정은 롤아웃이다.**

### 3-2. 범위 확장 190 ep — 현재 최종 (`HISTORY.md` §58)

기존 100 ep(±60 mm, 시드 91)는 그대로 두고 **x 바깥 띠(|dx| 60–100 mm, y ±60 mm)만 90 ep** 더 모아 합쳤다(새 띠의 30 % 는 큐브 yaw 를
접기 경계 ±10° 에서 뽑음). 하네스 덮어쓰기 변수로 돈다(기본값 = 원본 100 ep 실행). 경로는 **절대경로**로 준다 — 삭제 대상 검사가
`*/outputs/{il_raw,lerobot_ds,act}_*` 만 허용하고, 덮어쓰기를 주면서 `OUT`/`DS`/`CK` 를 빼면 원본을 지우지 않도록 rc 2.

```bash
cd /isaac-sim/volume/ur_ws && export ROS_DOMAIN_ID=42 GRASP_MODE=physical
H=src/ur_bringup/scripts/harness
# 수집 90 ep (§58.3: 90/90, 44,712 프레임, 정지 11.8 %)
OUT=$PWD/outputs/il_raw_red_left_band_phys SEED=301 TAG=cband \
RAND="--randomize-radius-x 0.10 --randomize-radius-y 0.06 --randomize-inner-x 0.06 --randomize-yaw-edge-frac 0.3 --randomize-yaw-edge-deg 10" \
  bash $H/collect_100.sh 90
# 합치기 = 상대 심볼릭 링크 190 개 (000000–099 → _100_phys, 000100–189 → _band_phys; 변환기는 이름 순서로 번호를 매긴다)
mkdir outputs/il_raw_red_left_190_phys && cd outputs/il_raw_red_left_190_phys
for i in $(seq 0 99); do e=$(printf episode_%06d $i); ln -s ../il_raw_red_left_100_phys/$e $e; done
for i in $(seq 0 89); do ln -s ../il_raw_red_left_band_phys/$(printf episode_%06d $i) $(printf episode_%06d $((100+i))); done
cd ../..
# 변환 (convert_100.sh 의 목적지는 OUT 이 아니라 DS) → 190 ep · 94,574 프레임 · 382 MB
RAW=$PWD/outputs/il_raw_red_left_190_phys DS=$PWD/outputs/lerobot_ds_red_left_190_phys \
REPO=tony/ur16e_pick_place_red_left_190_phys TAG=convert_190 bash $H/convert_100.sh
# 학습 전에 Isaac 을 내린다(§3 — collect_100.sh 는 Isaac 을 띄워 둔 채 끝난다) → 마지막 줄 CLEAN
bash $H/shutdown_rollout.sh
# 학습 200k (≈ 12.7 step/s, ≈ 4 h 23 m, loss 0.013, 67.7 epoch)
DS=$PWD/outputs/lerobot_ds_red_left_190_phys REPO=tony/ur16e_pick_place_red_left_190_phys \
CK=$PWD/outputs/act_red_left_190_phys STEPS=200000 SAVE_FREQ=20000 TAG=train_190 bash $H/train_100.sh
```

- ★ **중단된 학습을 `train_100.sh` 로 다시 돌리지 말 것** — 시작할 때 `rm -rf "$CK"` 한다. 이어 학습은
  `deps/.venv-ml/bin/lerobot-train --resume=true --config_path=$PWD/outputs/act_red_left_190_phys/checkpoints/last/pretrained_model/train_config.json`
  (실제 사용한 런처 `outputs/harness_logs/resume_train_190.sh` — 로컬 파일이 없으면 LeRobot 이 `--config_path` 를 Hub repo id 로 본다).
- ★ 실행 중인 하네스 `.sh` 를 고치지 말 것 — bash 가 바뀐 파일을 옛 바이트 위치에서 이어 읽는다(§58.2).
- 평가(§58.7, `outputs/harness_logs/eval_190.sh`, 10 회 × 70 s/시드): 기존 범위 시드 131·171 **19/20**, 넓은 범위
  (`RAND="--randomize-radius-x 0.10 --randomize-radius-y 0.06"`) 시드 211·251 **20/20** — 같은 배치에서 100 ep 200k 는 13/20(실패 7 회 전부
  |dx| > 60 mm 조준 실패). 140k 30/30, 100k 27/30. 성공 오차 4–6 mm. **yaw 추종은 아직 약하다**(기울기 −0.62…+0.05, 시연 −1.00).

### 3-B. 학습 (GR00T N1.7, VLA) — ✅ sim 검증 완료 (2026-09-13)

**§1·§2 (수집·변환)는 그대로다.** 같은 데이터셋을 쓰되, ACT 와 달리 **태스크를 필터하지 않는다** —
GR00T 는 `task` 문자열을 조건으로 받으므로 3종 혼합 데이터셋이 그대로 맞다(§30 의 ACT 제약과 반대).

이름은 `harness/groot_pipeline.sh` 의 physical 이름을 따른다: `il_raw_3task_v4_phys`(§1-3) →
`lerobot_ds_3task_v4_phys`(§2 와 같은 변환, `--repo-id tony/ur16e_pick_place_3task_v4_phys`) → `groot_3task_v4_phys_abs`.
**수집부터 롤아웃까지 한 번에는 `harness/groot_pipeline.sh 30`**(같은 이름으로 만든다 — 그 이름이 이미 있으면 거부하므로 §1-3 손 수집과 둘 중 하나만; 실제 실행은 용접 시대 v3 로만 했다 — §46, physical `v4_phys` 는 아직 안 돌렸다).
용접 시대 이름(`lerobot_ds_240_v2`·`3task_v3`)은 attach 데이터라 physical 수집과 섞지 말 것.

설치와 게이트는 [`SETUP.md`](SETUP.md) §2-C-2 를 먼저 볼 것 (`lerobot[groot]`, `HF_HOME`,
**gated `nvidia/Cosmos-Reason2-2B` 라이선스 동의 + 토큰을 `deps/hf_cache/token` 에**).

```bash
source src/setup/ml_env.sh      # HF_HOME · HF_HUB_OFFLINE=1 · OMP_NUM_THREADS=8 · WANDB_DISABLED (SETUP.md §2-C)
BASE=$(deps/.venv-ml/bin/python -c \
  "from huggingface_hub import snapshot_download; print(snapshot_download('nvidia/GR00T-N1.7-3B'))")

# OMP_NUM_THREADS=8 은 위 ml_env.sh 가 준다 (§45.4)
deps/.venv-ml/bin/lerobot-train \
  --policy.type=groot --policy.push_to_hub=false --wandb.enable=false \
  --policy.base_model_path="$BASE" \
  --policy.model_params_fp32=false \
  --policy.max_steps=10000 \
  --dataset.repo_id=tony/ur16e_pick_place_3task_v4_phys \
  --dataset.root=outputs/lerobot_ds_3task_v4_phys \
  --steps=10000 --batch_size=32 --num_workers=2 --save_freq=5000 \
  --output_dir=outputs/groot_3task_v4_phys_abs
```

| 인자 | 이유 |
|---|---|
| `--policy.base_model_path="$BASE"` | **repo id 금지** — `is_dir()` 판정에 걸려 체크포인트 사이드카가 경고 없이 무시되고 전처리가 lerobot 기본값으로 바뀐다. 학습은 그냥 돌아가서 알아챌 수 없다 |
| `--policy.model_params_fp32=false` | 기본 `true` 는 정적 **29.8/31.8 GiB** 로 **배치 크기와 무관하게** 32 GB 초과. `false` + batch 32 = 27.3 GB 실측 |
| `--steps=10000` + **`--policy.max_steps=10000`** | 10k = NVIDIA 레퍼런스 길이, **3 h 45 m**(기본 100k 는 39 h). `max_steps` 는 "unused" 주석과 달리 **warmup 스텝**을 결정한다 — 둘을 같이 바꾼다(§43.4) |
| **`OMP_NUM_THREADS=8`** (환경변수) | `data_s` 는 DataLoader 가 아니라 **메인 프로세스 전처리기** 시간이고, torch 기본 20 스레드에서 작은 텐서 연산이 과다할당으로 7배 느려진다. 8 스레드로 `data_s` 0.85→0.125 s, **처리량 2배**(23→51 샘플/s), 10k 스텝 ≈ 1 h 45 m 예상. 1/4/8/12 스레드 표는 §45.4 |
| `--batch_size=32 --num_workers=2` | 처리량은 배치가 좌우(8→32 에서 samples/s +38%, VRAM +2 GB). 워커는 2 면 충분(워커 대기 0.000 s 실측 — 병목은 메인 프로세스 전처리), **8 은 64 MiB `/dev/shm` 사망** |
| **상대 액션을 쓰지 않는다** | N1.7 은 상대로 사전학습됐고 학습도 되지만 **`async_inference` 서버가 서비스 못 한다**(§43.7). 절대 액션이 정본 |
| `--save_freq=5000` | 기본 20,000 > 10,000 이라 중간 체크포인트가 없다 |
| `--wandb.enable=false` | `GrootConfig.report_to` 기본값이 `wandb` |

**실측** (`HISTORY.md` §44.1, 용접 시대 `240_v2` 데이터셋): loss 0.995 → 0.014, 10.4 에폭, peak VRAM 27.5 GB, shm 14.5 MiB, `rc=0`.
전처리 5개(albumentations·dropout 0.2·percentile·256/0.95)가 체크포인트 고유값으로 기록됨 — repo id 를
줬다면 전부 lerobot 기본값이었다. 카메라 2대는 전처리 출력 `image_grid_thw (2,3)` 로 확인.

**롤아웃(§4)은 네 인자가 바뀐다**: `--policy_type=groot`, `--pretrained_name_or_path=<체크포인트>`,
`--actions_per_chunk=40`(ACT 는 50), **`--policy_device=cuda`**(기본 `cpu` 에서 bf16 이 조용히 죽는다 —
§43.6). 3태스크 판정은 `judge_rollout.py`(red→left 하드코딩)가 아니라 **`judge_task.py`** 로.
추론 지연 **80.8 ms**/청크(예산 1,333 ms). 롤아웃 7 ep/태스크 **3/8** → 30 ep/태스크 **14/29**(§46), `WRONG_OBJECT` 0 —
실패는 사전파지 높이에서 정지(§46.3). 설계·합격 기준·실측은
[`ur_bringup/docs/plan_groot_n17.md`](ur_bringup/docs/plan_groot_n17.md) §4·§8.

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
  --camera-res 320x240 --grasp-mode physical --with-camera --with-static-cam        # GUI 로 보려면 --headless 제거

ros2 launch ur_bringup ur16e_2f85_d405.launch.py use_sim:=true
ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py use_sim:=true ur_only:=false
```

### 4-2. 스트리밍 컨트롤러 + 초기 상태

```bash
ros2 launch ur_bringup policy_inference.launch.py use_sim:=true   # forward_position_controller 스폰(inactive) ★ 생략 금지
ros2 control list_controllers | grep forward_position_controller  # 없으면 아래 switch 가 실패한다

# ★ 학습과 같은 초기 조건을 만든다. 안 하면 정책이 얼어붙는다(§31.1)
python3 src/ur_bringup/isaac/common/switch_control_mode.py trajectory
ros2 service call /scene/reset_episode std_srvs/srv/Trigger
python3 src/ur_bringup/isaac/common/reset_pose.py ready        # error_code: 0 확인. ★ ready = 2026-09-17 실측 랑데부(§49.8); 2026-09-16 이전 데이터셋의 정책은 ready_v1, 2026-09-16 sim 스모크는 ready_v2
python3 src/ur_bringup/isaac/common/switch_control_mode.py streaming
```

> 시연 100개의 시작 자세 편차는 **0.0002 rad** — 사실상 한 점이다. READY 가 아닌 곳에서
> 시작하면 정책이 본 적 없는 상태이고, 명령은 나가는데 팔이 안 움직인다.

> **★ 제어 스택만 띄우고 `policy_inference.launch.py` 를 빠뜨리면** `switch_control_mode.py streaming` 이
> `controller 'forward_position_controller' is not loaded` 로 실패한다. 이 출력을 `/dev/null` 로 버리면
> 정책은 청크를 내고 서버는 전달했다고 하는데 **팔이 0° 도 안 움직이고**, 그대로 FAIL 로 채점된다(§44.2).
> 전환 후 `ros2 control list_controllers` 에서 `forward_position_controller ... active` 를 **눈으로 확인**할 것
> (`grep active` 는 `inactive` 에도 걸린다).

### 4-3. 정책 실행 (upstream LeRobot)

> `setup.sh ml` 이후 ACT 롤아웃은 **아무것도 내려받지 않는다** — 백본 resnet18(`IMAGENET1K_V1`)은 `deps/torch_home` 에 미리 있고,
> venv 의 모든 프로세스가 `ml_env.sh` 없이도 `TORCH_HOME`/`HF_HOME`/`PIP_CACHE_DIR` 기본값을 `deps/` 로 쓴다(SETUP.md §2-C).
> GR00T 는 `ml_env.sh` 의 `HF_HUB_OFFLINE=1` 로 오프라인 — 모델 파일 자체는 사용자의 HF 게이트 단계.

```bash
# 터미널 A — 정책 서버
deps/.venv-ml/bin/python -m lerobot.async_inference.policy_server --host=127.0.0.1 --port=8080

# 터미널 B — 로봇 클라이언트 (우리 코드는 --robot.type 플러그인 하나뿐)
deps/.venv-ml/bin/python -m lerobot.async_inference.robot_client \
  --robot.type=ur16e_ros --robot.id=sim \
  --policy_type=act \
  --pretrained_name_or_path=outputs/act_red_left_100_phys/checkpoints/last/pretrained_model \
  --actions_per_chunk=50 --task="put the red block on the left marker" \
  --server_address=127.0.0.1:8080
```

§3-2 의 190 ep 200k 체크포인트(현재 최종)면 `--pretrained_name_or_path=outputs/act_red_left_190_phys/checkpoints/200000/pretrained_model`(§3-1 의 100 ep 200k 는 `outputs/act_red_left_100_phys_long/checkpoints/200000/pretrained_model`).

**카메라 1대로 학습한 정책이면** 어댑터에도 알려야 한다(안 그러면 `/static_cam` 을 기다리다 실패):

```bash
  --robot.cameras_ros='{"wrist": "/camera/color/image_raw"}'
```

> `--task` 는 ACT 가 읽지 않는다(§30). 클라이언트가 요구해서 넣을 뿐이고, 정책이 VLA 가 되면
> 그때는 실제 입력이 된다 — **명령줄은 그대로**.

**GR00T 면** 위 클라이언트 명령에서 네 곳만 바뀐다 (`--robot.cameras_ros` 는 기본값이 이미 2대라 **빼야** 한다):

```bash
  --policy_type=groot \
  --pretrained_name_or_path=outputs/groot_3task_v4_phys_abs/checkpoints/last/pretrained_model \
  --policy_device=cuda \
  --actions_per_chunk=40 --task="put the blue block on the left marker"
  # --policy_device=cuda: ★ 기본 cpu 에서 bf16 이 조용히 죽는다 (§43.6)
```

서버는 `source src/setup/ml_env.sh` 한 쉘에서 띄운다(토크나이저를 `HF_HOME` 캐시에서 오프라인으로 연다 — 토큰 불필요). 3태스크
채점은 `ur_bringup/scripts/harness/judge_task.py --object blue --place left` — `WRONG_OBJECT`/`WRONG_PLACE`/`FAIL` 을
구분한다. 자동화 하네스: `ur_bringup/scripts/harness/groot_v8.sh <태스크당 N> 90 <체크포인트>`(`RADIUS`/`SEED` 는 수집과 같게),
수집부터 롤아웃까지 한 번에: `harness/groot_pipeline.sh 30`(§46.4).

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

### 4-5. 롤아웃 영상 (선택)

```bash
RECORD_DIR=$PWD/outputs/rollout_videos/act_red_left_100_phys \
  src/ur_bringup/scripts/harness/rollout_gui_then_n.sh 10 70
# 현재 최종(§3-2, 190 ep 200k): 세 번째 인자 = 체크포인트 (기본값은 60k 원본). SEED=(기본 131), RAND=(Isaac 무작위 인자 추가)
#   src/ur_bringup/scripts/harness/rollout_gui_then_n.sh 10 70 $PWD/outputs/act_red_left_190_phys/checkpoints/200000/pretrained_model
# 넓은 범위 평가(§58.7, x ±100 mm): 앞에 SEED=211 RAND="--randomize-radius-x 0.10 --randomize-radius-y 0.06"
# 100 ep 200k(§3-1): .../outputs/act_red_left_100_phys_long/checkpoints/200000/pretrained_model
```
시행마다 외부·손목 카메라를 나란히 담은 H.264 mp4 를 남기고 판정 결과를 파일명에 붙인다(`trial_03_SUCCESS.mp4`).
프레임은 Isaac `/clock` 기준이며 3배속으로 재생된다. 녹화기는 토픽만 구독하므로 결과에 영향이 없다(HISTORY.md §52).

---

## 5. 실측 기준값

| | 값 | 출처 |
|---|---|---|
| 에피소드 길이 | 중앙값 **14.3 s** (12.4–17.0, 평균 14.4) | `il_raw_red_left_100` 을 1-3 확인 스크립트로 2026-10-20 재측정(§54.3). 예전 표의 "사이클 11 s" 는 출처 절에 없던 값. 대기 제거 전 41.3 s(§32) |
| 정지 프레임 | **8.6%** | 같은 재측정(관절·그리퍼 변화 ≤ 1e-4). v2 는 65.8%(§31) |
| `grip_closed` | **0.52** | §35 접촉 시작점. 힘 평형점 0.599 가 아니다 |
| `grasp_z_offset` | **0.015** | §34 더 깊으면 핑거가 테이블과 간섭 |
| 에피소드 수 | **100** | §36/§38 50 은 랜덤 범위 가장자리에서 정렬 실패 |
| 롤아웃 | **9/10**, 평균 7.6 mm | §38 |
| (physical) 수집·롤아웃 | 100/100, 에피소드 16.6 s, 정지 10.0 % · 롤아웃 **8/10**(시드 131)·**9/10**(171), 평균 12.0 mm · 200k 이어 학습 **9/10·10/10**, 6.2 mm | §54.5·§54.6 |
| (physical, 현재 최종) 190 ep 200k | 100 ep + 바깥 띠 90 ep(합 94,574 프레임; 띠 90/90, 중앙값 16.6 s, 정지 11.8 %) · 기존 범위 **9/10·10/10**(시드 131·171), 4.0/6.0 mm · 넓은 범위(x ±100 mm, 시드 211·251) **20/20**, 4.7/6.0 mm (100 ep 200k 는 13/20) · yaw 추종 기울기 −0.62…+0.05(시연 −1.00) | §58.3·§58.7 |

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
