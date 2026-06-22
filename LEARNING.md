# LEARNING.md — IL(pick&place) + RL(insertion) 설계문서 (Isaac Lab 기반)

> 이 문서는 **설계/계획**이다(코드 아직 없음). UR16e 워크스페이스(`src/`, ROS2 Jazzy 제어 스택)에
> Imitation Learning 과 RL 을 **Isaac Lab** 으로 얹는 방향을 정리한다.
> 구성/실행 현황은 [`README.md`](README.md), 실물 절차는 [`HARDWARE.md`](HARDWARE.md), 이력은 [`HISTORY.md`](HISTORY.md).
>
> 조사 근거(파일 경로/클래스명)는 본문에 인라인으로 박아 두었다. Isaac Lab 위치 = `/isaac-sim/IsaacLab`
> (README 기준 v2.3.2), Isaac Sim 5.1.0, GPU = RTX 5090(32GB).

---

## 0. 목표 / 범위

| 학습 | 태스크 | 데이터/방식 | 산출물 |
|---|---|---|---|
| **Imitation Learning** | **pick & place** | **teleop** 시연 → (Mimic 증강) → BC | sim 롤아웃 성공 정책 |
| **Reinforcement Learning** | **insertion**(peg/너트 등 접촉 많은 조립) | sim 병렬 학습(PPO) + domain randomization | sim2real 견딜 정책 |
| **공통** | 배포 | 학습된 정책을 **ROS2 노드**로 → 기존 제어 스택 | sim·real 공용 추론 |

로봇: **UR16e + Robotiq 2F-85**(우리 베이크 USD), 카메라 D405(eye-in-hand) + 정적 카메라.

---

## 1. ★ 핵심 아키텍처 결정 — "학습은 오프라인(Isaac Lab), 배포만 ROS2"

"가능하면 ROS2 기반"의 정확한 답: **배포(inference)는 ROS2로, 학습(training)은 ROS2를 거치지 않는다.** 한계가 아니라 정석이다.

```
[ 학습 단계 ]  Isaac Lab (GPU 병렬 env, 직접 PhysX) ── ROS2 없음
   IL: teleop 단일 env → HDF5 시연 → Mimic 증강 → robomimic BC 학습
   RL: 수천 env 병렬 → PPO(rl_games/rsl_rl) → 정책 checkpoint
        │  export (policy.pt / .onnx)
        ▼
[ 배포 단계 ]  ROS2 정책 러너 노드 ── 기존 sim↔real 인터페이스 그대로
   /joint_states(+카메라) → policy(obs)→action → ros2_control → Isaac/실물 UR16e
```

- RL 은 표본 효율을 위해 수천 개 env 동시 구동이 필수 → ROS2 메시지 패싱으로는 throughput 불가.
  (같은 머신 `oht_bolting` 벤치: 64env 기준 helical 8,900 step·env/s — ROS2로 재현 불가능한 수치.)
- IL teleop 기록은 Isaac Lab recorder 와 env 가 밀결합(HDF5/episode) → Lab 루프에서 수집.
- **배포 단계는 우리가 만든 sim↔real ROS2 공용 인터페이스를 정확히 재사용**한다. Isaac Lab 도
  UR ROS 추론 예제를 동봉(§5) → 같은 패턴.

➡ 결론: 학습 코드는 기존 `ur_bringup`(ROS2 제어)을 **건드리지 않는 새 영역**에 두고(세트 분리 원칙 [[set-separation-rule]]),
배포 노드만 ROS2 패키지로 추가한다.

---

## 2. ★ 가장 중요한 통합 발견 — 컨트롤러 인터페이스 불일치

학습 정책은 **매 스텝 액션**(관절 위치 델타, ~30–60Hz)을 낸다. 그런데 현재 우리 실행 컨트롤러는
`scaled_joint_trajectory_controller`(**궤적 기반**, follow_joint_trajectory = 여러 waypoint+타이밍).
정책의 단일-스텝 델타를 궤적 컨트롤러로 흘리면 jerky/overshoot 발생.

**해결**: 배포용으로 ros2_control 에 **스트리밍 위치 컨트롤러**를 추가한다.
- 후보: `position_controllers/JointGroupPositionController`, 또는 UR 드라이버의
  `forward_position_controller` / `passthrough_trajectory_controller`(UR 권장 경로 — **확인 필요**).
- cuMotion/MoveIt(궤적) 와 정책(스트리밍)은 **컨트롤러를 전환**해 공존(둘 다 같은 HW 인터페이스).
- Isaac Lab gear_assembly UR 배포가 쓰는 액션 = `RelativeJointPositionActionCfg`, scale 0.025 rad/step,
  base 프레임 기준(아래 함정). 우리도 동일 규약을 따른다.

이 한 가지가 배포 단계의 핵심 작업이다. IL·RL 모두 같은 배포 경로를 공유한다.

---

## 3. 재사용할 기존 자산 (조사 결과)

### 3-A. 같은 머신의 `oht_bolting` — RL insertion 의 최고 참고자료 (정독·평가 완료)
`/isaac-sim/standalone_examples_my/oht_bolting/` : **UR16e 볼팅/언볼팅(OHT 휠) RL** 프로젝트. Isaac Lab v2.3.2,
**DirectRLEnv** + GridCloner, **rl_games(주)+rsl_rl(부)**. 자세 평가는 4개 영역 정독으로 확정(아래).

**성숙도 (정직한 평가)**
| 영역 | 상태 |
|---|---|
| **kinematic insertion** (joint-pos PD) | ✅ **solid/production** — sim 휠 삽입 100% 성공 보고(Phase 7), rock-steady |
| 보상/커리큘럼 | ⚠️ **prototype** — regime 4~17 반복 튜닝(만능 공식 없음, 가설→테스트→수정 사다리) |
| **접촉-resolved 물리 + F/T + 실물 배포** | 🔴 **미완(research)** — Phase 9(접촉)·Phase 5(실물) **시작 안 함**. F/T 채널 존재하나 wrench≈0 |

**바로 차용할 것 (gold)**
1. **`rl/ur16e_cfg.py`**: UR16e ArticulationCfg + EE-factory(`make_ur16e_with_ee_cfg(ee)`, `EndEffectorCfg` 디스크립터로
   drive/mimic/passive 분리). sim2real 액추에이터 튜닝(servoj 매칭: shoulder K=8000/D=400, elbow 5000/200,
   wrist 2000/40; effort=datasheet 330/150/28 N·m). → 우리 `UR16E_2F85_CFG` 의 기반.
2. **최적 템플릿 = `rl/oht_wheel_insert_env.py`(70KB)**: joint-position-delta(scale 0.025) + 종합 보상(distance,
   alignment, progress, cumulative-gated, milestone, seat-ramp, overtravel, radial attractor) + 3-조건 게이트
   (radial<5mm ∧ align<15° ∧ gripper closed). 우리 peg-in-hole 로 **fork → 휠 FSM 제거 → 그리퍼 kin 만 교체**(~80% 복붙).
3. **rl_games/rsl_rl 파이프라인**: `train_rl_games.py`/`play_rl_games.py`, env-var 커리큘럼(`OHT9_*`), `logs/rl_games/...`,
   `.pth`/`.pt`. (★ ONNX export 는 없음 → 배포용으로 우리가 추가 필요.)

**★ 핵심 교훈 (우리 위험 크게 절감)**
- **컨트롤러**: Cartesian impedance 는 UR16e 에서 무동작 시 7.4mm/step jitter → **joint-position PD(K=800/D=40)로 전환**
  해 안정·게이트 99.9%. 이는 NVIDIA UR 배포 레시피(정책=joint delta, impedance 는 *배포 시* 컨트롤러)와 일치 →
  우리 §2/§7 결정을 확증.
- **termination-avoidance trap**: per-step 양수 보상 + goal 이 episode 종료면 정책이 목표 직전에 "주차". → **success(지표)와
  termination(soft-limit/timeout) 분리**. plateau 시 `_get_dones` 부터 의심.
- **kinematic vs 접촉**: kinematic 이 SDF 대비 ~100배 throughput → 학습은 kinematic, 검증은 SDF. 단 **kinematic→SDF
  zero-shot 0%**(재학습 필요). 보상은 cumulative 게이팅 필수, 목표 과다 시 엉뚱한 것 최적화.
- 기타 함정: cloned env 에서 instanceable 메시 사라짐(Flatten 후 instanceable 해제), joint-layout assert, grasp-frame
  양쪽 패드 평균.

**우리와의 차이 / 주의**
- **2F-140 사용**(M20 너트용; NGC 에 2F-85 없음). **우리는 자체 2F-85 USD 보유가 강점** → §4 참조.
- **★ insertion 이 kinematic**(대상물이 그리퍼에 glue, 실접촉/중력 off). 즉 *진짜 접촉 많은 insertion*(peg 가 구멍 벽과
  접촉·힘 발생)은 거기서도 **미해결**(Phase 9). 우리가 "RL for insertion"의 본질(접촉)을 원하면 그 어려운 부분은 새로 풀어야 함
  → §10 결정사항.

### 3-B. Isaac Lab 빌트인 템플릿
- **로봇**: `isaaclab_assets/.../universal_robots.py` → `UR10e_ROBOTIQ_2F_85_CFG`
  (variant `{"Gripper":"Robotiq_2f_85"}`; 그리퍼 액추에이터 = `gripper_drive`[finger_joint]·
  `gripper_finger`[.*_inner_finger_joint]·`gripper_passive`[knuckles]). Franka 도 동일 2F-85 패턴(`franka.py`).
- **IL 태스크**: `manager_based/manipulation/{stack,pick_place,lift,place,reach}`.
  - `stack/config/ur10_gripper/` 에 **UR + 그리퍼 stack 태스크**가 이미 등록됨(IK-rel 변형) → 우리 시작점.
  - 액션: `DifferentialInverseKinematicsActionCfg`(teleop용 EE 델타) + `BinaryJointPositionActionCfg`(그리퍼).
- **Teleop+기록**: `scripts/environments/teleoperation/teleop_se3_agent.py`(keyboard/spacemouse/gamepad/handtracking,
  7D=Δpose6+gripper), `scripts/tools/record_demos.py`(HDF5, 성공 에피소드만 export).
- **IL 학습**: `scripts/imitation_learning/isaaclab_mimic/`(annotate→generate, MimicGen식 증강) +
  `robomimic/`(train/play/robust_eval, BC·BC-RNN). Mimic 은 태스크가 `ManagerBasedRLMimicEnv` 상속 +
  subtask 신호 구현 필요.
- **RL insertion 빌트인**: `direct/factory`(Franka, task-space, peg/gear/nut, solver iter=192),
  `direct/forge`(접촉+force+dynamics randomization). UR-네이티브 대안 =
  `manager_based/manipulation/deploy/gear_assembly/config/ur_10e/`(조인트-스페이스, 2F-85/2F-140).
- **RL 학습기**: `scripts/reinforcement_learning/{rsl_rl,rl_games,skrl,sb3,ray}/{train,play}.py`.
- **ROS 배포 예제**: `manipulation/deploy/gear_assembly/config/ur_10e/ros_inference_env_cfg.py`(+`deploy/mdp/`) —
  **UR10e 를 ROS 로 추론**하는 실제 템플릿(obs_order/action scale/joint names 노출, vision pose 입력).

---

## 4. 로봇 자산 포팅 — `UR16E_2F85_CFG`

우리 USD: `ur_bringup/isaac/assets/ur16e_with_2f85.usd`(IL/RL 학습용 권장; 카메라/커플링 메시는 시각용),
`ur16e_2f85_d405.usd`(비전 태스크용). 빌드 스크립트 `isaac/common/build_ur16e_2f85.py` 가 이미
instanceable 제거·articulation root 정리·finger_joint drive 튜닝(K=20,D=1)을 반영.

**방침**: `oht_bolting/rl/ur16e_cfg.py` 의 패턴을 차용하되 USD 만 우리 2F-85 로 교체.
- arm 액추에이터: oht_bolting 의 sim2real 튜닝값(8000/400, 5000/200, 2000/40, effort 330/150/28) 채택.
- gripper: `finger_joint`(drive) + `.*_inner_finger_joint`(mimic) + knuckles(passive) — 우리 USD 의 실제
  관절명 확인 후 정규식 확정(검증 명령은 §8).
- `articulation_root_prim_path` : 우리 USD 는 `/UR16e/root_joint`. Isaac Lab 은 `{ENV_REGEX_NS}/Robot` 아래
  스폰하므로 **상대 경로/auto-search** 동작 확인 필요(함정).
- 출력: `src/ur_isaaclab_tasks/.../robots/ur16e.py` 에 `UR16E_2F85_CFG`.

---

## 5. IL 파이프라인 (pick & place, teleop)

1. **태스크 정의**: `stack/config/ur10_gripper` 를 베이스로 UR16e+2F-85 stack(또는 pick&place) env.
   - `scene.robot = UR16E_2F85_CFG`, `ee_frame`(tool0/gripper_frame), `cube`/`bin` 스폰.
   - 액션: `DifferentialInverseKinematicsActionCfg`(arm, dls IK) + `BinaryJointPositionActionCfg`(gripper).
   - 관측: state(joint, eef pose, object rel) + (선택) **D405/정적카메라 RGB**(`concatenate_terms=False`).
   - 성공 termination(필수, 기록 라벨용) + Mimic 용 subtask 신호.
   - gym 등록: `Isaac-PickPlace-UR16e-2F85-IK-Rel-v0`.
2. **Teleop 수집**: `record_demos.py --task ... --teleop_device {keyboard|spacemouse} --num_demos 10~20`
   → `datasets/ur16e_pickplace_raw.hdf5`(성공분만).
3. **Mimic 증강(권장)**: `annotate_demos.py --auto` → `generate_dataset.py --generation_num_trials 300~500`
   → 수백 합성 시연(소수 인간 시연으로 성공률 ↑).
4. **BC 학습**: `robomimic/train.py --algo bc_rnn --dataset <generated>` → checkpoint.
5. **롤아웃**: `robomimic/play.py --checkpoint ...` 로 sim 성공률 측정.

권장: **Mimic + robomimic 조합**(인간 시연 10~20개 → 증강 → BC-RNN). 비전 정책은 2단계로(우선 state 정책 검증).

---

## 6. RL 파이프라인 (insertion)

**방침**: **`oht_bolting/rl/oht_wheel_insert_env.py` 를 fork** 해 UR16e+2F-85 peg-in-hole 로 특화(평가 결과 §3-A).
일반 factory(Franka)보다 UR16e-네이티브 + Phase 7 검증된 보상/제어를 그대로 물려받는 게 유리.
1. **에셋**: peg/hole. Isaac NGC `Factory/factory_{peg,hole}_8mm.usd` 또는 우리 스케일 cylinder. 접촉이면 SDF.
2. **환경(fork)**: wheel env 에서 휠 FSM(grasp state machine·release·retract) 제거, peg/hole 로 rename, 그리퍼 kin 만 교체.
   - 액션: **joint-position-delta**(scale 0.025) + 그리퍼 — **배포 컨트롤러와 동일 규약**(§2, oht_bolting 확정).
   - 관측: joint pos/vel + 대상물 상대 pose/axis/extent/align/radial + 게이트 플래그(+노이즈) — **privileged 금지**.
   - 보상: §3-A 의 검증된 12항 그대로 시작(distance+align+progress+cumulative-gated+milestone+seat+overtravel+radial+success).
   - **termination 과 success 분리**(trap 회피), 제어 dt=1/120.
   - domain randomization: 대상물 pose ±10mm/±5°, actuator gain/friction(sim2real).
3. **★ kinematic vs 접촉 결정**(§10): (a) oht_bolting 식 **kinematic**(빠른 PoC, 실접촉 아님) 또는
   (b) **접촉-resolved**(factory/forge SDF; 진짜 insertion, ~100배 느림·미해결 난제). 권장: (a)로 파이프라인 세우고
   (b)로 단계 상승.
4. **학습**: `train_rl_games.py`(rl_games, PPO+LSTM) 또는 rsl_rl, env 128~4096, env-var 커리큘럼(`OHT9_*` 스타일).
5. **플레이/export**: `play_rl_games.py` → checkpoint. **ONNX/JIT export 추가**(oht_bolting 엔 없음) → ROS 배포용.

---

## 7. ROS2 배포 (IL·RL 공통)

> **확정(oht_bolting+NVIDIA 레시피)**: 정책은 **joint-position delta** 를 내고, 실물 측은 **impedance/스트리밍 위치
> 컨트롤러**가 받는다(학습엔 Cartesian impedance 안 씀 — sim 불안정). 우리 §2 결정과 정확히 일치.

1. **스트리밍 컨트롤러 추가**(§2): ros2_control 에 위치 스트리밍 컨트롤러(JointGroupPositionController 또는
   UR forward_position_controller/passthrough). cuMotion/MoveIt(궤적) 와는 컨트롤러 전환으로 공존.
2. **정책 러너 노드**(`ur_policy_deploy` ROS2 패키지):
   - 구독: `/joint_states`(+ 비전 pose 토픽), 정책 forward, **action*scale(0.025) 후 target=current+Δ** 발행.
   - 정책 입력 = export 한 ONNX/JIT(§6-5). 주파수: 학습과 동일(예 60–120Hz). obs **순서/스케일/관절명**은
     학습 env 의 `ros_inference_env_cfg` 메타와 일치(하드코딩 금지).
3. **함정(Isaac Lab UR 배포 문서 기준)**:
   - **base vs base_link**: UR 정책은 `base`(base_link 에서 Z 180° 회전) 기준 학습 → 프레임 정합 필수.
   - obs 순서/액션 스케일 하드코딩 금지(메타에서 읽기), 제어 주파수 일치, 그리퍼는 별도 동기.
4. **sim↔real 동일**: sim 은 topic_based/Isaac, real 은 ur_robot_driver — 정책 러너·컨트롤러는 동일.

---

## 8. 제안 디렉토리 구조

```
src/
├── ur_bringup/                      # (기존) ROS2 제어/MoveIt/cuMotion/nvblox — 건드리지 않음
├── ur_isaaclab_tasks/              # (신규) Isaac Lab 학습 — ROS 아님, /isaac-sim/python.sh 로 실행
│   ├── robots/ur16e.py             #   UR16E_2F85_CFG (oht_bolting 패턴 + 우리 USD)
│   ├── pick_place/                 #   IL: env cfg + mimic env + mdp(obs/term/subtask) + agents(robomimic json)
│   ├── insertion/                  #   RL: env cfg + mdp(reward/obs) + agents(ppo cfg)
│   ├── deploy/                     #   *_ros_inference_env_cfg.py (obs순서/스케일 메타)
│   └── README.md                   #   실행 명령 모음(record/annotate/generate/train/play)
├── ur_policy_deploy/               # (신규) ROS2 패키지(ament_python): 정책 러너 노드 + 스트리밍 컨트롤러 yaml + launch
├── datasets/   (gitignore)         # teleop/합성 HDF5
└── policies/   (gitignore 또는 포인터)  # 학습 checkpoint/onnx
```
- `ur_isaaclab_tasks` 는 Isaac python 에 `pip install -e` 또는 PYTHONPATH 로 등록(gym.register).
- `ur_policy_deploy` 만 colcon 빌드(ROS2). 세트 분리·sim↔real 공용 인터페이스 유지.

---

## 9. 단계별 로드맵 (제안)

| 단계 | 내용 | 산출/검증 | 위험 |
|---|---|---|---|
| **L0 자산 포팅** | `UR16E_2F85_CFG` + 빈 env reset/step/관절 구동 | sim 에서 팔+그리퍼 구동·관절명 확인 | 낮음(템플릿 있음) |
| **L1 IL 태스크** | UR16e pick&place env + teleop 1회 동작 | teleop 으로 직접 집기 성공 | 중(IK/EE offset 튜닝) |
| **L2 IL 데이터+학습** | record→(mimic)→robomimic BC | sim 롤아웃 성공률 ≥ 목표 | 중(시연 품질/subtask) |
| **L3 RL insertion** | oht_bolting 패턴 적응 + PPO | sim 삽입 성공률 곡선 | 높음(접촉 물리/보상) |
| **L4 ROS2 배포** | 스트리밍 컨트롤러 + 정책 러너 노드 | sim 에서 정책으로 plan-free 실행 | 중(컨트롤러/프레임) |
| **L5 실물** | HW 연결 후 동일 노드로 추론 | (HW 준비 시) | sim2real |

먼저 **L0** 권장(IL·RL 공통 토대, 위험 최저).

---

## 10. 열린 결정사항

1. **★ RL insertion 물리 수준**: (a) **kinematic**(oht_bolting 식, 빠른 PoC, 대상물 glue·실접촉 아님) vs
   (b) **접촉-resolved**(SDF/forge, 진짜 insertion, ~100배 느림·sim2real 난제, oht_bolting 도 미해결).
   → (a)로 시작해 파이프라인·배포까지 세운 뒤 (b)로 상승 권장. **목표가 "접촉 제어 학습"이면 (b) 필수**.
2. **IL 베이스 태스크**: `stack`(성숙·UR 등록됨) vs `pick_place`(단일 물체, pinocchio 의존). → stack 권장.
3. **RL insertion 대상물**: peg-in-hole(단순) vs 너트/볼트(oht_bolting 자산 재활용). → peg 로 시작 후 확장.
4. **Teleop 장치**: keyboard(즉시) vs spacemouse(품질↑, 장치 필요). → 보유 장치 확인 필요.
5. **배포 컨트롤러**: JointGroupPositionController vs UR `forward_position_controller`/passthrough. → 실측 검증.
6. **2F-85 통합 방식**: (A) `robotiq_2f85` EndEffectorCfg 작성 + `build_arm_with_ee.py` 베이크(EE 교체 유연) vs
   (B) 우리 기존 `ur16e_with_2f85.usd` 에 ArticulationCfg 직접(즉시). → A 권장. **확인 필요**: 우리 2F-85 USD 의
   관절명 + USD-네이티브 4-bar 링키지 유무(있으면 `mimic_mirror_drive=False`, 없으면 True+drive PD).
7. **`ur_isaaclab_tasks` 형태**: 독립 Python 패키지 vs Isaac Lab 외부 태스크 확장. → 독립 패키지 권장.
8. **oht_bolting 코드 재사용 범위**: 결정됨 → **패턴 참고 후 우리 트리에 재구현(2F-85 기준)** (다른 프로젝트와 결합/2F-140
   의존 회피). 핵심 파일(`ur16e_cfg.py`·`oht_wheel_insert_env.py`·rl_games cfg)을 vendoring 후 적응.
