# LEARNING.md — **RL(insertion)** 설계문서 (Isaac Lab 기반)

> ## 📌 범위 (2026-09-06 정리) — 읽기 전에
>
> **이 문서는 이제 RL(insertion) 전용이다.** 원래 IL + RL 을 함께 다뤘으나 IL 절반은 분리됐다.
>
> | 부분 | 상태 |
> |---|---|
> | **RL (insertion)** — §3-A, §4, §6, §7, §8~§10 | ✅ **유효.** 이 워크스페이스에서 추후 진행 가능 |
> | **IL (pick&place)** — 구 §5, §1 의 IL 경로, Mimic → robomimic BC | ❌ **제거됨** → [`ur_bringup/docs/plan_il_vla.md`](ur_bringup/docs/plan_il_vla.md) 가 정본 |
> | **§2 컨트롤러 인터페이스 불일치** | ➡️ `plan_il_vla.md` §2.3 으로 **이관**(IL·RL 공통이라 거기서 관리) |
> | **§3-A `oht_bolting`** | ⚠️ **다른 워크스페이스**(`/isaac-sim/standalone_examples_my/oht_bolting/`) 자산. 이 ws 범위 밖 — **패턴 참고용으로만** |
>
> **IL 이 왜 빠졌나**: Isaac Lab Mimic 은 사람 데모에 강체 변환을 걸어 증강하는데 **UR 은 6-DoF 라
> 변환 궤적이 도달 불가·특이점에 걸린다**(Isaac Lab 문서: 어려운 경우 후보 성공률 1% 미만).
> 대신 `to_do.md` M3 의 cuMotion pick&place 상태머신으로 데모를 생성한다 — 매 포즈마다 IK·충돌을
> 새로 풀어 실패 궤적이 안 나온다. 상세는 `plan_il_vla.md` §3.
>
> **➡ 그래서 RL 은 IL 과 갈라진다**: IL 은 Isaac Lab 없이 가지만, **RL 은 수천 env 병렬이 필수라
> Isaac Lab 을 우회할 수 없다.** 바로 아래 **착수 전 관문**을 먼저 통과해야 한다.
>
> **섹션 번호는 고의로 유지**했다(다른 문서가 `LEARNING.md §2` 등을 참조). 구 §5 자리는 포인터로 남긴다.

> 이 문서는 **설계/계획**이다(코드 아직 없음). UR16e 워크스페이스(`src/`, ROS2 Jazzy 제어 스택)에
> RL 을 **Isaac Lab** 으로 얹는 방향을 정리한다.
> 구성/실행 현황은 [`README.md`](README.md), 실물 절차는 [`HARDWARE.md`](HARDWARE.md), 이력은 [`HISTORY.md`](HISTORY.md).
> IL/VLA 는 [`ur_bringup/docs/plan_il_vla.md`](ur_bringup/docs/plan_il_vla.md), 작업레이어는 [`to_do.md`](to_do.md).
>
> 조사 근거(파일 경로/클래스명)는 본문에 인라인으로 박아 두었다. Isaac Sim 6.0.1, GPU = RTX 5090(32GB).
> (~~Isaac Lab 위치 = `/isaac-sim/IsaacLab` v2.3.2~~ — **2026-09-06 실측: 미설치.** 상단 **착수 전 관문** 참조.)

---

## ⚠️ 착수 전 관문 — 먼저 읽을 것 (2026-09-06 실측)

IL 과 달리 RL 은 병렬 env 처리량이 본질이라 Isaac Lab 을 건너뛸 수 없다. **착수 시 이 순서로 확인한다.**

| # | 관문 | 현재 상태 |
|---|---|---|
| 1 | **Isaac Sim 6.0.1 과 호환되는 Isaac Lab 버전 확인** | ❓ 미확인 — Isaac Lab 은 릴리스마다 지원 Isaac Sim 버전이 고정. **여기서 막히면 아래가 전부 무의미** |
| 2 | Isaac Lab 설치 | ❌ **미설치** (`/isaac-sim/IsaacLab` 없음) |
| 3 | PyTorch (sm_120 포함 휠) | ❌ **미설치** — system/Isaac python 양쪽 다 없음. `plan_il_vla.md` §6.3 의 arch 확인 규약 따를 것 |
| 4 | **instanceable USD 변환** | ❌ 현재 instanceable prim **0개**. 병렬 env 처리량에 직결 (§4) |
| 5 | 에셋 자체(articulation/드라이브/충돌/2F-85 물리) | ✅ **완료** — 실측 결과 `plan_il_vla.md` §5 참조 |

> ⚠️ apt 로 ML 스택을 깔 때는 **`SETUP.md` §2-B-1 핀 정책**을 따를 것 — 이 컨테이너는 다른 프로젝트와 공유된다.

---

## 0. 목표 / 범위

| 학습 | 태스크 | 데이터/방식 | 산출물 |
|---|---|---|---|
| **Reinforcement Learning** | **insertion**(peg/너트 등 접촉 많은 조립) | sim 병렬 학습(PPO) + domain randomization | sim2real 견딜 정책 |
| **배포** | 학습된 정책을 **ROS2 노드**로 → 기존 제어 스택 | | sim·real 공용 추론 |

로봇: **UR16e + Robotiq 2F-85**(우리 베이크 USD), 카메라 D405(eye-in-hand) + 정적 카메라.

> IL(pick&place)은 이 문서 범위 밖 → [`ur_bringup/docs/plan_il_vla.md`](ur_bringup/docs/plan_il_vla.md).

---

## 1. ★ 핵심 아키텍처 결정 — "학습은 오프라인(Isaac Lab), 배포만 ROS2"

"가능하면 ROS2 기반"의 정확한 답: **배포(inference)는 ROS2로, 학습(training)은 ROS2를 거치지 않는다.** 한계가 아니라 정석이다.

```
[ 학습 단계 ]  Isaac Lab (GPU 병렬 env, 직접 PhysX) ── ROS2 없음
   RL: 수천 env 병렬 → PPO(rl_games/rsl_rl) → 정책 checkpoint
        │  export (policy.pt / .onnx)
        ▼
[ 배포 단계 ]  ROS2 정책 러너 노드 ── 기존 sim↔real 인터페이스 그대로
   /joint_states(+비전) → policy(obs)→action → ros2_control → Isaac/실물 UR16e
```

- RL 은 표본 효율을 위해 수천 개 env 동시 구동이 필수 → ROS2 메시지 패싱으로는 throughput 불가.
  (`oht_bolting` 벤치: 64env 기준 helical 8,900 step·env/s — ROS2로 재현 불가능한 수치.)
  **이것이 IL 과 갈라지는 지점이다** — IL 은 단일 env 로도 데이터를 만들 수 있어 Isaac Lab 없이
  `to_do.md` M3 상태머신으로 가지만, RL 은 병렬 env 가 본질이라 Isaac Lab 이 필수다.
- **배포 단계는 우리가 만든 sim↔real ROS2 공용 인터페이스를 정확히 재사용**한다. Isaac Lab 도
  UR ROS 추론 예제를 동봉(§3-B) → 같은 패턴.

➡ 결론: 학습 코드는 기존 `ur_bringup`(ROS2 제어)을 **건드리지 않는 새 영역**에 두고(세트 분리 원칙),
배포 노드만 ROS2 패키지로 추가한다.

---

## 2. 컨트롤러 인터페이스 불일치 → **`plan_il_vla.md` §2.3 으로 이관됨**

> **이 절의 내용은 IL·RL 공통이라 한 곳에서 관리한다.
> 정본: [`ur_bringup/docs/plan_il_vla.md`](ur_bringup/docs/plan_il_vla.md) §2.3.**

요지만 남긴다 — 정책은 **매 스텝 액션**(관절 위치 델타, ~30–60Hz)을 내는데 현행 실행 컨트롤러는
`scaled_joint_trajectory_controller`(궤적 기반)라 그대로 흘리면 jerky/overshoot 가 난다.
**해결책은 이미 설치된 UR 컨트롤러로 확인됐다**(2026-09-06 실측):
`forward_position_controller` / `ur_controllers::PassthroughTrajectoryController`.
cuMotion(궤적)과 정책(스트리밍)은 같은 HW 인터페이스 위에서 **컨트롤러 전환**으로 공존한다.

**RL 고유 사항만 여기 유지**: Isaac Lab gear_assembly UR 배포가 쓰는 액션 =
`RelativeJointPositionActionCfg`, **scale 0.025 rad/step**, `base` 프레임 기준(§7 함정).
`oht_bolting` 도 동일 규약(joint-position-delta scale 0.025)이라 **RL 은 이 규약을 따른다.**

---

## 3. 재사용할 기존 자산 (조사 결과)

### 3-A. `oht_bolting` — RL insertion 참고자료 (**다른 워크스페이스**, 정독·평가 완료)

> ⚠️ **범위 주의**: `oht_bolting` 은 **이 워크스페이스가 아니다**
> (`/isaac-sim/standalone_examples_my/oht_bolting/`). 여기서는 **교훈·패턴만 참고**하고,
> 코드 결합이나 2F-140 의존을 만들지 않는다(§10-8).
> 아래 내용은 2026-06 시점의 조사 결과이므로, RL 착수 시 그 ws 의 현재 상태를 다시 확인할 것.

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

### 3-B. Isaac Lab 빌트인 템플릿 (RL 관련만)

> Isaac Lab 미설치 상태(상단 **착수 전 관문**)이므로 아래 경로는 **설치 후 재확인** 대상이다. 버전에 따라 이동/개명 가능.

- **로봇**: `isaaclab_assets/.../universal_robots.py` → `UR10e_ROBOTIQ_2F_85_CFG`
  (variant `{"Gripper":"Robotiq_2f_85"}`; 그리퍼 액추에이터 = `gripper_drive`[finger_joint]·
  `gripper_finger`[.*_inner_finger_joint]·`gripper_passive`[knuckles]). Franka 도 동일 2F-85 패턴(`franka.py`).
  → **우리 USD 의 실제 관절명과 이 정규식이 맞는지는 §4 에서 실측으로 확인됨.**
- **RL insertion 빌트인**: `direct/factory`(Franka, task-space, peg/gear/nut, solver iter=192),
  `direct/forge`(접촉+force+dynamics randomization). UR-네이티브 대안 =
  `manager_based/manipulation/deploy/gear_assembly/config/ur_10e/`(조인트-스페이스, 2F-85/2F-140).
- **RL 학습기**: `scripts/reinforcement_learning/{rsl_rl,rl_games,skrl,sb3,ray}/{train,play}.py`.
- **ROS 배포 예제**: `manipulation/deploy/gear_assembly/config/ur_10e/ros_inference_env_cfg.py`(+`deploy/mdp/`) —
  **UR10e 를 ROS 로 추론**하는 실제 템플릿(obs_order/action scale/joint names 노출, vision pose 입력).

> IL 쪽 빌트인(`stack`/`pick_place` 태스크, `teleop_se3_agent.py`, `record_demos.py`,
> `isaaclab_mimic/`, `robomimic/`)은 **이 문서 범위 밖**이라 제거했다. 필요해지면
> `plan_il_vla.md` §3.1 의 "Mimic 으로 돌아갈 경우" 항목을 참조.

---

## 4. 로봇 자산 포팅 — `UR16E_2F85_CFG`

우리 USD: `ur_bringup/isaac/assets/ur16e_with_2f85.usd`(RL 학습용 권장; 카메라/커플링 메시는 시각용),
`ur16e_2f85_d405.usd`(비전 태스크용). 빌드 스크립트 `isaac/common/build_ur16e_2f85.py` 가 이미
instanceable 제거·articulation root 정리·finger_joint drive 튜닝(K=20,D=1)을 반영.

### 2026-09-06 에셋 실측 — §10 의 "확인 필요" 항목들이 해소됨

USD 를 직접 열어 감사한 결과(전체는 `plan_il_vla.md` §5):

| 항목 | 실측값 |
|---|---|
| articulation root | `ArticulationRootAPI` @ `/UR16e/root_joint`, defaultPrim `/UR16e`, metersPerUnit 1.0 |
| **2F-85 폐루프** | **없음 — 완전한 트리.** 보조관절 5개에 `PhysxMimicJointAPI`(전부 `finger_joint` 참조, gearing ±1.0) |
| 그리퍼 관절명 | 마스터 `finger_joint` / `right_outer_knuckle_joint` / `left·right_inner_finger_joint` / `left·right_inner_finger_knuckle_joint` |
| `finger_joint` 한계 | 0~47° (≈0.82 rad) |
| 충돌 근사 | `convexDecomposition` 7 + `convexHull` 11, 근사 없는 콜라이더 0 |
| **instanceable** | **0개** — RL 병렬 env 전에 변환 필요(상단 **착수 전 관문**) |

**드라이브 게인(실측, USD 에 이미 들어있음)**

| 관절 | stiffness | damping | maxForce |
|---|---|---|---|
| shoulder_pan / shoulder_lift | 1963.78 | 7.855 | 330.0 |
| elbow | 1963.78 | 7.855 | 150.0 |
| wrist_1 / wrist_2 / wrist_3 | 741.39 | 2.966 | 56.0 |
| finger_joint | 20.0 | 1.0 | 26.0 |

**방침** (실측 반영):
- **2F-85 는 이미 `PhysxMimicJointAPI` 로 4절 링크가 풀려 있다** → §10-6 의 "USD-네이티브 4-bar 링키지
  유무" 질문은 **해소**. 별도 물리 수정 없이 그대로 쓴다. Isaac Lab 그리퍼 액추에이터 분리
  (`gripper_drive`/`gripper_finger`/`gripper_passive`)는 위 실측 관절명으로 정규식만 맞추면 된다.
- arm 액추에이터: **위 실측 게인으로 시작**하고, sim2real 이 안 맞으면 `oht_bolting` 튜닝값
  (8000/400, 5000/200, 2000/40, effort 330/150/28)을 참고해 조정.
  ⚠️ 두 값의 차이가 크다 — oht_bolting 은 servoj 매칭용으로 따로 튜닝한 값이므로 **맹목 복사 금지**.
- `articulation_root_prim_path` : 우리 USD 는 `/UR16e/root_joint`. Isaac Lab 은 `{ENV_REGEX_NS}/Robot` 아래
  스폰하므로 **상대 경로/auto-search** 동작 확인 필요(함정).
- ⚠️ **instanceable 변환 시 주의**: `oht_bolting` 교훈 — cloned env 에서 instanceable 메시가 사라지는 사례
  (Flatten 후 instanceable 해제). 변환 후 병렬 env 에서 시각/충돌 메시 존재를 반드시 확인.
- 출력: `src/ur_isaaclab_tasks/.../robots/ur16e.py` 에 `UR16E_2F85_CFG`.

---

## 5. ~~IL 파이프라인~~ → **이 문서에서 제거됨**

> **IL(pick&place) 정본은 [`ur_bringup/docs/plan_il_vla.md`](ur_bringup/docs/plan_il_vla.md) 다.**
> (섹션 번호를 유지하려고 자리만 남긴다.)

원래 여기 있던 계획 — *Isaac Lab `stack/config/ur10_gripper` 기반 env → `record_demos.py` teleop 수집
10~20개 → Mimic 증강 → robomimic BC-RNN 학습* — 은 **폐기**됐다.

**폐기 이유** (요약, 상세는 `plan_il_vla.md` §3.1):
1. Mimic 의 강체 변환 증강이 **UR 6-DoF 에서 도달 불가/특이점**에 걸린다(어려운 경우 후보 성공률 1% 미만).
2. Isaac Lab 미설치 + Isaac Sim 6.0.1 호환 미확인 + `ManagerBasedRLMimicEnv` 포팅 + instanceable 변환 +
   HDF5→LeRobot 변환까지 **도입 비용이 크다**.
3. 대안이 이미 계획돼 있었다 — `to_do.md` **M3 cuMotion pick&place 상태머신**이 물체 포즈를 랜덤화하며
   실행되면 그 자체가 데모 생성기이고, **매 포즈마다 IK·충돌을 새로 풀어** 실패 궤적이 안 나온다.
4. 실물은 **UR freedrive 티칭**(`ur_controllers::FreedriveModeController`, 설치 확인됨)으로
   leader/follower 없이 사람 궤적을 얻는다.

**현행 IL 스택**: LeRobot 데이터셋(단일 진실) → ACT(데이터 품질 진단) → 소형 VLA → GR00T LoRA.

> **RL 착수 시 재활용 가능한 부분**: 위 IL 계획의 *env 정의 방식*(scene.robot=`UR16E_2F85_CFG`,
> `ee_frame`, 성공 termination 분리)은 RL env 에도 그대로 쓰인다 → §6 참조.

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

## 7. ROS2 배포 (RL 정책)

> **확정(oht_bolting+NVIDIA 레시피)**: 정책은 **joint-position delta** 를 내고, 실물 측은 **impedance/스트리밍 위치
> 컨트롤러**가 받는다(학습엔 Cartesian impedance 안 씀 — sim 불안정). 우리 §2 결정과 정확히 일치.
>
> IL/VLA 정책의 배포는 `plan_il_vla.md` §2.2/§2.3(정책 서버 + 소켓, 스트리밍 컨트롤러) 참조.
> **스트리밍 컨트롤러 계층은 둘이 공유**하므로 한 번만 만들면 된다.

1. **스트리밍 컨트롤러 추가**(§2): ros2_control 에 위치 스트리밍 컨트롤러.
   **2026-09-06 실측으로 후보가 확정됐다** — apt `ur_controllers` 가 `forward_position_controller` 와
   `ur_controllers::PassthroughTrajectoryController` 를 이미 제공한다.
   cuMotion/MoveIt(궤적) 와는 컨트롤러 전환으로 공존.
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
│   └── docs/plan_il_vla.md          #   IL/VLA 정본 (별도 트랙)
├── ur_isaaclab_tasks/              # (신규) Isaac Lab RL 학습 — ROS 아님, /isaac-sim/python.sh 로 실행
│   ├── robots/ur16e.py             #   UR16E_2F85_CFG (우리 USD + 실측 게인, §4)
│   ├── insertion/                  #   RL: env cfg + mdp(reward/obs) + agents(ppo cfg)
│   ├── deploy/                     #   *_ros_inference_env_cfg.py (obs순서/스케일 메타)
│   └── README.md                   #   실행 명령 모음(train/play/export)
├── ur_policy_deploy/               # (신규) ROS2 패키지(ament_python): 정책 러너 노드 + 스트리밍 컨트롤러 yaml + launch
└── policies/   (gitignore 또는 포인터)  # 학습 checkpoint/onnx
```
- `ur_isaaclab_tasks` 는 Isaac python 에 `pip install -e` 또는 PYTHONPATH 로 등록(gym.register).
- `ur_policy_deploy` 만 colcon 빌드(ROS2). 세트 분리·sim↔real 공용 인터페이스 유지.
- **IL/VLA 산출물은 여기 두지 않는다** — LeRobot 데이터셋/정책은 `plan_il_vla.md` 트랙에서 관리.
  단 `ur_policy_deploy`(스트리밍 컨트롤러 + 정책 러너)는 **두 트랙이 공유**한다.

---

## 9. 단계별 로드맵 (제안)

| 단계 | 내용 | 산출/검증 | 위험 |
|---|---|---|---|
| **L−1 환경 관문** | Isaac Lab(6.0.1 호환) + torch(sm_120) 설치, instanceable 변환 | 상단 **착수 전 관문** 5개 항목 통과 | **높음** — 1번 항목에서 막히면 전체 재검토 |
| **L0 자산 포팅** | `UR16E_2F85_CFG`(실측 게인 §4) + 빈 env reset/step/관절 구동 | sim 에서 팔+그리퍼 구동·관절명 확인 | 낮음(에셋 실측 완료) |
| **L3 RL insertion** | insertion env + 보상 설계 + PPO | sim 삽입 성공률 곡선 | 높음(접촉 물리/보상) |
| **L4 ROS2 배포** | 스트리밍 컨트롤러 + 정책 러너 노드 | sim 에서 정책으로 plan-free 실행 | 중(컨트롤러/프레임) |
| **L5 실물** | HW 연결 후 동일 노드로 추론 | (HW 준비 시) | sim2real |

> ~~L1 / L2 (IL 태스크·데이터·학습)~~ 는 이 트랙에서 제거 — `plan_il_vla.md` 로 이동.
> 단계 라벨은 다른 문서 참조 때문에 **재번호하지 않았다**(L1·L2 결번).

**먼저 L−1 을 권장한다.** 기존에는 L0 가 "IL·RL 공통 토대, 위험 최저"라 첫 단계였지만, IL 이 분리되면서
L0 는 RL 전용 비용이 됐다. **Isaac Lab 호환성이 확인되기 전에는 L0 에 투자하지 말 것** —
그 관문에서 막히면 RL 접근 자체를 재검토해야 한다.

---

## 10. 열린 결정사항

1. **★ RL insertion 물리 수준** ⏳ 미정: (a) **kinematic**(oht_bolting 식, 빠른 PoC, 대상물 glue·실접촉 아님) vs
   (b) **접촉-resolved**(SDF/forge, 진짜 insertion, ~100배 느림·sim2real 난제, oht_bolting 도 미해결).
   → (a)로 시작해 파이프라인·배포까지 세운 뒤 (b)로 상승 권장. **목표가 "접촉 제어 학습"이면 (b) 필수**.
2. ~~IL 베이스 태스크~~ → **범위 밖**(`plan_il_vla.md`).
3. **RL insertion 대상물** ⏳ 미정: peg-in-hole(단순) vs 너트/볼트. → peg 로 시작 후 확장.
4. ~~Teleop 장치~~ → **범위 밖.** (IL 쪽 결론: sim 은 cuMotion 상태머신, 실물은 UR freedrive.
   DualSense 는 그리퍼 개폐 입력으로 축소 — `plan_il_vla.md` §3.)
5. **배포 컨트롤러** ✅ **해소(2026-09-06 실측)**: apt `ur_controllers` 가
   `forward_position_controller` 와 `PassthroughTrajectoryController` 를 제공.
   `JointGroupPositionController` 를 별도로 쓸 이유 없음. 남은 것은 실기동 검증뿐.
6. **2F-85 통합 방식** ✅ **해소(2026-09-06 실측)**: USD 에 **`PhysxMimicJointAPI` 로 4절 링크가 이미 풀려
   있고 강체 그래프가 완전한 트리**다(§4). 별도 EE 베이크(A안) 없이 **(B) 기존 `ur16e_with_2f85.usd` 에
   ArticulationCfg 직접**으로 간다. 관절명도 실측 완료 → §4 표 참조.
   (EE 교체 유연성이 나중에 필요해지면 그때 A안으로 전환)
7. **`ur_isaaclab_tasks` 형태** ⏳ 미정: 독립 Python 패키지 vs Isaac Lab 외부 태스크 확장. → 독립 패키지 권장.
8. **oht_bolting 코드 재사용 범위** ⚠️ **재검토 필요**: 원래 "핵심 파일을 vendoring 후 적응"으로 결정했으나,
   **`oht_bolting` 은 다른 워크스페이스**(`/isaac-sim/standalone_examples_my/oht_bolting/`)이고
   이 ws 범위 밖으로 정리됐다. → **코드 복사보다 §3-A 에 정리된 "교훈"만 가져오는 쪽**이 안전하다
   (컨트롤러=joint-position PD, termination/success 분리, kinematic↔SDF zero-shot 0%, instanceable 함정).
   실제 vendoring 여부는 RL 착수 시점에 그 ws 상태를 다시 보고 판단.

> **신규 (2026-09-06)**
> 9. **Isaac Lab ↔ Isaac Sim 6.0.1 호환 버전** ❓ **최우선 미확인** — 상단 **착수 전 관문** 1번.
>    여기서 막히면 1·3·7 을 논할 필요가 없다.
> 10. **RL 과 IL 정책의 배포 통합 범위** ⏳ — `ur_policy_deploy`(스트리밍 컨트롤러 + 러너)를
>    두 트랙이 공유하기로 했는데(§8), 정책 I/O 규약(obs 순서/스케일/주파수)이 서로 다르다.
>    러너를 하나로 두고 정책별 어댑터를 둘지, 노드를 분리할지 결정 필요.
