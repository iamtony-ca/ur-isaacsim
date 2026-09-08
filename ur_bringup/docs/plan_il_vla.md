# UR16e Imitation Learning / VLA 프로젝트 계획

> 정리 시점: 2026-09-01 · **개정: 2026-09-06** (에셋 실측 + 환경 실측 반영, 데이터 생성 엔진 교체)
> 목표: UR16e + Robotiq 2F-85 대상으로 LeRobot ACT → 소형 VLA → GR00T로 확장하는 모방학습/VLA 파이프라인 구축

> **문서 위치**: 이 문서가 **IL/VLA 정본**이다.
> - 작업레이어(perception + pick&place 상태머신)는 [`../../to_do.md`](../../to_do.md) 가 정본이며,
>   그 **M3 상태머신이 이 문서의 데이터 생성 엔진을 겸한다**(§3).
> - `../../LEARNING.md` 와의 관계 — **은퇴가 아니라 분할**:
>   · **IL 절반**(Mimic → robomimic BC)은 **이 문서로 대체됨** (LeRobot → ACT → 소형 VLA → GR00T)
>   · **RL insertion 절반은 유효하게 살아있다** — 추후 이 워크스페이스에서 RL 을 진행할 수 있음
>   · `oht_bolting` 참조는 **다른 워크스페이스**의 자산이라 이 ws 범위 밖 (패턴 참고용으로만)
>   · §2(컨트롤러 인터페이스 불일치)는 이 문서 §2.3 으로 이관
>
> **RL 이 나중에 들어와도 이 문서의 결정 대부분은 그대로 쓰인다** — §2.3 의 스트리밍 컨트롤러 전환,
> §2.6 의 액션 공간·그리퍼 매핑, §5 의 에셋 실측값(드라이브 게인·mimic joint), §6.3 의 sm_120 규약은
> IL/VLA 전용이 아니다. RL 도입 시 달라지는 건 **학습 루프와 병렬 env 요구**뿐이고, 그때는
> Isaac Lab + instanceable USD(§5) 가 다시 필요해진다 — **그래서 §5 의 instanceable 항목을 지우지 않고 남겨뒀다.**

> **2026-09-06 개정 요약** — 무엇이 바뀌었나
> 1. **데이터 생성 엔진 교체**: Isaac Lab Mimic → `to_do.md` M3 **cuMotion 상태머신 + 실물 freedrive** (§3)
> 2. **실물 freedrive 티칭을 앞으로**: leader/follower 없이 사람 궤적 확보 가능 (§1, §3)
> 3. **ROS2 경계 재정의**: 정책 추론만 소켓 밖, 로봇 I/O·기록·안전은 기존 ROS2 스택 재사용 (§2.3)
> 4. **모델 사다리에 소형 VLA 추가**: ACT → 소형 VLA → GR00T (§2.7)
> 5. **§5 체크리스트 실측 완료** — 2F-85 물리는 **이미 해결되어 있었음** (§5)

---

## 1. 하드웨어 현황

| 항목 | 상태 |
|---|---|
| UR16e | 보유 (실물), Isaac Sim 환경에 구현 완료 |
| Robotiq 2F-85 | 보유, UR16e 단독 부착 가능. **sim 물리까지 완료**(§5) |
| RealSense D405 / D435 / D455 | 보유 |
| 카메라 + 그리퍼 동시 마운트 | **없음 → 제작하기로 확정.** 실물 데이터의 유일한 실질 블로커 → **지금 병렬 착수** |
| Leader / Follower 텔레오퍼레이션 장비 | 없음 → **불필요해짐**(아래 freedrive) |
| **UR freedrive (핸드가이딩) 티칭** | **가능.** `ur_controllers::FreedriveModeController` 가 apt 로 이미 설치됨 |
| GPU | RTX 5090 (32GB, **sm_120 / Blackwell**) |
| 입력 장치 | PS5 DualSense (그리퍼 개폐 입력용으로 축소) |

**★ freedrive 가 이 프로젝트의 전제를 바꾼다.** 팔을 손으로 직접 끌어 시연하고 `/joint_states` 를 기록하면,
추가 하드웨어 없이 **사람 궤적을 지터 없이** 얻는다. "leader/follower 가 없어서 실물 데모를 못 모은다"는
제약이 사라지므로, 실물 데이터 수집을 로드맵 맨 끝(F)에 둘 이유가 없다 → §3·§4 참조.
(⚠️ 16 kg 가반 900 mm 팔이다. 속도·영역 제한과 비상정지 동선 확보가 선행 조건.)

**소프트웨어 환경 실측 (2026-09-06)** — 로드맵 착수 전에 채워야 할 공백:

| 항목 | 상태 |
|---|---|
| Isaac Sim | 6.0.1 ✅ (세트1~3 + cuMotion + nvblox 전수 재검증, `HISTORY.md` §14) |
| **PyTorch** | ✅ **구축 완료(2026-09-06)** — `deps/.venv-ml` 격리 venv, **torch 2.11.0+cu128, sm_120 실증**(RTX 5090 matmul). system/Isaac python 은 여전히 깨끗. `HISTORY.md` §19 |
| **LeRobot** | ✅ 0.6.1(`[dataset]` extra). **raw→LeRobot 변환 round-trip 검증 완료** |
| **Isaac Lab** | **미설치** — 다만 §3 개정으로 **크리티컬 패스에서 제외됨** |

---

## 2. 확정된 설계 결정

### 2.1 데이터 포맷: LeRobot 데이터셋

**이 프로젝트의 단일 진실(single source of truth).**

- ACT, pi0(openpi), GR00T N1.7이 모두 LeRobot 포맷을 소비
- GR00T는 `meta/modality.json`만 추가로 요구
- UR16e는 pi0·GR00T 어느 사전학습 믹스처에도 없음 → `new_embodiment` 파인튜닝이 필수
- **결론: 지금 ACT용으로 모으는 데이터가 그대로 VLA 자산이 된다**

> 코드는 다시 짤 수 있지만 잘못 모은 데이터는 되돌릴 수 없다. 미들웨어 선택보다 스키마 확정이 우선.

### 2.2 아키텍처: Policy Server / Thin Client

pi0와 GR00T가 **동일한 구조**를 쓴다는 점이 설계의 출발점.

- openpi: `scripts/serve_policy.py` → WebSocket(기본 8000), 로봇 측은 `openpi-client`만 설치
- GR00T: `scripts/inference_service.py --server` → 클라이언트가 요청

로봇 측 코드는 사실상 세 줄:
```
관측 딕셔너리 생성 → 소켓 전송 → action chunk 수신 후 실행
```

```
[Layer 3] 정책 컨테이너 (별도 Docker, GPU)
          openpi serve_policy.py / gr00t inference_service.py
              ↕ WebSocket / ZMQ   ← 프로세스·환경 경계
[Layer 2] 브리지 (관측 동기화, chunk 보간, 안전 게이트)
              ↕
[Layer 1] 하드웨어 (sim 또는 실물 UR16e)
```

**Layer 2/3를 소켓으로 끊는 것이 핵심.** ROS 2 Jazzy는 Python 3.12, openpi 계열은 다른 환경을 전제하므로 같은 venv에 넣으려는 시도는 거의 실패한다.

> **2026-09-08 실측 정정.** 이 서술은 **lerobot 계열(ACT/pi0/pi05/smolvla/groot)에는 해당하지 않는다.**
> `deps/.venv-ml` 에서 ROS 오버레이를 source 하면 `rclpy` 와 `torch(cu128, sm_120)` 가 **함께 임포트된다**.
> 따라서 lerobot 정책은 소켓 경계 없이 상류 `async_inference`(policy_server + robot_client)를 그대로 쓰고,
> 우리 것은 로봇 플러그인 `lerobot_robot_ur16e_ros`(`--robot.type=ur16e_ros`) 하나뿐이다.
> 정책 교체는 `--policy_type` 인자 변경이다.
>
> 단, 경계가 사라진 것은 아니다. **정확한 한계는 "순수 파이썬 ROS API 는 되고, numpy 에 링크된 ROS C 확장은
> 안 된다"** 이다. ROS Jazzy 는 numpy 1.26.4 로 빌드됐고 venv 는 lerobot 이 요구하는 numpy 2.2.6 이라,
> `cv_bridge` 를 임포트하면 프로세스가 코어 덤프한다
> (`A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.6`).
> 어댑터가 `sensor_msgs/Image` 를 numpy 로 직접 디코딩하는 이유가 이것이다.
>
> 소켓 경계가 **여전히 필요한 경우**: openpi 자체 서버, NVIDIA `Isaac-GR00T` 저장소를 lerobot 의 `groot`
> 정책 대신 직접 쓰는 경우 등 lerobot 밖의 정책 스택. (Seeed의 Jetson 배포 가이드도 Python 3.12에서 LeRobot ACT 추론 API 호출을 권장하지 않는다고 명시.)

### 2.3 ROS 2: "나중에"가 아니라 **경계를 긋는다** (2026-09-06 개정)

원래 "Phase 1은 ROS 2 없이"였으나, **이 워크스페이스에는 이미 sim/real 공용 ROS 2 스택이 검증된 채로 존재한다.**
이걸 우회하면 동작 중인 `ur_robot_driver` 옆에 RTDE 경로를 하나 더 만드는 셈이다. 질문을 셋으로 쪼개면
"나중에 도입"이 아니라 **어디를 소켓으로 끊을지**의 문제가 된다.

| 질문 | 답 |
|---|---|
| 정책 **추론**이 ROS 2를 지나가야 하나? | **아니오** — 별도 컨테이너 + 소켓 (§2.2) |
| 로봇 **I/O · 데이터 기록 · 안전 · 에피소드 리셋**은? | **예 — 기존 ROS 2 스택 그대로 재사용** |
| 주변 인프라(캘리브레이션, nvblox, perception)는? | **예** (`to_do.md` 가 이미 ROS 2 기반) |

소켓 경계는 **Python 환경 충돌** 때문에 필요한 것이지(ROS 2 Jazzy=3.12 vs openpi 계열), ROS 2 자체를
미루기 위한 것이 아니다. 로봇 쪽 코드는 여전히 세 줄이다.

#### ★ 이관 항목 — 컨트롤러 인터페이스 불일치 (구 `LEARNING.md` §2)

**정책은 매 스텝 액션(~30–60 Hz)을 내는데, 현재 실행 컨트롤러는 `scaled_joint_trajectory_controller`
(궤적 기반, `follow_joint_trajectory` = 여러 waypoint + 타이밍)다.** 단일-스텝 목표를 궤적 컨트롤러로
흘리면 jerky/overshoot 가 난다. 그리퍼도 같은 문제 — 현재 `position_controllers/GripperActionController`
(액션 기반)라 스트리밍과 맞지 않는다.

**해결책이 이미 설치되어 있다** (apt `ur_controllers`, 실측 확인):

| 컨트롤러 | 용도 |
|---|---|
| `forward_position_controller` | 정책 액션 스트리밍(관절 위치 직접) |
| `ur_controllers::PassthroughTrajectoryController` | UR 권장 경로 — 궤적을 드라이버로 통과시킴 |
| `ur_controllers::FreedriveModeController` | **실물 핸드가이딩 데모 수집**(§3) |
| `ur_controllers::ScaledJointTrajectoryController` | 현행 — cuMotion/MoveIt 실행 |

cuMotion(궤적)과 정책(스트리밍)은 **같은 HW 인터페이스 위에서 컨트롤러만 전환**해 공존한다.
→ 이건 F단계로 미룰 문제가 아니라 **초기에 한 번 푸는 작은 작업**이다.

**MoveIt2/cuMotion은 추론 루프에 넣지 않는다.** 정책은 수십 Hz로 절대 관절 위치 청크를 뱉고, MoveIt2는 A→B 계획 도구라 패러다임이 다르다. 다만 아래 용도로는 유용:
- 에피소드 리셋(충돌 없는 홈 복귀) — `reset_pose.py home` 이 이미 있음
- **안전 봉투** — VLA가 이상한 액션을 뱉을 때 차단. 16kg 팔 + 미학습 embodiment 조합에서는 필수
- **데모 생성**(§3) — 이게 이번 개정의 핵심 용도
- IK (EE pose 액션을 쓸 경우, 단 `ur_ikfast` 단독이 더 가벼움)

**참고 — 브리지의 비용**: RIO 논문이 SO-100 + RealSense 3대 환경에서 π0.5 롤아웃의 관측-행동 지연을 측정한 결과 LeRobot 파이프라인 581.2ms vs RIO 130.3ms(4.46배 차이). 계층을 순진하게 쌓으면 지연이 성능을 갉아먹는다. ROS 2 도입 시 intra-process / loaned message 경로 확보 필요.

### 2.4 언어: Python 우선, C++은 핫루프만

`ur_rtde`는 C++ 코어 + pybind11 바인딩이므로 "어느 쪽이 빠른가"는 잘못된 질문.

| 레이어 | 선택 |
|---|---|
| 데이터 수집, 정책 클라이언트 | **Python** (LeRobot `Robot` 클래스, `openpi-client`가 Python) |
| 500Hz `servoJ` 보간 루프 | 측정 후 필요 시에만 C++ |
| 안전 게이트 | 무관 |

이유:
1. 지연 예산이 추론(130~580ms)에 지배됨. servoJ 루프의 언어 차이는 마이크로초 단위
2. ROS 2를 쓰면 `ur_robot_driver`가 이미 C++ RT 루프를 제공 → 질문이 소멸

언어보다 영향이 큰 것: PREEMPT_RT 커널, `SCHED_FIFO` + `isolcpus`, UR 컨트롤박스 직결 네트워크.

### 2.5 카메라: 외부 1 + 손목 1 (2대)

- "2대"는 GR00T `so100_dualcam` 레퍼런스에서 온 숫자지 규칙이 아님. N1.7은 `--modality-config-path`로 직접 정의
- **개수보다 구성이 중요**: openpi DROID의 관측 키가 `exterior_image_1_left` + `wrist_image_left` — 사전학습 분포의 표준형
- 손목 카메라는 **D405**(근접용). 외부는 D435/D455
- 시뮬 카메라는 나중에 실물에도 존재해야 하므로 손목 마운트 제작 확정
- 카메라를 늘리면 Mimic 생성 처리량이 그대로 떨어짐 (렌더링이 병목)

**중요 (2026-09-06 정정)**: 손목 카메라의 sim 배치는 실제 마운트 설계 치수와 일치해야 한다. 원칙은
"마운트 오프셋을 먼저 정하고 sim에 반영"이지만, **sim 이 이미 특정 치수를 확정한 상태다**:

```
tool0 → camera_link :  xyz = (0, -0.067, 0.01847) m,  pitch 8°   (PickNik 브라켓 기준)
```
이 값은 `urdf/ur16e_2f85_d405/realsense_d405_macro.xacro` 와 `isaac/common/ur16e_isaac_ros2.py`
**양쪽에 중복 기재**되어 있고, 한쪽만 바꾸면 RViz ↔ Isaac 자세가 어긋난다(`CLAUDE.md` 함정 7).

➡ 따라서 할 일은 "먼저 정하기"가 아니라 둘 중 하나다:
1. **실물 브라켓을 이 명목 치수로 제작**한다 (권장 — sim/데이터가 이미 이 값으로 검증됨), 또는
2. 제작 치수가 다르면 **위 두 파일을 함께 갱신**하고 `build_ur16e_2f85.py` 로 USD 를 재베이크한다.

`to_do.md` 는 v1 에서 **정적 카메라 1대로 장애물+타깃을 모두** 처리하고 손목 카메라를 v2 로 미룬다.
IL 데이터셋의 `video.wrist` 키는 처음부터 스키마에 넣어두되, v1 수집분은 비워두거나 정적 카메라만으로
시작해도 된다 — **스키마를 나중에 바꾸는 것이 데이터를 다시 모으는 것보다 비싸다**(§2.1).

### 2.6 액션 공간: 절대 관절 위치

```
video.exterior      : (H, W, 3)   외부 정면 뷰
video.wrist         : (H, W, 3)   손목 뷰 (D405 위치)
state.single_arm    : (6,)        관절 위치 [rad]
state.gripper       : (1,)        0.0(열림) ~ 1.0(닫힘) 정규화
action.single_arm   : (6,)        목표 관절 위치 [rad]
action.gripper      : (1,)        목표 그리퍼 개폐
task                : str         자연어 지시문
```

선택 이유:
- 실행 시 IK가 루프에 없음 → 특이점 문제 회피
- ACT(ALOHA 방식)와 정합
- GR00T `so100_dualcam` 패턴(`--modality_keys single_arm gripper`)과 일치 → 문서화된 `new_embodiment` 경로

#### 기존 스택과 붙는 지점 (실측 기반, 반드시 스키마에 못 박을 것)

**① 그리퍼 정규화 매핑**

```
state.gripper / action.gripper  0.0 = 열림  ~  1.0 = 닫힘
    ↕
finger_joint                    0.0 rad     ~  0.8 rad   (USD 한계 0~47° ≈ 0.82 rad, URDF limit 0~0.8)
```
`gripper_controller` 는 `position_controllers/GripperActionController` 이고 `joint: finger_joint` 하나만
명령한다(`config/common/ur16e_2f85_controllers.yaml`).

**② ★ 그리퍼 상태는 `finger_joint` **만** 기록한다**

URDF 와 USD 가 **보조관절의 이름·부호 규약이 서로 다르다.** 공통인 것은 마스터 `finger_joint` 하나뿐:

| | 보조관절 이름 |
|---|---|
| URDF (`robotiq_2f85_macro.xacro`) | `robotiq_85_right_knuckle_joint`, `..._left/right_inner_knuckle_joint`, `..._left/right_finger_tip_joint` (mimic multiplier ∓1) |
| USD (Isaac 에셋) | `right_outer_knuckle_joint`, `left/right_inner_finger_joint`, `left/right_inner_finger_knuckle_joint` (PhysX mimic gearing ±1) |

현재 제어 스택은 `finger_joint` 만 명령하므로 **지금은 무해**하지만, 데이터셋에 보조관절을 함께 기록하거나
sim↔real 을 관절 이름으로 매핑하는 순간 조용히 깨진다. **`state.gripper` 는 `finger_joint` 스칼라 하나.**

**③ 액션 기록 규약** — 데이터셋의 `action.single_arm` 은 **정책이 재현해야 할 목표 관절 위치**다.
§3 의 상태머신 생성 경로에서는 컨트롤러로 보낸 궤적 waypoint 가 아니라, **다음 스텝의 관측 관절 위치**를
액션으로 기록하는 편이 안전하다(궤적 보간 타이밍과 기록 rate 불일치를 피함). §6.5 의 시간 동기화 항목 참조.

### 2.7 모델 순서: ACT → **소형 VLA** → GR00T (2026-09-06 개정)

**ACT를 "쉬운 단계"가 아니라 "데이터 품질 진단 도구"로 사용.**

ACT는 80M 규모라 5090에서 몇 시간이면 학습 완료. **ACT가 데모 50개로 태스크를 못 배우면 모델 문제가 아니라 데이터 문제** — 타임스탬프 정렬, 액션 공간, 데모 일관성 중 하나가 깨진 것. 3B로 확인하면 시간이 10배 든다.

**개정 이유 — ACT(80M, 언어 없음) → GR00T(3B, LoRA) 점프가 너무 크다.** §6.2 가 이미 "32GB 단일 카드는
LoRA 레퍼런스(48GB) 아래"라고 판정했는데, 그 도박의 결과를 E단계에 가서야 알게 된다. 사이에 **LeRobot
생태계 안의 소형 언어조건 VLA**(SmolVLA 계열, ~0.5B급)를 한 칸 넣는다:

| 단계 | 모델 | 얻는 것 |
|---|---|---|
| 1 | **ACT** (~80M, 언어 ✗) | **데이터 품질 판정.** 여기서 실패하면 아래는 볼 것도 없음 |
| 2 | **소형 VLA** (~0.5B, 언어 ✓) | **"내 데이터로 언어 조건이 먹히는가"를 3B 도박 전에 확인.** 포맷 변환 0(LeRobot 유지), 32GB 여유 |
| 3 | **GR00T** (3B, LoRA) | 최종 목표. 언어 일반화 + 사전학습 전이 |

§2.8 이 설계한 **ACT vs VLA 동일조건 비교**가 2단계에서 이미 가능해진다 — 실험을 앞당기고 비용을 줄인다.

#### ★ 실측 해소 (2026-09-07) — "버전명 확인 필요" 항목

설치된 **lerobot 0.6.1 안에 GR00T N1.7 이 정책으로 들어 있다**(`lerobot/policies/groot/groot_n1_7.py`,
`lerobot-train --policy.type=groot`). 즉 **ACT → 소형 VLA → GR00T 가 프레임워크·데이터 스키마·학습
명령까지 전부 동일**하다. 별도 Isaac-GR00T 레포/변환 파이프라인이 필요 없다.

| 항목 | 값 (`policies/groot/configuration_groot.py` 실측) |
|---|---|
| 베이스 모델 | `nvidia/GR00T-N1.7-3B` |
| 백본 | `nvidia/Cosmos-Reason2-2B` (N1.7 은 Cosmos/Qwen3-VL. Eagle VLM 은 **N1.7 이전** 전용) |
| **N1.5** | **지원 제거됨** — 쓰려면 `lerobot==0.5.1` 로 고정해야 한다. 본문 표기 "N1.7" 이 맞고 §7 의 N1.5 링크가 낡았다 |
| 새 로봇 | `embodiment_tag = "new_embodiment"` ← UR16e 가 여기 해당 |
| 액션 | `chunk_size = 40`, `n_action_steps = 40` |
| **기본 튜닝 범위** | `tune_llm=False`, `tune_visual=False`, **`tune_projector=True`, `tune_diffusion_model=True`, `tune_vlln=True`** |

**§6.2 의 "3B 도박" 판정이 완화된다.** 기본값이 이미 **LLM·비전 백본을 얼려 두고 projector +
diffusion head 만 학습**하는 부분 파인튜닝이다. §6.2 가 걱정한 건 *전체* 파인튜닝(H100/L40 권장,
LoRA 레퍼런스 48GB)이었는데, 기본 경로는 그보다 훨씬 가볍다. → **32 GB 단일 5090 에서 가능성이
꽤 높다. 다만 아직 실측 전이므로 "가능"이라고 단정하지 말 것** — E단계 진입 시 제일 먼저 잴 것.

이 때문에 **2단계(소형 VLA)의 성격도 바뀐다**: "3B 를 감당할 수 있나"를 미리 보는 보험의 가치는 줄고,
"내 데이터로 언어 조건이 먹히나"를 싸게 확인하는 가치만 남는다. 데이터가 좋으면 **2단계를 건너뛰고
바로 GR00T 로 가는 것도 합리적**이다 — D단계 결과를 보고 정한다.

### 2.8 첫날부터 지킬 것: 언어 조건

**ACT는 언어 비조건부, GR00T·pi0는 언어 조건부.**

태스크 설명 없이 단일 태스크만 모으면 GR00T 파인튜닝은 돌아가지만 언어 일반화가 사라져 **3B짜리 비싼 ACT**가 된다.

- 에피소드마다 LeRobot `task` 필드에 실제 지시문 기록
- **태스크를 최소 2~3종 변형으로 수집** (예: "빨간 블록을 그릇에" / "파란 블록을 그릇에" / "블록을 왼쪽 상자에")
- 이렇게 모으면 동일 데이터로 ACT(언어 무시) vs GR00T(언어 사용)를 **같은 조건에서 비교** 가능 — 이 비교가 프로젝트를 튜토리얼 실행이 아닌 실험으로 만든다

---

## 3. 데모 수집 전략 (2026-09-06 전면 개정)

> **개정 전**: Isaac Lab Mimic 으로 사람 데모 10~20개를 증강.
> **개정 후**: **sim = cuMotion 상태머신 자동 생성 / real = UR freedrive 티칭.** Mimic 은 옵션으로 강등.

### 3.1 왜 Isaac Lab Mimic 을 크리티컬 패스에서 빼는가

Mimic 은 사람 데모를 subtask 로 쪼갠 뒤 **각 구간에 강체 변환을 걸어 재조합**한다. 그런데 이 메커니즘이
**우리 로봇에서 가장 잘 깨지는 부분**이다 — §6.1 이 스스로 지적했듯 UR 은 6-DoF 라 변환된 EE 궤적이
도달 불가·손목 특이점에 걸린다(Isaac Lab 문서: 어려운 경우 후보 성공률 **1% 미만**).
즉 "10개를 1000개로"라는 Mimic 의 가치제안이 하필 이 팔에서 가장 취약하다.

여기에 도입 비용이 붙는다 (2026-09-06 실측):

| Mimic 경로가 요구하는 것 | 현재 상태 |
|---|---|
| Isaac Lab 설치 + **Isaac Sim 6.0.1 호환 버전** | **미설치, 호환성 미확인** — 계획의 숨은 최대 리스크였음 |
| `ManagerBasedRLMimicEnv` 서브클래스 (`target_eef_pose_to_action` 등) | 미착수 |
| **instanceable USD 변환** | instanceable prim **0개** (§5) |
| Isaac Lab `ArticulationCfg` = **세 번째 로봇 정의** | URDF·USD 이미 2개. 동기화 함정(`CLAUDE.md` 함정 7)이 하나 더 늘어남 |
| HDF5 → LeRobot 변환 스크립트 | 미작성 (§6.5 가 "예상보다 시간 먹는 지점"으로 지목) |

### 3.2 sim 데이터: pick&place 상태머신이 **곧 데모 생성기**

> **★ 2026-09-07 재정렬.** 이 문서가 **이 프로젝트의 정본**이다. `to_do.md` 는 **다른 프로젝트**의
> 문서이므로 그 마일스톤(M0~M5)·결정(D1~D9)에 우리 일정을 묶지 않는다. 아래 내용은
> 그 아이디어를 **이 저장소 안으로 옮겨 온 것**이며, 구현체는 `ur_bringup/scripts/pick_place_demo.py` 다.
>
> **두 가지가 바뀌었다**:
> 1. **pose 소스 = Isaac GT** (`/scene/object_pose`). foundation perception(FoundationStereo/SAM3/
>    FoundationPose)은 **쓰지 않는다** — §7-B 가 이미 못박았듯 정책 입력은 RGB + 관절뿐이고,
>    상태머신은 *정책이 흉내낼 궤적을 만들 뿐*이라 GT 로 조준하든 추정 pose 로 조준하든
>    **데이터셋 내용이 달라지지 않는다.** perception 을 먼저 세우는 건 데이터 수집 일정만 미룬다.
>    나중에 필요하면 `-r /scene/object_pose:=/target/pose` **remap 한 줄**로 교체된다.
> 2. **이건 트렁크가 아니라 보조 경로다.** 이 프로젝트의 목표는 *실물 OMY-L100 teleop* → ACT → GR00T
>    이고, 상태머신은 ⓐ **사람 없이 파이프라인 전체를 검증**하는 구동기, ⓑ sim 대량 데이터 생성기다.
>    실물엔 GT 가 없으므로 이 스크립트는 **sim 전용 도구**다(실물 데모는 §3.3/§3.5 경로).

원래 아이디어는 GT-free pick&place 상태머신이었다:

```
HOME → DETECT → PRE_GRASP → REFINE → GRASP → CLOSE → LIFT → TRANSFER(회피) → PLACE → OPEN → 복귀
```

**물체 포즈를 랜덤화하며 이걸 반복 실행하면 그 자체가 데모 생성기다.** Mimic 과 결정적으로 다른 점:

> Mimic 은 사람 데모 **하나를 변환**해 놓고 도달 가능하기를 기대한다.
> 상태머신은 **매 포즈마다 cuMotion 이 IK·충돌을 새로 푼다.** 애초에 실행 불가능한 궤적이 생성되지 않는다.

그리고 전부 **2026-09-06 에 검증된 자산**을 재사용한다 — cuMotion plan+execute(오차 0.0003 rad),
nvblox 실시간 회피(A/B PASS), `gripper_controller`, XRDF 충돌 sphere, Isaac 물체/장애물 스폰.
**Domain randomization 도 Isaac Lab 없이 가능하다** — Isaac Sim 6.0.1 에 `isaacsim.replicator.*` 가 내장.

**정직한 트레이드오프**: 플래너 데모에는 사람 데모의 *스타일·다중모드성*이 없다. 정책이 "플래너처럼"
움직인다. pick&place 수준에서는 수용 가능하지만, 사람다운 다양성이 필요해지면 **같은 LeRobot 스키마로
teleop/freedrive 데모를 섞으면 된다**(스키마가 하나라서 가능한 일 — §2.1). Mimic 도 나중에 옵션으로 남는다.

### 3.3 real 데이터: UR **freedrive** 티칭 (leader/follower 대체)

§6.4 가 "sim 정책은 zero-shot 전이되지 않는다"고 선언한 이상, **쓸 수 있는 정책의 재료는 실물 데이터**다.
그 취득 수단이 이미 손에 있다 — `ur_controllers::FreedriveModeController` (apt 설치 확인됨).

- 팔을 손으로 끌어 시연 → `/joint_states` 기록. **추가 HW 0, 블루투스 지터 0, 사람 궤적 그대로.**
- **그리퍼 개폐만 별도 입력** — DualSense 트리거 또는 펜던트 버튼 → `gripper_controller` 액션.
- ACT 가 민감해하는 "데모 매끄러움"(§6.5) 문제가 게임패드 6-DoF teleop 대비 근본적으로 유리하다.
- 유일한 블로커는 **손목 마운트**(§1) → 제작을 지금 병렬로 시작한다.
- ⚠️ **안전 선행**: 16 kg 가반 900 mm 팔. 속도·영역 제한, 비상정지 동선, 초기에는 저속 확인.

### 3.4 두 데이터 소스의 역할 분담

| | sim (상태머신) | real (freedrive) |
|---|---|---|
| 양 | 많음 (자동, 밤새 생성) | 적음 (사람 시간) |
| 역할 | **파이프라인 검증 + 사전학습/co-train 데이터** | **파인튜닝 → 쓸 수 있는 정책** |
| 스키마 | **동일 (LeRobot)** — 그래서 co-training 가능 | ← |

### 3.5 leader 장치 로드맵 — 키보드/DualSense → OMY leader → GELLO

보유/계획: **키보드·PS5 DualSense**(지금) → **ROBOTIS AI OMY leader**(실물 보유, 시도 예정)
→ 필요 시 **GELLO** 구매. 이들은 **제어 경로가 다르다**:

| 장치 | 경로 | action 출처 |
|---|---|---|
| 키보드 / DualSense | EE twist → MoveIt Servo → `forward_position_controller` (IK 있음) | leader 없음 → `next_state` |
| **OMY leader / GELLO** | **관절 직결** → `forward_position_controller` (**IK 없음** — 아래 실측으로 확인) | **leader 관절값**(ALOHA 관례) |
| UR freedrive(실물) | 명령 없음, 사람이 팔을 끔 | 없음 → `next_state` |

**세 경로 모두 `forward_position_controller` 로 수렴한다**(§2.3). 그래서 T1 에서 만든 스트리밍
컨트롤러가 그대로 재사용되고, 기록기는 `--action-source`(`next_state` | `topic`) 하나로 전부 커버한다.
OMY/GELLO 연결 시 `--action-source topic --action-topic /<leader>/joint_states` 로 바꾸면 끝이고,
**기록기 코드는 손대지 않는다**.

#### ★ OMY ↔ UR16e 기구학 대조 (2026-09-06 실측 — 앞선 추측을 정정)

> **정정**: 여기 원래 "OMY 는 UR16e 와 관절 배치가 달라 **IK 리타게팅이 필요**할 것"이라고 적혀 있었다.
> 실제 모델을 뜯어본 결과 **틀렸다. 관절 직결이 성립한다.**

`robotis_mujoco_menagerie/robotis_omy/omy.xml`(MJCF) 과 우리 `ur16e_sim.urdf.xacro` 의
관절축을 base 좌표계로 환산해 비교:

| # | UR16e (URDF 실측) | OMY (MJCF 실측) |
|---|---|---|
| J1 | **Z (yaw)** | **Z (yaw)** |
| J2 | **Y (pitch)** | **Y (pitch)** |
| J3 | **Y (pitch)** | **Y (pitch)** |
| J4 | **Y (pitch)** | **Y (pitch)** |
| J5 | **Z (yaw)** | **Z (yaw)** |
| J6 | **Y (pitch)** | **Y (pitch)** |

**둘 다 `yaw–pitch–pitch–pitch–yaw–pitch` + 오프셋 손목의 동일 구조.** 링크 길이만 다르다:

| | 상완 | 전완 | 손목 오프셋 |
|---|---|---|---|
| UR16e | 478.4 mm | 360 mm | 174.2 / 119.9 / 116.6 mm |
| OMY | **247 mm** | **219.5 mm** | 121.5 / 113 / 115.5 mm |

OMY 는 UR16e 의 **축소판**(링크 기준 약 절반, reach 기준 약 62% — L100 560 / F3M 580 vs UR16e 900 mm)
— 즉 GELLO 가 말하는 **"축소된 기구학적 동형 복제본"과 같은 관계**다. 그러므로 **IK 리타게팅 불필요**.

⚠️ **관절 한계 — 반드시 `L100`(리더) 기준으로 볼 것** (2026-09-06 공식 사양으로 **정정**)

| # | **OMY-L100 (리더, 우리가 실제 잡는 것)** | UR16e | 판정 |
|---|---|---|---|
| J1 | ±180° (±3.1416) | ±360° (±6.2832) | ⊂ 안전 |
| J2 | **−70° ~ +100°** (−1.222 ~ +1.745) | ±360° | ⊂ 안전이나 **가장 좁음 → 실질 작업영역 제약** |
| J3 | ±180° (±3.1416) | **±180° (±3.14159)** | **사실상 동일 — 마진 0** |
| J4 | ±180° | ±360° | ⊂ 안전 |
| J5 | ±180° | ±360° | ⊂ 안전 |
| J6 | ±180° | ±360° | ⊂ 안전 |
| J7 | −90° ~ +60° (그리퍼 트리거) | 2F-85 로 **별도 스케일 매핑** | — |

> **정정**: 여기 원래 *"`J3` 한계가 OMY **±2.62 rad** ⊂ UR16e ±3.1416 → UR 한계를 넘길 수 없다 = 안전 방향"*
> 이라고 적혀 있었다. 그 ±2.62 rad(=±150°)는 **F3M(팔로워)** 값이고, 우리가 손으로 잡을 **L100(리더)은
> ±180°** 라서 UR16e elbow 한계와 **거의 정확히 같다**. 즉 **"리더가 더 좁아서 안전하다"가 J3 에는 성립하지 않는다.**
> → `omy_to_ur16e` 브리지에서 **관절별 clamp(UR 한계의 ±95% 권장)를 반드시 넣을 것.**
>
> 반대로 **J2 는 −70°~+100° 로 매우 좁다.** 안전 문제는 아니지만 **teleop 으로 도달 가능한 UR 자세가
> 그만큼 줄어든다** — pick&place 시연 자세를 이 범위 안에서 잡아야 한다. (L100 은 F3M 의
> 한계-대-한계 복제본이 **아니다**: F3M 은 J1·J2·J4~J6 ±360°, J3 ±150°.)

사양 출처·상세(액추에이터/질량/관성)는 아래 **「OMY HW 사양 — 공식 데이터 실측」** 참조.

#### 참고 저장소 검토 (2026-09-06)

| 저장소 | 판정 |
|---|---|
| [`ROBOTIS-GIT/open_manipulator`](https://github.com/ROBOTIS-GIT/open_manipulator) | ★ **1순위 — L100 공식 URDF + ros2_control 리더 스택.** Apache-2.0, 2026-08-31 갱신. 아래 §「공식 URDF」 |
| [`charlie8612/lerobot_teleoperator_omy`](https://github.com/charlie8612/lerobot_teleoperator_omy) | ✅ **대안(경량 경로)**. 아래 참조 |
| [`ROBOTIS-GIT/robotis_mujoco_menagerie`](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie) | ⚠️ **F3M(팔로워) 모델** — 리더 근거로 쓰지 말 것(§21 에서 실제로 오류 유발) |
| [`ROBOTIS-GIT/physical_ai_tools`](https://github.com/ROBOTIS-GIT/physical_ai_tools) | ⬜ **채택 안 함, 참고만** |

**`lerobot_teleoperator_omy` = 우리가 필요한 절반이 이미 구현되어 있다**
- LeRobot **Teleoperator 플러그인**, `omy_leader` 로 자동 등록 → `lerobot-teleoperate`/`lerobot-record` 에서 바로 사용
- **ROS 2 불필요** — Dynamixel Protocol 2.0 `GroupSyncRead` 직접 읽기(4 Mbps)
- 모터: J1–3 XH540-W150(ID 1–3), J4–6 XC330-T288(ID 4–6), 그리퍼 XC330-T181(ID 7)
- `get_action()` → `joint_1.pos`…`joint_6.pos` + `gripper.pos`, **라디안**
- `pip install git+https://github.com/charlie8612/lerobot_teleoperator_omy.git`
- ★ README 원문: *"Mapping these onto a **different follower's** joint space is **intentionally left to a
  downstream LeRobot processor**, so this teleoperator stays a generic OMY-L100 reader."*
  → **저자가 follower 매핑을 의도적으로 비워뒀다.** OMY 전용이 아니라 범용 리더 = 우리 상황에 정확히 맞음.

**`physical_ai_tools` 를 안 쓰는 이유**: ROBOTIS 공식 "LeRobot + ROS 2" 스택(`physical_ai_server`,
`physical_ai_manager`, `rosbag_recorder`)이지만 **OMY/AI Worker 중심**이라 UR16e 에 그대로 못 쓴다.
우리는 이미 `il_recorder.py` 로 동등 기능을 검증했다(30 Hz 정확, LeRobot round-trip 완료).
참고 가치는 **그들의 LeRobot 스키마 키 명명·에피소드 관리 방식**을 §2.6 과 대조하는 정도.

#### ★ 공식 URDF — `ROBOTIS-GIT/open_manipulator` (2026-09-06 실측)

**L100 URDF 가 공식으로 존재한다.** Apache-2.0, 660★, 최종 갱신 2026-08-31.

```
open_manipulator_description/urdf/omy_l100/omy_l100.urdf          # 평면 URDF (xacro 불필요)
open_manipulator_description/urdf/omy_l100/omy_l100.urdf.xacro
open_manipulator_description/meshes/omy_l100/{base_unit,link1..link7}.stl
open_manipulator_description/ros2_control/omy_l100_{position,current}.ros2_control.xacro
open_manipulator_bringup/{launch,config}/omy_l100_leader_ai*        # ★ 리더 전용 런치/설정
```

**URDF 로 확정된 것** (F3M 추론이 아니라 **리더 자신의 모델**):

| 항목 | 값 | 의미 |
|---|---|---|
| 축 순서 @base | `+Z +Y +Y +Y +Z +Y` | UR16e 와 **동일** → 관절 직결 재확인 |
| 링크 치수 | 94 / 265.8 / 222 / 52.5 / 46 / 44.5 / 125 mm | **도면(PNG)·STEP 과 완전 일치** (교차검증 3중) |
| **J5 부호** | L100 `+Z` vs UR16e `wrist_2` **`−Z`** | **★ 반전 필요: `q_ur[4] = −q_omy[4]`** |
| 영점 자세 | L100 q=0 = **수직 상방** / UR16e q=0 = **수평 전방** | **J2 오프셋 ≈ −90°** 필요 |
| 가동 관절 | 7 (`joint1..6` + `rh_r1_joint`) | DOF 6+그리퍼 표기 불일치 해소 |

⚠️ **URDF 의 관절 한계는 믿지 말 것** — 전 관절이 일괄 `±180°` 로 적혀 있어
**공식 사양의 `J2 −70°~+100°` 를 반영하지 않는다.** clamp 는 **UR16e 한계**를 기준으로 걸 것.

⚠️ **L100 은 UR16e 의 축소 복제본이 아니다** (GELLO 와 다른 점):

| | 측면(lateral) 오프셋 누적 | 전완 대비 비율 |
|---|---|---|
| UR16e | **+290.7 mm** (wrist_1 +174.1, wrist_3 +116.6) | 0.48 |
| OMY-L100 | **−46 mm** (−52.5, +52.5, −46) | 0.24 |

**크기뿐 아니라 부호까지 다르다.** 따라서 손목 3축(J4/J5/J6)에는 *정답인* 오프셋이 존재하지 않고,
**조작감 기준으로 실물에서 튜닝**해야 한다. (평행축 J2/J3/J4 는 방향만으로는 오프셋이 안 풀린다 —
자세 정합 조건으로 풀면 해가 512개 나온다. 이건 수학적 한계이지 데이터 부족이 아니다.)
→ **IL 데이터에는 영향 없음**: 기록되는 것은 UR16e 자신의 state/action 이고, 리더는 입력장치일 뿐이다.

#### ★ 리더 연결 경로 2가지 — 중력보상 여부가 갈림길

`omy_l100_leader_ai` 설정을 열어보니 L100 은 **수동 암이 아니라 중력보상되는 ros2_control 장치**다
(`gravity_compensation_controller` + `spring_actuator_controller`, effort 명령, `update_rate: 300 Hz`).

| | **A. `lerobot_teleoperator_omy`** | **B. ROBOTIS `open_manipulator` 리더 스택** |
|---|---|---|
| 방식 | Dynamixel Protocol 2.0 직접, **ROS 불필요** | ros2_control + `dynamixel_hardware_interface` |
| 중력보상 | ❌ **없음** | ✅ **있음** (+ 스프링 에뮬레이션) |
| 출력 | `get_action()` dict (ML venv 안) | **`/joint_states` 네이티브** |
| 우리 기록기 연결 | 브리지 추가 필요 | **`--action-source topic` 그대로** (수정 0) |
| 추가 빌드 | 없음 | `dynamixel_hardware_interface`, `robotis_interfaces` + 자체 컨트롤러 3종 |

> **권장: B.** 결정 요인은 **중력보상**이다. IL 은 에피소드 50~100개를 사람이 직접 끌어야 하는데,
> 보상 없는 1.46 kg 암을 계속 들고 있으면 피로가 데이터 품질로 직결된다. 게다가 B 는
> `/joint_states` 를 그대로 뱉어서 **`il_recorder.py` 를 한 줄도 안 고쳐도 된다**(§2.5 의 장치무관 설계가 여기서 회수됨).
> 의존이 소스 2개 + 컨트롤러 3종뿐이라 **기존 vcs+colcon 오버레이 격리 방식에 그대로 들어맞는다**.
> A 는 **폴백**으로 유지 — 실물에서 B 의 중력보상 튜닝이 오래 걸리면 A 로 먼저 파이프라인을 뚫는다.

#### OMY leader 연결 시 실제 할 일

| # | 작업 | 비고 |
|---|---|---|
| 1 | 리더 스택 구축 (**경로 B 권장**) | `ur16e.repos` 에 `open_manipulator` + `dynamixel_hardware_interface` + `robotis_interfaces` 추가 → colcon 오버레이. 폴백은 A(`deps/.venv-ml`) |
| 2 | **부호·오프셋 캘리브레이션** (**URDF 로 대부분 확정됨**) | 확정: **J5 부호 반전**, **J2 −90° 오프셋**. 미확정: 손목 J4/J6 오프셋(구조 비동형 → 조작감 기준 실물 튜닝) + **다이나믹셀 엔코더 영점이 URDF 영점과 일치하는지 1회 확인** |
| 3 | **`omy_to_ur16e` 브리지 노드**(신규) | L100 7관절 → UR16e 관절명 `JointState` 발행 → `forward_position_controller`. **IK 없음** |
| 4 | 안전 게이트 | **관절별 clamp(UR 한계 ±95%) + 속도 상한 + deadman 필수**(16 kg 가반). ★ L100 J3 는 UR elbow 와 한계가 같아 **마진이 없다** — 위 정정 참조 |
| 5 | 기록 | `il_recorder.py --action-source topic --action-topic /omy_leader/joint_states` — **기록기 수정 불필요** |

#### OMY HW 사양 — 공식 데이터 실측 (2026-09-06)

출처 ①  `ur_bringup/docs/robotis/` (사용자 제공): `OMY-L100.pdf` · `OMY-L100.dwg` ·
`OMY-L100.stp`(7.2 MB, CREO `PR44_P08_ASM`, 2025-06-11, **단위 mm**) · `omy_l100_kr_03_layout.png`
출처 ②  <https://docs.robotis.com/docs/systems/omy/specifications/hardware>

| | **OMY-L100 (리더)** | **OMY-F3M (팔로워)** |
|---|---|---|
| DOF | **6** (+그리퍼 J7 = 모터 7개) | 6 |
| Reach | **560 mm** | 580 mm |
| 무게 | **1.46 kg** | 13.5 kg |
| 전원 | 12 VDC | 24 VDC |
| 가반 / 반복정밀도 | – / – | 3 kg / ±0.05 mm |
| 액추에이터 | J1–3 **XH540-W150**, J4–6 **XC330-T288**, J7 **XC330-T181** | J1–2 YM080-230-A099-RH, J3–6 YM070-210-A099-RH |
| 분해능 | −2,048 ~ 2,048 pulse/rev | −262,144 ~ 262,144 pulse/rev |
| 호스트 / 버스 | **U2D2 (USB 2.0)** / TTL Multidrop **4 Mbps** | Ethernet / RS485 4 Mbps |
| 엔드이펙터 | 트리거 핸들 | RH-P12-RN + RealSense D405 |

> ★ **DOF 표기 불일치 주의**: 제품 시트(PNG)는 `DOF 7`, 웹 문서는 `DOF 6`. **둘 다 맞다** —
> 웹은 팔 관절만(6), PNG 는 그리퍼 J7 을 포함(7). 플러그인이 모터 7개를 읽는 것과 일치한다.
> 액추에이터 배치는 **PNG·웹 문서·`lerobot_teleoperator_omy` README 3자가 완전히 일치**(교차검증 OK).

**링크 질량·관성** — 웹 문서에 L100/F3M 전 링크의 질량과 **COG 기준 3×3 대칭 관성텐서**
(Ixx/Iyy/Izz + 곱관성 Ixy/Ixz/Iyz)가 있다. **단위는 문서에 명시됨: 질량 `g`, 관성 `g·mm²`.**
L100 링크 질량(g): J1 197.93 / J2 495.47 / J3 98.66 / J4 26.69 / J5 26.69 / J6 109.13 / J7 4.06.
F3M 링크 질량(g): 2064.88 / 3679.54 / 2386.59 / 1400.23 / 1400.23 / 400.15.
→ **URDF 로 옮길 때 `kg`·`kg·m²` 로 환산 필요** (질량 ×1e−3, 관성 ×1e−9).

> ⚠️ **웹 문서에 DH 파라미터·좌표계 그림·관절 회전방향 규약은 없다.** 그래서 위 표 4번의
> **부호·오프셋 캘리브레이션은 문서로 대체 불가 — 실물로 측정해야 한다.**
>
> 미확인 잔여: `OMY-L100.dwg`(CAD 원본), `.stp` 의 조립 변환행렬(→ L100 링크 길이 추출 가능).
> **관절 직결에는 링크 길이가 무관**하므로 착수 차단 요인은 아니다.

### 3.6 DualSense 위치 (축소)

원안의 6-DoF teleop 주력에서 **그리퍼 개폐 입력 + 보조 teleop** 으로 축소. 유지할 설정:
- 커널 `hid-playstation` 으로 Ubuntu 24.04 네이티브 인식, **USB-C 유선**(블루투스 지터 회피)
- 6-DoF teleop 을 쓸 경우: 저역통과 필터 + 속도 상한 필수

**SpaceMouse 구매 보류 유지.** sim 은 상태머신이, real 은 freedrive 가 커버하므로 구매 근거가 더 약해졌다.

---

## 4. 로드맵 (2026-09-06 개정)

| 단계 | 내용 | 기간 | 산출물 |
|---|---|---|---|
| **−1** | ✅ **ML 환경 구축 완료**(torch 2.11+cu128 sm_120 + lerobot 0.6.1) / ⬜ **손목 마운트 제작(병렬, 미착수)** | — | `setup/setup.sh ml` 로 재현. 마운트는 F′ 선행조건이라 **지금 시작해야 함** |
| **0** | ✅ **완료(2026-09-07)** — 공개 LeRobot 데이터셋으로 **ACT 관통** | 실제 반나절 | `lerobot/svla_so101_pickplace` 500스텝, loss 13.5→2.54. sm_120 실동작·피크 VRAM 6.4 GB 확인. 재현 버그 2개 발견·수정 (`HISTORY.md` §24) |
| **B′** | ~~Isaac Lab 포팅~~ → **`to_do.md` M3 상태머신** (M1/M2 perception 은 **뺌** — `to_do.md` D10) | 단축됨 | pick&place 상태머신. **IL 데이터 엔진 겸함**. pose 는 Isaac GT `/scene/object_pose` |
| **C′** | 상태머신 + **물체 포즈 랜덤화 → LeRobot 데이터셋 자동 생성** | 1~2주 | 자체 sim 데이터셋. 태스크 **3종 × 언어 지시문**(§2.8) |
| **D** | 같은 데이터로 ACT 학습 + closed-loop 롤아웃 | 1주 | **데이터 품질 판정** |
| **E** | **소형 VLA → GR00T LoRA**, ACT와 동일조건 비교 | 2~3주 | 언어 일반화 검증 (§2.7) |
| **F′** | **freedrive 실물 데모** + sim 데이터 co-train → 파인튜닝 | 마운트 완성 후 | **처음으로 "쓸 수 있는 정책"** |

**변경점**
- **A단계(Franka 튜토리얼) 삭제** — Isaac Lab/Mimic 을 쓰지 않으므로 검증할 대상이 없다.
  그 역할(툴체인 관통)은 0단계의 LeRobot 관통이 대신한다.
- **B 는 크게 줄었다** — 2F-85 물리는 **이미 완료**되어 있고(§5), instanceable 변환도 불필요해졌다.
  대신 `to_do.md` M0~M3 가 그 자리에 들어간다(어차피 할 일이었다).
- **F 가 F′ 로 앞당겨졌다** — freedrive 로 leader/follower 없이 실물 데모가 가능하므로(§3.3),
  마운트만 되면 바로 진입한다.

**0단계를 건너뛰지 말 것.** 공개 데이터셋으로 학습이 실제로 도는지 먼저 확인해야, D단계에서 실패했을 때
"데이터 문제"와 "환경 문제"를 분리할 수 있다. 특히 이 머신은 **sm_120** 이라 확인 가치가 크다(§6.3).

---

## 5. UR16e 에셋 점검 — **2026-09-06 실측 완료**

`isaac/assets/ur16e_with_2f85.usd` · `ur16e_2f85_d405.usd` 를 직접 열어 감사한 결과. **대부분 이미 통과했다.**

| 항목 | 실측 결과 | 판정 |
|---|---|---|
| Articulation 정의 | `ArticulationRootAPI` @ `/UR16e/root_joint`, defaultPrim `/UR16e`, metersPerUnit 1.0 | ✅ |
| **Instanceable** | **instanceable prim 0개** | ⚠️ **Isaac Lab 병렬 env 를 쓸 때만 문제** → §3 개정으로 **현재 불필요** |
| 액추에이터 config | 구동관절 7개에 DriveAPI(force) — 아래 표 | ✅ 그대로 이식 가능 |
| 충돌 메시 | `convexDecomposition` 7 + `convexHull` 11, 근사 없는 콜라이더 0 | ✅ |
| **2F-85 부착 + 물리** | **이미 해결됨** — 아래 참조 | ✅ |

**드라이브 게인 (실측값 — `ImplicitActuatorCfg` 등으로 그대로 옮기면 됨)**

| 관절 | stiffness | damping | maxForce |
|---|---|---|---|
| shoulder_pan / shoulder_lift | 1963.78 | 7.855 | 330.0 |
| elbow | 1963.78 | 7.855 | 150.0 |
| wrist_1 / wrist_2 / wrist_3 | 741.39 | 2.966 | 56.0 |
| finger_joint | 20.0 | 1.0 | 26.0 |

### ★ 2F-85 물리 — **이미 처리되어 있다** (개정 전 내용은 틀렸음)

개정 전 이 문서는 "링크를 버리고 prismatic 평행 관절 2개 + mimic 으로 근사"를 권장했다.
**실측 결과, 현재 에셋은 그보다 나은 방식으로 이미 되어 있다:**

- 보조관절 5개에 **`PhysxMimicJointAPI`** 적용, 전부 마스터 `finger_joint` 참조 (gearing ±1.0)
  - `right_outer_knuckle_joint`(−1), `right_inner_finger_joint`(−1), `right_inner_finger_knuckle_joint`(+1),
    `left_inner_finger_knuckle_joint`(+1), `left_inner_finger_joint`(+1)
- **강체 그래프가 완전한 트리** — body1(자식)로 두 번 등장하는 링크 0개 = **폐루프 없음**
- `finger_joint` 한계 0~47°(≈0.82 rad), Robotiq 스펙과 일치
- 세트2/3 sim 에서 개폐·자기충돌·파지가 이미 검증됨 (`HISTORY.md` §8)

즉 이 문서가 근거로 들었던 `Robotiq_2F_140_physics_edit` 와 **같은 종류의 물리 수정본이 이미 우리 에셋에
들어있다.** prismatic 근사로 되돌릴 필요 없고, revolute + mimic 이 파지 물리에 더 유리하다.
➡ **"최대 위험 요소"였던 항목이 사실상 완료 상태.** B단계 "1~2주" 추정은 여기서 크게 줄어든다.

### 참고: Isaac Lab이 기본 제공하는 UR 설정 (Mimic 경로로 돌아갈 경우에만 유효)

`UR10_CFG`(그리퍼 없음), `UR10E_ROBOTIQ_GRIPPER_CFG`(2F-140), `UR3e_CFG`, `UR3e_GRIPPER_CFG`(2F-140), `UR3e_SHORT_SUCTION_CFG`
→ **UR16e도 2F-85도 없음.** 다만 UR16e를 이미 구현해두었으므로 이 단계 비용은 크게 감소.

---

## 6. 알려진 위험 요소

### 6.1 UR 6-DoF 의 구조적 제약 ★ 최우선 감시 (§3 개정으로 **완화**)

- Franka는 7-DoF → null space 여유가 있어 변환된 EE 궤적이 대체로 도달 가능
- **UR은 6-DoF → IK 해가 이산적이고 손목 특이점에 걸림**

이것이 Mimic 을 크리티컬 패스에서 뺀 근본 이유다(§3.1). Isaac Lab 문서: Mimic 후보 데모 성공률이
**간단한 경우 70%, 어려운 경우 1% 미만**이며 "로봇 자체의 복잡도"에 달렸다고 명시.

**개정된 대응**: 상태머신 경로에서는 **cuMotion 이 매 포즈마다 IK·충돌을 새로 풀기 때문에 도달 불가 궤적이
애초에 생성되지 않는다.** 대신 감시 지표가 바뀐다:

- **C′ 단계에서 "샘플링한 물체 포즈 대비 상태머신 성공률"을 반드시 로깅.** 낮으면 데모가 아니라
  **작업 공간 배치**를 고쳐야 한다는 신호다(아래).
- 실패 원인을 `plan 실패` / `grasp 실패` / `충돌` 로 분류해 기록 — 이게 데이터 품질의 1차 방어선.
- **UR16e 는 도달거리 900mm 급 대형 팔**이므로 테이블탑 태스크의 작업 공간 배치를 별도 검토.
  물체 샘플링 영역을 특이점(어깨 위, 완전 신장, wrist_2≈0)에서 떨어뜨려 잡는다.
- Mimic 으로 돌아갈 경우: **cuMotion 을 후보 궤적의 사전 필터**(도달성·충돌 검사)로 쓸 수 있다.
  `cumotion/ur16e_2f85.{urdf,xrdf}`(충돌 sphere 포함)가 이미 있으므로 "생성 후 폐기"가 아니라
  "생성 전 제약"으로 다룰 수 있다.

### 6.2 RTX 5090 (32GB) VRAM 한계

- **ACT** (~80M): 여유 충분 — **실측(2026-09-07, §24)**: 피크 **6,441 MiB / 32,607 MiB**
  (batch 8, 카메라 2대, 480×640). Isaac 이나 perception 과 동시 구동할 여지가 충분하다.
  단 아래 "동시에 돌리지 말 것"은 VRAM 보다 **처리량** 이유로 여전히 유효.
- **소형 VLA** (~0.5B): 여유 있음 → **§2.7 이 이 단계를 넣은 이유가 여기다.** 3B 도박 전에
  언어 조건 검증을 끝낸다
- **GR00T (3B) 전체 파인튜닝**: 어려움. NVIDIA 문서가 최적 성능 기준 H100/L40 노드를 권장, LoRA는 A6000 2장 또는 RTX 4090 2장(=48GB)을 사용했다고 명시. 32GB 단일 카드는 그 아래
  → **LoRA + gradient checkpointing + 배치 축소**가 현실적. 안 되면 클라우드 GPU 단기 임대
- **추론**: 문제없음. 단일 샘플 처리에서 최신 GPU 간 차이 작고, denoising step 4면 충분
- **Isaac Sim과 학습을 동시에 돌리지 말 것.** 데이터 생성 → 학습을 순차 실행.
  (C′ 의 데모 생성은 무인 실행이 가능하므로 밤에 돌리고 낮에 학습하는 식으로 분리)

### 6.3 sm_120 (Blackwell) 호환성 — **이 머신에서 이미 실증됨**

Blackwell은 compute capability 12.0. PyTorch 안정 릴리스가 sm_90까지만 컴파일되던 시기에 `CUDA error: no kernel image is available for execution on the device`가 대량 보고됨. **더 나쁜 경우는 조용한 성능 저하** — GPU가 인식되고 텐서도 올라가는데 호환 모드로 떨어지는 상태.

> **★ 이 경고는 이론이 아니다.** 2026-09-06 이 컨테이너에서 정확히 같은 부류의 문제를 겪었다 —
> apt `nvblox_node` 가 **sm_75 전용 + PTX 폴백 없이** 빌드되어 RTX 5090 에서 기동 즉시
> `cudaErrorInvalidDevice: invalid device ordinal` 로 죽었고, 소스 재빌드로만 해결됐다
> (`HISTORY.md` §14). cuMotion 은 sm_120 을 포함해 무사했다 — **패키지마다 지원 arch 가 다르다.**

**습관화할 것 — 새 이진 배포물을 받으면 먼저 arch 를 확인한다:**
```bash
nvidia-smi --query-gpu=name,compute_cap --format=csv      # 5090 -> 12.0 (= sm_120)
cuobjdump --list-elf <바이너리|.so> | grep -oE 'sm_[0-9]+' | sort -u
cuobjdump --list-ptx <바이너리|.so> | grep -oE 'compute_[0-9]+' | sort -u   # PTX 있으면 JIT 폴백 가능
```

PyTorch 확인 (현재 **torch 미설치**라 아직 실행 불가 — §1):
```python
import torch
print(torch.__version__, torch.version.cuda)
print(torch.cuda.get_arch_list())   # 'sm_120' 포함 확인
```

- 없으면 cu128 이상 휠로 교체
- **Ubuntu 24.04의 apt `nvidia-cuda-toolkit` 설치 금지** (CUDA 12.0이 들어와 Blackwell 사용 불가). NVIDIA 저장소에서 직접 설치.
  이 워크스페이스는 이미 CUDA 13.2 를 NVIDIA 레포에서 설치해 두었다(`SETUP.md` §2-B).
- ⚠️ **apt 로 ML 패키지를 깔 때는 `SETUP.md` §2-B-1 의 핀 정책을 따를 것** — 이 컨테이너는 다른 프로젝트와
  공유된다. NVIDIA 레포가 ROS 패키지를 덮어쓰는 사례를 실제로 확인했다.
- openpi는 JAX 주 백엔드라 Blackwell 지원이 별개 문제 → 막히면 커뮤니티 PyTorch 포트가 우회로

### 6.4 sim-to-real 갭

**Isaac Sim 렌더링으로 학습한 visuomotor 정책은 실물 UR16e에 zero-shot 전이되지 않는다.** VLA는 픽셀을 직접 보는데 sim 렌더와 실제 RealSense 이미지의 도메인 갭이 크다. 도메인 랜덤화·Cosmos 증강이 갭을 줄이지만 없애지는 못한다.

**따라서 시뮬 단계의 산출물은 "쓸 수 있는 정책"이 아니라 "검증된 파이프라인"이다.** 데이터 스키마, 학습 루프, 추론 서버, 청크 실행, 평가 하네스가 맞춰진 상태에서 나중에 데이터 소스만 실물로 교체한다.

### 6.5 데이터 파이프라인 리스크 (개정)

- **LeRobot 데이터셋 writer 를 직접 써야 한다.** Mimic 을 빼면서 HDF5→LeRobot 변환은 사라졌지만,
  대신 상태머신 실행을 LeRobot 포맷으로 **기록하는 코드**가 필요하다. (어차피 Mimic 경로에서도 변환기를
  써야 했으므로 순증이 아니다.) 여기가 예상보다 시간을 먹는 지점이니 **가장 먼저** 만들고 0단계 데이터로 검증할 것.
- **★ 시간 동기화 규약을 스키마와 함께 확정할 것.** §2.8/이 절이 "타임스탬프 정렬"을 데이터 품질 1순위
  용의자로 지목하는데 정작 방법이 미정이었다. 현재 sim 실측값:
  `/clock` 기반, 카메라 ~80 Hz, controller_manager 100 Hz, physics 60 Hz.
  **기록 rate 를 하나 정하고(예: 30 Hz) 관측·액션을 그 그리드에 리샘플**한다. 카메라 프레임과
  `/joint_states` 를 어느 쪽 타임스탬프에 맞출지도 명시.
- **에피소드 성공 판정 + 자동 리셋을 정의할 것.** D단계 "closed-loop 롤아웃"의 성공 기준이 없으면
  §2.8 이 설계한 ACT vs VLA 비교가 성립하지 않는다. 판정 조건(물체가 목표 영역 안 & 그리퍼 열림 & 정지)과
  리셋(`reset_pose.py home` + 물체 재배치)을 하네스로 만들어 둔다.
- **데이터셋 버전 관리/백업 정책.** "잘못 모은 데이터는 되돌릴 수 없다"(§2.1)고 선언한 이상,
  수집 배치마다 버전 태그 + 스키마 해시 + 백업 위치를 남긴다.
- 물리 리플레이는 결정론적이지 않을 수 있음 → **데모는 필요한 것보다 넉넉히 만들고 성공한 것만 사용.**
  상태머신 경로에서는 성공 판정이 자동이라 이 필터링이 공짜다.
- 게임패드 6-DoF teleop은 SpaceMouse·VR보다 데모가 거칠고, ACT는 데모 매끄러움에 민감. GR00T SO-101 튜닝 스레드에 파인튜닝은 됐으나 동작이 덜컹거리고 `denoising-steps`·`action_horizon` 조정으로도 개선이 어려웠다는 보고 존재.
  → **freedrive(§3.3) 가 이 리스크의 직접적 완화책이다.**
- **ROS 2 브리지 관련 커뮤니티 저장소는 거의 전부 SO-100/SO-101 기준**이며 UR 계열 검증 사례 미확인. 개인/연구실 규모라 유지보수 보장 없음.
  → 우리는 **자체 ROS 2 스택이 이미 검증**되어 있으므로(§2.3) 이들 저장소는 참고용에 그친다.
- **라이선스 확인** — GR00T / openpi / 소형 VLA 각각의 상업 사용 조건.
  `to_do.md` 가 SAM3·ultralytics(AGPL) 를 이미 검토한 만큼 일관성 차원에서 착수 전 확인.

---

## 7. 참고 링크

**모델 / 프레임워크**
- [openpi (Physical Intelligence)](https://github.com/Physical-Intelligence/openpi) · [remote inference 문서](https://github.com/Physical-Intelligence/openpi/blob/main/docs/remote_inference.md)
- [Isaac-GR00T](https://github.com/NVIDIA/Isaac-GR00T) · [new embodiment 파인튜닝](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/3_0_new_embodiment_finetuning.md)
- [GR00T N1.5 SO-101 포스트트레이닝 블로그](https://huggingface.co/blog/nvidia/gr00t-n1-5-so101-tuning)

**시뮬레이션**
- [Isaac Lab — Teleoperation and Imitation Learning (Mimic)](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/teleop_imitation.html)
- [Isaac Lab — Augmented Imitation Learning (Cosmos)](https://isaac-sim.github.io/IsaacLab/main/source/overview/imitation-learning/augmented_imitation.html)
- [MimicGen 논문](https://arxiv.org/pdf/2310.17596)

**하드웨어 / ROS 2**
- [ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/)
- [UR RTDE Guide](https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/)
- [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) · [문서](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/)

**ROS 2 ↔ LeRobot 브리지 (참고용, SO-101 기준)**
- [ycheng517/lerobot-ros](https://github.com/ycheng517/lerobot-ros) — Jazzy 테스트, 게임패드 6-DoF teleoperator 포함
- [sacovo/lerobot_ros](https://github.com/sacovo/lerobot_ros) — 토픽 기반 관측/행동, 서비스로 에피소드 관리
- [rosetta](https://discourse.openrobotics.org/t/announcing-rosetta-a-ros-2-lerobot-bridge/50657) — rosbag2 → LeRobot 변환
- [konu-droid/Isaac-GR00T-ros2](https://github.com/konu-droid/Isaac-GR00T-ros2) — GR00T + ROS 2 사례

---

## 7-B. 구현 진행 상황

| 단계 | 상태 |
|---|---|
| **T1. teleop 조종 파이프라인** (Servo + 스트리밍 컨트롤러 + 패드/키보드 + 그리퍼) | ✅ **완료·검증** (2026-09-06, `HISTORY.md` §15) |
| T2. 씬 준비 | 🔶 **진행중** |
| ├ T2-1 **파지** — 접촉 물리 실패 → **D5 fixed-joint attach 로 해결·검증**(물체 +1.62 m 추종, 관절 건전) | ✅ |
| ├ T2-2 씬 요소(작업면·물리 물체·place 마커·랜덤화·`/scene/reset_episode`) | ✅ |
| ├ T2-3 **외부 RGB 카메라** — `/static_cam/color/{image_raw,camera_info}` 추가 + TF 를 `static_cam_tf.launch.py` 로 분리 | ✅ |
| └ T2-5 실물 외부 카메라 런치(sim 과 동일 토픽) | ⬜ |
| T3. **LeRobot 데이터셋 writer** — `il_recorder.py`(raw) + `raw_to_lerobot.py`(변환). 30Hz 정확, action 규약 검증 완료 | ✅ *(실제 LeRobot 변환은 ML 환경 생긴 뒤 검증)* |
| **T3-B. OMY-L100 리더 스택**(경로 B) — 빌드 7개, apt 2개·**업그레이드 0**, mock 으로 컨트롤러 4종 active + `/leader/joint_trajectory` **300 Hz** 검증 | ✅ **완료** (2026-09-06, `HISTORY.md` §22 / `SETUP.md` §2-D) |
| ├ **`omy_to_ur16e` 브리지** — J5 반전 + J2 −90° + UR 한계 ±95% clamp + slew + **engage 게이트** + watchdog. 합성 입력 **7/7 검증** | ✅ **완료** (`scripts/omy_to_ur16e.py`) |
| ├ **Isaac sim 실기동 검증** — `virtual_omy_leader.py` + `teleop_omy.launch.py` 로 전 구간. **전체 추종오차 0.244°**(브리지 매핑 0.021°), 그리퍼 연동, disable 즉시정지, trajectory 복귀 | ✅ **완료** (`HISTORY.md` §22) |
| └ 실물 L100 연결 후 손목 J4/J6 오프셋·엔코더 영점 튜닝 | ⬜ *(U2D2 연결 필요 — 파라미터만 조정, 코드 수정 불필요)* |
| **T3-C. 0단계 — 공개 데이터셋 ACT 관통** — `svla_so101_pickplace` 500스텝(loss 13.5→2.54), sm_120 실동작, 피크 VRAM 6.4 GB. `lerobot[training]` 누락 + `/dev/shm` 64 MiB 함정 발견·수정 | ✅ **완료** (2026-09-07, `HISTORY.md` §24) |
| **T3-D. pick&place 상태머신** `scripts/pick_place_demo.py` — GT pose(`/scene/object_pose`) → cuMotion + **직선 접근/후퇴**(`compute_cartesian_path`) → 2F-85. TCP 는 TF 로 실측. **사람 없이 파이프라인을 끝까지 돌리는 구동기** | ✅ **sim 검증 완료** (2026-09-07, 1/1 SUCCESS, 마커에서 14 mm — `HISTORY.md` §25) |
| └ ⚠️ **런치는 반드시 `ur_only:=false`** — 기본 `true` 면 move_group 이 그리퍼 없는 모델을 써서 **모든 Cartesian goal 이 SUCCESS 를 반환하며 무동작**한다 | — |
| **T4-A. 파이프라인 관통** — 상태머신 → `il_recorder` → `raw_to_lerobot` → LeRobot v3.0 → ACT 학습. 4 에피소드/7,136프레임, 재판독 시 §2.6 스키마 그대로, ACT loss 15.3→3.1 | ✅ **완료** (2026-09-08, `HISTORY.md` §25) |
| T4-B. 대량 수집 + closed-loop 롤아웃 | ⬜ *선행: **태스크 3종 × 언어 지시문**(§2.8 — 지금은 1종이라 VLA 가 언어를 무시한다), 에피소드의 정지 프레임 정리(현재 1,686프레임≈56초 중 상당수가 대기)* |
| T5. (선택) 소형 VLA → **GR00T N1.7** | ⬜ *lerobot 0.6.1 내장 확인(§2.7) — 별도 레포 불필요* |

> ### ★ IL/VLA 스택에 **필요 없는 것** (2026-09-06 확정)
> 정책 입력은 **RGB + 관절뿐**이다. 따라서 teleop→IL→VLA 경로는 아래를 **쓰지 않는다**:
> - **nvblox** (ESDF 장애물 매핑) — 정책은 depth 도 ESDF 도 안 본다
> - **cuMotion / MoveIt 을 추론 루프에** — 정책이 직접 관절 목표를 낸다 (§2.3)
> - **foundation perception**(FoundationStereo/SAM3/FoundationPose) — 그건 `to_do.md` 의
>   *고전적* pick&place 스택용이다. IL 정책은 픽셀에서 바로 행동을 낸다
>
> **IL 최소 기동 (4개면 끝)**:
> ```
> ① Isaac  --scene pick_place --table --grasp-attach --with-camera --with-static-cam
> ② 제어    ur16e_2f85_d405.launch.py use_sim:=true
> ③ TF      static_cam_tf.launch.py            ← 그래서 nvblox 런치에서 분리했다
> ④ teleop  teleop_servo.launch.py + teleop_dualsense.launch.py
> ```
> MoveIt/cuMotion 은 **에피소드 리셋과 안전 봉투**로만 필요하면 곁들인다(§2.3). nvblox·cuMotion 은
> `to_do.md` 의 scripted pick&place 와 (나중에) RL 쪽에서 계속 쓰인다 — 버리는 게 아니라 **트랙이 다르다.**

> **T1 에서 확정된 규약** (T3 스키마와 직결):
> - 액션 경로 = `/forward_position_controller/commands`(Float64MultiArray, 6관절, yaml 의 `joints` 순서)
> - 그리퍼 = `finger_joint` 하나, `GripperCommand` 액션, 0.0 열림 ~ 0.8 닫힘
> - **에피소드 리셋 자세 = `ready`** (`home` 은 팔꿈치 특이점이라 teleop/Servo 가 거부)
> - teleop 은 **deadman 유지 중에만** 동작 → 데모 기록 시 deadman 해제 구간은 "정지" 프레임으로 남는다.
>   기록기에서 잘라낼지 남길지 T3 에서 결정할 것.

---

## 8. 다음 액션 (2026-09-06 개정)

1. **손목 마운트 제작 착수 (병렬, 지금)** — 리드타임이 가장 길고 F′ 의 유일한 블로커.
   **§2.5 의 명목 치수 `xyz=(0,-0.067,0.01847)`, pitch 8° 로 제작**하거나, 다르면 URDF·Isaac 스크립트를
   함께 갱신하고 USD 재베이크.
2. **ML 환경 구축** — torch(sm_120 포함 휠) 설치. `SETUP.md` §2-B-1 핀 정책 준수(공유 컨테이너).
3. ~~**0단계 착수**~~ → **완료(2026-09-07, `HISTORY.md` §24).** sm_120 실동작 확인, 피크 VRAM 6.4 GB.
   학습 실행 시 **`UR_WS_TORCH_SHM_FIX=1`** 를 붙일 것(`/dev/shm` 64 MiB 함정 — `SETUP.md` §2-C).
4. **LeRobot 데이터셋 writer + 시간 동기화 규약 확정** (§6.5) — 데이터를 모으기 전에 만든다.
5. **태스크 3종 정의** — 언어 지시문 포함, §2.8 참조. `to_do.md` M0 의 물체/exemplar 선정과 함께 정한다.
6. **`to_do.md` M0~M3 진행** — 완료 시점이 곧 C′ 의 데이터 엔진 완성 시점.

~~기존 UR16e 에셋 점검~~ → **완료(2026-09-06, §5).** 2F-85 물리는 이미 해결되어 있었고,
instanceable 미변환은 Isaac Lab 을 쓰지 않기로 하면서 무의미해졌다.