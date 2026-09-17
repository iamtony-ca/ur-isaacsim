# GELLO(`gello_software`) 대조 검토 및 벤치마크 항목 (2026-09-17)

> 대상: [`wuphilipp/gello_software`](https://github.com/wuphilipp/gello_software) HEAD `204f53a`
> (`ur_ws/tempp/gello_software`, repo 밖 참고 클론 — `ros2/`(FR3 ROS 2 판)·`gello/factr/`(중력보상) 포함 전부 읽음).
> 우리 쪽: `scripts/omy_to_ur16e.py`·`il_recorder.py`·`omy_leader_calib.py`·`teleop_omy.launch.py`·`CHECKLIST.md` E/H.
> 실행 이력·재현 로그는 `HISTORY.md` §49. 이 문서는 **판정과 남은 BM 항목**만 유지한다.

## 1. 결론 한 줄

전제는 같다(관절 1:1 affine 매핑, IK 없음). 안전장치는 우리가 더 많다. **GELLO 가 하는 핵심 하나 —
매핑 후 팔로워 명령을 action 으로 기록 — 를 우리는 문서와 달리 실제로 못 하고 있었다**(결함, 수정).
그 외 GELLO 의 장점 3건을 반영했고, 각각 "동작한다"와 "이득이다"를 분리해서 검증했다(§3).

## 2. 대조표

| 항목 | GELLO | 우리 (OMY-L100 → UR16e) | 판정 |
|---|---|---|---|
| 리더 → 팔로워 매핑 | `q = sign·(raw − offset)`, offset 은 π/2 배수 | `q_ur = sign·q + offset`, 연속값 | 동등 |
| 오프셋 캘리브 | `gello_get_offset.py`: 알려진 자세에서 π/2 격자 브루트포스 + `start_joints` 로 ±2π wrap | `omy_leader_calib.py --mode match`: 팔로워를 눈으로 따라 잡고 잔차 최소 오프셋 산출, `check`/`verify` | 우리가 OMY(ros2_control, rad) 에 맞음 |
| 정렬(engage) | 팔로워가 리더에게 **접근**(스텝당 0.05 rad × 25, 초기 차 0.8 rad 초과면 거부), 직선 보간, 충돌검사 없음 | 고정 랑데부(`/sync`, MoveIt 충돌검사) + engage 게이트 0.15 rad | **채택** → `/sync_to_leader`(§3-③) |
| 운전 중 안전 | 정렬 후 매 사이클 \|action − state\| > 0.5 rad 면 `exit()` | 상시 slew(`max_joint_speed`)·관절 clamp 0.95·워치독 0.5 s·데드맨·engage 게이트 | **채택** → 리더 속도 가드(§3-②) |
| 리더 필터 | EMA α = 0.99 (사실상 없음) | 없음 | 동등 |
| 리더 스프링/댐퍼 | ROS 2 FR3 판만: Dynamixel 내부 PID(kp 280 ≈ 0.25 Nm/rad, 600 mA 제한) | ROBOTIS `gravity_compensation` + `spring_actuator` 컨트롤러(300 Hz), E-1 ⑤ 드리프트 점검 | 이미 커버 |
| 리더 통신 | 57600 baud → 25 Hz 상한, 1 Mbps → 100 Hz, `latency_timer=1` 권장 | 300 Hz, `setup.sh udev` 가 `latency_timer=1`, `check_env.sh` 검사 | 이미 커버 |
| 실물 UR 경로 | `ur_rtde servoJ` 500 Hz, lookahead 0.2 s, gain 100, 그리퍼는 URCap 소켓 63352 | `ur_robot_driver` `forward_position_controller` 100 Hz(드라이버 내부 servoj), 그리퍼 `GripperCommand` + deadband | **실물 BM 항목**(§5) |
| 데이터 기록 | `action = agent.act(obs)` = 매핑 후 명령, 프레임마다 pickle, 마지막 5 프레임 폐기, 30 프레임 미만 스킵, pygame 창 S/Q | `il_recorder.py` 서비스/패드, 30 Hz JPEG+JSON, `action_source next_state|topic`, `min_episode_frames 10` | **결함 수정**(§3-①) + `trim_tail_frames`(§3-④) |
| 시뮬 | MuJoCo menagerie(`sim_ur`) | Isaac + `virtual_omy_leader.py` | 범위 밖 |
| 양팔 / Quest / SpaceMouse / FACTR 힘반영 | 있음 | 없음(OMY 스택에도 없음) | 범위 밖 |

## 3. 반영한 4건 — "동작하는가" 와 "이득인가"

### ① [결함 수정] action 기록 토픽 = `/omy_bridge/command_joint_states`

- **문제**: 문서(`CHECKLIST` H·`HARDWARE` 4-B·`plan_il_vla` §3.5)가 `action_topic:=/leader/joint_states` 를 지시했지만
  그 토픽은 이름이 `joint1..6`(UR 이름 아님), 값은 매핑 전. 기록기는 "saved" 를 보고하고 action 은 전부 null,
  변환기는 `None + None` 으로 사망. 재현으로 확인(§49.1).
- **수정**: 브리지가 매 tick 실제 보낸 명령(매핑·clamp·slew 후)을 UR 이름 JointState 로 발행, 기록기는 UR 팔 관절이
  없는 action 토픽이면 시작 거부·null action 이면 저장 거부.
- **동작**: ✅ 기록 → null 0 → 변환 성공(§49.3 T1).
- **이득인가**: 필수(없으면 리더 action 기록 자체가 불가). 다만 **"리더 명령 action 이 `next_state` 보다 낫다"는
  사실은 이 워크스페이스에서 입증되지 않았다.** Isaac 실측(§49.4): 명령이 state 를 앞서는 양은 **1 프레임(33 ms),
  잔차 0.028°** — 프레임당 이동 0.139° 의 20%. 즉 **sim 에서는 topic action ≈ next_state**. 둘이 갈라지는 건 실물의
  추종 지연·접촉으로 팔이 못 따라갈 때이고(ALOHA/GELLO 가 리더 값을 쓰는 이유), 그 효과는 실물에서 재야 한다(§5).

### ② [안전] 리더 속도 가드 `max_leader_speed` (기본 20 rad/s)

- **처음 설계는 틀렸다**: GELLO 처럼 **절대 각도** 0.5 rad/샘플로 만들었더니, 벽시계 리더의 3.1 rad/s 정상 동작에서
  300 Hz 스트림이 PC 쪽에서 0.16 s 끊긴 순간 29° 로 **오탐**(§49.4). 사람은 ≤ 5 rad/s, 엔코더 wrap 은 수백 rad/s 라
  **속도(Δq/Δt, 수신시각, Δt ≥ 1 ms)** 가 옳은 기준.
- **동작**: ✅ 3.1 rad/s 정상 동작 오탐 없음 / 0.3 s 끊김 후 재개 오탐 없음 / 글리치 1샘플(115° in 3 ms = 621 rad/s) 차단,
  status `leader_jump` 유지 → 재`enable` 은 engage 게이트가 판단.
- **오탐 2(sim 시간)**: `use_sim_time` 에서 같은 `/clock` 틱의 두 샘플이 Δt 0 → 1.2° 가 20 rad/s 로 판정. Δt 는 벽시계로,
  스텝은 `min_leader_jump` 0.1 rad 이상이어야 판정(§49.5). 실물엔 무관, sim 회귀를 위해 필요.
- **이득인가**: ✅ 실제 구멍을 막는다 — 워치독은 "발행 중단"만 잡고, 값 튐은 slew 가 `max_joint_speed` 로 충실히
  따라갔다. 비용은 파라미터 1개. 한계: 끊김(< 0.5 s) 직후에 글리치가 겹치면 속도가 희석돼 놓칠 수 있다(그 경우
  slew 상한이 최후 방어).

### ③ [편의] `/omy_bridge/sync_to_leader`

- **동작**: ✅ mock+MoveIt: 팔로워가 리더 매핑 자세로 이동 후 engage 성공, `/sync` 회귀 통과(§49.3 T2).
- **GELLO 보다 나은 점 검증**: 리더를 **자기충돌 자세**(UR `[0,−90,+170,−90,0,0]°`)에 두고 호출 → MoveIt 이
  `wrist_2_link`–`upper_arm_link` 접촉으로 계획 거부 → `sync_failed`, **팔은 0° 도 안 움직임**(§49.4). GELLO 의
  직선 보간은 그대로 갔을 자세.
- **이득인가**: 캘리브/튜닝 중 "리더를 랑데부에 맞춰 놓는" 수고를 없앤다 — 조작 편의이지 데이터 품질 이득은 아니다.
  **수집은 여전히 `/sync`**(에피소드 시작 자세 통일, `plan_il_vla` §2.6).

### ④ [실험용] `il_recorder.py trim_tail_frames` (기본 0)

- **동작**: ✅ `ros2 param set` 으로 5 → 프레임/JPG 104=104, meta 기록(§49.3 T1). stop 시점에 읽어 조용한 no-op 없음.
- **이득인가**: **미검증.** 이 컨테이너에 사람 데모 raw 가 없다(수집은 전부 스크립트 `pick_place_demo`). 근거는
  §46 의 실패 양상(사전파지 높이 **정지**)과 "정지 버튼을 누르는 꼬리는 항상 정지"라는 추정뿐. 그래서 기본 0 이고,
  **`scripts/il_tail_stats.py <raw_dir>`** 로 데이터셋의 꼬리 정지 프레임 수를 먼저 재고(중앙값 ≈0 이면 무의미,
  10+ 이면 시도) A/B 로 판단한다.

## 4. 반영하지 않은 것과 이유

| 항목 | 이유 |
|---|---|
| Dynamixel 내부 PID 스프링/댐퍼 | ROBOTIS 컨트롤러가 담당(중력보상 포함). E-1 ⑤ 로 드리프트만 확인 |
| π/2 격자 오프셋 | OMY 는 ros2_control 이 rad 로 이미 처리, 우리 `match` 가 상위호환 |
| baud/`latency_timer` | 300 Hz·udev·check_env 로 이미 있음. 단 BM 항목으로 실측(§5 ③) |
| 리더 EMA 필터 | GELLO 도 사실상 없음(α 0.99). 필요해지면 브리지에 1차 IIR 1줄 |
| FACTR 힘반영(팔로워 토크 → 리더) | OMY 리더 스택에 인터페이스 없음, 실물 이후 별도 과제 |
| 양팔·Quest·SpaceMouse·MuJoCo | 범위 밖 |

## 5. 실물 벤치마크 항목 (UR16e 도착 후, `HARDWARE.md` 4-B 시점)

GELLO 수치를 기준점으로 삼는다. 측정 도구는 이미 있는 것으로 충분하다(`/omy_bridge/command_joint_states` vs
`/joint_states` 를 같이 구독 — §49.4 의 lag probe 방식, `omy_leader_calib.py --mode check`).

| # | 항목 | GELLO 기준점 | 우리 측정 방법 | 합격선(제안) |
|---|---|---|---|---|
| ① 추종 지연 | `servoJ` 500 Hz, lookahead 0.2 s, gain 100 | 리더 사인(0.25 rad, 8 s) 중 명령 vs state 최적 k (sim: 1 프레임/0.028°; **headless Isaac 은 실시간 3.8배라 sim 절대값은 무의미**, §49.5) + 브리지 `slew capped N%` 경고로 `max_joint_speed` 병목 여부 | k ≤ 3 프레임(100 ms), 잔차 < 0.5°, cap ≤ 2.0 rad/s 에서 보호정지 없음 |
| ② 오버슈트/진동 | (수치 없음) | 리더 스텝 0.2 rad 후 state 의 최대 초과량·정착 시간 | 초과 < 5 %, 정착 < 0.5 s |
| ③ 리더 발행률·잡음 | 25/50/100 Hz(baud 별), `latency_timer=1` | E-1 ①: `ros2 topic hz /leader/joint_states`, 정지 시 관절 std | ≥ 250 Hz, std < 0.01 rad |
| ④ 그리퍼 반응 | URCap 소켓, 매 사이클 `move(pos,255,10)` | 트리거 스텝 → `finger_joint` 도달 시간(`GripperCommand` preempt 포함) | < 0.5 s |
| ⑤ action vs next_state 괴리 | (GELLO 는 항상 리더 값) | 실물 에피소드에서 \|action[t] − state[t+1]\| 분포, 접촉 구간 분리 | 갈림 구간이 있으면 topic action 채택 근거 |
| ⑥ 꼬리 정지 | 마지막 5 프레임 폐기 | `il_tail_stats.py` 중앙값 | 10+ 프레임이면 `trim_tail_frames` A/B |

## 6. 검증 상태 요약

| | 동작 | 이득 |
|---|---|---|
| ① action 토픽 | ✅ T1 | 필수(결함). "리더 action > next_state" 는 sim 에서 **차이 없음**, 실물 ⑤ 로 판단 |
| ② 속도 가드 | ✅ 오탐 2종 없음 + 글리치 차단 | ✅ (초안의 절대각 기준은 오탐 — 폐기) |
| ③ sync_to_leader | ✅ T2 + 충돌 거부 | 편의 ✅, 데이터 품질 무관 |
| ④ trim_tail_frames | ✅ | **미검증**, 기본 0, `il_tail_stats.py` 로 먼저 잰다 |
