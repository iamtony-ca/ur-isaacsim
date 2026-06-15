# QnA — UR16e sim+real 스택

개념 이해를 돕는 질문/답변 모음. (구성/실행은 [`README.md`](../README.md), 재현은 [`SETUP.md`](SETUP.md))

---

## Q1. `ur16e.launch.py` 와 `ur16e_moveit.launch.py` 의 차이는?

계층이 다릅니다 — `ur16e.launch.py` = *로봇+제어(토대)*, `ur16e_moveit.launch.py` = 그 위의 *모션 플래닝(두뇌)*. 보통 **둘 다** 실행.

| | `ur16e.launch.py` | `ur16e_moveit.launch.py` |
|---|---|---|
| 역할 | **로봇 + ros2_control 백엔드** | **MoveIt2 모션 플래닝 + RViz** |
| 띄우는 것 | robot_state_publisher, ros2_control_node(또는 실물 ur_robot_driver), 컨트롤러 스포너 | move_group(플래닝) + RViz MotionPlanning |
| `use_sim` 분기 | **있음** (sim=topic_based / real=ur_robot_driver) | 없음 — `use_sim`은 `use_sim_time`만 결정 |
| 내부 | 직접 노드+컨트롤러 (우리 패키지) | 공식 `ur_moveit_config/ur_moveit.launch.py` 래퍼 |
| `follow_joint_trajectory` | **노출**(컨트롤러) | 그 액션을 **호출**해 계획 궤적 실행 |
| 단독 실행 | 가능(직접 trajectory) | **불가** (컨트롤러/`/joint_states` 선행 필요) |
| 필수 | 필수(먼저) | 선택(그 다음) |

```
ur16e.launch.py        →  로봇 + 컨트롤러 + /joint_states   [필수, 먼저]
        ↑ follow_joint_trajectory 액션
ur16e_moveit.launch.py →  move_group 이 그 액션으로 실행     [그 다음]
```

---

## Q2. nav2 기반 navigation system 과 비교하면? (이동 ↔ 매니퓰레이션)

nav2 navigation 스택에 익숙하다면, 매니퓰레이션 스택을 다음과 같이 거의 1:1 로 대응시켜 이해하면 쉽습니다.

> 한 줄: nav2가 *"베이스를 어디로 어떻게 굴릴지"* 계획·제어하듯, MoveIt은 *"팔을 어떤 관절 궤적으로 움직일지"* 계획·실행한다. 둘 다 아래에 **"로봇+ros2_control 토대"** 가 있어야 동작.

### 컴포넌트 대응

| 계층 | Nav2 (이동) | MoveIt (매니퓰레이션) | 어느 런치 |
|---|---|---|---|
| 토대: 하드웨어+제어 | 베이스 드라이버 + ros2_control + `diff_drive_controller`(→`/cmd_vel`), robot_state_publisher, `/odom`·`/joint_states`·TF | ros2_control + `scaled_joint_trajectory_controller`, robot_state_publisher, `/joint_states`·TF | **`ur16e.launch.py`** |
| 계획(global planner) | `planner_server` (NavFn/Smac) | `move_group` 의 모션 플래너 (OMPL/Pilz) | `ur16e_moveit.launch.py` |
| 세계 표현(충돌) | costmap (global/local) | **planning scene** | `ur16e_moveit.launch.py` |
| 추종/실행 | `controller_server` (DWB/RPP) → `/cmd_vel` | 궤적 시간화 후 `follow_joint_trajectory` 로 JTC 추종 | 계획=moveit / 추종=`ur16e.launch.py` JTC |
| 오케스트레이션 | `bt_navigator` (behavior tree) | `move_group` 액션 서버 | `ur16e_moveit.launch.py` |
| 목표 명령(액션) | `/navigate_to_pose` | `/move_action` | — |
| GUI | RViz Nav2 패널 + "2D Goal Pose" | RViz MotionPlanning 패널 + 인터랙티브 마커 | `ur16e_moveit.launch.py` |

### 계층 매핑
```
[nav2 세계]                              [매니퓰레이션 세계]
robot bringup (base + ros2_control)  ↔  ur16e.launch.py        ← 토대(필수)
   └ /cmd_vel 로 바퀴 구동                 └ follow_joint_trajectory 로 팔 구동
nav2_bringup (planner+controller+BT) ↔  ur16e_moveit.launch.py ← 두뇌(선택)
   └ /navigate_to_pose                    └ /move_action
```
nav2도 베이스 bringup이 먼저 떠야 그 위에 올라가듯, 우리도 `ur16e.launch.py` → `ur16e_moveit.launch.py` 순.

### 핵심 개념 차이 (혼동 주의)
1. **계획 공간**: nav2 = 2D 평면 `SE(2)`(x,y,θ) on costmap / MoveIt = 6관절 **configuration space**(또는 3D Cartesian+IK) + 풀 3D 충돌 지오메트리.
2. **추종 방식**: nav2 `controller_server`는 **온라인 로컬 플래너**라 매 주기 `/cmd_vel` 재계산(폐루프 재계획). MoveIt은 보통 **전체 궤적을 한 번 계획**해 JTC가 추종(온라인 재계획은 MoveIt Servo 영역).
3. **"controller" 용어 주의**: nav2의 controller_server(로컬 플래너) ≠ ros2_control의 controller. ros2_control 의 `scaled_joint_trajectory_controller` 는 오히려 nav2 의 **`diff_drive_controller`**(저수준 추종)에 대응.

### sim/real 스왑도 동일
nav2에서 베이스 하드웨어만 (실물 ↔ Gazebo/Isaac) 바꾸고 nav2 스택은 그대로 두듯, 우리도 `ur16e.launch.py` 의 `use_sim` 으로 하드웨어만 교체하고 MoveIt 은 sim/real 동일하게 올림.

> 정리: **`ur16e.launch.py` ≈ "베이스 bringup + diff_drive_controller"**, **`ur16e_moveit.launch.py` ≈ "nav2_bringup(planner+costmap+BT+NavigateToPose)"**.

---

## Q3. UR 을 RTDE 로 직접 제어(moveJ/moveL)하는 것과 MoveIt 방식은 뭐가 다른가?

핵심은 **"계획(planning)이 어디서, 무엇을 하느냐"**. moveJ/moveL 은 "직접 제어"처럼 보이지만 실제로는 **로봇 컨트롤러(PolyScope/URControl) 내부의 점-대-점 모션 생성기**를 호출하는 것.

### 직접 제어 (ur_rtde 라이브러리 moveJ/moveL — MoveIt/ros2_control 없이)
```
[앱] --(목표 pose/관절)--> [RTDE 클라이언트 노드] --RTDE--> [UR 컨트롤러]
                                                            └ moveJ/moveL 내부 실행
```
- `rtde_control.moveJ/moveL(target)` → **UR 펌웨어가 직접** IK(moveL) + 사다리꼴 속도 프로파일 + 관절 이동.
- "지능"이 **로봇 안**에 있음. ROS 는 목표 하나만 던짐. 단순·저지연·의존성 적음. 하지만:
  - moveJ=관절 직선, moveL=Cartesian 직선 → **경로 고정**, 중간 장애물 있으면 **그대로 충돌**.
  - 환경/자기충돌/부착물 모델 없음. **UR 전용**(RTDE/URScript 종속, sim·타로봇 이식 불가).
- → 작업 공간이 알려져 있고 자유공간에서 단순 점-대-점이면 이 방식으로 충분(충돌회피·복잡경로 불필요).

### MoveIt 방식 (현재 표준 스택)
```
[내 앱] --(목표 포즈/관절)--> [move_group: OMPL 플래너 + planning scene]
                                   └ 충돌 없는 전체 궤적 생성
        --(follow_joint_trajectory)--> [JTC / ur_robot_driver] --> [UR] └ 궤적 추종
```
"지능"이 **ROS(PC)** 로 이동, 로봇은 궤적 추종자. 추가로: ① 충돌 회피 경로계획(OMPL) ② planning scene(테이블/부착물/점군) ③ 제약 있는 IK ④ 하드웨어 독립(sim/real/타로봇) ⑤ pick&place 등 상위 파이프라인.

### 비교

| | RTDE moveJ/moveL 직접 | MoveIt |
|---|---|---|
| 계획 위치 | **로봇 펌웨어 안** | **ROS move_group** |
| 경로 | 관절/Cartesian 직선(고정) | 충돌 없는 임의 경로 |
| 충돌 회피 / 환경 인식 | ❌ | ✅ (planning scene) |
| 이식성(sim/타로봇) | ❌ UR 전용 | ✅ 표준 인터페이스 |
| 복잡도/지연 | 낮음·단순 | 높음·무거움 |
| 적합 작업 | 자유공간 점-대-점 | 클러터/장애물/복잡 조작 |

### 오해 풀기
- MoveIt 은 moveJ 를 **대체가 아니라 그 위에 "계획 두뇌"를 얹는 것**. 결국 MoveIt 도 관절 궤적을 만들어 로봇에 흘려보냄(실물 UR 은 ur_robot_driver 가 External Control 로 servoJ류 스트리밍).
- 새 스택에서도 **"직접 moveJ류"는 여전히 가능**: MoveIt 없이 `follow_joint_trajectory` 직접 전송(데모가 그 방식 ≈ moveJ), 또는 `forward_position_controller`/`passthrough_trajectory_controller` 사용. **충돌 회피 불필요하면 MoveIt 레이어 생략 가능**, ros2_control 토대(`ur16e.launch.py`)만으로 점-대-점 충분.

> 정리: 직접제어 = "로봇 내부 모션 생성기를 RTDE로 직접 호출"(단순·UR전용·충돌무시), MoveIt = "ROS에서 충돌 없는 궤적 계획 → 표준 컨트롤러로 실행"(이식성·충돌회피·생태계). 작업 성격에 따라 선택.
