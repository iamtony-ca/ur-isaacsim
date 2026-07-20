# QnA — UR16e sim+real 스택

개념 이해를 돕는 질문/답변 모음. (구성/실행은 [`README.md`](README.md), 재현은 [`SETUP.md`](SETUP.md))

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

---

## Q4. UR16e 에 Robotiq 2F-85 그리퍼를 붙이고 MoveIt 이 충돌까지 인식하게 하려면? (개념)

팔 단독 세트(`ur16e_*`)와 병렬로 그리퍼 세트(`ur16e_2f85_*`)를 새 파일로만 추가했다. 핵심 개념 4가지.

### ① 그리퍼는 관절 1개만 제어 — 나머지는 mimic
2F-85 는 평행 4절 링크라 **실제 자유도(DOF)는 `finger_joint` 1개**뿐이고 나머지 5관절은 그것에 종속된다.
- **ROS 쪽**: ros2_control 은 `finger_joint` 만 명령/상태 교환. URDF 의 나머지 관절은 `<mimic joint="finger_joint">`
  로 선언 → robot_state_publisher 가 `finger_joint` 값으로 TF 를 계산(그래서 RViz 에서 양손가락이 평행으로 움직임).
- **Isaac 쪽**: USD 의 나머지 관절에 `PhysxMimicJointAPI` 가 걸려 있어 물리적으로 자동 추종.
- 그래서 `gripper_controller`(`position_controllers/GripperActionController`)는 `GripperCommand` 액션으로
  `finger_joint` 위치(0=open … ~0.8=close) 하나만 보낸다. (참고: robotiq_description 매크로의 원본 master 는
  `robotiq_85_left_knuckle_joint` 라 그대로 쓰면 우리 `/joint_states` 로 안 움직임 → master 를 `finger_joint` 로 다시 명명)

### ② URDF 이름과 SRDF 이름이 같아야 SRDF 가 적용된다 (왜 `ur16e`)
MoveIt 은 로봇 모델을 만들 때 **URDF(`robot_description`)** 와 **SRDF(`robot_description_semantic`)** 를 합치는데,
**둘의 `<robot name="X">` 가 동일해야** 합친다. 다르면 *"Semantic description is not specified for the same robot
as the URDF"* 경고와 함께 **SRDF 를 통째로 무시**한다.

SRDF 의 핵심은 `disable_collisions`(인접 링크 충돌 무시 규칙). UR 팔은 인접 링크가 원래 맞닿아 있어서, SRDF 가
무시되면 **현재 자세가 곧 충돌**로 잡혀 플래닝이 `error_code -10 (START_STATE_IN_COLLISION)` 로 거부된다.
→ 그리퍼 변형 URDF 라도 `<robot name>` 을 ur_moveit_config SRDF 의 이름인 **`ur16e`** 로 둬야 SRDF 가 적용된다.

### ③ "collision 반영" = 그리퍼 collision 메시 + 그리퍼용 disable 규칙을 함께
그리퍼 형상을 MoveIt 이 충돌 계산에 쓰려면 URDF 에 **collision 메시**가 있어야 한다(`robotiq_2f85_macro.xacro
collision:=true`). 하지만 팔 전용 SRDF 에는 그리퍼 링크용 disable 규칙이 없어서, 그냥 켜면 그리퍼 내부 링크끼리·
그리퍼와 장착 손목이 충돌로 오인돼 또 `-10`. 그래서 전용 SRDF(`srdf/common/ur16e_2f85.srdf.xacro`)가
- **그리퍼 내부 모든 쌍 + 그리퍼↔손목/tool0** → `disable_collisions` (시작자세 오인 방지)
- **그리퍼↔팔 몸통(upper_arm/forearm/shoulder/base) + 환경** → 그대로 검사 (진짜 충돌은 잡음)

이 SRDF 를 move_group 에 넣는 게 `ur16e_2f85_moveit.launch.py`(공유 `ur16e_moveit.launch.py` 와 별개).
같은 패턴이 **GRP-ES-CPL-077 커플링**(`ur_to_robotiq_link`)과 **세트3 의 카메라/브라켓**(`camera_link`/`camera_adapter_link`)
에도 그대로 확장된다 — 마운트 인접쌍은 disable, 팔 몸통 쌍은 enable. (이 SRDF 는 세트2/3 공용.)

### ④ 결과: 자기충돌/환경충돌을 실제로 거부
`/check_state_validity` 로 보면 그리퍼를 팔 몸통에 박는 포즈에서 `robotiq_85_*_link <-> upper_arm_link` 같은
접촉이 잡히고(valid=False), 그 포즈를 목표로 보내면 MoveGroup 이 `FAILURE` 로 거부한다
(`isaac/ur16e_2f85/selfcollision_demo.py` 로 실증). 반대로 그리퍼 내부/손목 마운트는 disable 돼 정상 자세는 통과.

> 정리: 그리퍼는 **DOF 1개(`finger_joint`)+mimic** 로 제어하고, MoveIt 충돌 인식은 **collision 메시 + (이름이 일치하는)
> 결합 SRDF** 의 disable 범위로 정해진다 — 내부/마운트는 끄고, 몸통/환경은 켠다.

---

## Q5. 같은 그리퍼를 sim 과 실물에서 어떻게 같은 코드로 굴리나? (`robotiq_driver` 연동)

팔의 `use_sim` 스위치(sim=topic_based / real=ur_robot_driver)와 **같은 발상**을 그리퍼에도 적용한다.
그리퍼는 `ros2_control` 하드웨어 플러그인만 바꾸고 그 위(컨트롤러·액션·데모·SRDF)는 1벌을 공유한다.

| | sim (Isaac) | 실물 |
|---|---|---|
| 하드웨어 플러그인 | `joint_state_topic_hardware_interface/JointStateTopicSystem` (Isaac 토픽) | `robotiq_driver/RobotiqGripperHardwareInterface` (Modbus RTU; 손목장착=`/tmp/ttyUR` tool-comm 브리지, 벤치=USB-RS485) / 점검용 `mock_components/GenericSystem` |
| 런치 | `ur16e_2f85.launch.py` (팔+그리퍼 한 CM) | 손목 장착: `ur16e_2f85_real.launch.py` / 벤치: `robotiq_2f85_real.launch.py` (`gripper` 네임스페이스 별도 CM) |
| 액추에이트 조인트 | `finger_joint` | `finger_joint` (동일) |
| 컨트롤러/액션/데모 | `gripper_controller` · `GripperCommand` · `gripper_demo.py` | **동일** (데모는 `--action`/`--joint-states-topic` 로 네임스페이스만) |

### 실물 2F-85 는 PC 가 아니라 UR 손목에 붙는다 — 그럼 PC 는 어떻게 닿나
2F-85 는 Robotiq 커플링으로 **UR 손목 tool 커넥터**에 물려 24V + RS-485(Modbus RTU)를 UR 에서 받는다.
제어 PC 와는 직접 선이 없으므로, ROS2 가 닿는 표준 경로는 **ur_robot_driver 의 tool communication 브리지**다:
`ur_control.launch.py use_tool_communication:=true tool_voltage:=24` 가 `ur_tool_comm` 노드를 띄워 UR tool
시리얼(TCP 54321)을 **가상 시리얼 `/tmp/ttyUR`** 로 미러링 → `robotiq_driver` 가 그 `com_port` 를 연다.
`ur16e_2f85_real.launch.py` 가 이 브리지(팔)+그리퍼를 한 번에 띄운다(브리지가 먼저 떠야 해서 `gripper_startup_delay`
로 그리퍼를 늦춤). `/dev/ttyUSB0` 직결은 그리퍼를 PC 에 USB-RS485 로 따로 배선한 **벤치** 경우뿐. (펜던트
URCap 으로만 쓰면 PC 는 그리퍼에 접근 못 함.)

### 왜 실물은 팔과 "별도 controller_manager" 인가
팔은 이더넷(RTDE), 그리퍼는 시리얼(Modbus) — **물리 채널이 다른 별개 장치**다. 두 `ros2_control_node`(=CM)
가 한 머신에서 같은 `/controller_manager` 이름을 쓰면 충돌하므로, 그리퍼는 `gripper` 네임스페이스
(`/gripper/controller_manager`)로 띄운다. 그리퍼 TF subtree 는 `tool0` 에 붙어 팔 TF 트리에 자연히 합쳐진다.

### 왜 그리퍼 조인트 이름을 `finger_joint` 로 유지할 수 있나
`robotiq_driver` 는 URDF `ros2_control` 의 **첫 조인트(`info_.joints[0]`) 이름을 그대로** 쓴다(이름 비종속).
그래서 robotiq_description 원본의 `robotiq_85_left_knuckle_joint` 대신 sim 과 같은 `finger_joint` 로 둘 수 있고,
나머지 5관절은 실물에선 하드웨어가, ROS 표시는 RSP 의 `<mimic>` 가 따라간다 → 데모/SRDF 1벌 공유.

### 함정 (실물 그리퍼 고유)
- **네임스페이스 CM → 컨트롤러 yaml 은 wildcard 노드키(`/**/controller_manager` …)** 여야 매칭된다.
  평문 `controller_manager:` 는 `/gripper/controller_manager` 와 안 맞아 컨트롤러가 type 미정의로 로드 실패.
- **`robotiq_activation_controller`** 는 e-stop 후 재활성화용(`~/reactivate_gripper` 서비스)이며 `reactivate_gripper`
  GPIO 이름을 prefix 없이 하드코딩 → prefix 비우고 실물에서만 스폰.
- 점검은 `use_fake_hardware:=true`(mock) 로 하드웨어 없이 컨트롤러/액션 경로까지 검증 가능.

> 정리: **"하드웨어 플러그인만 교체"** 가 핵심 — sim/real 이 `finger_joint`·컨트롤러·액션·데모·SRDF 를 공유하고,
> 실물은 채널이 달라 `gripper` 네임스페이스 별도 CM 으로만 분리한다.

---

## Q6. 손목에 RealSense D405 를 붙이는 best practice? (eye-in-hand)

### 연결은 그리퍼와 정반대 — 카메라는 PC 에 직결
그리퍼는 UR tool I/O(RS-485) 경유였지만, D405 는 **USB 3.1 영상**이라 UR 를 거치지 못하고 **제어 PC 에 USB-C
직결**한다. 케이블은 팔 따라 정리하고 손목(±360°) 회전분 **서비스 루프**를 둔다. (그래서 그리퍼처럼 ros2_control
하드웨어가 아니라, 별도 카메라 드라이버 노드 `realsense2_camera` 가 PC 에서 돈다.)

### TF / optical frame 이 전부
URDF 가 `tool0 → camera_link → camera_{color,depth}_optical_frame`(REP-103, z-forward/x-right/y-down) 을
정의한다(`realsense_d405_macro.xacro`). 인식 결과를 로봇 좌표로 옮기는 정합은 **이 `tool0→camera` 변환 정확도**가
좌우한다. sim 은 USD 카메라 배치가 곧 정확한 ground truth(캘리브 불필요), 실물은 **hand-eye 캘리브**
(`easy_handeye2`/MoveIt Hand-Eye)로 추출해 URDF mount 에 반영한다. ⇒ **sim 카메라 pose 와 URDF mount 는
한 쌍으로 맞춰야 한다.**

### 마운트는 상용 브라켓 위에 — 그리고 충돌까지 인식
카메라를 임의 좌표에 띄우지 말고 **실제 브라켓** 위에 올린다. 여기선 PickNik `ur_realsense_camera_adapter`
(오픈 메시)를 쓰고, e-Series 표준 스택 **flange → 카메라 브라켓 → GRP-ES-CPL-077 커플링 → 2F-85** 순으로 쌓는다
(브라켓이 flange 에 먼저 붙고 그리퍼는 커플링에 체결). 카메라 시팅은 cradle 표면과 **평행(pitch 8°)·밀착(gap 0)**
이 되도록 origin 을 맞춰 sim 형상이 실제 기구와 같게 한다. 브라켓·카메라 body 는 **collision 메시**를 켜고
(`collision:=true`) SRDF 로 마운트 인접쌍만 disable → 팔이 카메라/브라켓을 자기 몸체에 박는 자세를 MoveIt 이
거부한다(환경 장애물 회피는 OctoMap, 자기간섭 회피는 collision+SRDF — 상보적). 상세 치수/검증은
[`HISTORY.md`](HISTORY.md) §9.

### sim/real perception parity
Isaac 의 `ROS2CameraHelper`(rgb/depth/depth_pcl) + `ROS2CameraInfoHelper` 를 **realsense2_camera 와 동일한
토픽명/인코딩/프레임**으로 발행(`/camera/color/image_raw` rgb8, `/camera/depth/image_rect_raw` 32FC1,
`/camera/depth/color/points`, `/camera/{color,depth}/camera_info` K with HFOV≈87°). depth 는 같은 센서 렌더라
color 에 자동 정렬. 그래서 인식 노드는 sim/real 무수정으로 동일하게 붙는다.

### 다운스트림(cuMotion / DepthAnything+FoundationPose) 까지 한 인터페이스
- **cuMotion**(GPU 모션플래닝): depth + camera_info + 정확한 TF(+로봇 충돌 sphere 모델)로 **카메라 기반 충돌
  월드**(nvblox/ESDF)를 만들어 회피. → 지금 깔린 depth/TF 가 그대로 입력.
- **FoundationPose**(6-DoF 포즈) + **DepthAnything**(단안 깊이): RGB + (color 에 정렬된) depth + camera_info +
  TF 필요. → 동일 인터페이스. 객체 mesh 만 추가하면 된다.

> 정리: 카메라는 **PC USB3 직결** + **표준 optical frame/camera_info/토픽** + **hand-eye(실물)/USD(sim) 로 맞춘
> tool0→camera** 가 핵심. 이 인터페이스 한 벌이 octomap·cuMotion·FoundationPose 입력으로 그대로 재사용된다.

---

## Q7. 실물 그리퍼를 URCap 으로 제어해도 되나? sim(Isaac)과 호환되게 하려면? (2FG14 dual-tool)

Q5 는 2F-85(세트2/3)의 sim/real 스왑을 다뤘다. 여기서는 **URCap vs ROS2 드라이버 "소유권"** 과, ROS2 오픈소스
드라이버가 없는 **세트4 OnRobot 2FG14** 를 어떻게 호환되게 설계하는지를 다룬다. (실물 런북 예정은
[`HARDWARE.md`](HARDWARE.md) §6.)

### 핵심 원칙 — 팔의 `use_sim` 패턴을 그리퍼에 그대로
sim/real 호환의 정답은 팔과 똑같다: **위(액션)는 sim/real 1벌, 아래(`ros2_control` `<hardware>` 플러그인)만
교체.** 단일 계약 = `control_msgs/GripperCommand` 액션. 지금 sim `gripper_controller`(=`GripperActionController`)가
이미 이 계약으로 도는 것을 확인함 → **top-level 계약은 이미 확보, 남은 건 real 백엔드뿐.**

```
상위앱/MoveIt ──GripperCommand──▶ gripper_controller ── ros2_control <hardware> (use_sim 로 스왑) ──┐
   sim  → topic_based_ros2_control/TopicBasedSystem → Isaac finger joint                            │
   real → 벤더 하드웨어 인터페이스(2F-85=robotiq_driver / 2FG14=신규) → 실물 그리퍼 ─────────────────┘
```

### 실물 제어 경로 3가지 — 그리고 URCap 의 위치
| 경로 | 그리퍼 소유자 | ROS2/Isaac 연동 |
|---|---|---|
| **(A) URCap 단독** | UR 펜던트(PolyScope) 프로그램이 tool RS-485/URScript 로 개폐 | ❌ 제어 루프가 로봇 컨트롤러 안에 갇힘 — PC/ROS 못 봄 |
| **(B) tool-comm 브리지 + ROS2 HW** | tool I/O = **User(ROS)** 소유, `ur_robot_driver` 가 `/tmp/ttyUR` 미러 | ✅ **2F-85 실물 방식**(Q5) |
| **(C) 외부 직결** | USB-RS485(Robotiq) 또는 OnRobot **Compute Box** Modbus TCP(이더넷) | ✅ (B/C 둘 다 sim 호환) |

**(A)와 (B)/(C)는 배타적**이다 — 그리퍼를 누가 소유하느냐가 갈린다(둘이 동시에 tool 포트를 못 잡음). 그래서
§2 2F-85 설정에도 tool I/O **Controlled by = User**(URCap 이 tool I/O 를 잡지 못하게)라고 못박혀 있다.

**B vs C 는 sim 문제가 아니다** — B·C 는 **둘 다 실물 백엔드**라 sim 에선 topic_based 로 스왑되므로 sim 호환은
동일(비호환은 A 뿐). 그리퍼 1개면 **B 가 배선 간단**(tool I/O 끝, 박스 불필요), HEX F/T 센서·F/T 데이터 쓰면
**C**(Compute Box 허브). SW(레지스터맵·GripperCommand)는 B/C 거의 동일 → 부품 후 결정. 경로 상세는 [`HARDWARE.md`](HARDWARE.md) §6.

### "실물을 URCap 으로 제어해도 Isaac 과 연동되나?"
- **순수 URCap → 안 된다.** URScript 명령은 로봇 컨트롤러 내부 이벤트일 뿐 ROS2 메시지가 되지 않아 Isaac 에
  도달할 방법이 없다(Isaac 엔 PolyScope 도 실물 그리퍼도 없음).
- **연동시키려면 명령의 출발점을 ROS2(액션)로 올려야** 한다. sim/real 차이는 액션 **아래(하드웨어 백엔드)**에만 둔다.
- (절충) ROS 에서 `ur_robot_driver` 의 `urscript_interface` 로 개폐 URScript 스니펫을 쏘는 방법도 가능하지만,
  sim 엔 그 개념이 없어 여전히 별도 경로가 필요 → 결국 "위는 액션 통일 + real 은 그 액션을 URScript 전송으로
  구현하는 어댑터"가 된다. `ros2_control` 하드웨어 인터페이스보다 덜 깔끔하니 (B)/(C) 를 권장.

### 우리 그리퍼별 ROS2 wrapper 현황
- **Robotiq 2F-85 (세트2/3) — 있음, 이미 vendored+빌드.** `ros2_robotiq_gripper`(PickNik): `robotiq_driver`
  (`RobotiqGripperHardwareInterface`, Modbus RTU/serial) + `robotiq_controllers` + `serial`. → 경로(B), 해결됨.
- **OnRobot 2FG14 (세트4 dual-tool, 지금 그리퍼) — 성숙한 1st-party ROS2 드라이버 없음.** 단 **참고 구현 확보**
  (tonydle `OnRobot_ROS2_Driver`, MIT — ROS2 `ros2_control` ActuatorInterface, `IModbusConnection` 로 **RS-485[B]/TCP[C]
  통일**, RG 레지스터맵 격리; 형제 `UR_OnRobot_ROS2`/`ur_onrobot`). → 신규 드라이버는 "새로 짜기"가 아니라 **이 레퍼런스를
  2FG14 레지스터맵으로 포팅 + `GripperActionController` 추가**. Modbus 는 **경로 B/C 어느 쪽이든**(둘 다 sim 호환) 위
  `GripperCommand` 에 연결(2F-85 배선 미러링: `gripper` 네임스페이스 별도 CM, wildcard 노드키, `tool0`→EOAT 밑 TF,
  조인트 이름 sim 과 동일 + `<mimic>`).

> 정리: **URCap 단독은 sim 과 호환 불가**(로봇 컨트롤러에 갇힘). 호환의 핵심은 **`GripperCommand` 액션 1벌 +
> `<hardware>` 플러그인만 스왑**. 2F-85 는 끝났고(`robotiq_driver`), 2FG14 는 **참고구현(tonydle `OnRobot_ROS2_Driver`)을
> 2FG14 레지스터맵으로 포팅**해 같은 구조로 붙이면 된다(Modbus 경로 B/C 는 배선/HEX-F/T 로 갈리고 **둘 다 sim 호환** —
> 부품 후 결정). sim 2FG14 는 이미 topic_based→Isaac 검증됨.

## Q8. STEP(CAD)→Isaac 반영 시 자주 나오는 개념들 (핑거 분할·원점·collider·import 방향)

세트4 CAD 파이프라인(`STEP_TO_SIM.md`)을 돌리며 반복되는 4가지 개념 질문. 파이프라인 스크립트의 *왜* 에 해당.

### Q8-a. 핑거 STEP 은 왜 left/right 를 나눠 받나? 모터 1축·대칭이면 세트로 받으면 안 되나?
**STEP 자체는 정지 형상이라 open/close 와 무관**하다 — 세트로 받든 나눠 받든 파일 안에 움직임은 없다. 나누는
이유는 *변환 결과물(sim 링크 구조)* 때문:
- Isaac/URDF 에서 개폐하려면 **움직이는 관절 1개 = 서로 다른 rigid body 2개**가 필요. 관절은 "부모 바디→자식
  바디" 사이에만 생긴다. 두 핑거가 **하나의 solid 로 융합(boolean union)** 되어 오면 sim 에선 **단일 rigid link
  1개** → 낄 관절이 없어 개폐 불가 + collision mesh 도 통째로 얼어붙어 파지 충돌체크가 무의미해진다.
- "모터 1축이라 자동 대칭"은 맞다(그래서 관절도 하나만 구동하고 나머지는 `<mimic>` 미러). 하지만 *대칭으로
  움직이려면 그 전에 좌·우가 독립 링크로 존재*해야 mimic 이 걸린다. 대칭은 "안 나눠도 되는 이유"가 아니라
  "나눈 두 링크를 어떻게 구동하냐"의 문제.
- **진짜 요구는 "파일 개수"가 아니라 "solid body 분리"** 다: ①파일 2개, 또는 ②파일 1개 + **solid body 2개**
  (assembly STEP) 는 둘 다 OK(HOOPS 가 바디별 Xform 으로 넣어줌 → 스크립트로 좌/우 분리 가능). ③파일 1개 +
  **body 1개로 융합** 만 ❌(경계·관절축을 손으로 추정해 잘라야 함). 즉 **"boolean union 하지 말고 좌·우를 별개
  바디로"** 가 핵심이고, 별도 파일로 주면 그게 자동 보장될 뿐.
- CAD 저장 자세(열림/닫힘)는 무관 — 변환 후 `normalize_part.py` 로 정규화하고 개폐 범위는 config 가 정한다.

### Q8-b. 핑거 링크의 원점(origin)은 어디에 잡나? 바디 접촉면 중심을 좌·우 공유하면?
먼저 "원점 두 개"를 구분: **①링크 프레임**(핑거 mesh 원점, `normalize_part.py` 가 잡음) vs **②관절 원점**
(`joint origin`, 부모 프레임에서 q=0 시 자식이 앉는 위치). 튜닝·충돌에 실제로 영향 주는 건 ①.

**권장: 바디 중심 공유 ✕ → 각 핑거의 "자기 슬라이더 장착면 중심"에 개별로.**
- **Z = 장착면 법선(=접근/툴 축)**, 단 **슬라이드(개폐) 축은 Z 에 수직인 Y**(현재 `axis [0,∓1,0]`)까지 같이
  고정해야 한다. Z 만 맞추면 부족 — "Z=법선 + Y=개폐방향" 둘 다 정규화해야 mimic 이 깔끔.
- 개별 원점이 나은 이유: ①개폐 오프셋이 **관절에 남아** `q` 가 "장착면에서 이동량"으로 직접 읽힘(우리가 튜닝한
  `OPEN ±0.031 ↔ CLOSE ±0.006` 이 관절 limit 로 그대로 보임; 바디중심 공유면 ±31mm 가 mesh 에 파묻힘) ②mesh 가
  자기 프레임에 얹혀 충돌 geometry 가 링크와 함께 이동 ③`normalize_part.py` 규약("모든 부품=자기 장착면→원점")과 일치.
- **바디중심 공유가 노리던 "대칭(left=mirror(right))"은 개별 원점이어도 공짜로 따라온다** — 두 장착면 자체가 Y=0
  에 대해 거울상이라 관절 원점 `(0,+Δ,h)`↔`(0,−Δ,h)`·축 `+Y`↔`−Y`·mesh 미러 → 정확한 거울 대칭. 이점만 얻고 손해 없음.
- 파지 접촉점(ㄷ-채널 접촉)은 링크 원점으로 삼지 말 것 — **TCP/grasp 프레임으로 따로**(우리는 이미
  `--grasp-xyz 0,0,0.179`). 링크 원점은 관절이 물리적으로 있는 장착면이어야 q=0 정의가 물리와 일치.

### Q8-c. "collider" 가 뭐야?
**PhysX(물리엔진)가 충돌·접촉 판정에 쓰는 보이지 않는 충돌 형상** — 눈에 보이는 render mesh 와 별개 레이어.
- STEP 을 넣으면 기본은 **render-only** → 물리적으로 "유령"(관통·미끄러짐). prim 에 **Collider 활성화**하면
  `UsdPhysics.CollisionAPI` 가 붙어 PhysX 가 충돌 물체로 인식(접촉력·마찰·파지). GUI: prim 우클릭 → Add →
  Physics → Colliders Preset (Property 패널 Collider 체크).
- **충돌 세계가 둘**임을 기억: **Isaac PhysX = USD Collider**(물리 재생 — 핑거가 휠을 실제 잡는 검증) vs
  **MoveIt = URDF collision mesh**(`export_eoat_meshes.py`, 모션플래닝 충돌체크). collider 는 Isaac 물리 재생용.
- **함정 — 근사 타입(`approximation`) 선택이 중요**(collider = 이 셋을 말함):
  - `convexHull` — 빠르나 **오목부를 메꿈**. 핑거 ㄷ-채널·휠 보어에 쓰면 속이 꽉 차 헛충돌/삽입 불가.
  - `convexDecomposition` — 오목을 볼록 조각들로 분해. **오목 EOAT 부품엔 이게 정답**(정적 장애물 파이프라인도 이래서 3종 제공).
  - `none`(triangle mesh) — 정확하나 **static 전용**(dynamic rigid body 불가). 테이블·지그용.
  - `sdf` — 오목 dynamic 까지 정확(무겁지만 정밀 파지/삽입).

### Q8-d. GUI import 시 원점 orientation 을 바꿀 수 있나?
**부분적으로만.** "원점을 이 면에, Z 를 이 축으로" 같은 임의 재배치는 import 단계에서 **안 된다.**
- Import 다이얼로그에서 되는 것 = **Up Axis(Y↔Z, 우리 `iUpAxis=2`)** 와 **Units/metersPerUnit(`0.001`)** 뿐.
  up-axis 는 **전체 import 를 통째로 회전**시키는 것이지 부품별 장착면 정렬이 아니다.
- 안 되는 것 = 개별 부품 원점을 특정 면/점으로 옮기기, 장착축을 +Z 로 맞추기. **local 원점 = STEP 저작 CAD
  글로벌 좌표계 그대로**이기 때문. CAD 에서 정렬해 저장했으면 그대로, 아니면 import 로는 못 고침.
- Import **후** GUI 에서 Xform translate/orient·Edit Pivot 은 가능하나 **mesh 위에 xformOp 를 얹는 것**이지 원점을
  재저작(bake)하는 게 아님 → reference/관절 시 원점이 CAD 원점 기준이라 헷갈리고 **재현 안 됨**(손 정렬).
- **그래서 GUI 대신 `normalize_part.py`** — 장착면→원점, 축→+Z 를 mesh 에 **baked** 해서 `assets/cad/normalized/`
  로 내보냄(config 숫자만 있으면 항상 같은 결과, 손 정렬 0). GUI 는 그 결과를 **눈으로 확인·미세 튜닝**하는 용도
  (CLAUDE.md "headless 는 치수만, 방향/자세 최종 튜닝은 Isaac GUI 확인 필수"의 역할 분담).
- 정석: mech 팀이 **CAD 저작 단계에서 원점=장착면·축 정렬로 저장**(Q8-a/b 요청과 이어짐) → import·normalize 둘 다 깔끔.
