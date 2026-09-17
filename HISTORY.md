# HISTORY — UR16e ROS 2 + Isaac Sim 구축 이력 (누적 기록)

> 📌 이 문서는 **누적 이력/검증 로그/디버깅 교훈**을 모은 기록이다(날짜별 검증 결과, 함정, 근거 포함).
> **현재 상태 기준 요약은 [`README.md`](README.md)**, 실물 연결 절차는 [`HARDWARE.md`](HARDWARE.md).
> 새 변경/검증은 계속 이 문서에 덧붙인다. (이하 본문은 초기 구축 시점부터의 기록)

---

# UR16e — ROS 2 Jazzy + Isaac Sim 5.1.0 (sim & real 공용 제어 스택)

워크스페이스를 **UR16e 단일 암**으로 정리하고,
**하나의 ROS 2 (Jazzy) 소프트웨어로 Isaac Sim 시뮬레이션과 실물 UR16e를 모두** 구동하도록 구성한 결과 정리.

- 작업 디렉터리: `/isaac-sim/ur_ws` (ROS2 colcon workspace), git repo 는 `src/`
- 환경: Docker, ROS 2 **Jazzy**, **Isaac Sim 5.1.0**, GPU RTX 5090
- 핵심 패키지: **`src/ur_bringup`** (이번에 신규 작성)

---

## 1. 아키텍처 — 공통 인터페이스 + 교체형 백엔드

```
            상위 앱 / MoveIt2                      ← sim/real 100% 동일
                   │  follow_joint_trajectory (action)
        scaled_joint_trajectory_controller
                   │  ros2_control
        ┌──────────┴───────────────┐
   use_sim:=true              use_sim:=false
   topic_based 하드웨어          ur_robot_driver (RTDE)
   (joint_state_topic_hw)        + 공식 UR 컨트롤러
        │ /isaac_joint_states         │ URCap
        │ /isaac_joint_commands       │
   Isaac Sim 5.1.0              실물 UR16e
   (ROS2 bridge OmniGraph)
```

`ur16e.launch.py`의 **`use_sim` 인자 하나**로 ros2_control 하드웨어만 바뀌고, 그 위(MoveIt·컨트롤러·앱)는 동일.
(UR 공식 `ur_simulation_gz`가 Gazebo에 쓰는 패턴을 Isaac Sim으로 대체한 형태)

---

## 2. 워크스페이스 구성

UR-only 표준 스택만 남도록 정리 완료. 현재 `src/` 트리:

```
src/
├── ur_bringup/                      ← [핵심] use_sim 디스패처, controllers, sim xacro, Isaac 스크립트+데모
├── topic_based_hardware_interfaces/ ← [필요] sim 백엔드 하드웨어 (vcstool 관리, 태그 0.2.1 — §3/§6)
├── ur16e.repos                      ← 소스 의존성 고정 (topic_based @ 0.2.1)
├── README.md / SETUP.md / qna.md    ← 문서 (구성 / 재현 매뉴얼 / 개념 Q&A)
```
> `topic_based_hardware_interfaces/` 는 `.gitignore` 처리되어 src 저장소에 박히지 않고 `vcs import` 로 받음.

자체 코드는 **`ur_bringup/` 한 곳**뿐이고, 나머지는 모두 UR 공식 패키지
(`ur_description`/`ur_robot_driver`/`ur_moveit_config`/`ur_controllers` — apt) + `topic_based`(0.2.1)에 의존.
(RTDE 직접제어 vs MoveIt 의 개념 차이는 `qna.md` Q3 참고)

`ur_bringup` 안의 파일은 **세트별 하위 폴더**로 구분된다. 각 카테고리(`launch`/`config`/`urdf`)는
세트 폴더(`ur16e` / `ur16e_2f85` / `ur16e_2f85_d405`)로 나뉘고, **여러 세트가 공유하는 파일은 `common/`** 에 둔다
(세트 간 재사용 — 상위 세트는 하위/공유 파일을 수정하지 않고 경로로 참조):

```
ur_bringup/
├── launch/
│   ├── ur16e/            ur16e.launch.py, ur16e_moveit.launch.py
│   ├── ur16e_2f85/       ur16e_2f85[_moveit|_real].launch.py, robotiq_2f85_real.launch.py
│   └── ur16e_2f85_d405/  ur16e_2f85_d405[_moveit].launch.py
├── config/
│   ├── common/           ur16e_2f85_controllers.yaml          (세트2·3 공유)
│   ├── ur16e/            ur16e_controllers.yaml
│   ├── ur16e_2f85/       robotiq_2f85_real_controllers.yaml
│   └── ur16e_2f85_d405/  sensors_3d.yaml
├── urdf/
│   ├── common/           robotiq_2f85_macro.xacro, ur16e_2f85_sim.ros2_control.xacro  (세트2·3 공유)
│   ├── ur16e/            ur16e_sim.urdf.xacro, ur16e_sim.ros2_control.xacro
│   ├── ur16e_2f85/       ur16e_2f85_sim.urdf.xacro, robotiq_2f85_real.{urdf,ros2_control}.xacro
│   └── ur16e_2f85_d405/  ur16e_2f85_d405_sim.urdf.xacro, realsense_d405_macro.xacro
├── srdf/common/          ur16e_2f85.srdf.xacro                 (세트2·3 공유)
├── isaac/
│   ├── common/           ur16e_isaac_ros2.py, moveit_plan_execute_demo.py, build_ur16e_2f85.py, convert_dae_to_usd.py
│   ├── ur16e_2f85/       gripper_demo.py, selfcollision_demo.py
│   ├── ur16e_2f85_d405/  octomap_demo.py, convert_bracket.py
│   └── assets/           합성 USD (세트 공유 버킷)
└── meshes/               PickNik 브라켓 메시 (세트3)
```
> `ros2 launch ur_bringup <파일명>` 은 폴더와 무관하게 이름으로 찾으므로 실행 명령은 그대로(하위폴더 경로 불필요).
> xacro `$(find ...)` include·launch 의 PathJoin·isaac 스크립트는 위 경로로 참조하도록 갱신됨.

**세 세트를 독립적으로 유지** — 각각 따로 띄울 수 있고, 상위 세트는 하위 세트 파일을 건드리지 않는다.

| 세트 | 런치 | 설명 |
|---|---|---|
| **1. UR16e 단독** | `ur16e.launch.py` (+`ur16e_moveit.launch.py`) | 팔만 |
| **2. UR16e + 2F-85** | sim `ur16e_2f85.launch.py` (+`ur16e_2f85_moveit.launch.py`) / real `ur16e_2f85_real.launch.py` | 팔 + **GRP-ES-CPL-077 커플링** + 그리퍼, collision-aware (§8) |
| **3. UR16e + 2F-85 + D405** | `ur16e_2f85_d405.launch.py` (+`ur16e_2f85_d405_moveit.launch.py`, Isaac 카메라 플래그) | 세트2 + **PickNik 카메라 브라켓** + eye-in-hand D405 + depth→OctoMap 충돌회피 (§9) |

> 세트마다 새 파일로 분리(상위 세트는 하위 세트의 ros2_control/SRDF/controllers/매크로를 **재사용**하되 그 파일은 수정하지 않음).
> 상세: **§8 그리퍼**, **§9 D405 카메라**.

---

## 3. 설치 (1회)

```bash
# (1) 표준 스택 바이너리 — sudo 필요
sudo apt update && sudo apt install -y \
    ros-jazzy-ur ros-jazzy-moveit ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers ros-jazzy-ros2-control-cmake
#   ros-jazzy-ur 메타패키지 = ur_robot_driver/ur_description/ur_moveit_config/ur_controllers/ur_calibration/...
#   (ros-jazzy-topic-based-ros2-control 은 apt 에 없음 → 아래 소스로 빌드)

# (2) sim 백엔드 하드웨어 — vcstool 로 0.2.1 고정 (★ 중요, §6 참고)
cd /isaac-sim/ur_ws
vcs import src < src/ur16e.repos          # topic_based_hardware_interfaces @ 0.2.1

# (3) 빌드 (워크스페이스 루트에서)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to ur_bringup \
    --cmake-args -DBUILD_TESTING=OFF      # 0.2.1 의 ros_testing 빌드 의존 회피
source install/setup.bash
```

> 단계별 재현 매뉴얼 전체는 **[`src/SETUP.md`](SETUP.md)** 참고 (apt 목록, vcs, 검증, 트러블슈팅 포함).
> 개념 Q&A (런치 구조, nav2 와의 비교 등)는 **[`src/qna.md`](qna.md)**.

---

## 4. 실행

모든 터미널에서 먼저: `source /opt/ros/jazzy/setup.bash && source /isaac-sim/ur_ws/install/setup.bash && export ROS_DOMAIN_ID=0`

### 두 런치 파일의 역할 (계층이 다름 — 보통 둘 다 실행)

| | `ur16e.launch.py` | `ur16e_moveit.launch.py` |
|---|---|---|
| 역할 | **로봇 + ros2_control 백엔드** (토대) | **MoveIt2 모션 플래닝 + RViz** (두뇌) |
| 띄우는 것 | robot_state_publisher, ros2_control_node(또는 실물 ur_robot_driver), 컨트롤러 스포너 | move_group(플래닝 파이프라인) + RViz MotionPlanning |
| `use_sim` 분기 | **있음** (sim=topic_based / real=ur_robot_driver) | 없음 — `use_sim`은 `use_sim_time`(클럭)만 결정 |
| 내부 | 직접 노드+컨트롤러 구성 (우리 패키지) | 공식 `ur_moveit_config/ur_moveit.launch.py` 래퍼 |
| `follow_joint_trajectory` 액션 | **노출**(컨트롤러) | 그 액션을 **호출**해 계획 궤적 실행 |
| 단독 실행 | 로봇 제어 가능(직접 trajectory) | **불가** — 컨트롤러/`/joint_states` 선행 필요 |
| 필수 여부 | 필수 (먼저) | 선택 (플래닝/GUI 필요 시, 그 다음) |

```
ur16e.launch.py        →  로봇 + 컨트롤러 + /joint_states     [필수, 먼저]
        ↑ follow_joint_trajectory 액션 노출
ur16e_moveit.launch.py →  move_group 이 그 액션으로 실행       [그 다음]
```
`use_sim` 스위치는 토대(`ur16e.launch.py`)에만 있고, MoveIt 은 sim/real 동일하게 올라감(시간만 `use_sim_time`).

### A) Isaac Sim 시뮬레이션
```bash
# 터미널 1 — Isaac Sim (씬 + ROS2 그래프 자동 구성)
#   GUI(로봇 보기): 환경 로드 필수 (조명). headless 면 --headless 추가, 헤드리스 테스트면 --no-env 도 가능
/isaac-sim/python.sh /isaac-sim/ur_ws/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py            # GUI
/isaac-sim/python.sh /isaac-sim/ur_ws/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py --headless  # headless

# 터미널 2 — 제어 (Isaac 가 /clock·/isaac_joint_states 발행 시작한 뒤)
ros2 launch ur_bringup ur16e.launch.py use_sim:=true

# 터미널 3 — MoveIt2 + RViz (선택)
ros2 launch ur_bringup ur16e_moveit.launch.py use_sim:=true

# 프로그램적 plan+execute 데모
python3 /isaac-sim/ur_ws/src/ur_bringup/isaac/common/moveit_plan_execute_demo.py
```
**기동 순서 중요:** Isaac → (안정화) → control → (`/joint_states` 유효 확인) → move_group/RViz.
Isaac 재시작 시 move_group/RViz 도 재시작(§6).

### B) 실물 UR16e
```bash
ros2 launch ur_bringup ur16e.launch.py use_sim:=false robot_ip:=<UR16e_IP>
ros2 launch ur_bringup ur16e_moveit.launch.py use_sim:=false
# 하드웨어 없이 드라이버 경로 점검:
ros2 launch ur_bringup ur16e.launch.py use_sim:=false use_mock_hardware:=true
```
실물 전제조건: ① 같은 서브넷 + `ping <ip>` ② External Control URCap (또는 `headless_mode:=true`)
③ `ur_calibration` 기구학 추출(권장).

---

## 5. 검증 결과 (2026-06-15)

| 항목 | 결과 |
|---|---|
| `colcon build` (ur_bringup + 0.2.1 hw) | ✅ |
| Isaac `ur16e_isaac_ros2.py`: UR16e 로드 + ROS2 그래프 + `/clock`·`/isaac_joint_states`(~250Hz) | ✅ |
| sim: 컨트롤러 active, `/joint_states` 유효 | ✅ |
| **sim: MoveIt plan+execute** (홈→목표, 오차 0.0084 rad) | ✅ |
| **sim: Isaac GUI + RViz 동시 표시 + plan+execute** (오차 0.0071 rad) | ✅ |
| real 경로(`use_mock_hardware:=true`): ur_robot_driver + 공식 UR 컨트롤러 active | ✅ |
| **real 경로: MoveIt plan+execute** (mock, 오차 0.0074 rad) | ✅ |
| 실물 UR16e 물리 연결 | ⏳ 로봇 IP/URCap/캘리브레이션 갖춘 뒤 |

---

## 6. 핵심 교훈 / 함정 (디버깅으로 확정)

1. **topic_based 하드웨어는 태그 `0.2.1`** (classic `export_state_interfaces()` API).
   `main`/1.0.0/1.1.0 의 신 API `set_state(name,value)` 는 apt `ros-jazzy` ros2_control 4.44.0 에서
   exported state interface 를 갱신하지 못해 **`/joint_states` 전체가 NaN** → MoveIt 상태 없음 → RViz 가
   NaN 포즈 렌더링 중 **SIGSEGV**. (드라이버 명령 경로 `write()` 는 정상이라 직접 trajectory 는 동작해 혼동됨)
2. **Isaac UR16e USD 의 articulation root 는 `/UR16e/root_joint`** (USD 에 default prim 없음, ArticulationRootAPI 가
   고정 베이스 조인트에 적용). OmniGraph 노드 targetPrim/robotPath 를 `/UR16e` 가 아니라 여기로.
3. **sim xacro 에 `effort` state interface 필요** — Isaac 의 PublishJointState 가 effort 까지 발행하므로
   position/velocity/effort 3개 모두 선언.
4. **sim 은 `use_sim_time:=true` + Isaac `/clock` 필요** — clock 없거나 너무 느리면 컨트롤러 활성화가
   `Switch controller timed out` 으로 실패. Isaac 없이 제어 스택만 테스트하려면 `use_sim_time:=false`.
5. **Isaac GUI 는 환경(조명) 로드 필수** — `--no-env` 로 띄우면 광원이 없어 뷰포트가 까맣게 보임(로봇은 존재).
6. **기동 순서** — Isaac 안정화 후 control, 그 다음 move_group/RViz. Isaac 재시작 시 move_group 이
   과도기 NaN 상태를 캐싱해 plan 이 `error_code -4` 로 실패하므로 move_group/RViz 도 재시작.

---

## 7. 파일 맵 (`src/ur_bringup/`)

| 파일 | 내용 |
|---|---|
| `launch/ur16e/ur16e.launch.py` | `use_sim` 디스패처. 인자: `use_sim, use_sim_time, robot_ip, use_mock_hardware, headless_mode, launch_rviz` |
| `launch/ur16e/ur16e_moveit.launch.py` | sim/real 공용 MoveIt2 (공식 `ur_moveit.launch.py` 래퍼) |
| `config/ur16e/ur16e_controllers.yaml` | sim 컨트롤러 (`scaled_joint_trajectory_controller` 이름으로 MoveIt 기본값과 정합, `update_rate:100`) |
| `urdf/ur16e/ur16e_sim.urdf.xacro` | UR16e(ur_description) + topic_based ros2_control |
| `urdf/ur16e/ur16e_sim.ros2_control.xacro` | topic_based 하드웨어 블록 (position cmd, pos/vel/eff state) |
| `isaac/common/ur16e_isaac_ros2.py` | Isaac Sim 씬+ROS2 OmniGraph (standalone, 세 세트 공용, `--asset-path`, `--with-camera`) |
| `isaac/common/moveit_plan_execute_demo.py` | 프로그램적 MoveIt plan+execute 데모 (관절 목표, 세 세트 공용) |
| `isaac/common/convert_dae_to_usd.py` | 범용 메시(.dae/.stl/.obj)→USD 변환기 (**미터 단위 고정**, 100x 스케일 방지). 커플링/브라켓 USD 생성용 |
| `isaac/README.md` | Isaac OmniGraph 배선표 + GUI/순서/트러블슈팅 |
| **세트 2 — UR16e + 2F-85 (그리퍼)** | — |
| `urdf/ur16e_2f85/ur16e_2f85_sim.urdf.xacro` | UR16e + **GRP-ES-CPL-077 커플링**(`ur_to_robotiq`, tool0→gripper_mount_link +11mm) + 2F-85 top-level (`<robot name="ur16e">`) |
| `urdf/common/ur16e_2f85_sim.ros2_control.xacro` | topic_based 하드웨어 (팔 6관절 + `finger_joint`) |
| `urdf/common/robotiq_2f85_macro.xacro` | 2F-85 매크로 (robotiq 메시, `finger_joint` master + mimic 5, `collision:=true`) |
| `config/common/ur16e_2f85_controllers.yaml` | 팔 JTC + `gripper_controller` (GripperActionController) |
| `launch/ur16e_2f85/ur16e_2f85.launch.py` | 그리퍼 세트 제어 브링업 (sim/Isaac) |
| `srdf/common/ur16e_2f85.srdf.xacro` | 팔 SRDF + 그리퍼·커플링·카메라/브라켓 `disable_collisions` (세트2/3 공용, collision-aware) |
| `launch/ur16e_2f85/ur16e_2f85_moveit.launch.py` | 그리퍼 전용 MoveIt2 (위 SRDF, collision-aware) |
| `isaac/common/build_ur16e_2f85.py` | UR16e+2F-85 단일 articulation USD 합성. 인자: `--out`/`--gripper-z`/`--coupling-usd`/`--coupling-z`/`--camera-mount-usd` (커플링·브라켓·standoff 를 USD 에 베이크 → Isaac EE 가 URDF/RViz 와 일치). 기본=세트2 커플링, 옵션=세트3 |
| `isaac/assets/ur_to_robotiq_coupling.usd` | GRP-ES-CPL-077 커플링 visual USD (robotiq_description 커플링 메시 → convert_dae_to_usd) |
| `isaac/ur16e_2f85/gripper_demo.py` | 그리퍼 open/close 데모 (GripperCommand, `--action`/`--joint-states-topic` 인자) |
| `isaac/ur16e_2f85/selfcollision_demo.py` | 자기충돌 감지/거부 실증 (`/check_state_validity` + MoveGroup) |
| **세트 2 — 실물(real) 추가** | — |
| `urdf/ur16e_2f85/robotiq_2f85_real.ros2_control.xacro` | 실물 하드웨어 블록: `robotiq_driver/RobotiqGripperHardwareInterface` (또는 `use_fake_hardware`=mock), `finger_joint` |
| `urdf/ur16e_2f85/robotiq_2f85_real.urdf.xacro` | 그리퍼 단독 URDF (root=`tool0`, 팔 TF에 접속) |
| `config/ur16e_2f85/robotiq_2f85_real_controllers.yaml` | `gripper_controller` + `robotiq_activation_controller` (wildcard 노드키, `gripper` 네임스페이스용) |
| `launch/ur16e_2f85/robotiq_2f85_real.launch.py` | 실물 그리퍼 단독 브링업 (`gripper` 네임스페이스, 팔과 별도 CM) — 벤치 USB-RS485 |
| `launch/ur16e_2f85/ur16e_2f85_real.launch.py` | **실물 결합** 런치: 팔(ur_control + tool comm 브리지 `/tmp/ttyUR`) + 그리퍼 한 번에 (손목 장착 표준) |
| **세트 3 — UR16e + 2F-85 + D405 (카메라)** | — |
| `urdf/ur16e_2f85_d405/ur16e_2f85_d405_sim.urdf.xacro` | 세트2 + **PickNik 브라켓**(camera_adapter_link, flange flush) 스택 + eye-in-hand D405 top-level (`<robot name="ur16e">`). 스택: tool0→브라켓(mount)→커플링(+7mm)→2F-85(+18mm), 브라켓에 D405 거치 |
| `urdf/ur16e_2f85_d405/realsense_d405_macro.xacro` | D405 매크로 (camera_link + color/depth optical frames, REP-103, optical +11.5mm, `collision` 파라미터; 세트3 는 `collision:=true`) |
| `meshes/picknik_ur5_realsense_camera_adapter_rev2.dae` | PickNik `ur_realsense_camera_adapter` 브라켓 visual 메시 (vendored) |
| `meshes/picknik_ur5_realsense_camera_adapter_rev2_collision.stl` | 같은 브라켓 collision 메시 (MoveIt 충돌검사용) |
| `isaac/assets/picknik_camera_adapter.usd` | 위 브라켓 .dae → USD (Isaac GUI 표시용, build 스크립트가 베이크) |
| `isaac/assets/ur16e_2f85_d405.usd` | 세트3 Isaac 씬 (커플링+브라켓+gripper standoff 베이크된 단일 articulation) |
| `launch/ur16e_2f85_d405/ur16e_2f85_d405.launch.py` | 세트3 제어 브링업 (d405 URDF 로드, 그리퍼 controllers 재사용) |
| `launch/ur16e_2f85_d405/ur16e_2f85_d405_moveit.launch.py` | 세트3 MoveIt2 + `use_octomap`(depth→OctoMap, 기본 on) |
| `config/ur16e_2f85_d405/sensors_3d.yaml` | MoveIt PointCloudOctomapUpdater (`/camera/depth/color/points`→OctoMap) |
| `isaac/ur16e_2f85_d405/octomap_demo.py` | octomap 활성 plan+execute + octomap 충돌검사 probe 데모 |
| **세트 3 — 실물(real) D405 추가** | — |
| `urdf/ur16e_2f85_d405/d405_real.urdf.xacro` | 실물 카메라 TF 단독 URDF (root=`tool0`, realsense_d405 매크로, `cam_xyz`/`cam_rpy`=hand-eye 인자). 전용 RSP 가 발행, sim 프레임명과 일치 |
| `config/ur16e_2f85_d405/d405_real.yaml` | `realsense2_camera` D405 파라미터 (color+depth+align+pointcloud, `publish_tf:=false`, sim 동일 토픽; wildcard 노드키) |
| `launch/ur16e_2f85_d405/d405_real.launch.py` | 실물 D405 단독 브링업 (realsense2_camera 노드 + 카메라 TF RSP, namespace `camera`, USB3, `enable_camera` 토글) |
| `launch/ur16e_2f85_d405/ur16e_2f85_d405_real.launch.py` | **실물 결합**: 팔(RTDE)+손목 2F-85+D405 한 런치 (`ur16e_2f85_real.launch.py`+`d405_real.launch.py` 재사용) |
| `README.md` | 패키지 상세 + 실물 전제조건 |

---

## 8. Robotiq 2F-85 그리퍼 (UR16e + 2F-85 세트)

UR-단독 세트와 **병렬**로, 팔 끝에 **GRP-ES-CPL-077 커플링 + Robotiq 2F-85** 를 붙인 세트. 그리퍼는 Isaac USD
에서 실제로 물리 구동되고, ROS 에서는 단일 `finger_joint` 만 명령하며(나머지 5관절은 ROS=`<mimic>`/RSP,
Isaac=`PhysxMimicJointAPI` 로 자동 연동), MoveIt 은 그리퍼·커플링 형상까지 **충돌 인식**하며 플래닝한다.

#### ★ End-effector coupling — GRP-ES-CPL-077 (e-Series 손목 직결)
최신형 e-Series UR16e 는 Robotiq **AGC-ES-UR-KIT-85** 키트의 **GRP-ES-CPL-077** 커플링으로 그리퍼를 손목 tool
커넥터에 바로 물린다(M8 female, 컨트롤러로 빼는 별도 케이블 없음). URDF 는 robotiq_description 의
`ur_to_robotiq` 매크로로 `tool0 → ur_to_robotiq_link(커플링) → gripper_mount_link (+11mm) → 2F-85` 를 세우고,
SRDF 가 커플링 인접쌍을 disable 한다. Isaac USD 에도 커플링 standoff/visual 이 베이크돼 EE 가 RViz 와 일치.

### 추가 설치 (1회)
```bash
sudo apt install -y ros-jazzy-robotiq-description      # RViz 용 정식 2F-85 메시 + ur_to_robotiq 커플링 매크로
# Isaac UR16e + 커플링 + 2F-85 단일 articulation USD 합성 (1회) → isaac/assets/ur16e_with_2f85.usd
#   --gripper-z 0.011 = 커플링 두께(+11mm), --coupling-usd = 손목에 베이크할 커플링 visual
/isaac-sim/python.sh /isaac-sim/ur_ws/src/ur_bringup/isaac/common/build_ur16e_2f85.py \
    --out  /isaac-sim/ur_ws/src/ur_bringup/isaac/assets/ur16e_with_2f85.usd \
    --gripper-z 0.011 \
    --coupling-usd /isaac-sim/ur_ws/src/ur_bringup/isaac/assets/ur_to_robotiq_coupling.usd --coupling-z 0.0
#   (커플링 visual USD 는 동봉됨. 재생성하려면: convert_dae_to_usd.py <robotiq coupling 메시> ur_to_robotiq_coupling.usd)
```

### 실행 (시뮬, 3 터미널 + 데모)
```bash
# 1) Isaac — 합성된 2F-85 씬을 --asset-path 로 지정 (스크립트 수정 불필요)
/isaac-sim/python.sh /isaac-sim/ur_ws/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/ur_ws/src/ur_bringup/isaac/assets/ur16e_with_2f85.usd
# 2) 제어 (팔 JTC + gripper_controller)
ros2 launch ur_bringup ur16e_2f85.launch.py
# 3) MoveIt2 + RViz — ★ 그리퍼 전용 (공유 ur16e_moveit.launch.py 아님!)
ros2 launch ur_bringup ur16e_2f85_moveit.launch.py
# 데모
python3 /isaac-sim/ur_ws/src/ur_bringup/isaac/ur16e_2f85/gripper_demo.py           # open/close
python3 /isaac-sim/ur_ws/src/ur_bringup/isaac/ur16e_2f85/selfcollision_demo.py     # 자기충돌 거부 실증
```
그리퍼는 `GripperCommand` 액션(`/gripper_controller/gripper_cmd`, position 0=open … ~0.8=close)으로 제어.

### 함정 (그리퍼 고유, 디버깅으로 확정)
- **그리퍼 변형 URDF 의 `<robot name>` 은 반드시 `ur16e`** — 아니면 ur_moveit SRDF(이름 `ur16e`)가
  적용되지 않아 인접 링크가 충돌로 잡혀 `error_code -10 (START_STATE_IN_COLLISION)`. (개념: `qna.md` Q4)
- **그리퍼 master 조인트 = `finger_joint`** 로 통일 — robotiq_description 매크로 원본 master 는
  `robotiq_85_left_knuckle_joint` 라 우리 `/joint_states` 로 안 움직임. 그래서 매크로를 다시 써 master 를
  `finger_joint` 로, 나머지를 그 `<mimic>` 으로.
- **collision 을 켜면 전용 SRDF 동반 필수** — `robotiq_2f85_macro.xacro collision:=true` 가 collision 메시를
  내보내고, `srdf/common/ur16e_2f85.srdf.xacro` 가 그리퍼-내부 + 그리퍼↔손목 쌍을 `disable_collisions` 한다
  (그리퍼↔팔몸통/환경은 유지 → 실제 충돌은 검사됨). 이 SRDF 를 쓰는 게 `ur16e_2f85_moveit.launch.py`.

### 검증 결과 (2026-06-18)
| 항목 | 결과 |
|---|---|
| 합성 USD 단일 articulation, `finger_joint` 구동 + mimic 5개 자동 추종 | ✅ |
| `gripper_controller` active, `GripperCommand` open/close (0↔0.6) | ✅ |
| RViz 실제 2F-85 메시 평행 개폐 (finger_joint → mimic, RSP) | ✅ |
| 팔 plan+execute + 그리퍼 open/close **동시** | ✅ |
| collision-aware: 그리퍼 메시 planning scene 반영, 팔 plan SUCCESS(오차 ~0.01, `-10` 없음) | ✅ |
| **자기충돌 실증**: 그리퍼↔upper_arm/forearm/shoulder 접촉 감지, MoveGroup 목표 **거부**(error 99999) | ✅ |

### 실물(real) 그리퍼 — `robotiq_driver` 연동 (2026-06-19)

sim 의 `topic_based` 와 동일한 `finger_joint`/`gripper_controller`/SRDF/데모를 **그대로** 쓰되, 백엔드만
실물 드라이버로 교체한 세트. 그리퍼는 팔(RTDE)과 **별개 물리 장치**(시리얼 Modbus RTU)이므로,
팔의 `/controller_manager` 와 충돌하지 않게 **`gripper` 네임스페이스의 별도 controller_manager** 로 띄운다.

#### ★ 물리 연결 — 2F-85 는 UR 손목에 붙지, 제어 PC 에 직접 안 붙는다
e-Series UR16e 에서 2F-85 는 Robotiq 커플링으로 **UR 손목 tool 커넥터**에 물려 24V + RS-485(Modbus RTU,
115200 8N1)를 받는다. 제어 PC 와는 직접 선이 없다. PC(ROS2)가 그리퍼에 닿는 길은:

| 토폴로지 | 연결 | `com_port` | 런치 |
|---|---|---|---|
| **A. 손목 장착(표준)** | 그리퍼→UR tool→**ur_robot_driver tool comm 브리지**(UR tool 시리얼을 TCP 54321→가상 시리얼) | `/tmp/ttyUR` | `ur16e_2f85_real.launch.py` (팔+브리지+그리퍼 한 번에) |
| B. 벤치 직결 | 그리퍼→USB-RS485 어댑터→PC | `/dev/ttyUSB0` | 팔 `ur16e.launch.py` + 그리퍼 `robotiq_2f85_real.launch.py` 따로 |
| (참고) URCap | UR 펜던트가 제어 | — | PC 에서 ROS2 제어 불가 |

```bash
# 의존성 (apt 에 robotiq_driver 없음 → 소스 빌드). serial + ros2_robotiq_gripper 가 src/ur16e.repos 에 고정됨
cd /isaac-sim/ur_ws && vcs import src < src/ur16e.repos
colcon build --symlink-install --packages-select serial robotiq_driver robotiq_controllers \
    --cmake-args -DBUILD_TESTING=OFF

# ── A. 손목 장착(표준): 팔 + tool comm 브리지 + 그리퍼를 한 런치로 ──
ros2 launch ur_bringup ur16e_2f85_real.launch.py robot_ip:=<UR16e_IP>
#   내부: ur_control.launch.py use_tool_communication:=true tool_voltage:=24 → /tmp/ttyUR 생성,
#         gripper_startup_delay(기본 8s) 뒤 robotiq_2f85_real.launch.py(com_port:=/tmp/ttyUR) 기동

# ── B. 벤치 직결(USB-RS485) ──
ros2 launch ur_bringup ur16e.launch.py use_sim:=false robot_ip:=<UR16e_IP>          # 팔
ros2 launch ur_bringup robotiq_2f85_real.launch.py com_port:=/dev/ttyUSB0           # 그리퍼

# ── 무하드웨어 점검 (팔 mock + 그리퍼 mock, 브리지 off) ──
ros2 launch ur_bringup ur16e_2f85_real.launch.py \
    use_mock_hardware:=true use_fake_hardware:=true use_tool_communication:=false
# (그리퍼만 점검) ros2 launch ur_bringup robotiq_2f85_real.launch.py use_fake_hardware:=true

# 데모 (네임스페이스된 액션/상태 토픽 지정)
python3 /isaac-sim/ur_ws/src/ur_bringup/isaac/ur16e_2f85/gripper_demo.py \
    --action /gripper/gripper_controller/gripper_cmd --joint-states-topic /gripper/joint_states
```

- **드라이버는 joint 이름 비종속** — `info_.joints[0]` 을 그대로 쓰므로 sim 과 동일하게 `finger_joint`
  로 통일(데모/SRDF 1벌 공유). 실물에선 단일 `finger_joint` position 만 명령, 5 mimic 은 RSP 가 계산.
- **`gripper` 네임스페이스 → 컨트롤러 yaml 은 wildcard 노드키(`/**/...`)** 필수. 평문 `controller_manager:`
  키는 FQN(`/gripper/controller_manager`)과 안 맞아 `type 미정의` 로 컨트롤러 로드 실패.
- **`robotiq_activation_controller`** 는 실물에서만 스폰(`use_fake_hardware:=false`) — `reactivate_gripper`
  GPIO(prefix 없는 이름 하드코딩)를 잡아 e-stop 후 `~/reactivate_gripper` 서비스로 재활성화.
- **tool comm 전제조건**: UR tool I/O 를 RS-485/Robotiq 로 설정, `tool_voltage:=24`, PC 사용자가
  `/tmp/ttyUR` 쓰기 권한(`dialout`). 브리지(`ur_tool_comm`)가 먼저 떠야 robotiq_driver 가 열 수 있어
  `gripper_startup_delay`(기본 8s)로 그리퍼 기동을 늦춘다.

| 항목 (real, `use_fake_hardware:=true` 로 무하드웨어 검증) | 결과 |
|---|---|
| `serial`+`robotiq_driver`+`robotiq_controllers` 소스 빌드, 플러그인 등록 | ✅ |
| `/gripper/controller_manager`: `joint_state_broadcaster` + `gripper_controller` active | ✅ |
| `GripperCommand` open/close (0↔0.6) `reached_goal=True`, `goals_ok=True` | ✅ |
| `/gripper` RSP 가 `robotiq_85_*` TF subtree 발행(tool0 접속, mimic 추종) | ✅ |
| 실물(`robotiq_driver`, `/dev/ttyUSB0`) | 미검증 — 실물 그리퍼/시리얼 어댑터 필요 |

---

## 9. RealSense D405 (eye-in-hand 카메라)

타깃 리그는 **UR16e + 2F-85 + D405**. 카메라는 손목에 달려 파지 직전 근거리(약 7~50cm) RGB-D 를 본다.
파지/비전 + MoveIt octomap 충돌회피용이며, 추후 **cuMotion**(GPU 모션플래닝) · **DepthAnything + FoundationPose**
연동을 염두에 두고 **표준 토픽·optical frame** 으로 깔았다.

#### ★ 연결 — 그리퍼와 정반대 (PC 직결)
| | 그리퍼 2F-85 | **D405** |
|---|---|---|
| 신호 | RS-485 (저속) | **USB 3.1 영상** |
| 경로 | UR 손목 tool I/O 경유 (`/tmp/ttyUR`) | **제어 PC 에 USB-C 직결** (UR 거치지 않음) |

→ 카메라는 UR tool I/O 로 못 보낸다. **PC USB3 직결**, 케이블은 팔 따라 정리 + 손목 회전분 서비스 루프 확보.

#### ★ 마운트 브라켓 — PickNik `ur_realsense_camera_adapter`
카메라는 임의 좌표가 아니라 **실제 상용 브라켓** 위에 거치한다. PickNik 의 오픈 브라켓(D415/L515용, D405 대표
형상으로 사용)을 vendored(`meshes/picknik_*_rev2.dae` visual + `_collision.stl`). 스택 순서는 PickNik UR 하드웨어
가이드대로 **flange(tool0) → 카메라 브라켓(flush) → 커플링(+7mm) → 2F-85(+18mm)** — 브라켓이 먼저 flange 에
붙고(M6 나사가 브라켓+커플링 관통), 그리퍼는 커플링에 체결. D405 는 브라켓 cradle 에 안착된다.

- **카메라 시팅(seating) 정밀 튜닝**: D405 뒷면이 cradle 에 평행·밀착하도록 `realsense_d405` origin 을
  `xyz="0 -0.067 0.01847" rpy="0 ${-pi/2 + radians(8)} ${pi/2}"` 로 둠. pitch 는 PickNik 공칭 6° 가 아니라
  **8°**(visual 메시에서 측정한 cradle 표면 법선과 평행 → 0° 잔차), 높이 0.01847 은 gap=0(밀착) 해.
- Isaac GUI 에서 브라켓·커플링 visual 이 보이도록 `build_ur16e_2f85.py` 가 두 메시를 손목에 베이크
  (`assets/ur16e_2f85_d405.usd`), 카메라 박스(42×42×23mm)는 `--with-camera` 가 런타임에 추가.

#### 좌표/캘리브 (핵심)
- URDF 가 `tool0 → camera_adapter_link(브라켓) → camera_link → camera_{color,depth}_optical_frame`(REP-103, z-forward) 정의.
- **sim**: USD 카메라 = 정확한 ground truth → 캘리브 불필요. **real**: `tool0→camera` 를 **hand-eye 캘리브**
  (`easy_handeye2`/MoveIt Hand-Eye)로 추출해 URDF mount 에 반영. 브라켓이 명목 위치를 주고, 캘리브가 보정.
- 따라서 URDF mount origin 과 Isaac 카메라 pose 는 **함께 맞춰야 하는 한 쌍**.

#### 실행 (sim, 세트 3)
```bash
# T1: Isaac — 세트3 씬(커플링+브라켓 베이크) + 카메라 그래프 (--with-camera)
#   ★ --asset-path 는 반드시 절대경로 (상대경로는 Isaac 에셋서버 기준으로 붙어 로봇 로드 실패)
/isaac-sim/python.sh /isaac-sim/ur_ws/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/ur_ws/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd --with-camera
# T2: 제어 — d405 description 로드(카메라 프레임이 robot_description/TF 에 포함)
ros2 launch ur_bringup ur16e_2f85_d405.launch.py
# T3: MoveIt + RViz — depth→OctoMap 충돌회피 (use_octomap 기본 on)
ros2 launch ur_bringup ur16e_2f85_d405_moveit.launch.py
# 데모: octomap 활성 plan+execute + octomap 충돌검사 probe
python3 /isaac-sim/ur_ws/src/ur_bringup/isaac/ur16e_2f85_d405/octomap_demo.py
```
#### 실행 (real, 세트 3) — D405 USB3 직결

sim 의 Isaac 카메라를 **실물 `realsense2_camera` 드라이버**로 교체한 세트. sim 과 **토픽·인코딩·optical
frame 이 동일**(`/camera/...`)하게 깔아 인식 스택(OctoMap/cuMotion/FoundationPose)을 무수정 재사용한다.
그리퍼(2F-85)와 정반대로 카메라는 **UR tool 버스가 아니라 PC 에 USB3 직결**이라 팔/그리퍼와 독립 노드로 뜬다.

```bash
# (1회) 드라이버 — apt (소스 빌드 불필요)
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-librealsense2
#   ★ 부분 업그레이드 주의: realsense2_camera 가 최신이면 구버전 diagnostic_updater 와
#     ABI 불일치로 노드가 dlopen 실패(`undefined symbol: diagnostic_updater::Updater::Updater`)
#     -> SIGABRT. 같이 올려 맞춘다:
#       sudo apt install -y ros-jazzy-diagnostic-updater ros-jazzy-diagnostic-msgs

# A. 카메라 단독 (USB3 직결) — realsense2_camera + 카메라 TF(tool0→camera_*) 한 번에
ros2 launch ur_bringup d405_real.launch.py
#   TF 만 점검(장치 없이): enable_camera:=false  → camera_state_publisher 만 떠 tool0→camera_link 발행
#   hand-eye 결과 반영: cam_xyz:="x y z"  cam_rpy:="r p y"  (아래 참고)

# B. 전체 실물 결합 (팔 RTDE + 손목 2F-85 + D405) 한 런치 — 세트2 real 을 재사용
ros2 launch ur_bringup ur16e_2f85_d405_real.launch.py robot_ip:=<UR16e_IP>

# 그 위에 MoveIt + depth→OctoMap (sim 과 동일 토픽, 실시간만)
ros2 launch ur_bringup ur16e_2f85_d405_moveit.launch.py use_sim:=false

# 무하드웨어 dry-run (팔 mock + 그리퍼 mock + 카메라 TF 만, 장치/브리지 off)
ros2 launch ur_bringup ur16e_2f85_d405_real.launch.py \
    use_mock_hardware:=true use_fake_hardware:=true use_tool_communication:=false enable_camera:=false
```

- **TF 단일 소스 = URDF** — 드라이버는 `publish_tf:=false`(yaml), 카메라 프레임(`tool0→camera_link→*_optical_frame`)은
  `urdf/ur16e_2f85_d405/d405_real.urdf.xacro` 를 로드한 **전용 robot_state_publisher**(namespace `camera`)가 발행한다.
  루트가 `tool0` 라 팔 TF 트리에 tool0 에서 합류(그리퍼 real 패턴과 동일). sim 프레임명과 1:1 일치.
- **hand-eye 캘리브레이션** — `tool0→camera_link` 외부파라미터는 `cam_xyz`/`cam_rpy` 런치(=xacro) 인자로 노출.
  기본값은 sim 명목 마운트(`0 -0.067 0.01847` / `0 -1.4311700 1.5707963` = `0, -π/2+8°, π/2`). 실물에선
  `easy_handeye2` 또는 MoveIt Hand-Eye Calibration 으로 추출한 값을 이 두 인자에 넣는다(브라켓이 명목, 캘리브가 보정).
- **토픽명 주의** — 노드를 namespace `camera` 로 띄워 `/camera/<topic>` 을 노린다. realsense-ros 버전에 따라
  `/camera/camera/<topic>` 으로 중첩될 수 있다(`camera_name` 이 `camera_namespace` 아래로). 그 경우 `camera_name:=''`
  로 두거나 remap 으로 sim 토픽명에 맞춘다. 프로파일은 `rs-enumerate-devices -c` 로 지원목록 확인 후 yaml 수정.

#### 검증 결과 (2026-06-19, sim)
| 항목 | 결과 |
|---|---|
| `--with-camera` 카메라 그래프 빌드(render product + CameraHelper×3 + CameraInfoHelper×2) | ✅ |
| `/camera/color/image_raw`(rgb8 640×480 ~90Hz), `/depth/image_rect_raw`(32FC1 ~100Hz), `/depth/color/points` | ✅ |
| `camera_info` K=[fx=fy=334.2, cx=320, cy=240] → **HFOV≈87°**(D405 일치), frame=`camera_color_optical_frame` | ✅ |
| 토픽명·인코딩·optical frame 이 **realsense2_camera 실물과 동일** (sim/real parity) | ✅ |
| PickNik 브라켓+커플링 baked USD(`ur16e_2f85_d405.usd`) 단일 articulation, Isaac GUI 에 브라켓/커플링 visual 표시 | ✅ |
| 카메라 시팅(pitch 8°, z 0.01847): D405 뒷면 cradle 평행·밀착(gap=0, 메시 교차 검사로 확정) | ✅ |
| **카메라/브라켓 collision-aware**: home pose valid, 간섭 pose 거부(`forearm_link↔camera_adapter_link`/`↔camera_link`) | ✅ |
| **실물(real) 구성**: `d405_real.launch.py`(카메라+TF) / `ur16e_2f85_d405_real.launch.py`(팔+그리퍼+카메라) 작성 | ✅ |
| real 무하드웨어 검증: `d405_real.urdf.xacro` 파싱, `camera_state_publisher` 가 `tool0→camera_link` 발행(=명목 마운트, RPY [0,-82°,90°]) | ✅ |
| 실물 D405(`realsense2_camera`, USB3) 영상/depth/points 발행 + hand-eye 캘리브 | 미검증 — 실물 D405 + `ros-jazzy-realsense2-camera` 설치 필요 |

> 다운스트림 염두: cuMotion 은 depth+camera_info+정확한 TF(+로봇 sphere 모델), FoundationPose 는 RGB+정렬 depth+
> camera_info+TF 를 요구 → 지금 깔린 인터페이스가 그대로 입력이 된다. depth 는 같은 센서 렌더라 color 에 정렬됨.

#### depth → MoveIt OctoMap 충돌회피
move_group 의 planning scene monitor 가 D405 포인트클라우드를 **OctoMap** 으로 적분해 충돌형상으로 넣는다
(카메라가 보는 장애물을 플래닝이 회피). 로봇 자기 링크는 self-filter(padding)로 제외.

- `config/ur16e_2f85_d405/sensors_3d.yaml`: `occupancy_map_monitor/PointCloudOctomapUpdater` ← `/camera/depth/color/points`
  (sim·real 동일 토픽), `max_range:=1.5`, `max_update_rate:=5`.
- `ur16e_2f85_d405_moveit.launch.py`(세트3) 가 `MoveItConfigsBuilder.sensors_3d()` 로 로드 + move_group 에
  `octomap_frame:=world`, `octomap_resolution:=0.02` 전달. **`use_octomap`(기본 true)** 인자로 토글 —
  `false` 면 perception 플러그인 없이 세트2와 동일한 collision-aware MoveIt(공유 SRDF).
- 카메라 body·브라켓은 **collision on**(`collision:=true`, 세트3 URDF) — 팔이 카메라/브라켓을 자기 몸체에 박는
  자세를 MoveIt 이 거부한다. 공용 SRDF 가 마운트 인접쌍(wrist_3·tool0·flange·커플링·그리퍼·서로) 28개를 disable
  해 START_STATE 오검출을 막고, **카메라/브라켓 ↔ 팔 몸체(forearm/upper_arm/shoulder/base/wrist_1·2)는 enabled**
  로 둬 자기간섭을 잡는다. (환경 장애물은 OctoMap 이 담당 — 둘은 상보적.)
- **런타임 의존(중요)**: 플러그인이 **`ros-jazzy-moveit-ros-perception`** 에 있다(미설치 시 `sudo apt install`).
  프레임워크 `moveit_ros_occupancy_map_monitor` 만으로는 부족.

```bash
# (1회) perception 플러그인 — sudo (octomap 사용 시 필수)
sudo apt install -y ros-jazzy-moveit-ros-perception
#   ★ 부분 업그레이드 주의: perception 만 최신이면 libgeometric_shapes.so 버전 불일치로 플러그인 로드 실패.
#     스택 정렬: sudo apt install -y $(dpkg -l | awk '/^ii.*ros-jazzy-moveit/{print $2}') ros-jazzy-geometric-shapes
# Isaac(--with-camera) + 세트3 제어 + 세트3 MoveIt(octomap 기본 on) 띄운 뒤:
ros2 launch ur_bringup ur16e_2f85_d405_moveit.launch.py       # use_octomap:=true (기본)
# 확인: planning scene 의 OctoMap 채워졌는지 (component 32 = OCTOMAP)
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 32}}" \
    | grep -E "id=|resolution=|data="     # id='OcTree', resolution=0.02, data=[...] 비어있지 않음
# RViz: PlanningScene 디스플레이의 "Show OctoMap" → 카메라 시야의 복셀 표시
```

#### 검증 결과 (2026-06-19, sim, end-to-end)
| 항목 | 결과 |
|---|---|
| `moveit-ros-perception`(+geometric_shapes) 정렬 후 octomap updater 로드 (`Listening to '/camera/depth/color/points'`, target=`world`) | ✅ |
| `/filtered_cloud` 발행 ~5.5Hz(=max_update_rate), raw 307200 → 범위내 self-filter 후 ~59745 pts | ✅ |
| planning scene `OcTree`(frame `world`, resolution 0.02) **복셀 적분 확인** (`GetPlanningScene` component 32) | ✅ |
| `use_octomap:=false` → perception 없이 기존 collision-aware 그리퍼 MoveIt 그대로 | ✅ |

---

## 10. 종료
```bash
pkill -f ur16e_isaac_ros2.py ; pkill -f "ros2 launch ur_bringup" ; pkill -f "lib/rviz2/rviz2"
```
> 주의: 같은 컨테이너에서 다른 워크스페이스가 GPU/ROS 를 쓸 수 있으므로 광범위한 `pkill`(예: `python3`,
> `ros2_control_node`) 은 피하고 위처럼 워크로드 전용 패턴만 사용.

---

## 11. cuMotion (GPU 모션플래닝, MoveIt 플러그인) — 2026-06-22

NVIDIA **Isaac ROS cuMotion** 을 MoveIt planning pipeline 으로 통합. GPU(cuMotion 엔진, cuRobo 후속)로
플래닝하고 실행은 기존 `scaled_joint_trajectory_controller`(sim/real 공용). **sim plan+execute 검증 완료**
(MoveGroup SUCCESS, 도달 오차 0.0003 rad).

### 설치 (apt, bare-metal)
- Isaac ROS 4.x = **Jazzy + Ubuntu 24.04** 라인(우리 환경 일치). 레포 `isaac-ros release-4 noble main`.
- 전제 레포 3개: Isaac ROS + **CUDA 13**(`cuda-keyring`, `cuda-toolkit-13-0` 하드 의존, 수 GB) +
  **VPI 4**(`libnvvpi4`, NVIDIA Jetson OTA x86_64 `r38.2`). 셋 다 있어야 `gxf-isaac-*`/`nitros` 연쇄 해결.
- 패키지: `ros-jazzy-isaac-ros-cumotion[-moveit/-examples/-robot-description]`. 엔진은 **deb 번들**
  (`libcumotion_impl.so`) — 런타임 tarball 불필요. CUDA 드라이버(580, CUDA13 호환)는 기존 설치로 충분
  (시스템 CUDA 툴킷은 없었음 → deb 가 `cuda-toolkit-13-0` 로 끌어옴).

### UR16e 로봇설정(XRDF)
- cuMotion 은 URDF + XRDF(cspace/충돌 sphere/self-collision) 필요. UR16e 는 ur5e/ur10e 만 기본 제공 → 생성.
- UR16e 는 shoulder/wrist/base + 2F-85 메시를 UR10e 와 공유, upper_arm/forearm 만 다름 → NVIDIA `ur10e_robotiq_2f_85.xrdf`
  를 베이스로 **upper_arm/forearm/coupling sphere 만 재생성**(standalone cuMotion 휠 `create_collision_sphere_generator`
  로 메시→sphere, collision origin 으로 링크프레임 변환). `tool0`→`ur_to_robotiq_link` 치환.
- 결과 vendored: `ur_bringup/cumotion/ur16e_2f85.{urdf,xrdf}` + `gen_xrdf.py`(재현). cuMotion 노드 로드 검증됨.
- 휠은 venv(`deps/.venv-cumotion`, python3.12-venv)로 설치(엔진 deb 와 별개, 오프라인 생성 전용). Isaac python 은
  휠 플랫폼 태그 거부 → 시스템 venv 사용.

### 통합 / 실행
- `launch/ur16e_2f85_d405/ur16e_2f85_d405_cumotion_moveit.launch.py`: cuMotion planner 노드(ComposableNode,
  `StaticPlanningSceneServer` 동반 필수) + move_group 에 `isaac_ros_cumotion` pipeline 추가(기본값). 실행은 기존 제어.
- 제어 위에서 `ros2 launch ... cumotion_moveit` → `moveit_plan_execute_demo.py`. RViz 에서 planner 선택 가능.
- world 충돌회피: `read_esdf_world:=true` + nvblox(D405 depth→ESDF). 기본 off.

### 겪은 함정 (확정)
1. `cuda-toolkit-13-0`/`libnvvpi4` "not installable" → CUDA·VPI 레포 누락. 3개 레포 다 추가하면 해결.
2. **부분 업그레이드 ABI 깨짐(중요)**: cuMotion/realsense 가 `diagnostic_updater` **4.2.7**(`Updater(...,double,bool)`)
   을 끌어오면, 구버전(4.44) `controller_manager`(`Updater(...,double)` 기대)가 `undefined symbol` 로 죽어
   **sim/real 제어 전체 마비**. → ros2_control 스택을 **4.45.2 로 동반 업그레이드**(메타패키지 말고 실제 패키지 직접 지정:
   controller-manager/controller-interface/hardware-interface/-msgs/joint-trajectory-controller/joint-state-broadcaster/position-controllers).
   둘 다 표준 ROS 레포(packages.ros.org)에 있음.
3. JTC 가 cuMotion 궤적 goal 거부: `Velocity of last trajectory point ... is not zero`(종단 잔여속도 ~1e-3) →
   컨트롤러 yaml 에 `allow_nonzero_velocity_at_trajectory_end: true`(OMPL 등 종단속도 0 궤적엔 무해).

### 검증 결과 (2026-06-22, sim)
| 항목 | 결과 |
|---|---|
| cuMotion 4.4 + CUDA13 + VPI4 설치, planner GPU 초기화(MotionPlan action server) | ✅ |
| UR16e XRDF/URDF 생성 + cuMotion 노드 로드("Robot description loaded successfully") | ✅ |
| move_group 에 `isaac_ros_cumotion` pipeline 등록 + planner 노드 기동 | ✅ |
| **plan+execute** (조인트 목표, MoveGroup SUCCESS, 오차 **0.0003 rad**, 기존 컨트롤러로 실행) | ✅ |
| world ESDF(nvblox) 연동 | ✅ — §12 참고 |

## 12. nvblox 실시간 장애물 회피 (cuMotion + nvblox) — 2026-06-22

cuMotion 의 `read_esdf_world` 를 **nvblox** 에 연결해 "카메라가 본 장애물을 GPU 가 실시간 회피"를 sim 에서 검증.
파이프라인: `정적 카메라 depth → robot_segmenter(로봇 마스킹) → nvblox(3D ESDF, base_link) → cuMotion`.

### 설치 (apt)
- `ros-jazzy-isaac-ros-nvblox`, `ros-jazzy-isaac-ros-cumotion-robot-segmenter`(레포는 §11 의 Isaac ROS release-4).
- realsense2_camera 가 `diagnostic_updater` 4.2.7 을 끌어와 ABI 깨짐 → `ros-jazzy-diagnostic-updater/-msgs` 동반 업그레이드(§11-2 와 동류).

### 구성 (자체 코드)
- 설정 `config/ur16e_2f85_d405/nvblox_cumotion.yaml`: nvblox_base 위 overlay. **3D ESDF**, voxel 0.02,
  `global_frame: base_link`, workspace bounding_box(±0.9, z 0.1~1.3, 바닥 제외), 근거리 통합거리 제한.
- 런치 `launch/ur16e_2f85_d405/ur16e_2f85_d405_nvblox.launch.py`: robot_segmenter(ComposableNode) + nvblox_node +
  `base_link→static_cam_depth_optical_frame` 정적 TF 를 한 번에. depth 토픽 기본 = 정적카메라(`/static_cam/depth/*`),
  `depth_image:=`/`depth_info:=` 로 교체 가능. `use_robot_segmenter`/`static_cam_tf` 인자.
- Isaac `ur16e_isaac_ros2.py`: `--with-static-cam`(워크스페이스 오버룩 정적 depth 카메라, `--static-cam-xyz/-target`),
  `--obstacle`(데모용 박스, `--obstacle-pose/-size`), 그리고 **시작 시 home 자세 초기화**(SingleArticulation;
  USD 기본 전관절0=팔 수평 대신 팔 위로). 정적카메라 포즈는 런치 `SCAM_TF` 쿼터니언과 동기해야 함.
- cuMotion moveit 런치: `read_esdf_world:=true` 면 RViz 설정을 `config/ur16e_2f85_d405/cumotion_nvblox.rviz`
  (nvblox `tsdf_layer` 복셀 + workspace 마커 표시)로 자동 전환.

### ★ 핵심 교훈 — eye-in-hand 가 아니라 **정적(외부) 카메라**로 매핑
처음엔 eye-in-hand D405 로 nvblox 를 돌렸으나, 손목카메라는 시야 대부분이 로봇 자신 + 팔과 함께 움직여 TSDF 가
로봇으로 오염 → cuMotion 이 **시작자세를 `Invalid c-space position: world collision detected` 로 거부**(모든 plan 실패).
NVIDIA 매니퓰레이션 레퍼런스처럼 **워크스페이스를 내려다보는 정적 카메라**로 매핑하니 깔끔하게 해결. eye-in-hand
D405 는 파지/비전 전용으로 유지(역할 분리). segmenter 는 정적카메라 시야에서도 로봇을 빼 자기충돌 잔상 방지.

### 겪은 함정 (확정)
1. **프레임 불일치(중요)**: cuMotion 은 ESDF 를 자기 로봇 base 프레임(XRDF `set_base_frame`=`base_link`)으로 요청.
   nvblox `global_frame` 이 다르면(`world`) → `Requested ... in base_link frame but nvblox is mapping in world frame.
   Sending empty grid` → cuMotion `World update failed`. → nvblox `global_frame: base_link` 로 맞춤(정적 base 라 OK).
2. **segmenter 잔상**: 마스킹 버퍼 부족 시 로봇 잔상이 ESDF 에 남아 시작자세 충돌 → `additional_buffer_distance` 0.12.
3. **eye-in-hand 자기오염**(위 교훈) → 정적 카메라로 전환.
4. RViz 복셀 viz: `static_esdf_pointcloud`/`static_occupancy_grid` 는 미발행 → **`tsdf_layer`(~15Hz)** 사용.
5. 노드 多 → Fast DDS SHM 포트 포화(`fastrtps_port ... open_and_lock_file failed`). plan/execute 는 정상이나
   신규 CLI/RViz 구독이 불안정할 수 있음 → 필요시 전체를 UDP 전송(`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`)으로 재기동.

### 검증 결과 (2026-06-22, sim)
| 항목 | 결과 |
|---|---|
| 정적 카메라 depth 발행(`/static_cam/depth/*`, frame `static_cam_depth_optical_frame`) | ✅ (~50–110Hz) |
| robot_segmenter masked depth(`/cumotion/camera_0/world_depth`) | ✅ (~40–70Hz; eye-in-hand 0.9Hz 대비 개선) |
| nvblox 3D ESDF(base_link) 서빙 + cuMotion 읽기 | ✅ `Successfully wrote requested ESDF` / `Updated ESDF grid successfully` |
| **빈 워크스페이스 plan**(home 시작) | ✅ `success: true` (18pt) |
| **plan+EXECUTE 전체 루프**(ESDF 활성, home→target) | ✅ `error_code=1`, Isaac 팔 이동, 오차 0.0003 rad |
| **장애물(박스) 매핑 + 회피 반영** | ✅ 아래 A/B 대조로 확정 |
| GUI(Isaac + RViz) 기동 | ✅ (복셀 viz 는 DDS SHM 포화 시 불안정 — §함정5) |

### 회피 A/B 대조 검증 (2026-06-22, sim) — 데모 `isaac/ur16e_2f85_d405/nvblox_obstacle_demo.py`
**동일한 task-space(pose) goal** 을 그리퍼(`gripper_frame`)로 보내, 장애물 부피 안의 점이
ESDF 켤 때만 막히는지(=정말 장애물 때문인지) 대조로 확인. 장애물 박스 = Isaac `--obstacle`
기본(center (0.5,0.3,0.6), size (0.12,0.12,0.5) → x[0.44,0.56] y[0.24,0.36] z[0.35,0.85]).

| 목표점 (gripper_frame, base) | `read_esdf_world:=true`(장애물 매핑) | `read_esdf_world:=false`(장애물 없음) |
|---|---|---|
| FREE = (0.5, **−0.3**, 0.6) 자유공간 | ✅ 성공 (`error_code=1`) | ✅ 성공 |
| OBST = (0.5, **+0.3**, 0.6) 장애물 내부 | ❌ 실패 (`INVERSE_KINEMATICS_FAILURE`) | ✅ **성공** |

→ 같은 점(도달 가능: 거울상 −y 와 ESDF-off 양쪽에서 성공)이 **ESDF 켤 때만** 막힘 =
nvblox 가 매핑한 장애물을 cuMotion 의 collision-aware IK 가 실제로 회피. (기존 "c-space goal 거부"
보다 강한 대조군 증명.) 데모는 ESDF 켠 상태에서 FREE 성공·OBST 실패면 `PASS` (exit 0).

**함정(이번에 확정)**: cuMotion **task-space(pose) goal 의 target link 는 XRDF end-effector
(`gripper_frame`)여야 함** — `tool0` 로 보내면 `Target link 'tool0' does not match end effector
'gripper_frame'` 로 거부(joint-space goal 은 무관, 그래서 `moveit_plan_execute_demo.py` 는 안 걸림).

---

## 13. 테이블(베이스 바닥) 충돌 회피 + 장애물 운영 — 2026-06-28

nvblox 스택을 sim 에서 재가동해 plan&execute 를 시험하던 중, cuMotion 이 **UR 베이스가 놓인 테이블/바닥을
뚫고 내려가는 경로**를 생성하는 문제를 확인하고 cuMotion 내장 ground plane 으로 해결. 더불어 정적 장애물
박스를 GUI 에서 조정·영속화하는 절차와, 그 과정에서 부딪힌 함정들을 정리.

### ★ 테이블 충돌 — perception 이 아니라 **cuMotion ground plane** 으로
- **원인**: nvblox `workspace_bounds_min_height_m: 0.1` 이라 **base_link 기준 z<0.1 은 ESDF 에 아예 안 담김** →
  테이블 상판(z≈0)이 cuMotion 에 "빈 공간"으로 보임. 게다가 **베이스 밑 테이블은 정적 카메라가 봐도 로봇에 가려져
  (occlusion) 절대 매핑 불가** → perception 으로 풀 문제가 아님(테이블은 알려진 고정 기하).
- **해결**: cuMotion(cuRobo) 내장 파라미터 **`add_ground_plane:=true`** 사용. base_link 밑
  `ground_plane_z_offset`(기본 -0.05) 에 2×2m 바닥면을 항상 두어 그 아래로 내려가는 경로를 거부.
  카메라 무관·occlusion 무관. `launch/ur16e_2f85_d405/ur16e_2f85_d405_cumotion_moveit.launch.py` 의
  `cumotion_planner` 파라미터에 `add_ground_plane: True` + `ground_plane_z_offset: -0.05` 박아 영속화.
- **역할 분담 확정**: nvblox ESDF = **z>0.1 의 카메라가 본 동적 장애물**, ground plane = **base 밑 알려진 테이블/바닥**.
- **튜닝**: 팔이 여전히 테이블을 긁으면 `ground_plane_z_offset` 을 0 쪽으로(-0.02~0.0), home 에서조차 충돌로
  plan 이 막히면 더 아래로(-0.08). 값만 바꾸고 cuMotion+moveit 레이어만 재기동.

### 정적 장애물 박스(`--obstacle`) 운영 — GUI 조정값 영속화
- 박스는 Isaac 스크립트가 **매 실행 코드로 생성**: `UsdGeom.Cube`(size=1.0) + `AddScaleOp(scale)`.
  즉 **`xformOp:scale` = 박스 실제 크기(m)** 이고 `--obstacle-size` 와 1:1.
- GUI 스케일 기즈모로 크기를 바꾸면 `scale` 만 바뀜 → **USD stage 저장은 무의미**(스크립트가 저장 stage 를
  재로드하지 않고 매번 새로 만듦). **영속화 = 스크립트 기본값(`--obstacle-size`) 갱신**이 정답.
- 이번 변경: `--obstacle-size` 기본 `0.12,0.12,0.5` → **`0.12,0.5,0.1`**(GUI Scale 과 일치, y 로 넓은 슬랩),
  `--obstacle-pose` 기본 `0.5,0.3,0.6` → **`0.5,0.1,0.6`**(GUI Translate 과 일치). 크기·위치 바뀌어도
  nvblox 가 카메라로 재매핑 → cuMotion 자동 회피(추가 수정 불필요).

### 운영 함정 (이번에 확정)
1. **정적 카메라 prim 을 GUI 에서 끌지 말 것**: `/World/static_cam` 의 Isaac 실제 포즈는 nvblox 가 발행하는
   **고정 TF `base_link→static_cam_depth_optical_frame`([1.10,0,1.10], look-at [0.30,0,0.15])** 와 짝.
   GUI 에서 옮기면 카메라만 이동하고 TF 는 그대로 → **depth 역투영이 어긋나 ESDF 가 엉뚱한 곳에 매핑**.
   실수로 옮겼으면 **Isaac 재시작**이 가장 깔끔(스크립트가 명목 포즈로 결정적 재생성). orientation 은 look-at
   계산값이라 손으로 못 맞춤.
2. **"plan 성공인데 팔이 안 움직임" ≠ 고장**: 보통 ① **시작자세 ≈ 목표(home→home)** 라 실행할 모션이 없거나,
   ② RViz 에서 Plan&Execute 를 연타/마커 재조작해 앞 실행이 **PREEMPTED('stop' 이벤트)**. 로그상
   `planning succeeded` 뒤 `Received event 'stop'`→`PREEMPTED`. 확인법: 직접 trajectory 를
   `/scaled_joint_trajectory_controller/joint_trajectory` 로 비-home 값 publish → Isaac 팔이 따라오면 경로 정상.
3. **GUI prim 조작 중 `KeyError: NoneType`(`omni.kit.manipulator.prim`)**: 선택/조작 핸들러의 비치명 예외 —
   articulation/물리엔 영향 없음.

### 검증 결과 (2026-06-28, sim)
| 항목 | 결과 |
|---|---|
| 전체 스택 clean 재기동(Isaac→제어→nvblox→cuMotion+RViz), static cam TF=[1.10,0,1.10] 일치 | ✅ |
| `add_ground_plane` 활성(`/cumotion_planner` 파라미터 `True`, z_offset -0.05) | ✅ |
| 직접 trajectory(shoulder_pan 0.5/lift −0.5) → Isaac 팔 추종 후 home 복귀 (명령경로 정상) | ✅ |
| 장애물 박스 크기 `0.12,0.5,0.1` 로 스크립트 영속화 | ✅ |
| 테이블 관통 경로 회피(ground plane) | ⏳ 사용자 RViz plan&execute 로 확인 중 |

---

## 14. Isaac Sim 5.1.0 → **6.0.1** 환경 이식 검증 — 2026-09-06

컨테이너가 Isaac Sim **6.0.1**(`6.0.1-rc.7+release.42383`)로 바뀐 뒤, 기존 스택 전체가 그대로 도는지
바닥부터 재검증. **자체 코드(`ur_bringup`)는 Isaac 6.0.1 때문에 고친 곳이 한 줄도 없다.** 막힌 것은
전부 ① 새 컨테이너에 아직 안 깔린 apt 패키지, ② **GPU 아키텍처(sm_120)** 문제였다.

### 환경
| 항목 | 값 |
|---|---|
| Isaac Sim | 6.0.1-rc.7 (`isaacsim.ros2.bridge` 5.1.2) |
| ROS 2 | Jazzy / Ubuntu 24.04 |
| GPU | RTX 5090 (**compute_cap 12.0 = sm_120**), 드라이버 580.173.02 |
| ros2_control | 4.45.2 (controller_manager/hardware_interface), ros2_controllers 4.40.1 |
| MoveIt | 2.12.4 / UR 스택 3.8.0 |

### Isaac Sim 6.0.1 API — 무수정 동작
`isaacsim.core.api.SimulationContext`, `isaacsim.core.utils.{extensions,prims,stage,viewports}`,
`isaacsim.storage.native.get_assets_root_path`, `isaacsim.core.prims.SingleArticulation`,
OmniGraph 노드(`isaacsim.ros2.bridge.ROS2{Context,PublishJointState,SubscribeJointState,PublishClock,
CameraHelper,CameraInfoHelper}`, `isaacsim.core.nodes.{IsaacReadSimulationTime,IsaacArticulationController,
IsaacCreateRenderProduct}`) — **전부 5.1.0 과 동일하게 유효**. 세트2/3 의 베이크된 USD
(`ur16e_with_2f85.usd`, `ur16e_2f85_d405.usd`)도 그대로 로드된다.

새로 뜨는 **비치명 경고**(동작 영향 없음):
- `[ROS2 Publish Joint State] Reading from targetPrim is deprecated. Connect an Isaac Read Joint State node
  and use its outputs instead.` — 6.x 권고사항. 관절 발행은 정상.
- `camera_info_utils: Forcing fy to fx (334.2222154 != 334.2222302)` — 렌더러가 정사각 픽셀을 가정.
- `DLSS increasing input dimensions: Render resolution of (320,240) is below minimal input resolution of 300`
- `omni.timeline: direct use of ITimeline callbacks is deprecated`

### ★ 이번에 확정된 함정 — nvblox apt 바이너리는 RTX 50 에서 못 돈다
`nvblox_node` 가 기동 직후 SIGABRT:
```
terminate called after throwing an instance of 'thrust::...::system::system_error'
  what():  parallel_for failed: cudaErrorInvalidDevice: invalid device ordinal
```
원인은 파라미터도 TF 도 아니고 **바이너리에 이 GPU 용 코드가 없어서**다:
```
cuobjdump --list-elf /opt/ros/jazzy/lib/nvblox_ros/nvblox_node | grep sm_   ->  sm_75   (PTX 없음)
nvidia-smi --query-gpu=compute_cap                                          ->  12.0    (= sm_120)
```
sm_75(Turing) 전용으로만 컴파일되어 있고 PTX 폴백이 없으니 JIT 도 불가능 → **소스 빌드가 유일한 해결책**
(`isaac_ros_nvblox` @ `release-4.6`, `-DUSE_NATIVE_CUDA_ARCHITECTURE=1`). 절차는 `SETUP.md` §2-B-4.
**cuMotion 은 무관** — `libcumotion.so` 는 `sm_75/86/89/120` 을 모두 포함해 apt 판이 그대로 돈다.
> 교훈: Isaac ROS apt 바이너리는 패키지마다 지원 arch 가 다르다. 새 GPU 로 옮기면
> `cuobjdump --list-elf <바이너리> | grep sm_` 로 먼저 확인할 것.

### ★ NVIDIA apt 레포는 ROS 패키지를 덮어쓴다 (공유 머신 필수 대비)
`isaac.download.nvidia.com/isaac-ros/release-4` 를 추가하면 ROS 공식 패키지를 **더 높은 버전으로 가로챈다**:
`robotiq_description` 0.0.1-3 → **9.0.1**, `moveit_task_constructor_core` → **99.99.0** 등.
같은 컨테이너의 다른 워크스페이스까지 같이 갈리므로, 레포보다 **핀을 먼저** 넣는다
(`/etc/apt/preferences.d/99-nvidia-isolate.pref`, Pin-Priority 100 — `SETUP.md` §2-B-1).
핀 적용 후 `apt-cache policy ros-jazzy-robotiq-description` 의 Candidate 가 ROS 쪽으로 돌아오면 정상.

또한 메타패키지를 피해 설치 규모를 통제했다(실측):
| 조합 | 결과 |
|---|---|
| `isaac-ros-cumotion{,-moveit,-examples,-robot-description}` + `isaac-ros-nvblox` (문서의 옛 목록) | **432 신규 + 17 업그레이드** (libc6·python3.12 포함) |
| cuMotion 4종(examples 제외) + segmenter + `nvblox-ros/-msgs/-rviz-plugin` | **90 신규 + 0 업그레이드** ✅ |
`nvblox_examples_bringup`(triton/tensor_rt/visual_slam/detectnet/unet 을 끌고 옴)에서 우리가 쓰는 건
`nvblox_base.yaml` 하나뿐이라 `config/ur16e_2f85_d405/vendor/nvblox_base.yaml` 로 vendoring 하고,
`ur16e_2f85_d405_nvblox.launch.py` 가 apt 판이 있으면 그걸, 없으면 vendored 판을 쓰도록 고쳤다.

### ★ 이번에 확정된 함정 2 — nvblox 4.6 의 "미관측 = 장애물" 기본값

nvblox 를 sm_120 으로 소스빌드해 살려도, cuMotion 이 **모든 시작자세**를 거부했다(home 포함):
```
Invalid c-space position: world collision detected.
Failed call to 'planToTaskSpaceTarget()': 'initial_cspace_position' [[0 -1.5706 0 0 0 0]] is invalid.
trajopt: INVALID_INITIAL_CSPACE_POSITION
```
`Updated ESDF grid successfully` 는 계속 찍히므로 nvblox↔cuMotion 연결 자체는 정상. ESDF 를 직접 떠서
값 분포를 보니 원인이 드러났다 — cuMotion 이 요청하는 AABB(511,560 복셀) 중 **307,666 개가 `-1000.0`**:
```
ros2 param get /nvblox_node esdf_and_gradients_unobserved_value  ->  -1000.0   (노드 기본값)
```
`-1000` 은 nvblox 의 **미관측(unobserved) 센티널**이고, cuMotion 은 거리 ≤ 0 을 "장애물 내부"로 읽는다.
정적 카메라 1 대로는 워크스페이스 박스의 대부분이 영영 미관측이고, **결정적으로 로봇 자신은 segmenter 가
depth 에서 일부러 지우므로 팔이 있는 공간이 바로 미관측**이다 → 어떤 자세든 "world collision".

**해결**: `config/ur16e_2f85_d405/nvblox_cumotion.yaml` 에 `esdf_and_gradients_unobserved_value: 1000.0`
(미관측 = FREE). 장애물은 카메라가 실제로 본 것에서만 나오고, 자기충돌/planning scene 체크는 그대로 유효.
적용 후 음수(=충돌) 복셀 **307,666 → 378 개**(진짜 장애물 내부만)로 정상화, A/B 데모 `PASS`.
> `ros2 param set` 으로는 안 바뀐다(둘 다 노드 기동 시 1회만 읽음). yaml 수정 후 재기동해야 한다.
> segmenter 의 `additional_buffer_distance` 도 같은 이유로 런치 인자 `segmenter_buffer` 로 노출했다
> (기본 0.12; 팔 잔상이 의심되면 0.2~0.3).

### 검증 결과 (2026-09-06, sim)
| 항목 | 결과 |
|---|---|
| `colcon build --packages-up-to ur_bringup` (topic_based 0.2.1 + ur_bringup) | ✅ |
| `colcon build --packages-select serial robotiq_driver robotiq_controllers` | ✅ |
| 세트1 Isaac 6.0.1 기동 → `/isaac_joint_states`(6관절, NaN 없음)·`/clock` | ✅ |
| 세트1 컨트롤러 활성(joint_state_broadcaster + scaled_joint_trajectory_controller) | ✅ |
| 세트1 MoveIt(OMPL) plan+execute (`moveit_plan_execute_demo.py`) | ✅ 오차 **0.0097 rad** |
| 세트2/3 URDF 생성(`robotiq_description` apt 설치 후, 메시 경로 전수 확인) | ✅ 24 / 31 links |
| 세트3 USD 로드 + eye-in-hand·정적 카메라·데모 장애물 | ✅ |
| 세트3 카메라 4토픽(`color/image_raw`,`depth/image_rect_raw`,`depth/color/points`,`static_cam/depth/*`) | ✅ 약 80 Hz |
| 세트3 `/isaac_joint_states` 12관절(팔6+2F-85 6) | ✅ |
| 세트3 컨트롤러 3종 활성(+`gripper_controller`) | ✅ |
| cuMotion move_group(`pipeline isaac_ros_cumotion`) plan+execute | ✅ 오차 **0.0003 rad** (기존 기록과 동일) |
| `robot_segmenter` 로드/카메라 포즈/intrinsics | ✅ |
| nvblox apt 바이너리 | ❌ sm_120 미지원 → 소스 빌드로 대체 |
| nvblox 소스 빌드(`USE_NATIVE_CUDA_ARCHITECTURE=1`) | ✅ `libnvblox_lib.so`/`libnvblox_ros_lib.so` = **sm_120** |
| nvblox 노드 기동 + 파이프라인 | ✅ 마스킹 depth 24 Hz → `tsdf_layer` 11 Hz, `/nvblox_node/get_esdf_and_gradient` 정상 |
| robot_segmenter 마스킹 실측 | ✅ 307,200 → 299,773 px (buffer 0.12) / 294,226 px (0.25) |
| cuMotion 이 nvblox ESDF 읽기 | ✅ `Initialized grid from nvblox` + `Updated ESDF grid successfully` |
| ESDF 미관측 처리 수정 후 충돌 복셀 | ✅ 307,666 → **378** |
| **실시간 회피 A/B (`nvblox_obstacle_demo.py`)** | ✅ **PASS** (FREE 성공 / OBST 거부) |

### 이번에 새 환경에서 추가로 설치해야 했던 것
새 컨테이너에는 `ur`/`moveit`/`ros2_control` 계열만 있었고 다음이 없었다:
`ros-jazzy-robotiq-description`(**없으면 세트2/3 URDF 가 아예 생성 안 됨**),
`ros-jazzy-realsense2-camera`+`librealsense2`(세트3 real), Isaac ROS cuMotion/nvblox 일체.
전부 `SETUP.md` §2 / §2-B 에 반영. 워크스페이스는 `build/install/log` 가 없는 상태여서 처음부터 빌드했다.

---

## 15. Teleoperation (MoveIt Servo) — IL pick&place 1단계 — 2026-09-06

`plan_il_vla.md` 의 teleop→IL 파이프라인 **1단계: "팔을 손으로 조종할 수 있게 만들기"** 구현·검증.
정책 실행(IL/VLA·RL)도 **같은 스트리밍 경로**를 쓰므로, 이 단계는 학습 배포의 토대이기도 하다.

```
게임패드/키보드 → TwistStamped → servo_node → /forward_position_controller/commands
                                            → ros2_control → Isaac / 실물 UR16e
                └→ /gripper_controller/gripper_cmd (GripperCommand)
```

### 추가된 것
| 파일 | 역할 |
|---|---|
| `config/common/ur16e_2f85_controllers.yaml` | **`forward_position_controller`**(JointGroupPositionController) 추가 |
| `config/common/ur16e_servo.yaml` | MoveIt Servo 설정 (**Jazzy 스키마 기준 신규 작성**) |
| `launch/common/teleop_servo.launch.py` | servo_node + 스트리밍 컨트롤러(inactive) 스폰 |
| `launch/common/teleop_dualsense.launch.py` | joy_node + teleop_joy |
| `scripts/teleop_joy.py` | 게임패드 → twist/그리퍼. **deadman 필수**. (CMakeLists `install(PROGRAMS)` 추가) |
| `isaac/common/switch_control_mode.py` | trajectory ↔ streaming 컨트롤러 전환 (정책 배포 때도 사용) |
| `isaac/common/reset_pose.py` | **`ready` 자세 추가** (아래 함정) |

### ★ 함정 1 — `home`/`up`/`zero` 는 전부 특이점이라 teleop 이 안 먹는다
전부 `elbow_joint = 0`(팔 완전신장) = **팔꿈치 특이점**. Servo 가 정상적으로 거부한다:
```
/servo_node/status → code: 2, "Very close to a singularity, emergency stop"
```
배선이 다 맞는데도 **"명령은 나가는데 팔이 안 움직이는"** 증상으로 보인다(`/forward_position_controller/commands`
는 50Hz 로 발행됨). → **`reset_pose.py ready`**(`[0,-1.5707,1.5707,-1.5707,-1.5707,0]`, 팔꿈치 90°) 로
옮기고 시작할 것. 데모 기록 시 에피소드 리셋 자세도 `home` 이 아니라 `ready` 를 쓴다.

### ★ 함정 2 — Servo yaml 은 예제를 그대로 못 쓴다 (두 겹의 구조 요구)
1. `moveit_servo/config/*.yaml` 예제는 **flat** 이다(데모가 ParameterBuilder 로 로드하기 때문).
   런치에서 `--params-file` 로 주면 죽는다: `Cannot have a value before ros__parameters`.
2. 감싸도 또 죽는다: `parameter 'moveit_servo.move_group_name' is not initialized`.
   generate_parameter_library 인스턴스명이 `moveit_servo` 라 **한 겹 더** 필요하다.

최종 구조:
```yaml
/**:
  ros__parameters:
    moveit_servo:
      move_group_name: ur_manipulator
      ...
```
3. **`ur_moveit_config/config/ur_servo.yaml` 을 재사용하지 말 것** — 구(rewrite 이전) Servo API 용이라
   `planning_frame`/`ee_frame`/`robot_link_command_frame`/`low_latency_mode`/`system_latency_compensation`/
   `collision_check_type`/`num_outgoing_halt_msgs_to_publish`/`low_pass_filter_coeff`/`use_gazebo` 등
   Jazzy 스키마에 없는 키투성이다. 권위 있는 스키마 = `moveit_servo/config/servo_parameters.yaml`.

### ★ 함정 3 — 팔 컨트롤러 2개는 상호배타
`scaled_joint_trajectory_controller`(궤적)와 `forward_position_controller`(스트리밍)는 **같은 position
command interface** 를 잡으므로 동시에 active 불가. teleop 런치는 스트리밍 쪽을 **`--inactive`** 로 스폰하고,
`switch_control_mode.py` 가 원자적으로 교체한다. `reset_pose.py` 는 궤적 컨트롤러를 쓰므로
**trajectory 모드에서만** 동작한다.

### 검증 결과 (2026-09-06, sim 세트3)
| 항목 | 결과 |
|---|---|
| servo_node 기동 (`Servo initialized successfully`) | ✅ |
| 컨트롤러 모드 전환 trajectory ↔ streaming | ✅ 원자적 전환, 상태 조회 정상 |
| `switch_command_type(TWIST)` | ✅ success |
| twist → 팔 이동 (`ready` 자세에서) | ✅ max 0.199 rad / 3s, status `code 0 No warnings` |
| **[A] deadman 미입력 시 정지** | ✅ max\|d\| = 0.0000 (안전) |
| **[B] deadman + 스틱 → 이동** | ✅ max 0.353 rad |
| **[C] deadman 해제 → 즉시 정지** | ✅ max\|d\| = 0.0000 |
| **[D] R2 → 그리퍼 닫힘** | ✅ finger_joint 0.000 → 0.723 |
| **[E] L2 → 그리퍼 열림** | ✅ 0.723 → 0.000 |
> 패드 미연결 상태라 **합성 `/joy` 메시지**로 검증했다(DualSense 축 배치 그대로). 실물 패드 연결 시
> `ros2 topic echo /joy` 로 축/버튼 인덱스만 확인해 런치 인자로 덮으면 된다.
> 패드 없이 즉시 확인하려면 `ros2 run moveit_servo servo_keyboard_input`.

### ★ 함정 4 — `/joint_states` 가 sim(7관절) 과 real(6관절) 로 달랐다 (구조 결함, 수정함)

teleop 을 **실물 경로(mock 하드웨어)** 로 검증하다 발견. Servo 가 영원히
`Waiting to receive robot state update` 에서 멈춘다.

원인: sim 은 controller_manager 하나가 팔+`finger_joint` 를 모두 `/joint_states` 에 발행하지만,
real 은 그리퍼가 **별도 네임스페이스의 CM** 이라 `/gripper/joint_states` 로만 나가고 전역
`/joint_states` 는 **팔 6관절뿐**이었다. Servo 의 로봇모델(팔+그리퍼)이 요구하는 `finger_joint` 가
영영 안 와서 CurrentStateMonitor 가 완성되지 않는다.

> 이건 teleop 만의 문제가 아니다. README §1 이 내세우는 **"토픽·프레임은 sim/real 동일"** 원칙이
> `/joint_states` 에서 이미 깨져 있었고, **IL 데이터 기록기도 real 에서 그리퍼 상태를 조용히 놓쳤을** 것이다.

**수정**: `robotiq_2f85_real.launch.py` 의 `joint_state_broadcaster` 를 전역 토픽에 발행하도록
리맵(`--controller-ros-args "-r joint_states:=/joint_states"`), 런치 인자 `global_joint_states`(기본 true)로 제어.
`/joint_states` 에 발행자가 여럿인 것은 표준 ROS 패턴이며 robot_state_publisher 와 MoveIt
CurrentStateMonitor 가 알아서 병합한다.
검증: real(mock) `/joint_states` 에서 **7관절 전부 수신**(3438 msg / 6s), Servo 정상 진입.

### 검증 결과 — 실물 경로 (2026-09-06, mock 하드웨어)
| 항목 | 결과 |
|---|---|
| `ur16e_2f85_d405_real.launch.py`(mock) 기동 | ✅ |
| 실물 드라이버가 `forward_position_controller`·`freedrive_mode_controller` 를 **이미 제공** | ✅ (우리가 추가할 필요 없음) |
| `switch_control_mode.py` 실물 CM 에서 동작 | ✅ (`ur_controllers/ScaledJointTrajectoryController` 인식) |
| teleop_servo 런치 (spawner 가 기존 컨트롤러 tolerate) | ✅ `Controller already loaded, skipping` |
| `/joint_states` 7관절 (수정 후) | ✅ |
| twist → 팔 이동 | ✅ max **0.8175 rad** |
➡ **teleop 계층은 sim/real 무수정 공용**. `use_sim_time` 만 다르다.

### 남은 것 (2단계 이후)
씬 준비(물체·place 타깃·**외부 RGB 카메라** — 현 정적카메라는 depth 전용) → LeRobot 데이터셋 writer
(30Hz 리샘플, `finger_joint` 스칼라만, 태스크 3종 언어 지시문) → ACT 학습 → closed-loop 롤아웃.

---

## 16. T2 씬 구성 + **파지 물리 검증 (미완료 — 차단 원인 규명)** — 2026-09-06

`plan_il_vla.md` T2(씬 준비). **T2-1 파지 검증을 최우선으로 앞당겼다** — 씬을 다 만든 뒤에
"그리퍼가 물체를 못 잡는다"를 발견하는 것이 최악이기 때문. 결과적으로 그 판단이 맞았다.

### 사전 구조조사에서 확인한 것 (직접 짜기 전에)
| 항목 | 실제 |
|---|---|
| `--obstacle` 박스 | **시각 전용**(rigid body/collider 없음). 파지 대상으로 못 씀 |
| 정적 카메라 | **depth 전용, RGB 없음** → IL 정책 입력이 없다 (T2-3 과제) |
| 물체/물리 스폰 코드 | 리포지토리에 **전무** |
| `gripper_demo.py` | `finger_joint` 움직임만 검증 — **물체 파지는 미검증이었음** |
| Isaac `DynamicCuboid`/`FixedCuboid`/`VisualCuboid`/`PhysicsMaterial` | **제공됨** → rigid body+collider+mass+재질을 한 호출로. 단 6.0.1 에서 `isaacsim.core.api` 는 **`extsDeprecated/`** 소속(동작함). 후속 = `isaacsim.core.experimental.objects` |

### 구현한 것 (동작 확인됨)
- `ur16e_isaac_ros2.py` `--scene pick_place` : 작업면(`--table`) + **물리 물체**(DynamicCuboid,
  질량/마찰 인자) + place 마커(VisualCuboid). 기존 nvblox/obstacle 경로 불변(전부 opt-in).
- `/scene/object_pose` (GT, **검증 전용 — 데이터셋/정책 입력 아님**)
- **`/scene/reset_episode`** (std_srvs/Trigger) — 물체 재배치(+`--randomize-object`/`--seed`).
  **sim/real 공용 리셋 계약**의 sim 절반(real 은 같은 서비스명으로 사람에게 배치 요청).
- `isaac/ur16e_2f85/grasp_test.py` : 접근→하강→close→lift 후 **물체 z 상승**으로 PASS/FAIL 판정.

### ★ 파지 검증 결과 = **FAIL. 원인 3개 규명**

**① 하드코딩 관절 waypoint 는 함정 — UR 손목 측면 오프셋**
`shoulder_pan=0` 이면 tool0 가 y=0 이 아니라 **y=+0.174 m**(UR16e 손목 오프셋). 손으로 짠 파지
자세가 물체에서 17 cm 빗나가 "파지 실패"로 보였다. → IK/플래너로 자세를 구해야 한다.

**② 물체가 MoveIt planning scene 에 없다 → 팔이 물체를 쳐서 날린다**
접근 중 물체가 (0.55,0) → (0.47,−0.46) 로 **46 cm** 밀려났다. 플래너는 물체를 모르므로 그리퍼를
그대로 관통시킨다. 완화책으로 접근고도 상향(0.30 m)+단계적 수직 하강을 넣었으나 근본책은
**물체를 planning scene 에 collision object 로 넣고 파지 순간 attach** 하는 것(`to_do.md` M3 가 이미 계획).

**③ ★★ 실패가 관절을 망가뜨리고, 그 뒤 모든 테스트가 거짓 실패한다**
실패한 파지 시도가 반복되면서:
```
finger_joint = -1.17 rad      (한계 0~0.8 완전 이탈 — 물체가 손가락을 강제로 벌림)
wrist_2_joint = -69.0 rad     (약 11회전 감김)
```
그러면 MoveIt 이 **모든** plan 을 거부한다:
```
CheckStartStateBounds: Joint 'wrist_2_joint' ... outside bounds by [-68.2953]
  should be in [-6.28319, 6.28319]  ->  move_action error_code -26 (START_STATE_INVALID)
```
- 이 `-26` 은 "자세 도달 실패"처럼 보이지만 실제로는 **시작상태 불량**이다. 혼동 주의.
- `/isaac_joint_states`(Isaac 원본)에도 −69 가 그대로 나온다 → 우리 `sum_wrapped_joint_states`
  파라미터 탓이 **아니고**, sim 관절이 실제로 감긴 것.
- **`reset_pose.py ready` 가 `error_code 0` 을 반환하면서도 wrist_2 를 복구하지 못한다.**
  (그 스크립트의 존재 이유가 바로 이 windup 복구인데, 이 상황에서는 듣지 않는다.)
- **유일하게 확실한 복구 = Isaac 재기동.** 재기동 후 `wrist_2=0.0, finger_joint=0.0` 확인.

> **작업 규율**: 파지 실험을 반복할 때는 **매 실패마다 관절 상태를 확인**하고, 한계 이탈이 보이면
> 곧바로 Isaac 을 재기동할 것. 안 그러면 이후 모든 결과가 오염되고 엉뚱한 원인을 쫓게 된다.
> (이번에 실제로 그렇게 몇 라운드를 낭비했다.)

### ★ D5 폴백(fixed-joint attach) 구현 — **성공. 파이프라인 뚫림** (2026-09-06)

접촉 물리를 튜닝하는 대신 `to_do.md` D5 폴백을 넣었다: 그리퍼가 닫히면 물체를 **USD fixed joint 로
그리퍼에 용접**하고, 열리면 푼다. IL 데이터 수집에 필요한 건 "닫으면 물체가 따라온다"는 사실뿐이다.

**구현** (`ur16e_isaac_ros2.py --grasp-attach`)
- `UsdPhysics.FixedJoint` + 현재 상대 pose 를 localPos/Rot 로 보존
  (참조 패턴: `isaacsim.robot_setup.assembler.robot_assembler._create_fixed_joint`.
   그 모듈은 GUI 확장 소속이라 standalone 에 끌어오지 않고 계산만 인라인)
- **용접 중 물체 collider 를 끈다** — 안 그러면 손가락이 계속 물체를 파고들어 관절이 한계를 이탈한다
- 자동(그리퍼 닫힘+근접) + 수동 서비스 `/scene/attach_object`·`/scene/detach_object`,
  상태는 `/scene/grasp_active`
- **자동은 의도된 설계**: teleop 중 조작자는 트리거만 당기면 되고, 실물과 동작이 같다.
  서비스를 손으로 불러야 하면 데모 타이밍이 왜곡되고 sim/real 대칭이 깨진다.
- **정책에는 보이지 않는다**: 정책은 이미지+관절만 본다(실물에서는 실제 마찰이 그 역할).
  **grasp 상태를 데이터셋에 넣지 말 것.**

**★ 함정 4 — URDF↔USD 이름 불일치가 링크에도 있고, `base_link` 는 이름이 겹친다**
처음에 `--grasp-link robotiq_85_base_link`(URDF 이름)로 프림을 찾다가 조용히 실패했다. USD 실제 이름:

| URDF | USD 실제 경로 |
|---|---|
| `robotiq_85_base_link` | `/UR16e/wrist_3_link/gripper/Robotiq_2F_85/`**`base_link`** |
| `robotiq_85_left/right_finger_tip_link` | `left_inner_finger` / `right_inner_finger` |

**그리퍼 베이스가 로봇의 `base_link` 와 이름이 같다** → 이름 기반 조회는 엉뚱한 프림을 잡는다.
→ 인자를 **USD 프림 경로**로 바꾸고, 못 찾으면 **에러 로그**를 남기게 했다(조용한 실패가 한 라운드를 먹었다).

**함정 5 — 수동 attach 를 자동 해제가 즉시 취소**
`/scene/attach_object` 로 붙여도 그리퍼가 열려 있으면(`finger_joint≈0`) 자동 해제 규칙이 바로 떼어냈다.
→ `manual` 플래그를 두어 서비스로 붙인 것은 `detach_object` 전까지 유지.

**검증 결과 (2026-09-06, sim)**
| 항목 | 결과 |
|---|---|
| 프림 경로 해석 | ✅ 실패 0건 |
| `/scene/attach_object` | ✅ `welded (manual hold)` |
| **팔 이동 시 물체 추종** | ✅ **(0.550, 0.000, 0.040) → (0.009, 0.375, 1.665)** = **+1.62 m 상승** |
| `/scene/detach_object` → 낙하 | ✅ z 0.040 복귀, `/scene/grasp_active=false` |
| **관절 건전성** | ✅ `finger_joint=0.0`, `wrist_2=0.0` — **한계 이탈 없음**(함정 ③ 동시 해소) |

> **D5 가 관절 붕괴(③)까지 고친다**: 용접 경로는 손가락을 물체에 밀어넣지 않으므로
> `finger_joint` 가 한계를 벗어나지 않고, 뒤따르던 wrist windup 도 발생하지 않는다.

### 남은 일 (T2)
1. ~~파지~~ → **D5 로 해결.** 단 `grasp_test.py` 의 **scripted 접근 경로**는 아직 물체를 쳐낸다
   (플래너가 물체를 모른 채 스윕). **teleop 에서는 사람이 위치를 잡으므로 이 문제가 없다** —
   즉 **teleop 데모 수집은 지금 진행 가능**하다. scripted pick&place(M3)에서 다시 다룬다.
2. T2-3 정적 카메라 **RGB 추가**(depth 유지) — IL 정책 입력.
3. T2-5 실물 외부 카메라 런치(`d405_real` 패턴 재사용, 동일 토픽).

---

## 17. T2-3 정적 카메라 RGB + TF 분리 — 2026-09-06

IL/VLA 정책은 **RGB + 관절**만 먹는데, 정적(외부) 카메라가 **depth 만** 내보내고 있었다. 즉
`video.exterior` 로 쓸 입력이 없었다. RGB 를 추가하고, 그 과정에서 구조 문제 하나를 같이 고쳤다.

### 한 것
1. **정적 카메라 그래프에 RGB + ColorInfo 추가** (`ur16e_isaac_ros2.py --with-static-cam`)
   - `/static_cam/color/image_raw`, `/static_cam/color/camera_info` (frame `static_cam_color_optical_frame`)
   - depth 는 그대로 유지 → nvblox 경로 불변
   - **손목 카메라와 같은 render product 패턴**(RGB·depth 를 한 센서에서) 을 그대로 복제
2. **★ 정적 카메라 TF 를 `launch/common/static_cam_tf.launch.py` 로 분리**
   - 문제: 이 TF 의 유일한 발행자가 **nvblox 런치**였다. teleop/IL 은 외부 카메라가 필요하지만
     ESDF 매퍼를 돌릴 이유가 없는데, TF 를 얻으려면 nvblox 를 켜야 했다.
   - nvblox 런치는 이제 이 공용 런치를 `IncludeLaunchDescription` 으로 포함(포즈 정의는 한 곳).
   - color optical frame 은 depth frame 의 **identity 자식**으로 발행 — sim 은 두 이미지를 한
     render product 에서 뽑으므로 정확히 일치한다. **실물 D435/D455 는 baseline 이 있으므로**
     `publish_color:=false` 로 두고 카메라 드라이버가 프레임을 소유하게 한다.

### ★ IL/VLA 에는 nvblox 가 필요 없다 (확정)
정책 입력이 RGB+관절뿐이라 **nvblox·cuMotion·foundation perception 은 IL 경로에 불필요**하다.
IL 최소 기동은 **Isaac + 제어 + static_cam_tf + teleop** 4 개다(`plan_il_vla.md` §7-B).
nvblox/cuMotion 은 `to_do.md` 의 scripted pick&place 와 RL 트랙에서 계속 쓰인다 — **버리는 게 아니라 트랙이 다르다.**
(이번 nvblox 기동은 TF 분리에 대한 **회귀 검증** 목적이었다.)

### 검증 결과 (2026-09-06, sim)
| 항목 | 결과 |
|---|---|
| `/static_cam/color/image_raw` | ✅ 640x480 `rgb8`, **~54 Hz**, frame `static_cam_color_optical_frame` |
| 이미지 내용(빈 렌더 아님) | ✅ 픽셀 평균 133.3, 범위 [6, 222] |
| `/static_cam/color/camera_info` | ✅ 640x480, fx 334.22, cx 320.0, cy 240.0 |
| 손목 카메라와 형식 대칭 | ✅ 둘 다 640x480 `rgb8` |
| `static_cam_tf.launch.py` 단독 실행 | ✅ depth·color 두 프레임 모두 `[1.100, 0.000, 1.100]` (Isaac 포즈와 일치) |
| **nvblox 회귀** (TF 분리 후) | ✅ 노드 생존, TF 정상, `tsdf_layer` 10.6 Hz, ESDF 서비스 존재 |

---

## 18. T3 — IL 데모 기록기 (LeRobot) — 2026-09-06

teleop 시연을 학습 데이터로 남기는 단계. **스키마는 `plan_il_vla.md` §2.6 이 정본**이고, 코드는 그걸 따른다.

### ★ 왜 2단계로 나눴나 (ROS 에서 LeRobot 을 직접 못 쓴다)
ROS 2 Jazzy 파이썬 환경 실측:

| 있음 | 없음 |
|---|---|
| `cv_bridge`, `cv2` 4.6, `PIL`, `numpy`, `message_filters` | **`pyarrow`/`pandas`(parquet), `ffmpeg`/`av`(mp4), `lerobot`, `torch`** |

LeRobot 온디스크 포맷은 parquet + mp4 가 필수다. 그걸 위해 ML 스택을 ROS 프로세스에 넣는 것은
**격리 원칙**과 `plan_il_vla.md` §2.2(환경 경계) 에 정면으로 어긋난다. → **분리**:

```
[ROS2 Jazzy]                                   [ML 환경 (torch+lerobot)]
scripts/il_recorder.py ──raw 에피소드──▶ scripts/raw_to_lerobot.py ──▶ LeRobot 데이터셋
 (cv2 JPEG + JSON, 30Hz 리샘플)                  (공식 API로 parquet/mp4)
```
raw 포맷은 자기서술적(`meta.json` 에 스키마·관절명·그리퍼 범위·action 출처·rate 기록) + 버전 필드.

> **rosbag2 는 검토 후 기각**: 640x480 2대 무압축이 ~100 MB/s 이고, 리샘플·에피소드 경계·성공 라벨은
> 어차피 별도 구현이 필요하다.

### ★ teleop 장치 무관 설계 (키보드/DualSense → OMY leader → GELLO 로드맵 대응)
관측은 언제나 `/joint_states` 라 장치와 무관하다. **다른 건 action 이다**:

| 장치 | 제어 경로 | action 의 자연스러운 출처 |
|---|---|---|
| 키보드 / DualSense | EE twist → Servo → `forward_position_controller` | leader 없음 |
| **OMY leader / GELLO** | **관절 직결** → `forward_position_controller` (IK 없음) | **leader 관절값**(ALOHA 관례) — **§49.1 정정**: 실제로는 브리지의 매핑 후 명령 토픽이어야 한다 |
| UR freedrive(실물) | 명령 없음, 읽기만 | 없음 |

→ `--action-source` 파라미터:
- **`next_state`(기본)**: `action[t] = state[t+1]`. **모든 장치에서 동작**(freedrive 포함)
- **`topic`**: `action_topic` 으로 지정한 JointState(예: leader 팔의 관절) 를 그대로 기록.
  **OMY leader / GELLO 연결 시 이 모드로 바꾸면 끝** — 기록기 코드 변경 불필요.

### 인터페이스
- 서비스 `/il/start_episode`, `/il/stop_episode`(저장), `/il/discard_episode` — 전부 `std_srvs/Trigger`
  (새 인터페이스 패키지를 만들지 않으려고 task 는 ROS **파라미터**로 둠)
- 상태 `/il/status`(String) — `idle (ready)` / `idle (NOT ready: ...)` / `recording ...`
- **게임패드 버튼 연동**: Square=start, Triangle=stop+save, Cross=discard
  (`teleop_joy.py`). 조작자가 패드에서 손을 떼지 않아도 된다. 기록기가 없으면 무해하게 무시.
- `stop` 시 `/scene/reset_episode` 자동 호출(`auto_reset`), 프레임 수가 `min_episode_frames` 미만이면 자동 폐기

### 검증 결과 (2026-09-06, sim 세트3 + teleop 스택)
| 항목 | 결과 |
|---|---|
| 기록기 준비 감지 | ✅ `idle (ready)` — 카메라 2대 + 7관절 확인 후에만 |
| 에피소드 기록 | ✅ 228 프레임, 21 MB(JPEG q92, 2 cam) |
| 프레임 수 일치 | ✅ exterior 228 / wrist 228 / data.json 228 |
| **샘플링 정확도** | ✅ **33.3 ms ± 0.0 ms = 정확히 30.0 Hz** |
| 결측·NaN | ✅ 0 |
| **action 규약** | ✅ `action[t] == state[t+1]` 오차 **0.00e+00**, 마지막 프레임 반복 |
| 그리퍼 정규화 | ✅ [0, 1] 범위 |
| 이미지 무결성 | ✅ 456 장 전수 검사, 손상 0 |
| 이미지 내용 | ✅ 640x480 uint8, 첫↔끝 차이 exterior 4.8 / wrist 56.5 (정지화면 아님) |
| `discard_episode` | ✅ 디렉토리 삭제 확인 |
| 변환기 dry-run | ✅ 에피소드/프레임/태스크 집계 + **언어 다양성 경고 작동** |

### 남은 것
- 변환기의 **실제 LeRobot 변환은 ML 환경이 생긴 뒤 검증**해야 한다(현재 torch/lerobot 미설치).
  lerobot 릴리스마다 dataset API 가 달라지므로, 어긋나면 **변환기만 고칠 것** —
  기록기를 고치면 이미 모은 데이터가 무효가 된다.
- 태스크 3종 수집(§2.8): 지금은 1종만 넣어도 경고가 뜨도록 해 두었다.

---

## 19. ML 환경 구축 (torch sm_120 + LeRobot) — 2026-09-06

IL/VLA 트랙의 학습·데이터변환용 환경. **`plan_il_vla.md` §1 이 "미설치"로 비워뒀던 공백을 채웠다.**

### 격리 — 워크스페이스 로컬 venv
```
deps/.venv-ml/   (7.5 GB, include-system-site-packages=false)
```
기존 `deps/.venv-cumotion` 선례를 따랐다. `deps/` 는 git repo(`src/`) 밖이라 자동으로 추적 제외되고,
`rm -rf` 하면 완전히 원복된다. **시스템 python 과 Isaac python 에는 torch 를 넣지 않았고**,
`check_env.sh` 가 매번 두 곳의 누출을 실제로 검사한다.

### ★ 함정 1 — `python3-venv` 를 apt 로 깔면 안 된다
`python3 -m venv` 는 되지만 이 이미지엔 `ensurepip` 이 없어 venv 안에 pip 이 없다. 그런데
`apt install python3-venv python3-pip` 은 **python3.12 를 시스템 전역으로 업그레이드**한다(실측 7개 패키지).
우리 격리 규칙 위반이자 `apt_guarded_install` 이 거부할 대상.
→ **venv 안에만 `get-pip.py` 로 부트스트랩.** 이후 시스템 python 은 여전히 pip 없음을 확인했다.

### ★ 함정 2 — torch 는 반드시 CUDA 인덱스에서 (sm_120)
기본 PyPI `pip install torch` 는 sm_120 커널을 보장하지 않는다. 없으면 **GPU 는 보이고 텐서도 올라가는데
커널이 안 도는** — `HISTORY.md` §14 의 nvblox 와 똑같은 실패 모드다.
```bash
pip install --index-url https://download.pytorch.org/whl/cu128 torch torchvision
```
설치본: **torch 2.11.0+cu128 / torchvision 0.26.0+cu128**
(2026-09 기준 인덱스: `cu128`→2.11.0, `cu130`→2.14.0)

**검증(arch 목록만 보지 말고 실제 연산까지)**:
```
archs: sm_75 sm_80 sm_86 sm_90 sm_100 sm_120     <- sm_120 포함
GPU  : NVIDIA GeForce RTX 5090, capability (12, 0)
실제 GPU matmul(2048x2048): OK
```
`setup.sh ml` 은 이 검사를 **torch 직후 + lerobot 직후 두 번** 한다 —
나중 의존성이 torch 를 다른 빌드로 조용히 바꿔치기할 수 있기 때문. (이번엔 유지됨을 확인)

### ★ 함정 3 — `lerobot` 만으로는 부족, `lerobot[dataset]` 필요
`pip install lerobot` 만 하면 `lerobot.datasets` import 가
`'datasets' is required but not installed` 로 실패한다. → `lerobot[dataset]`.

### ★ 변환기 버그 2건 — 이번 검증으로 발견·수정
T3 에서 ML 환경이 없어 검증하지 못했던 `raw_to_lerobot.py` 의 API 가정이 **둘 다 틀렸다**:
1. **import 경로**: `lerobot.common.datasets...` (≤0.5) → **0.6+ 는 `lerobot.datasets...`**
   → 신경로 우선 + 구경로 폴백으로 수정
2. **`add_frame(frame, task=...)` 은 0.6 에서 없음** — `add_frame(self, frame)` 뿐이고
   **task 는 frame dict 안에** 넣어야 한다 → 수정
> 이래서 "**어긋나면 변환기만 고치고 기록기는 건드리지 않는다**"는 규칙이 중요하다.
> 기록기를 고쳤으면 이미 모은 raw 데이터가 무효가 됐을 것이다.

### torchcodec 경고는 무해 (조치 불필요)
lerobot 이 끌어오는 `torchcodec` 의 `.so` 는 **시스템 ffmpeg 공유 라이브러리**를 요구해 로드에 실패한다
(`imageio-ffmpeg` 는 바이너리만 제공). lerobot 이 **자동으로 pyav 로 폴백**하고 정상 동작한다.
→ 경고를 없애려고 apt 로 ffmpeg 을 깔지 말 것: 미관상 경고 하나에 시스템 전역 변경은 과하다.

### 검증 결과 (2026-09-06)
| 항목 | 결과 |
|---|---|
| venv 격리 | ✅ 시스템 python·Isaac python 모두 torch 없음 |
| torch sm_120 | ✅ arch 목록 포함 + **RTX 5090 실제 matmul 성공** |
| lerobot | ✅ 0.6.1 (`[dataset]` extra 포함) |
| lerobot 설치 후 torch 유지 | ✅ 2.11.0+cu128, sm_120 유지(바꿔치기 없음) |
| **raw → LeRobot 실제 변환** | ✅ 2 에피소드/48 프레임, **mp4 인코딩 성공** |
| 생성 레이아웃 | ✅ `data/*.parquet`, `videos/observation.images.{exterior,wrist}/*.mp4`, `meta/{info.json,stats.json,tasks.parquet,episodes/}` |
| **round-trip 재로드** | ✅ 2 ep / 48 frame / fps 30, **태스크 2종 복원**, `observation.state`(7,) `action`(7,) `images`(3,480,640) |
| `check_env.sh` | ✅ environment OK |

### 남은 것
- ACT 학습 실행(0단계) — 이제 환경은 준비됨
- `il_recorder` 로 모은 **실데이터**로 변환 재확인(이번엔 합성 raw 로 검증했다)

---

## 20. OMY leader ↔ UR16e 기구학 대조 + 참고 저장소 검토 — 2026-09-06

`plan_il_vla.md` §3.5 가 "OMY 는 UR16e 와 관절 배치가 달라 **IK 리타게팅이 필요**할 것"이라고
**추측**해 둔 것을 실제 모델로 검증했다. **결론: 그 추측은 틀렸다. 관절 직결이 성립한다.**

### 방법
- OMY: `ROBOTIS-GIT/robotis_mujoco_menagerie` → `robotis_omy/omy.xml` (MJCF) 의 joint axis/range/body pos
- UR16e: 우리 `ur16e_sim.urdf.xacro` 를 xacro 전개 후, 각 관절 `origin rpy` 를 누적 곱해
  **관절축을 base 좌표계로 환산**(UR URDF 는 축을 전부 `0 0 1` 로 두고 회전을 origin 에 넣기 때문에
  axis 벡터만 보면 비교가 안 된다 — 이게 함정)

### 결과 — 축 순서가 완전히 같다
| # | UR16e (실측) | OMY (실측) |
|---|---|---|
| J1 | **Z (yaw)** | **Z (yaw)** |
| J2 | **Y (pitch)** | **Y (pitch)** |
| J3 | **Y (pitch)** | **Y (pitch)** |
| J4 | **Y (pitch)** | **Y (pitch)** |
| J5 | **Z (yaw)** | **Z (yaw)** |
| J6 | **Y (pitch)** | **Y (pitch)** |

둘 다 `yaw–pitch–pitch–pitch–yaw–pitch` + **오프셋 손목**. 링크 길이만 다르다:

| | 상완 | 전완 | 손목 오프셋 |
|---|---|---|---|
| UR16e | 478.4 mm | 360 mm | 174.2 / 119.9 / 116.6 mm |
| OMY | 247 mm | 219.5 mm | 121.5 / 113 / 115.5 mm |

OMY ≈ UR16e 의 **축소판**(reach 580 vs 900 mm) → GELLO 가 말하는 **"축소된 기구학적 동형 복제본"**
과 같은 관계. **IK 리타게팅 불필요, 관절 1:1 매핑으로 충분.**

> ⚠️ **아래 관절 한계 서술은 §21 에서 정정되었다** (F3M 값을 L100 인 줄 알고 적었음).
> ~~`J3(elbow)` 한계만 다르다: OMY ±2.62 rad ⊂ UR16e ±3.1416 rad → 안전 방향.~~
> **실제로는 L100(리더) J3 = ±180° 로 UR16e elbow 와 한계가 같아 마진이 없다.** → §21

> 미확인(당시): `omy.xml` 은 그리퍼가 달린 **F3M(팔로워)** 모델이고 **L100(리더) 사양 미확인**.
> → **§21 에서 공식 HW 데이터로 해소.**

### 참고 저장소 판정
| 저장소 | 판정 | 근거 |
|---|---|---|
| `charlie8612/lerobot_teleoperator_omy` | ✅ **그대로 사용** | LeRobot Teleoperator 플러그인(`omy_leader`), **ROS 2 불필요**(Dynamixel Protocol 2.0 직접), `get_action()` → `joint_1..6.pos`+`gripper.pos`(rad). ★ README: *"Mapping these onto a different follower's joint space is **intentionally left to a downstream LeRobot processor**"* — **저자가 follower 매핑을 의도적으로 비워둔 범용 L100 리더**라 우리 상황에 정확히 맞음 |
| `ROBOTIS-GIT/robotis_mujoco_menagerie` | ✅ **기구학 근거** | 위 대조표의 출처 |
| `ROBOTIS-GIT/physical_ai_tools` | ⬜ **참고만** | ROBOTIS 공식 LeRobot+ROS2 스택이지만 **OMY/AI Worker 중심**이라 UR16e 에 부적합. 우리 `il_recorder.py` 가 이미 동등 기능 검증 완료(§18/§19). 스키마 키 명명 대조 정도의 가치 |

### OMY 연결 시 할 일 (§3.5 에 상세)
1. `lerobot_teleoperator_omy` 설치(`deps/.venv-ml`) 2. **부호·오프셋 캘리브레이션**(플러그인이 노출 안 함)
3. **`omy_to_ur16e` 브리지 노드**(관절 1:1, IK 없음) 4. 속도상한+deadman 5. 기록은 `--action-source topic` 그대로(→ **§49.1 정정**: 토픽은 `/omy_bridge/command_joint_states`)

### 교훈
문서에 **추측을 "확인 필요"로 남겨둔 것이 제 역할을 했다** — 나중에 실측으로 뒤집혔고, 그 사이
잘못된 전제(리타게팅 노드 필요)로 설계를 진행하지 않았다. 반대로 **추측을 단정으로 적었다면**
불필요한 IK 리타게팅 컴포넌트를 만들었을 것이다.

---

## 21. OMY-L100 공식 HW 데이터 검증 — §20 의 관절 한계 서술 정정 — 2026-09-06

사용자가 `src/ur_bringup/docs/robotis/` 에 OMY HW 자료를 배치. 전량 검증했다.

### 검증한 원본
| 파일 | 크기 | 내용 | 상태 |
|---|---|---|---|
| `omy_l100_kr_03_layout.png` | 218 KB | 제품 사양표 + 치수 도면 | ✅ 판독 |
| `OMY-L100.pdf` | 227 KB | 위 도면의 벡터 PDF | ⚠️ 벡터라 텍스트 추출 불가. **poppler 미설치**(격리 원칙) — PNG 에 같은 치수가 있어 손실 없음 |
| `OMY-L100.stp` | 7.19 MB | CREO `PR44_P08_ASM` (2025-06-11) | ✅ 구조 파싱 |
| `OMY-L100.dwg` | 640 KB | AutoCAD 원본 | ⬜ 미확인(전용 툴 필요) |

추가로 <https://docs.robotis.com/docs/systems/omy/specifications/hardware> 대조.

### ★ 정정 — §20 의 `J3 ±2.62 rad` 은 **F3M(팔로워)** 값이었다
우리가 손으로 잡을 것은 **L100(리더)** 인데, §20 은 MJCF(`omy.xml` = F3M)에서 읽은 값을
OMY 일반값으로 적었다. 공식 사양 기준 **둘은 한계가 다르다**:

| # | **L100(리더)** | F3M(팔로워) | UR16e | 판정 |
|---|---|---|---|---|
| J1 | ±180° | ±360° | ±360° | ⊂ 안전 |
| J2 | **−70°~+100°** | ±360° | ±360° | ⊂ 안전, **가장 좁음** |
| J3 | **±180°(±3.1416)** | ±150°(±2.62) | **±180°(±3.14159)** | **동일 — 마진 0** |
| J4–J6 | ±180° | ±360° | ±360° | ⊂ 안전 |
| J7 | −90°~+60°(그리퍼) | – | – | 2F-85 별도 매핑 |

**영향**: §20 의 *"리더가 더 좁아 UR 한계를 넘길 수 없다 = 안전 방향"* 은 **J3 에 대해 성립하지 않는다.**
→ `omy_to_ur16e` 브리지에 **관절별 clamp(UR 한계 ±95%) 필수**. `plan_il_vla.md` §3.5 에 반영함.
부수적으로 **L100 은 F3M 의 한계-대-한계 복제본이 아니다**(그래서 F3M 모델로 리더를 추론하면 안 됨).

### 확보한 사양 (요약, 상세는 `plan_il_vla.md` §3.5)
- **L100**: reach 560 mm / 1.46 kg / 12 VDC / J1–3 XH540-W150, J4–6 XC330-T288, J7 XC330-T181 /
  U2D2 USB2.0 + TTL Multidrop 4 Mbps / 2,048 pulse-rev
- **F3M**: reach 580 mm / 13.5 kg / 24 VDC / ±0.05 mm / YM080·YM070 / Ethernet + RS485 / RH-P12-RN + D405
- **질량·관성**: 양 모델 전 링크의 질량과 **COG 기준 3×3 대칭 관성텐서(곱관성 포함)** 제공.
  **단위가 문서에 명시되어 있다: 질량 `g`, 관성 `g·mm²`** → URDF 환산 시 ×1e−3, ×1e−9.
- **STEP**: `SI_UNIT(.MILLI.,.METRE.)` 로 **mm 확정**. 고유 부품 15종(`PR44_P08_*`: BASE_T, ELBOW,
  LINK_16PI_160, LINK_ADAPTER, HANDLE, TRIGGER, CRADLE_*, HOLDER_*) + 다이나믹셀/체결류 54종.
  바운딩박스 593×963×230 mm — 도면 전고 641 mm 보다 큰 것은 **거치대(CRADLE)와 벌어진 자세 포함**이라 정합.

### 함정 / 주의
1. **DOF 표기가 자료마다 다르다** — 제품시트(PNG) `DOF 7`, 웹 문서 `DOF 6`. **둘 다 맞다**:
   웹은 팔만, PNG 는 그리퍼 J7 포함. 모터는 7개. (플러그인이 7개를 읽는 것과 일치)
2. **DH 파라미터·좌표계 그림·회전방향 규약은 어디에도 없다.** → **부호/오프셋 캘리브레이션은
   문서로 대체 불가, 실물 측정 필수.** (이게 OMY 연결 작업의 실질 리스크)
3. 액추에이터 배치는 **PNG · 웹 문서 · `lerobot_teleoperator_omy` README** 3자가 일치 — 교차검증됨.

### 교훈
§20 에서 *"`omy.xml` 은 F3M 모델"* 이라고 **출처를 명시해 둔 덕분에** 이번에 어느 값이 오염됐는지
즉시 특정할 수 있었다. 수치를 적을 때 **어느 변종(리더/팔로워)에서 왔는지**까지 남길 것.

### 후속 — L100 공식 URDF 발견 (같은 날)
`ROBOTIS-GIT/open_manipulator`(Apache-2.0, 2026-08-31 갱신)에 **L100 URDF + STL + ros2_control +
리더 전용 런치**가 전부 있다. 받아서 검증한 결과:

- **축 순서 `+Z +Y +Y +Y +Z +Y`** — UR16e 와 동일. §20 결론이 **리더 자신의 모델로 재확인**됨(F3M 추론 아님).
- **링크 치수가 도면(PNG)·STEP 과 완전 일치** (94 / 265.8 / 222 / 52.5 / 46 / 44.5 / 125 mm) → 3중 교차검증.
- ★ **J5 부호 반전 필요**: L100 `joint5` = `+Z`, UR16e `wrist_2_joint` = **`−Z`**.
- ★ **영점 자세가 다름**: L100 q=0 = 수직 상방, UR16e q=0 = 수평 전방 → **J2 오프셋 ≈ −90°**.
- ⚠️ **URDF 관절 한계는 전부 `±180°` 로 뭉뚱그려져 있어 공식 사양(J2 −70~+100°)과 다르다.** 믿지 말 것.
- ⚠️ **L100 은 UR16e 축소 복제본이 아니다**: 측면 오프셋 누적이 UR16e **+290.7 mm** vs L100 **−46 mm**
  로 크기도 부호도 다름. 손목 J4/J6 오프셋은 **정답이 없고 조작감 기준 실물 튜닝**.
  (평행축 J2/J3/J4 는 자세 정합만으로 풀면 해가 512개 — 수학적 한계이지 데이터 부족이 아님.)
- ★ **L100 은 중력보상 리더다**: `omy_l100_leader_ai` 가 `gravity_compensation_controller` +
  `spring_actuator_controller` 를 300 Hz effort 로 돌린다. → **연결 경로를 B(ROBOTIS ros2_control 스택)로
  권장 변경**, 기존 A(`lerobot_teleoperator_omy`)는 폴백. B 는 `/joint_states` 를 네이티브로 뱉어
  **`il_recorder.py --action-source topic` 이 무수정으로 붙는다**(§18 장치무관 설계의 회수). (→ **§49.1 정정**: 리더 원토픽은 관절 이름이 달라 안 붙는다. 브리지 명령 토픽으로.)
  의존은 소스 2개(`dynamixel_hardware_interface`, `robotis_interfaces`) + 자체 컨트롤러 3종뿐.

> **정정**: §21 본문에 *"DH·좌표계·회전방향 규약이 없어 부호/오프셋은 실물 측정 필수"* 라고 적었으나,
> **URDF 가 축 방향과 영점 자세를 주므로 부호(J5 반전)와 J2 오프셋은 문서로 확정된다.**
> 실물 측정이 남는 범위는 **손목 J4/J6 오프셋 + 다이나믹셀 엔코더 영점 확인** 으로 좁혀졌다.

---

## 22. OMY-L100 리더 스택 구축 — 경로 B 채택, mock 검증 완료 — 2026-09-06

§21 의 A/B 비교에서 **B(ROBOTIS ros2_control 스택)** 로 결정하고 구축했다. **실물 미연결 상태에서
mock hardware 로 전 구간 검증 완료.**

### 격리 결과 (이 ws 의 최우선 제약)
```
apt:  0 upgraded, 2 newly installed, 0 to remove
      ros-jazzy-dynamixel-sdk 4.0.3 + ros-jazzy-dynamixel-interfaces 1.0.1  (공식 ROS 레포)
```
컨트롤러가 요구하는 `control_toolbox`/`kdl_parser`/`generate_parameter_library`/`rsl`/`tl_expected`/
`backward_ros`/`angles`/`realtime_tools` 는 **이미 맞는 버전으로 설치되어 있어 손댈 필요가 없었다.**
소스는 `ur16e.repos` 에 **SHA 고정** 3개(`open_manipulator` 1b741c0 / `dynamixel_hardware_interface` 3375b3d /
`robotis_interfaces` 9231cb1, 전부 `jazzy` 브랜치). 태그(5.1.1 등)는 main 에서 잘려 **L100 리더 파일 포함이
보장되지 않아** 쓰지 않았다.

### 빌드 7 / 제외 6
`open_manipulator_description`, `open_manipulator_bringup`, `om_gravity_compensation_controller`,
`om_spring_actuator_controller`, `om_joint_trajectory_command_broadcaster`,
`dynamixel_hardware_interface`, `robotis_interfaces` — 7개 전부 성공(8.8초).
제외: 메타/`collision`/`gui`(Qt)/`moveit_config`/`playground`/`teleop`.

### mock 검증 (하드웨어 0)
```
gravity_compensation_controller         active
spring_actuator_controller              active
joint_state_broadcaster                 active
joint_trajectory_command_broadcaster    active
/leader/joint_states       7관절(joint1..6 + rh_r1_joint)
/leader/joint_trajectory   300.0 Hz (min 0.003s / max 0.004s / std 0.00012s)
```

### 함정 (전부 실제로 밟음)
1. **ros2_control 4.45 controller_manager 는 `robot_description` 파라미터를 안 본다** —
   `/robot_description` **토픽**을 구독한다. `-p robot_description:=...` 로 `ros2_control_node` 를
   직접 띄우면 `Waiting for data on 'robot_description' topic` 에서 영원히 멈춘다.
   → `robot_state_publisher` 와 같이 띄울 것(공식 런치는 이미 그렇게 되어 있다).
2. **전부 `/leader` 네임스페이스** — `ros2 control list_controllers` 를 그냥 치면 `/controller_manager` 를
   찾다가 무한 대기(타임아웃으로만 끝남). `-c /leader/controller_manager` 필수.
3. **`use_self_collision_avoidance` 기본값 `true`** 인데 그 노드는 우리가 COLCON_IGNORE 한
   `open_manipulator_collision` 에 있다 → `false` 로 꺼야 뜬다.
4. **`open_manipulator_bringup` 이 `gz_ros2_control`·`ros_gz_*` 를 의존 선언**하지만
   `ament_python` 이라 colcon 이 해결하지 않아 **Gazebo 는 안 깔린다.**
   단 **`rosdep install` 을 돌리면 통째로 딸려온다** → `check_env.sh` 에 `ros_gz` 유입 감시를 넣었다.
5. **평면 `omy_l100.urdf` 에는 `ros2_control` 블록이 없다**(grep 0줄). 반드시 `.urdf.xacro` 경로.
6. `omy_l100.urdf.xacro` 의 `<gazebo>` 블록이 `$(find open_manipulator_bringup)/config/om_y_leader/...`
   를 참조하는데 **그 디렉터리는 jazzy 브랜치에 없다**(upstream 잔재). `$(find)` 는 패키지 경로만
   해석하고 파일 존재는 확인하지 않아 **전개는 성공**하고, Gazebo 를 안 쓰는 우리에겐 무해.

### 재현
`setup/setup.sh leader`(단계 신설, `--dry-run` 지원) + `check_env.sh` 에 11개 검사 추가.
절차·함정은 `SETUP.md` §2-D.

### `omy_to_ur16e` 브리지 — 같은 날 구현·검증
`scripts/omy_to_ur16e.py`. `/leader/joint_states` → `q_ur[i] = sign[i]*q_leader[i] + offset[i]`
→ `/forward_position_controller/commands` (+ 그리퍼는 `GripperCommand` 액션). **IK 없음.**
기본값 `sign=[1,1,1,1,-1,1]`, `offset=[0,−90°,0,0,0,0]` — 둘 다 §21 URDF 실측에서 나온 값이고
**전부 ROS 파라미터**라 실물 튜닝 시 코드를 안 고친다.

안전장치 5중(16 kg 가반 팔을 1.46 kg 장난감이 조종한다는 점을 잊지 말 것):
1. **기동 시 비활성.** `/omy_bridge/enable` 명시 호출 필요.
2. **★ engage 게이트** — 매핑된 리더 자세가 UR 현재 자세와 `engage_tol`(기본 0.15 rad) 이내가
   **아니면 거부**하고 어긋난 관절을 도(deg)로 알려준다. 이게 없으면 enable 하는 순간
   팔이 차이만큼 한 주기에 튄다. **여기서 제일 중요한 한 줄.**
3. 관절별 **clamp** = UR 한계 × `limit_margin`(0.95). ★ elbow 는 UR ±π 인데 L100 도 ±π 라
   마진이 0 이므로(§21) 이 clamp 가 유일한 방어선이다.
4. 관절별 **속도 상한**(slew). 리더 글리치가 전속 슬램이 되지 않게.
5. **watchdog** — 리더 데이터가 `leader_timeout`(0.5 s) 끊기면 자동 비활성.

**검증(하드웨어 0)**: 합성 `/leader/joint_states` + 합성 `/joint_states` 로 **7/7 PASS** —
거부/수락, 첫 명령이 로봇 현재자세와 일치(무점프), slew 0.01 rad/tick 정확, elbow 2.9845=π×0.95,
J5 부호 반전, watchdog. 거부 메시지가 `shoulder_lift_joint −90.0°` 를 지목해 **J2 오프셋이
실제로 걸린다는 것**까지 확인됐다. 이어 **실제 리더 스택(mock)** 과 붙여 QoS/토픽 호환 확인
(enable 이 "leader 없음"이 아니라 "UR16e `/joint_states` 없음"을 반환 = 리더 데이터는 수신 중).

> **함정**: `--symlink-install` 은 원본 파일 권한을 그대로 쓴다. `install(PROGRAMS)` 를 걸어도
> **소스에 실행 비트가 없으면 `ros2 run` 이 `No executable found`** 로 실패한다 → `chmod +x` 필수.

### Isaac sim 실기동 검증 — 2026-09-06
실물 연결 전에 sim 으로 전 구간을 돌렸다. **mock 리더는 못 쓴다**(effort 명령만 미러링해서
position 이 0 고정) → `scripts/virtual_omy_leader.py`(신규) 로 `/leader/joint_states` 를 합성.
**시작 시 UR16e 의 실제 `/joint_states` 를 읽어 역매핑**(`q_leader=(q_ur-offset)/sign`)해서
engage 게이트를 바로 통과하게 만든다. 나머지 체인(브리지→fpc→ros2_control→Isaac)은 전부 진짜다.
런치는 `launch/common/teleop_omy.launch.py`(신규, Servo 없음 — 관절직결이라 IK 가 필요없고
그래서 **Servo 경로의 elbow 특이점 제약(§15)도 없다**).

구성: Isaac(headless, `ur16e_with_2f85.usd`) → `ur16e_2f85.launch.py use_sim:=true` →
`reset_pose.py ready` → `teleop_omy.launch.py virtual_leader:=true` →
`switch_control_mode.py streaming` → `/omy_bridge/enable`.

**결과 (사인 ±14°, 이동폭 25.4°/20.0°, 14초, 샘플 1656)**
| 구간 | 평균 오차 |
|---|---|
| 리더 → 명령 (**브리지 매핑**) | **0.021°** |
| 명령 → Isaac (물리 추종) | 0.223° |
| **전체** | **0.244°** |

정지 관절은 0.000~0.013°. 그리퍼 `finger_joint` 0.691 rad 연동. `disable` 시 5초간 이동폭
**0.000°**(즉시 정지), `switch_control_mode.py trajectory` 로 복귀 정상. 역매핑 로그가
`q0=[0,0,90,−90,+90,0]°` 를 찍은 것도 **J5 부호 반전이 역방향으로도 맞다**는 확인이다.

### ★★ 함정 — 이것 때문에 측정을 한 번 통째로 오판했다
1. **`pkill -f "<런치파일명>"` 은 `ros2 launch` 부모만 죽인다.** 자식 노드
   (`omy_to_ur16e.py`, `virtual_omy_leader.py`)는 cmdline 이 스크립트 경로라 패턴에 안 걸려
   **살아남는다.** 그 상태로 다시 런치하면 **리더 2개 + 브리지 2개**가 같은 토픽에 서로 다른
   위상을 쏜다. 그때 나온 수치가 *"리더→명령 8°, 리더 속도 156,067°/s"* 였고 브리지 버그처럼
   보였지만 **전부 아티팩트**였다. 중복 제거 후 0.021°.
   → **측정 전에 반드시 `ros2 topic info <topic> | grep Publisher` 로 발행자 수가 1인지 확인할 것.**
2. **`pgrep -c ros2_control_node` 는 실행 중에도 0 을 반환한다** — 프로세스명이 15자로 잘려
   `ros2_control_no` 가 되기 때문. **`pgrep -cf` (전체 cmdline)** 를 쓸 것. 이걸 모르면
   "정리됐다"고 오판한다.
3. **`pkill -f <패턴>` 이 자기 셸을 죽인다** — 내 명령문에 그 문자열이 들어 있으면 셸의 cmdline 도
   매칭된다(실제로 두 번 당함). 패턴을 **런타임에 조립**(`A="robot_state"; A="${A}_publisher"`)
   하거나 PID 를 명시할 것. 공유 컨테이너라 광범위 pkill 이 금지된 것과 별개의 문제다.
4. 가상 리더가 사인 운동을 먼저 시작해 버리면 engage 게이트가 (정당하게) 거부한다
   — 첫 시도에서 elbow 14.3° 이탈로 거부됨. → `wait_for_engage`(기본 true)로
   **engaged 상태가 될 때까지 q0 을 유지**하게 했다. 실물에서도 같다: **리더를 맞춘 자세로
   가만히 잡고 engage** 해야 한다.

### 다음
실물 L100 연결(U2D2) 후 **손목 J4/J6 오프셋 + 엔코더 영점** 튜닝. 파라미터만 만지면 되고
코드/스키마는 그대로다. 기록은 `il_recorder.py --action-source topic --action-topic /leader/joint_states`. (→ **§49.1 정정**: `/omy_bridge/command_joint_states`)

---

## 23. 다른 PC 재현 — `bootstrap.sh` + `preflight` + 리더 캘리브레이션 도구 — 2026-09-07

실물 HW 작업을 **다른 PC** 에서 하기로 해서, "clone 후 두 줄" 로 끝나도록 정리했다.

### 추가한 것
| 파일 | 역할 |
|---|---|
| `setup/bootstrap.sh` | 새 머신 단일 진입점. `--dry-run` / `--no-ml` / `--with-udev`. **WS 를 자기 위치에서 역산**하므로 클론 경로가 달라도 동작 |
| `setup.sh` **`preflight`** 단계 | 읽기 전용 사전점검(Isaac 6.x·ROS Jazzy·GPU sm·디스크·git/curl/sudo·`ur16e.repos` 존재). **아무것도 깔기 전에** 걸러낸다 |
| `setup.sh` **`udev`** 단계 | U2D2 udev 규칙(`latency_timer=1`, mode 0666). **기본 실행에서 제외** — 유일하게 `/etc/udev` 에 쓰고 장치 없이는 무의미 |
| `scripts/omy_leader_calib.py` | 리더 실물 캘리브레이션. `check`(리더 단독) / **`match`(오프셋 자동 산출)** / `verify`(잔차) |
| `check_env.sh` 항목 | U2D2 존재·`latency_timer`·포트 권한·udev 규칙·다이나믹셀 모델파일 3종 (전부 warn, 실패 아님) |

### `omy_leader_calib.py --mode match` 가 핵심인 이유
실물에서 남은 미지수는 **엔코더 영점**(URDF 영점과 같다는 보장이 없음)과 **손목 J4/J6**
(L100 은 UR16e 축소복제본이 아니라 *유도되는 정답이 없음*, §21) 두 가지다. 리더를 UR16e 와
같은 모양으로 잡고 실행하면 `offset[i] = q_ur[i] − sign[i]·q_leader[i]` 를 평균내어 **둘을 한 번에**
구하고, 붙여넣기 가능한 형태로 출력한다. 측정 중 리더가 움직이면 경고한다.
**검증**: 알려진 오프셋 `[3.0, −90.0, 0.0, 12.0, 0.0, −7.5]°` 를 심은 합성 토픽으로 **오차 0 으로 복원**.

### 함정 (둘 다 이번에 실제로 터뜨림)
1. **`udev` 를 STAGES 에 넣자 인자 없는 `./setup.sh` 가 그것까지 실행**하게 됐다 — 하드웨어도 없는
   머신이 `/etc/udev` 에 쓰는 상황. → `STAGES`(유효한 이름)와 **`DEFAULT_STAGES`(기본 실행)를 분리**.
   부수효과가 있는 단계는 **기본 목록에서 빼고 명시적으로만** 돌게 할 것.
2. **`local venv="$WS/..." py="$venv/bin/python"` 가 `set -u` 에서 죽는다.**
   bash 는 `local` 의 **인자를 전부 전개한 뒤 대입**하므로 같은 줄의 앞 변수는 아직 unset 이다.
   `bootstrap.sh --dry-run` 이 `ml` 단계에서 `venv: unbound variable` 로 중단됐다 —
   **새 PC 사용자가 가장 먼저 치는 명령**이 이거라 치명적이었다. → 두 줄로 분리.
   (`stage_ml` 이 여태 동작했던 건 실제 실행 경로가 이 줄을 다르게 타서가 아니라,
   그동안 `--dry-run` 으로 `ml` 을 돌린 적이 없었기 때문이다.)

### 문서
`SETUP.md` **§0-B**(컨테이너→clone→bootstrap→검증→실물, 전제조건 표),
§0-A 단계표 갱신, `HARDWARE.md` §4-B ④ 를 캘리브 도구 절차로 교체, `README.md` §4 상단에 포인터.

### ★ 빈 워크스페이스 실기동 검증 — 2026-09-07
`--dry-run` 만으로는 부족해서 **실제로 빈 워크스페이스(`/isaac-sim/volume/ur_ws_fresh`)를 만들어
`bootstrap.sh` 를 끝까지 돌렸다.** "커밋 후 clone" 과 동일한 트리
(`git ls-files` + 추적안됨-미무시, vcs 디렉터리 제외)로 시작.

**결과**: `sources`(vcs import + nvblox 서브모듈) → `build`(topic_based·ur_bringup·그리퍼 드라이버 +
**nvblox_ros sm_120 소스빌드 2분 23초**) → `leader`(7패키지) → `ml`(venv+torch) → `verify`
전부 통과, 새 워크스페이스에서 **브리지 테스트 7/7 PASS**.

**dry-run 이 절대 못 잡는 버그 2개를 여기서 발견했다:**

1. **`set -u` + ROS `setup.bash` = 즉사.**
   `stage_build`/`stage_leader` 가 `source /opt/ros/jazzy/setup.bash` 하는 순간
   `AMENT_TRACE_SETUP_FILES: unbound variable` 로 죽는다. ROS 의 setup.bash 가 여러 변수를
   기본값 없이 참조하기 때문. **dry-run 은 source 자체를 건너뛰어서 안 걸린다.**
   → source 구간만 `set +u` / `set -u` 로 감쌌다(2곳).
2. **`python3 -m venv` 가 `ensurepip` 부재로 실패.**
   이 이미지엔 `python3-venv` 가 없고, 깔면 python3.12 전역 업그레이드(7개)라 우리 격리 원칙 위반.
   코드 주석은 "get-pip 로 부트스트랩한다" 였는데 **그 앞의 `python3 -m venv` 에서 이미 죽었다.**
   → **`python3 -m venv --without-pip`**. 이건 최적화가 아니라 **필수**다.

**교훈**: `--dry-run` 은 "무엇을 할지"만 보여준다. `source`·`venv` 처럼 **실행해야만 드러나는 경로**는
빈 워크스페이스 실기동으로만 검증된다. 스크립트를 고칠 때마다 이 절차를 반복할 것:
```bash
TESTWS=/isaac-sim/volume/ur_ws_fresh
{ git ls-files; git ls-files --others --exclude-standard; } | sort -u > /tmp/f.txt
tar -cf - -T /tmp/f.txt | (mkdir -p "$TESTWS/src" && tar -xf - -C "$TESTWS/src")
WS=$TESTWS $TESTWS/src/setup/bootstrap.sh
```

**부수 확인**: apt 는 `ros-jazzy-ur` + `ros-jazzy-ur-calibration` **2개가 실제로 신규 설치**됐다
(메타패키지 자체는 미설치 상태였음). 가드가 `0 upgraded, 0 removed` 를 확인한 **순수 추가**라
격리 원칙은 지켜졌지만, "이미 다 깔려 있으니 시스템 변경 0" 이라는 예상은 틀렸다.

### ★ vcs 의존성 SHA 고정 — 2026-09-07
소스 의존 7개는 **저장소에 커밋하지 않고 다른 PC 에서도 `vcs import` 로 받는다**(전부 `.gitignore`).
그러면 **`ur16e.repos` 의 `version:` 필드가 재현성의 전부**인데, 3개가 **브랜치**로 되어 있었다:
`ros2_robotiq_gripper`=`main`, `serial`=`ros2`, `isaac_ros_nvblox`=`release-4.6`.
브랜치는 움직이므로 **다음 달 다른 PC 가 clone 하면 다른 코드를 받는다** — 문서가 설명하는 것과
다른 트리 위에서 디버깅하게 된다.

→ 이 머신에서 검증된 SHA 로 **고정**:
`ros2_robotiq_gripper` 3b6cf8ff / `serial` d8d16067 / `isaac_ros_nvblox` dadbe96c.
`topic_based_hardware_interfaces` 만 태그 `0.2.1` 유지(문서가 이름으로 참조하고 재태깅 위험 없음),
검증 SHA 007cff10 을 주석으로 남김. **nvblox_core 서브모듈(24eee494)은 슈퍼프로젝트 SHA 안에
기록**되므로 함께 고정된다.

**검증**: 빈 디렉터리에 `vcs import` 를 실제로 돌려 **7개 전부 요청한 SHA 와 일치**함을 확인.
기본 브랜치가 아닌 곳의 SHA(`jazzy`, `ros2`, `release-4.6`)도 정상 체크아웃된다.

> SHA 를 올릴 때는 **반드시 빈 워크스페이스 재검증과 함께** 할 것(위 절차).

---

## 24. 0단계 — 공개 LeRobot 데이터셋으로 ACT 학습 관통 — 2026-09-07

`plan_il_vla.md` §4 의 **0단계**("공개 데이터셋으로 ACT 관통. 건너뛰지 말 것").
목적은 모델 성능이 아니라 **D단계에서 학습이 실패했을 때 "데이터 문제"와 "환경 문제"를 분리**할
근거를 미리 만들어 두는 것. 결과: **관통 성공**, 그리고 그 과정에서 재현 버그 2개가 드러났다.

### 데이터셋 선택 — `lerobot/svla_so101_pickplace`

lerobot 0.6.1 은 데이터셋 **v3.0** 포맷을 요구한다(`CODEBASE_VERSION = v3.0`). 후보를 HF API 로
`meta/info.json` 직접 조회해 확인했다:

| repo | ver | ep | frames | fps | cams |
|---|---|---|---|---|---|
| **`lerobot/svla_so101_pickplace`** | v3.0 | 50 | 11,939 | **30** | up, side |
| `lerobot/aloha_sim_transfer_cube_human` | v3.0 | 50 | 20,000 | 50 | top |
| `lerobot/aloha_sim_insertion_human` | v3.0 | 50 | 25,000 | 50 | top |
| `lerobot/pusht` | v3.0 | 206 | 25,650 | 10 | (없음) |

`svla_so101_pickplace` 를 골랐다 — **우리 목표 구성과 구조가 가장 가깝다**:
단일팔 + 그리퍼, 카메라 2대, **30 fps(= 우리 기록 규약, §18)**, 그리고 절대 관절위치 액션.

```
action            float32 [6]  shoulder_pan/lift, elbow_flex, wrist_flex/roll, gripper (.pos)
observation.state float32 [6]  (동일)
observation.images.{up,side}  video [480,640,3]
task = "pink lego brick into the transparent box"
```

우리 §2.6 스키마(`state.single_arm`(6) + `state.gripper`(1), 카메라 2대, 30 Hz, 절대 관절위치)와
**같은 모양**이다. 차이는 팔 관절이 5개(SO-101)냐 6개(UR16e)냐뿐.

### 결과 — 관통 성공

500 스텝(batch 8, 4 workers, RTX 5090):

| step | loss | l1_loss | kld |
|---|---|---|---|
| 50 | 13.544 | 0.834 | 1.271 |
| 200 | 3.484 | 0.670 | 0.281 |
| 500 | **2.540** | **0.513** | 0.203 |

- **sm_120 실동작 확인**(§6.3 검증 항목): torch 2.11.0+cu128, `get_arch_list()` 에 `sm_120` 포함, RTX 5090 에서 실제 학습.
- **피크 VRAM 6,441 MiB / 32,607 MiB**(§6.2 VRAM 한계 항목). ACT + 카메라 2대 + batch 8 기준.
  Isaac(+perception)과 동시 구동 여지가 충분하다.
- 처리량 **161 smp/s**, 500 스텝 32초.

### ★ 함정 1 — `lerobot[dataset]` 만으로는 학습이 안 된다

`lerobot-train` 이 첫 줄에서 죽는다:
```
ImportError: 'accelerate' is required but not installed. Install it with: pip install 'lerobot[training]'
```
`requirements-ml.txt` 에 `lerobot[dataset]` 만 있었다. **`[dataset]` 만으로도 데이터 변환
(`raw_to_lerobot.py`)은 멀쩡히 되기 때문에 §19 에서 이 누락을 못 잡았다** — 첫 실제 학습에서야 드러났다.

→ `lerobot[dataset,training]` 으로 수정. `pip install --dry-run` 으로 **순수 추가 13개
(accelerate, wandb 등)이고 torch 는 안 건드림**을 먼저 확인한 뒤 설치(격리 원칙).
설치 후 `get_arch_list()` 재확인 — sm_120 유지.
→ `check_env.sh` 에 **`accelerate` 임포트 검사 별도 추가**(lerobot 임포트만으론 못 잡으므로).

부수: `[training]` 설치 후 hub push 검증이 켜져 `--policy.push_to_hub=false` 가 필요해진다.

### ★★ 함정 2 — `/dev/shm` 64 MiB → DataLoader 워커가 **간헐적으로** 죽는다

```
RuntimeError: unable to allocate shared memory (shm) for file <...>: Resource temporarily unavailable (11)
RuntimeError: DataLoader worker (pid ...) exited unexpectedly
```

이 컨테이너의 `/dev/shm` 은 Docker 기본값 **64 MiB**. PyTorch 기본 공유전략(`file_descriptor`)은
워커→학습루프 배치 전달에 `/dev/shm` 을 쓰는데, **ACT 배치 하나가
8 × 카메라2 × 3×480×640 float32 ≈ 59 MiB** 라 한 배치도 겨우 들어간다.

**실측(동일 조건 3회 반복)**:

| | 결과 | 피크 shm |
|---|---|---|
| 수정 없음 | **SHM_CRASH / SHM_CRASH / OK** | 43 / 43 / 29 MiB |
| `UR_WS_TORCH_SHM_FIX=1` | **OK / OK / OK** | 29 / 36 / 15 MiB |

> **★ 이게 간헐적이라는 게 핵심이다.** 처음 500 스텝 런이 수정 없이 통과해서 한 번
> "해결됐다"고 잘못 판단했다. 결정적 실패보다 나쁘다 — 스모크는 통과하고 긴 학습 중간에 죽는다.
> 반드시 **반복 실행으로** 확인할 것.

`/dev/shm` 을 키우려면 컨테이너 재생성이 필요한데 **다른 프로젝트와 공유하는 머신**이라 불가.
→ 공유전략을 `file_system`(temp dir 사용, 2.2 TB 여유)으로 바꾼다.

**여기 도달하기 전에 조용히 실패한 시도 2개** (둘 다 "고친 것처럼 보였다"):

1. **부모 프로세스에만 `set_sharing_strategy("file_system")`** → 워커가 그대로 죽음.
   원인: lerobot 이 `dataloader_multiprocessing_context = "spawn"` 을 **명시적으로 박아 뒀다**
   (`configs/train.py:110`; 시스템 기본 start method 는 `fork`). spawn 된 워커는 부모 상태를
   상속하지 않는다 — lerobot 이 의도한 동작이다.
2. **venv site-packages 에 `sitecustomize.py`** → `get_sharing_strategy()` 가 여전히
   `file_descriptor`. 원인: **`/usr/lib/python3.12/sitecustomize.py` 가 이미 있고
   stdlib 경로가 site-packages 보다 앞서서** 우리 것이 임포트조차 안 된다.
   `import sitecustomize; sitecustomize.__file__` 로 확인.

**확정안 = `.pth` 파일.** `site` 가 **모든** 인터프리터(spawn 된 워커 포함)에서 `import` 로 시작하는
줄을 실행하고, 이름 충돌도 없다. 환경변수로 게이팅해 평소 venv python 기동 비용은 0:

```
deps/.venv-ml/lib/python3.12/site-packages/ur_ws_shm_fix.pth   # site 가 실행하는 한 줄
deps/.venv-ml/lib/python3.12/site-packages/ur_ws_shm_fix.py    # 실제 set_sharing_strategy
```
`setup.sh ml` 이 자동 설치하고 **설치 직후 실제 전략값으로 자체 검증**한다.
사용: `UR_WS_TORCH_SHM_FIX=1 ... lerobot-train ... --num_workers=4`.

**워커 도달 증명**(파일 존재가 아니라 실동작으로): spawn 된 DataLoader 워커 안에서
`get_sharing_strategy()` 를 읽어 반환시키고, `PYTHONHASHSEED=0` 으로 고정해
`hash("file_system")` 와 대조 → 일치.

**속도**(부수 효과지만 큼):

| | data_s | smp/s | 200 스텝 |
|---|---|---|---|
| `num_workers=0` (회피책) | 0.13 | 47 | 40 s |
| `UR_WS_TORCH_SHM_FIX=1` + 4 workers | **0.001** | **161** | **16 s** |

loss 는 동일(step 200 에서 3.483 vs 3.484) — **속도만 2.5배**.
`num_workers=0` 도 유효한 폴백이지만 D단계 장기학습엔 부담.

→ `check_env.sh` 는 **파일 존재가 아니라 실제 전략값**을 검사한다(함정 2 때문에
"설치된 것처럼 보이지만 안 도는" 상태가 실재했으므로).

### 변경 파일
`setup/requirements-ml.txt`(extra 수정) · `setup/setup.sh`(`_install_shm_workaround`) ·
`setup/check_env.sh`(accelerate + shm 실동작 검사) · `SETUP.md` §2-C · `plan_il_vla.md` §4/§6.2/§7-B.

### 남은 것
0단계는 **관통 확인**이 목적이라 여기서 끝. 정책 품질 평가는 자체 데이터가 생기는 **D단계**의 몫이다.

---

## 25. pick&place 상태머신 (T3-D) — sim 검증 완료 — 2026-09-07

`plan_il_vla.md` §3.2 의 sim 데모 생성기. **사람 없이 파이프라인 전체를 끝까지 돌리는 구동기**이자
sim 대량 데이터 생성기. 실물엔 GT 가 없으므로 **sim 전용 도구**이고, 실물 데모는 §3.3/§3.5 경로다.

```
/scene/object_pose (GT) ─▶ pick_place_demo.py ─▶ MoveGroup/cuMotion ─▶ JTC ─▶ Isaac
                                └─▶ /gripper_controller/gripper_cmd ─▶ 2F-85
READY → DETECT → PRE_GRASP → GRASP → CLOSE → LIFT → TRANSFER → PLACE → OPEN → RETRACT
```

**결과: 1/1 → 재실행 3/3 사이클 SUCCESS** (마커 오차 14 / 9 / 18 / 7 mm). 단계별 물체 좌표로 검증:

| 단계 | 물체 위치 |
|---|---|
| DETECT | (0.600, 0.000, 0.225) |
| LIFT | (0.608, 0.007, **0.369**) ← 실제로 들림 |
| TRANSFER | (0.608, **0.258**, 0.288) |
| PLACE | (0.606, 0.258, **0.225**) ← 테이블에 안착 |

**pose 는 토픽으로만 받는다** — 나중에 FoundationPose 가 `/target/pose` 를 내면
`-r /scene/object_pose:=/target/pose` **remap 한 줄**로 교체된다. 시뮬레이터를 직접 들여다보지 않는다.

### ★★ 함정 1 — MoveIt 은 "모르는 링크의 제약"을 "이미 충족된 제약"으로 취급한다

가장 위험한 종류의 실패다. **액션이 `error_code=1`(SUCCESS)을 돌려주는데 팔이 전혀 안 움직인다.**
로그엔 `PRE_GRASP: ok / GRASP: ok / LIFT: ok` 가 찍히고, 유일한 증상은 "물체가 안 집힌다"뿐이다.

```
[ERROR] Link 'gripper_frame' not found in model 'ur16e'
[WARN]  Position constraint link model gripper_frame not found in kinematic model. Constraint invalid.
[INFO]  Goal constraints are already satisfied. No need to plan or execute any motions
```

원인이 **두 겹**이었고 둘 다 조용하다:

1. **cuMotion 은 `tool0` 을 거부한다** — XRDF 가 `tool_frames: [gripper_frame]` 이라
   `Target link 'tool0' does not match end effector 'gripper_frame'`. 그런데 `gripper_frame` 은
   `cumotion/ur16e_2f85.urdf` 에만 있고 **런타임 URDF 엔 없었다** → 두 모델이 어긋나 있었다.
   → `urdf/common/robotiq_2f85_macro.xacro` 에 **identity 오프셋 별칭**으로 추가(기하 변화 0, USD 재베이크 불필요).
2. **`ur16e_2f85_d405_cumotion_moveit.launch.py` 의 `ur_only` 기본값이 `true`** —
   move_group 이 **그리퍼 없는 UR 팔 단독 모델**(`urdf/ur16e/ur16e_sim.urdf.xacro`)을 로드한다.
   런치 도움말 그대로 `false` 가 **"programmatic cuMotion"** 용이다. → **`ur_only:=false` 필수.**

> **검사 방법이 중요하다.** 아래 둘은 **작동하지 않는다**:
> - 로그에서 `"gripper_frame not found"` grep — 그 에러는 **제약이 평가될 때만** 찍히므로 기동 직후엔 항상 없다.
>   (이걸로 만든 가드가 통과하는 동안 모든 goal 이 무동작이었다.)
> - `robot_description` **파라미터** 조회 — `ur_only:=false` 면 모델이 **토픽**으로 오고 파라미터엔
>   팔 단독 기본값이 남아 있어 **거짓 실패**를 보고한다.
>
> **`/compute_fk` 로 물어야 한다** — move_group 자신의 기구학 모델이 답한다.
> `pick_place_demo.py` 의 `check_ee_link_known()` 이 시작 시 이걸 하고, 실패하면 거부한다.

### ★ 함정 2 — 낡은 `robot_state_publisher` 잔존 → move_group 이 구 URDF 를 latch

URDF 를 고치고 제어 스택을 재시작했는데도 안 먹었다. `ros2 topic info /robot_description` →
**Publisher count: 2**. 런치 부모를 죽여도 **자식 노드는 살아남는다**(§22 의 중복 노드 함정과 동일).
`/robot_description` 은 TRANSIENT_LOCAL 이라 늦게 뜬 move_group 이 **낡은 쪽을 latch** 했다.
→ 재시작 후 **RSP 가 정확히 1개인지 확인**할 것.

### ★ 함정 3 — `use_sim_time` + `/clock` 도착 전 마감시각 계산

`use_sim_time` 이면 첫 `/clock` 이 올 때까지 `now()` 가 0 이다. 그때 만든 마감시각은 `0 + timeout`
이라, 실제 sim 시각(수천 초)이 들어오는 **순간 모든 대기가 동시에 만료**된다.
증상은 "토픽이 죽은 것처럼 보이는 즉시 타임아웃" — 실제로 그렇게 오진했다.
→ `wait_for_clock()` 으로 **먼저 `/clock` 을 기다린 뒤** ROS 시간 마감시각을 만든다.
그 대기 자체는 **monotonic 시계**로 제한한다(기다리는 대상으로 그 대상을 잴 수 없다).

### ★ 함정 4 — TCP 를 그리퍼 닫힌 채로 재면 13.5 mm 틀린다

TCP = **패드 중점**인데 패드는 닫히면서 안쪽으로 접힌다. 고정 sleep 후 측정하니
같은 코드가 한 번은 `0.0983 m`, 다음엔 `0.0452 m` 를 냈다 — 그리퍼 타이밍에만 의존.
→ ① `finger_joint` 가 열림값에 도달할 때까지 대기 ② TF 값이 **연속 3회 0.5 mm 이내로 안정될 때까지**
샘플링. 안정 안 되면 추측하지 말고 실패.
(TCP 는 **TF 에서 실측**한다 — standoff 가 세트별로 다르고(세트2 +11 mm, 세트3 +18 mm)
하드코딩하면 `CLAUDE.md` 함정 7 을 밟는다. 실측 확인: `tool0 → gripper_frame` = **0.018 m** = 세트3 standoff.)

### ★★ 함정 5 — cuMotion 은 **관절공간** 경로다. 접근/후퇴를 planned move 로 하면 물체를 친다

cuMotion 은 minimum-jerk **관절공간** 궤적을 낸다 → 두 pose 사이가 **Cartesian 직선이 아니다.**
그 상태로 물체 위에서 하강하니 손가락이 물체를 쓸고 지나갔다. 단계별 물체 좌표로 특정:

| | 물체 위치 | 밀림 |
|---|---|---|
| PRE_GRASP 후 | (0.600, 0.000, 0.225) | — |
| GRASP 후 (planned) | (0.621, −0.010, 0.233) | **21 mm** |
| GRASP 후 (**linear**) | (0.609, −0.003, 0.229) | 9 mm |

→ **접근·후퇴(GRASP/LIFT/PLACE/RETRACT)는 `/compute_cartesian_path` + `/execute_trajectory`**
직선 이동으로. `fraction < 0.95` 면 **부분 실행하지 말고 실패 처리**(중간에 멈추면 물체에 못 닿는다).
PRE_GRASP·TRANSFER 같은 자유 이동만 cuMotion planned move 로 남긴다.

> **교훈: 단계마다 물체 좌표를 찍어라.** 안 찍었으면 이 전부가 뭉뚱그려 "grasp 실패"로만 보였다.

### ★ 함정 6 — 작업면 높이: 낮으면 안 닿고, 높으면 로봇을 친다

원래 장면은 **테이블 상면 z=0**(teleop 데모용, §16/§18). teleop 은 Servo 라 충돌검사가 없어 됐지만
**cuMotion 은 거부한다.** plan-only 로 실측한 임계값:

| gripper_frame z | 0.34 | 0.29 | 0.25 | 0.22 | 0.20 | 0.18 | 0.16 | 0.14 | 0.12 |
|---|---|---|---|---|---|---|---|---|---|
| 결과 | ok | ok | ok | ok | ok | **ok** | 실패 | 실패 | 실패 |

즉 패드가 닿을 수 있는 최저 높이는 `0.18 − 0.0983 = 0.082 m` 인데 물체 중심은 0.040 —
**애초에 닿을 수 없는 장면**이었다.

테이블을 0.20 으로 올리자 이번엔 **`wrist_2_joint` 가 −26.3 rad**(±6.28 밖) 로 튀어
모든 plan 이 `START_STATE_INVALID`. 1.0×1.2 m 슬래브가 x=0.55 중심이라 **스폰 시 팔과 겹쳤고**
PhysX 가 겹침을 풀면서 팔을 날려버린 것.
→ `--table-pose` / `--table-size` 를 **인자로 추가**. 검증된 조합:

```
--table-height 0.20 --table-pose 0.72,0.0 --table-size 0.70,0.90
--object-pose 0.60,0.0,0.235 --place-pose 0.60,0.25,0.0
```

### 변경 파일
`ur_bringup/scripts/pick_place_demo.py`(신규) · `urdf/common/robotiq_2f85_macro.xacro`(gripper_frame) ·
`isaac/common/ur16e_isaac_ros2.py`(`/scene/place_pose` latched 발행, `--table-pose`/`--table-size`) ·
`CMakeLists.txt` · `package.xml`(moveit_msgs/shape_msgs/control_msgs/geometry_msgs/tf2_ros).

### ★★★ 함정 7 (진짜 원인) — **컨트롤러가 팔이 도착하기 전에 SUCCEEDED 를 보고한다**

랜덤화를 켜자 파지가 계속 실패했다. 단계별 물체 좌표가 "하강 중 11 mm 밀림"까지는 보여줬지만
**왜** 밀리는지는 추측만 가능했다. 그래서 **명령 자세 vs 실제 도달 자세**를 찍게 했더니 즉시 나왔다:

```
GRASP: gripper_frame actual (0.6179, -0.0042, 0.3431)
       commanded             (0.600,   0.000,   0.323)
       err (+17.9, -4.2, +19.8) mm
```

MoveIt 이 `SUCCEEDED` 를 보고한 시점에 팔은 목표에서 **18 mm 앞, 20 mm 위**에 있었다.
sim 하드웨어(topic_based → Isaac articulation drive)가 위치 명령을 **뒤따라가는 중**이기 때문.
밀림 방향(+x)과 오차 방향(+x)이 정확히 일치한다.

**전체 인과사슬이 여기서 시작한다**:
> 정착 안 기다림 → 그리퍼가 18 mm 어긋난 채 하강 → 물체를 밀침 → 모서리로 물림 →
> 손가락이 용접 임계값(`grasp_close` 0.25)에 못 미친 채 정지 → **용접 안 됨** →
> 쥐어짜다 물체가 튕겨나감 → `grasp_failed`

→ **`wait_settled()`**: 매 이동 후 TF 로 실제 자세가 목표 3 mm 안에 들어올 때까지 대기(타임아웃 5 s).
"궤적 완료"가 아니라 **"실제 도착"** 을 기다린다.

> **틀린 가설 하나 기록**: 처음엔 접근 자세 공차(`ori_tol` 0.05 rad ≈ 3°)가 원인이라 보고
> 0.005/0.01 로 조였다. **밀림은 그대로 11 mm 였고**(가설 반증) 오히려 PRE_GRASP/LIFT 계획 실패까지
> 생겨 10 사이클 중 5연속 실패했다. 공차는 원복. **측정 없이 조인 것이 실수였다.**

### 참고 — 접근 공차 자체는 원인이 아니었다

고정 위치에선 4/4 성공이라 안 보였는데, `--randomize-object` 를 켜자 바로 실패했다.
단계별 물체 좌표가 원인을 그대로 보여줬다:

| 단계 | 물체 위치 |
|---|---|
| PRE_GRASP 후 | (0.672, −0.030, 0.225) 그대로 |
| GRASP 후 | (0.683, −0.034, 0.231) — **11 mm 밀림** |
| LIFT 후 | (0.194, 0.150, 0.040) — **튕겨나감** |

**연쇄**: `ori_tol`=0.05 rad(≈3°) → 0.15 m 하강 동안 측면 오차 ≈8 mm → 물체가 밀림 →
모서리로 물림 → 손가락이 **용접 임계값 `grasp_close`=0.25 에 못 미친 채 정지** → 용접 안 됨 →
쥐어짜다 튕겨나감. 즉 "파지 실패"의 뿌리는 **접근 자세 공차**였다.

→ **하강 직전 물체를 다시 읽고**, 2 mm 넘게 밀렸으면 **같은 높이에서 수평 보정(RECENTRE) 후
수직 하강**. 대각선으로 내려가면 그게 다시 물체를 친다. 50 mm 넘게 밀렸으면 yaw 도 못 믿으므로
`part_disturbed` 로 **버린다**(밀린 물체를 쫓아가는 데모는 흉내낼 가치가 없다).
공차 조이기는 **효과 없었다**(위 참조) — 남겨둔 건 RECENTRE 쪽이다.

### ★ 함정 8 — 파지 도중에 상태머신을 죽이면 **Isaac 재시작 말고는 복구가 없다**

중간에 `kill` 하면 물체를 문 채로 멈춰서 **`finger_joint` 가 한계 밖으로 밀려난다**(실측 −0.324,
한계 0~0.8). §16 이 기록한 그 현상이다. 이 상태에선 TCP 측정이 성립하지 않는다
(새 가드가 "추측하지 말고 실패"로 정확히 거부했다). 복구:

```
ros2 service call /scene/detach_object std_srvs/srv/Trigger
ros2 service call /scene/reset_episode std_srvs/srv/Trigger
# 그리퍼를 0.0 으로 한 번 명령
```
**이 복구는 물체가 손가락 사이에서 빠져나갔을 때만 통한다.** 물체가 끼어 있으면 안 된다 —
실측: −0.974 와 −0.341 두 번 모두 실패, `-0.324` 한 번만 성공. `recover_gripper()` 를
`pick_place_demo.py` 에 넣어 자동 시도하되, **실패하면 추측하지 않고 "restart Isaac" 으로 중단**한다.
무인 실행에서는 **애초에 사이클 도중에 죽이지 말 것.**

### il_recorder 연동

`pick_place_demo.py --ros-args -p record:=true` 로 매 사이클이 한 에피소드가 된다.
**성공만 저장하고 실패는 `discard`** 한다 — 물체를 쓰러뜨린 데모는 없는 것보다 나쁘다(§6.5).
기록 시작은 리셋이 안정된 **뒤**(DETECT 직후)라, 에피소드가 물체 순간이동으로 시작하지 않는다.
⚠️ 기록기는 **`auto_reset:=false`** 로 띄울 것 — 안 그러면 상태머신과 둘 다
`/scene/reset_episode` 를 불러 한 사이클에 두 번 리셋된다. 에피소드 수명주기는 상태머신이 소유한다.

### ★★ 함정 9 — 큐브 **중심**을 겨냥하면 손끝이 테이블에 닿는다

랜덤화 성공률이 1/5 로 주저앉았다. GRASP 도달 오차가 **z +21.5 mm, 즉 명령보다 높은 곳**에서
멈춘 게 단서였다 — 위치제어가 못 미친 게 아니라 **아래에서 물리적으로 막힌** 것.

그리퍼 기하를 실측해 배제부터 했다(열림/닫힘 각각 TF 로):

| 상태 | tip **링크** 간격 | 실제 패드 간격 |
|---|---|---|
| OPEN (`finger_joint`=0) | 135.5 mm | **84.8 mm** ← 실물 2F-85 의 85 mm 와 일치 |
| CLOSED (0.8) | 50.7 mm | 0 mm |

즉 tip 링크 원점은 패드 접촉면보다 각각 **25.35 mm 바깥**에 있을 뿐, 개폐 폭은 정상이고
50 mm 큐브는 한쪽당 17.4 mm 여유로 들어간다 → **옆면 충돌이 아니다.**
남는 건 아래, 즉 **테이블**이다. 큐브 중심(z=0.225)을 TCP 로 겨냥하면 손끝이 상면(0.20)에 닿는다.

→ **`grasp_z_offset` (기본 +15 mm)**: 큐브 **상부**를 잡는다. 패드는 닫히면 간격 0 이라 유지력은
같고, 실물 2F-85 로 바닥에 놓인 부품을 집을 때 하는 방식이기도 하다. PLACE 해제 높이도 같이 보정.
**결과: 1/5 → 4/5**, 마커 오차도 14 mm → **3~6 mm**.

### ★★ 함정 10 — `jump_threshold = 0.0` 은 "제한 없음"이 아니라 **검사 끄기**다

`grasp_z_offset` 적용 후에도 1건이 남았고, 성격이 달랐다: **물체는 7 mm 만 움직였는데
팔이 y 로 50 mm 엉뚱한 곳에 도착**했다. 막힌 게 아니라 **딴 데로 간 것**이고,
`compute_cartesian_path` 는 그 경로를 `fraction 1.00` 으로 보고했다.

원인은 요청의 `jump_threshold = 0.0`. 이 값은 **관절공간 점프 필터를 비활성화**한다
(메시지 주석: "If jump_threshold is set > 0, it acts as a scaling factor ... the returned path
is truncated before the step"). 그래서 하강 도중 **IK 분기가 뒤집히는** 불연속 경로가
"100% 해결"로 통과했고, 컨트롤러는 그걸 추종하지 못했다.

→ **`jump_threshold = 5.0`**. 점프가 있으면 경로가 **잘려서** `fraction` 이 떨어지고,
우리 `min_fraction`(0.95) 검사가 **깨끗하게 실패 처리**한다 — 팔이 날뛰는 대신.

### 참고 — 남아 있는 계통 오차

`PRE_GRASP` 는 매번 **z −7.2~−7.3 mm** 로 정착한다(x,y 는 ±0.1 mm). 위치 드라이브의
중력 처짐으로 보이며 재현성이 높다. 파지에는 `grasp_z_offset` 이 흡수하므로 현재는 문제 없지만,
**정밀도가 필요해지면 여기부터 볼 것.**

### ★ 함정 11 — 테이블을 플래닝 씬에 넣는 건 생각보다 까다롭다

플래너가 작업면을 모르면 **경로를 관통시키고 물리가 막는다**(위 함정 7 의 50 mm 오차가 그것).
실물에선 그게 곧 충돌이므로 테이블은 반드시 씬에 있어야 한다. 그런데 그냥 넣으면 안 된다:

| 시도 | 결과 |
|---|---|
| 테이블 없음 | 6/8 — 실패가 **물리적 충돌**로 발생 |
| 실제 높이로 등록 | **0/3, 전부 `plan_failed_grasp`** — 표면 위 부품을 집으려면 그리퍼가 표면에 근접해야 하는데 그게 전부 충돌 판정 |
| 20 mm 침하 (등록을 맨 앞에) | **`START_STATE_IN_COLLISION(-10)`** — Isaac 은 팔을 **수평으로 뻗은** 자세로 시작하고 그 높이가 대략 작업면 높이라, "로봇이 방금 알려준 상자 안에 이미 있는" 상태 |
| 20 mm 침하 + **READY 도달 후** 등록 | **5/8**, 마커 오차 **2~5 mm**(테이블 없을 때 3~8 mm), 실패는 **계획 단계에서 안전하게 거절** |

→ 채택: `table_sink`(기본 0.02) + **등록 시점을 READY 이후로**. 기하는 Isaac 이
`/scene/table_box` 로 발행하고 상태머신이 CollisionObject 로 등록 — **단일 출처**라 어긋날 수 없다.
성공률은 6/8 → 5/8 로 n=8 수준 노이즈지만 **정확도가 오르고 실패가 안전해진다.**
사이클 시간은 2~3배(정착 대기 + 충돌검사).

> **정석은 침하가 아니라 ACM.** `work_table` ↔ **그리퍼 링크만** 충돌 허용하면 팔은 완전히 막고
> 손가락만 통과시킬 수 있다. cuMotion 플러그인이 AllowedCollisionMatrix 를 존중하는지 **미확인**이라
> 미뤘다. 정밀도나 안전 마진이 필요해지면 여기부터.

### ✅ 파이프라인 관통 — 상태머신 → 기록 → LeRobot → ACT (2026-09-08)

`plan_il_vla.md` §7-B 가 "실제 LeRobot 변환은 ML 환경 생긴 뒤 검증"이라고 미뤄둔 **마지막 미검증 고리**를 닫았다.

```
pick_place_demo(record:=true) ─▶ il_recorder(auto_reset:=false) ─▶ raw(JPEG+JSON)
   ─▶ raw_to_lerobot.py (ML venv) ─▶ LeRobot v3.0 ─▶ lerobot-train --policy.type=act
```

| 단계 | 결과 |
|---|---|
| 기록 | 5 사이클 중 **4 성공 저장 / 1 실패 discard** — 실패 데모는 저장하지 않는다(§6.5) |
| raw | 4 에피소드 / **7,136 프레임** / 30 Hz, `meta.json` 이 태스크·관절·그리퍼 규약·스키마 출처까지 자기서술 |
| 변환 | **성공** (`done -> local/ur16e_pickplace`) |
| 재판독 | `observation.images.{exterior,wrist}` (3,480,640) / `observation.state`(**7**) / `action`(7) / `task` — **§2.6 스키마 그대로** |
| ACT 학습 | **동작** — loss 15.34 → 3.13, l1 0.588 → 0.275 (300 스텝) |

**변환기는 손댈 필요가 없었다** — 이미 lerobot 0.6.x API(`create`/`add_frame`/`save_episode`,
task 를 frame dict 안에)를 대상으로 작성돼 있었다. 0단계(§24)에서 학습 툴체인을 먼저 검증해 둔 덕에
"데이터 문제 vs 환경 문제"를 구분할 필요조차 없었다.

**★ 다음에 손볼 데이터 품질 이슈**: 에피소드가 **1,686 프레임(≈56초)** 로 길다. `wait_settled` 대기와
계획 시간이 전부 **정지 프레임**으로 들어간다. 정책이 "가만히 있기"를 배우므로 잘라내거나
기록을 이동 구간에만 켜는 편이 낫다. 지금은 파이프라인 검증이 목적이라 그대로 뒀다.

**★ 태스크는 아직 1종**이다. 변환기가 스스로 경고한다 —
"Fewer than 2 distinct task strings. A VLA trained on this will ignore language"(§2.8).
C′ 단계에서 **태스크 3종 × 언어 지시문**으로 모을 것.

### 언어 조건 태스크 3종 — 장면을 물체 2 × 목적지 2 로 확장 (2026-09-08)

**물체 1개 + 목적지 1개로는 지시문이 잉여다.** 무시해도 정답이므로 VLA 가 언어를 안 읽고,
3B 를 써서 ACT 를 얻는다(§2.8). 같은 관측에서 **지시문에 따라 다른 행동**이 나와야 한다.

| # | 지시문 | object_topic | place_topic |
|---|---|---|---|
| 1 | put the **red** block on the **left** marker | `/scene/objects/red/pose` | `/scene/places/left/pose` |
| 2 | put the **blue** block on the **left** marker | `/scene/objects/blue/pose` | `/scene/places/left/pose` |
| 3 | put the **red** block on the **right** marker | `/scene/objects/red/pose` | `/scene/places/right/pose` |

1↔2 는 **어떤 물체를**, 1↔3 은 **어디로** 가 다르다. 네 요소(블록 2, 마커 2)가 **항상 동시에** 장면에 있다.

**상태머신은 코드 변경 0** — `object_topic`/`place_topic` 파라미터가 이미 있었다.
pose 를 토픽으로만 받게 설계(§D10 exit strategy)한 것이 그대로 값을 했다.

Isaac 쪽 변경(`--object-names` / `--place-names` / `--object-spacing`):
- 이름별 스폰 + **개별 토픽** `/scene/objects/<n>/pose`, `/scene/places/<n>/pose`(latched).
  기존 `/scene/object_pose`·`place_pose` 는 **첫 번째 것의 별칭**으로 유지(이미 검증된 경로를 안 깬다).
- **리셋이 모든 물체를 재배치.** 방해물이 지난 에피소드 자리에 남으면 같은 지시문에 장면이 달라진다.
- **파지 시 TCP 에 가장 가까운 물체를 용접.** 그리퍼는 지시문을 모르므로 고정 물체를 물면
  엉뚱한 걸 잡고도 성공으로 위장할 수 있다.
- 색은 RGB 상 멀리 떨어뜨린다 — "red vs orange" 구분은 우리가 낼 의도가 없던 인식 문제다.

### ★★ 함정 12 — 실패한 사이클은 **그 자리에서** 그리퍼를 풀어야 한다

CLOSE 이후에 실패하면 손가락이 물체에 물린 채 남고, 2F-85 mimic 링키지가 한계 밖으로 밀린다
(실측 −0.558, −0.974 / 범위 [0, 0.8]). **이 상태는 Isaac 재시작 외에 복구가 없다**(§함정 8).
무인 수집에서는 **사이클 하나가 전체 수집을 끝장낸다.**

→ `run_cycle` 이 실패 시 **같은 프로세스 안에서** detach + reset + 그리퍼 open 을 즉시 수행
(`_release_after_failure`). 검증: 사이클 2 실패 직후 `cleanup ok (finger_joint 0.000)` 로 복구되어
수집이 계속됐다. 다음 프로세스가 시작할 때 고치려 해서는 **늦다** — 그때는 이미 못 푼다.

### ★★★ 미해결 — 2×2 장면에서 성공률 0. **용접(D5)이 거의 안 걸린다** (2026-09-08 시점)

물체 2 × 목적지 2 로 확장한 뒤 성공률이 **0/15** 로 무너졌다. 아래는 **측정으로 확정된 것과
아직 모르는 것**을 구분해 적는다. 내일 여기서 이어갈 것.

#### 확정된 사실

| 측정 | 결과 |
|---|---|
| 그리퍼 명령 추종 | 명령 0.28 → 실제 **0.2801** (용접 임계값 0.25 초과) — 그리퍼는 정상 |
| 도달성 (plan-only 스윕, x=0.60) | y **−0.30 ~ +0.40** 전 구간, 파지·접근 높이 **모두 ok** — 도달성은 원인이 아님 |
| Isaac TCP 소스 | **손가락 패드 프림 정상 해석** (base_link 폴백 아님) |
| 파지 순간 물체 추적 | GRASP 후 (0.597,−0.096,0.225) 정상 → CLOSE(0.80) → LIFT 후 **(1.258,−0.186,0.036)** = 발사됨 |
| 용접 발생 빈도 | **15 사이클 중 2회** |
| 2단계 닫기 적용 후 | **6/6 `weld_failed`** — 용접이 0.28 에서 안 걸림 |
| `weld skipped` 진단 로그 | **한 번도 안 찍힘** ← 가장 중요한 단서 |

#### 가장 유력한 다음 수순

`weld skipped` 경고는 `fj >= _g_close` 가 참일 때 찍히도록 넣었는데 **한 번도 안 나왔다.**
그런데 `finger_joint` 은 0.2801 로 임계값을 넘는다. 즉 `_grasp_step()` 이 그 분기에 **도달조차
못 하고 있다**는 뜻이고, 그 함수는 맨 앞에서 이렇게 조용히 빠져나간다:

```python
try:
    fj = float(_art.get_joint_positions()[_names.index("finger_joint")])
except Exception:
    return          # <-- 예외를 삼킨다
```

`_names`(= `_art.dof_names`)에 `finger_joint` 이 없으면 `ValueError` 로 **매번 조용히 return** 한다.
→ **먼저 `_art.dof_names` 를 찍어서 실제 DOF 이름을 확인할 것.** 이 except 가 원인을 가리고 있었다.
(2/15 만 걸렸던 것도 이 가설과 모순되지 않는지 함께 볼 것.)

#### 이번 라운드에서 넣은 것 (유지)

- **2단계 닫기** `grip_preclose`(0.28): 패드 간격 `84.8·(1−j/0.8)` mm 이므로 0.25→58.3, **0.33→50(접촉)**.
  0.80 을 한 번에 명령하면 몇 물리 스텝 만에 닫혀 용접이 끼어들 틈 없이 물체가 발사된다.
  0.28 에서 멈춰 용접을 건 뒤 마저 닫는다. 용접 실패 시 **더 닫지 않고 `weld_failed`** 로 중단.
- **`dropped_in_transfer` 검사**: 운반 중 놓친 걸 잡는다. 없을 때 **가짜 성공**이 있었다 —
  떨어뜨린 물체가 마커 57 mm 지점에 떨어져 60 mm 임계값을 통과했다. `place_tol` 0.06→**0.035**.
- **실패 후 READY 복귀**: 실패 경로가 팔을 아무 데나 두면 다음 사이클이 그 자세에서 계획을 시작해
  `plan_failed_pre_grasp` 로 연쇄 붕괴한다.
- **`/scene/reset_gripper`**(Isaac): 한계 밖으로 나간 링키지를 텔레포트로 되돌린다. **순서가 중요** —
  detach → reset_episode → **열림 명령** → 텔레포트 ×수회. 텔레포트를 먼저 하면 컨트롤러가
  닫힘 목표를 잡고 있어 즉시 되돌아간다(실측 −0.558 → 그대로 → 열림 후 −0.197 → 반복 후 0.000).
- **TCP 소스 배너 + `weld skipped` 진단**: 폴백 여부와 실제 거리를 기동/실패 시점에 드러낸다.
- 테이블 충돌 상자 **기본 OFF**(`table_sink: -1.0`). 측정상 성공률을 못 올리면서 계획 실패를 늘렸다.
  **실물에서는 반드시 켤 것**(+ ACM).

#### ★ 진행 방식에 대한 반성 (되풀이하지 말 것)

단일 물체에서 6/8 이 나온 뒤 **한 번에 여러 개를 바꿨다** — 물체 2개화, 좌표 3회 변경, 테이블 충돌,
`grasp_z_offset`, RECENTRE, `wait_settled`, `jump_threshold`, 판정 강화. 실패 원인 후보가 늘 여러 개라
하나씩 짚는 데 시간을 다 썼고, 고칠 때마다 새 문제를 만들었다(간격을 벌리려다 물체를 테이블 밖으로).

또한 **보고했던 6/8·4/5 는 느슨한 판정**(마커 60 mm, 낙하 검사 없음)으로 잰 값이라 부풀려져 있었다.
가짜 성공이 몇 개 섞였는지 알 수 없다. → **한 번에 하나만 바꾸고 매번 같은 판정으로 6 사이클 측정.**

### 남은 것
위 용접 원인 규명이 최우선. 이어서 정지 프레임 정리(에피소드 ≈56초 중 상당수가 대기),
(선택) ACM 로 테이블 충돌 정밀화.

## 26. pick&place 복구 — Isaac 2F-85 에셋과 URDF 의 3중 불일치 — 2026-09-08

§25 에서 물체 2개 × 목적지 2개로 확장한 뒤 성공률이 **0/15** 로 무너졌다. 원인은 확장 자체가
아니라, 그 전부터 있었으나 단일 물체 조건에서 **가려져 있던** NVIDIA 스톡 Robotiq 2F-85
에셋과 ROS `robotiq_description` URDF 의 불일치 세 가지였다. 최종 **6/6, 배치 오차 0.000~0.001 m**
(허용 0.035). 재현 검증은 저장소 산출물만으로 수행(명령줄 오버라이드 없음).

### 26.1 `finger_joint` 규약이 반대 — **★ 2026-09-09 철회됨. §28 을 먼저 읽을 것**

> **이 절의 결론은 틀렸다.** 에셋은 URDF 와 같은 규약(0=열림)이며, 반전은 필요 없었다.
> 아래 서술과 `--gripper-invert` 는 **없던 문제를 만든 것**이다. 근거·재현·후속 조치는 §28.

같은 관절값에서 두 모델의 개도가 정반대다(실측):

| `finger_joint` | URDF 팁 간격(TF) | Isaac 패드 간격(PhysX) |
|---|---|---|
| 0.0 | 135.5 mm (열림) | 0.0 mm (닫힘) |
| 0.4 | 95.8 mm | 39.7 mm |
| 0.8 | 50.7 mm (닫힘) | 84.9 mm (열림) |

행정 거리는 84.8 vs 84.9 mm 로 같고 방향만 반대 — `j_isaac = 0.8 − j_urdf`. 실물
`robotiq_driver` 규약이 `0=열림` 이므로 **에셋이 틀렸다.**

증상: 데모가 `OPEN` 을 보내면 Isaac 그리퍼가 **닫힌다.** 닫힌 채로 하강해 부품을 146 mm
밀어내고, `finger_joint` 이 자기 하한 밖(−0.754)으로 강제로 벌어지고, 팔이 55 mm 못 미친 채
막힌다. 그 상태에서 `reset_gripper` 로 관절을 강제 복구해도 다음 사이클에 반복된다.

수정: **sim 경계에서 토픽 값만 반전**(`--gripper-invert`, 기본 ON). 그래프는
`*_isaac_raw` 토픽과 대화하고, 우리 노드가 `finger_joint` 만 `0.8 − j` 로 바꿔 중계한다.
팔 관절 값은 그대로 통과하므로 ros2_control·TF·MoveIt·**IL 기록**이 전부 URDF 규약을 본다.

> **USD 관절 프레임을 직접 고치려는 시도는 2회 모두 실패했다.** 두 `localRot` 에 같은 회전을
> 곱해 축을 뒤집고 `localRot1` 에만 범위만큼 곱해 영점을 옮기는 계산은 대수적으로 맞고
> `Gf` 합성 순서도 실험으로 확인했으나, PhysX 가 만든 자세는 매번 달랐다. `finger_joint` 만
> 고치면 오른쪽 손가락이 각도 커플링을 통해 따라오다 깨져 비대칭이 된다(간격 50→112 mm).
> 에셋 수술은 §26.4 로 남긴다.

### 26.2 파지점 오프셋 31 mm 차이

데모는 URDF `robotiq_85_*_finger_tip_link` 기준 **98.3 mm** 를 파지점으로 쓴다. Isaac 패드
메시(`fingertipsstep`)의 실제 범위는 공구축 **110.4~148.4 mm**, 중심 **129.4 mm**.

31 mm 낮게 조준하니 손끝이 테이블 아래로 들어가야 하고, 불가능하니 팔이 걸려 멈춘다.
그 어긋난 자세에서 부착이 걸리면 부품이 TCP 보다 37~39 mm 아래 매달려 `dropped_in_transfer`
로 오판된다. 수정: sim 에서만 `tcp_offset: 0.1294`.

### 26.3 완전 개방 시 간섭 — 하강·해제 둘 다 망가뜨린다

같은 큐브에 개도만 바꿔 하강시킨 실측(손끝 vs 큐브 윗면):

| 개도 | 결과 |
|---|---|
| 0.00 | +15.7 mm 에서 막힘 |
| 0.10 / 0.20 | +11.7 mm 에서 막힘 |
| **0.30** | **−29.6 mm 통과** |

> **철회됨(§29.4).** 이 절의 "완전 개방 시 간섭"은 **존재하지 않는 현상**이었다. 당시
> `--gripper-invert` 가 켜져 있어 URDF 0.0 이 물리적으로는 *닫힘*이었고, 닫힌 그리퍼로
> 하강하니 당연히 막혔다. "원인 부품을 특정하지 못한" 것도 그런 부품이 없었기 때문이다.
> `grip_approach 0.30` 은 제거됐다(반쯤 열린 상태를 우연히 만들어 준 값일 뿐).

활짝 열수록 막힌다 — 폭이 아니라 **자세** 문제다(완전 개방 시 바깥·아래로 스윙하는 부품).
같은 간섭이 해제에도 작용해, 1.2 mm 오차로 정확히 놓은 부품을 **37 mm 쳐냈다**(허용 35 mm를
겨우 넘겨 `place_failed`). 수정: sim 에서만 `grip_approach: 0.30`, 접근·하강·해제에 사용.
`--grasp-close 0.35` / `--grasp-release 0.32` 가 이 값을 **위아래로 감싸야** 한다 — 낮으면
하강 내내 부착이 무장되고, 높으면 0.30 까지만 열어서는 부착이 안 풀려 부품을 든 채
다음 사이클로 넘어간다.

### 26.4 아직 남은 것 / 함정

- **에셋 정합이 근본 해법**(`build_ur16e_2f85.py` 재베이크). 위 셋은 전부 sim 전용 보정이다.
- **`convexHull` 가설은 기각.** 그리퍼 콜라이더가 전부 `convexHull` 이라 오목한 손가락이
  메워진다고 보고 `convexDecomposition` 으로 바꿨으나 **효과 없었다**(미달 23.2→37.9 mm).
  플래그(`--gripper-collision`)는 남겨두되, 효과가 확인된 수정이 아니다.
- **정적 USD 형상 추론이 물리와 3회 어긋났다**(관절 프레임 2회, 콜리전 근사 1회). 형상 값으로
  계산한 패드 간격 87 mm 는 실측 통과 폭과 끝내 맞지 않았다. **시뮬레이터에 직접 물어보는
  방식만 답을 줬다.**
- **`reset_gripper` 는 모든 그리퍼 관절을 에셋의 authored 영점(전부 0)으로** 보내야 한다.
  구동 관절만 "열림"으로, 나머지를 0 으로 보내면 물리적으로 불가능한 링키지가 된다
  (`finger_joint` 0.539 인데 패드 간격 91 mm).
- **`_attach` 의 상대자세는 반드시 물리 기반**으로 계산할 것. `UsdGeom.XformCache` 는 USD
  스테이지(정적 authored 자세)를 읽으므로, 시뮬레이션 링크 포즈가 아니다. 이걸로 고정관절을
  만들면 부착 순간 부품이 0.6 m 순간이동한다. 같은 이유로 `_tcp()` 도 두 패드에 **동일 좌표**를
  돌려주고 있었다(TCP 가 그리퍼 밑동으로 붕괴 → 부착이 8 mm 차이로 영구 불가).
- **링크 이름으로 조회 금지.** `body_names` 에 `base_link`(로봇)와 `base_link_0`(그리퍼)가
  둘 다 있어 `index("base_link")` 가 **로봇 베이스**를 준다. 위치로 판별할 것.

### 26.5 어제 수집한 IL 데이터는 폐기 대상

`outputs/lerobot_ds`(4 에피소드) 는 §26.1 보정 이전에 수집됐다. 기록된 `finger_joint` 은
0→0.8145 이지만 그 값은 Isaac 원값이므로 **의미가 반대**다. 손목 카메라 프레임을 열어보면
데모가 "닫힘"이라 기록한 순간 손가락이 활짝 벌어져 있고, 부품은 D5 부착으로만 따라간다.
**이 데이터로 학습한 정책은 실물에서 집어야 할 때 손을 벌린다.** ACT 손실이 잘 떨어진 것은
데이터 내부적으로 일관됐기 때문이며, 그 일관성이 현실과 반대였을 뿐이다. 재수집 필요.

### 26.6 재현 방법

`config/common/pick_place.yaml`(공통) + `config/common/pick_place_sim.yaml`(sim 전용) +
`launch/common/pick_place_demo.launch.py`. **코드 기본값은 실물 기준**이고 sim 보정만 덮는다 —
방향이 중요하다. sim 파일을 빠뜨리면 sim 이 요란하게 실패하지만, 반대로 두면 실물 팔이
31 mm 낮게 조준해 테이블을 들이받는다. 확인:

```
use_sim:=true    tcp_offset pinned by parameter: 0.1294 m
use_sim:=false   measured tcp_offset ... = 0.0983 m
```

## 27. IL 파이프라인 전 구간 관통 — 수집 → 변환 → 학습 → 추론 — 2026-09-09

> **★ 데이터 관련 주의.** 이 절의 배관(수집→변환→학습→추론)은 유효하지만, 여기서 수집한
> **21 에피소드는 폐기 대상**이다. §28 의 그리퍼 규약 오판 때문에 물체를 실제로 문 적이 없다.

§26 에서 pick&place 를 6/6 으로 되살린 뒤, LeRobot 상류 파이프라인으로 **추론까지** 연결했다.
루프 안의 자체 코드는 **로봇 어댑터 하나**뿐이다.

### 27.1 아키텍처 — 상류를 그대로, 어댑터만 우리 것

```
Layer 3  lerobot.async_inference.policy_server   상류
         정책 (act / pi0 / pi05 / smolvla / groot) 상류
Layer 2  lerobot.async_inference.robot_client    상류
         lerobot_robot_ur16e_ros                 ★ 우리 것, 유일
Layer 1  ROS 2 + ros2_control + Isaac / 실물 UR16e
```

VLA 전환은 `--policy_type=groot --pretrained_name_or_path=nvidia/GR00T-N1.7-3B` 로 **인자만 바뀐다**.
`RobotClientConfig` 가 `policy_type` / `robot: RobotConfig` / `task`(언어 지시)를 받으므로 코드 변경이 없다.

**어댑터는 상류의 공식 플러그인 규약을 따른다**: 배포판 이름이 `lerobot_robot_*` 이면
`register_third_party_plugins()` 가 자동 임포트하고, 설정 클래스 `XConfig` → 로봇 클래스 `X` 로
`make_device_from_device_class` 가 생성한다. 그래서 `PYTHONPATH` 조작도 wrapper 도 없다.
(CLAUDE.md 가 참조하는 OMY 리더 플러그인 `lerobot_teleoperator_omy` 도 같은 규약 — 나중에 그대로 붙는다.)

### 27.2 ★ 환경 경계의 정확한 위치

`plan_il_vla.md` §2.3 은 "ROS 2 와 정책 환경은 같은 venv 에 못 넣는다"고 했으나 **부분적으로 틀렸다**.

| | 결과 |
|---|---|
| `deps/.venv-ml` 에서 `rclpy` + `torch(cu128, sm_120)` | **공존한다** |
| numpy 1.x 로 빌드된 ROS **C 확장**(`cv_bridge`) | **프로세스 코어 덤프** |

ROS Jazzy 는 numpy 1.26.4, venv 는 lerobot 이 요구하는 2.2.6 이라
`A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.6` 로 죽는다.
→ 어댑터는 `sensor_msgs/Image` 를 **numpy 로 직접 디코딩**한다(스트라이드·인코딩 처리 포함).
**RGB 로 뽑아야 한다** — 학습 데이터가 RGB(`il_recorder` cv2 JPEG → `raw_to_lerobot` PIL `.convert("RGB")`)라,
BGR 을 넣으면 정책은 돌지만 색이 뒤집힌 입력을 받는 **조용한 실패**가 된다.

소켓 경계가 여전히 필요한 경우: openpi 자체 서버, NVIDIA `Isaac-GR00T` 저장소 직결 등 **lerobot 밖** 스택.

### 27.3 그 밖의 경계 함정

- **`pyserial`**: 상류 `robot_client` 는 로봇 타입을 전부 임포트해 등록하는데, ROS 오버레이를 source 하면
  `/opt/ros/jazzy/.../dynamixel_sdk` 가 보이고 그게 `serial` 을 요구한다. 우리 하드웨어와 무관하지만 없으면
  클라이언트가 우리 로봇에 도달하기도 전에 죽는다.
- **스트리밍 컨트롤러 스폰**: `forward_position_controller` 는 yaml 에 정의돼 있지만 teleop 런치에서만
  스폰됐다. 없으면 `switch_control_mode.py` 가 실패하고, 그걸 놓치면 정책이 아무도 안 듣는 토픽에 발행해
  **팔이 안 움직인다 = 정책이 고장난 것처럼 보인다**. → `policy_inference.launch.py`(스폰만 한다).
- **lerobot 버전 핀**: `requirements-ml.txt` 에 버전이 없었다. 어댑터가 `lerobot.robots.robot.Robot` 을
  상속하므로 다른 PC 에서 최신판이 깔리면 조용히 깨진다 → `==0.6.1` 고정.
  **git clone 대신 PyPI 핀**이 "공식 오픈소스를 그대로"에 더 충실하다(로컬 수정이 섞일 수 없다).

### 27.4 데이터 수집 — 해상도와 그리퍼 분포

**해상도 320×240 으로 결정.** 근거는 두 정책의 동작이 정반대라는 것:

| | 리사이즈 | 입력 |
|---|---|---|
| ACT | **안 함**(ResNet 완전 컨볼루션) | 데이터셋 해상도가 곧 모델 입력 |
| GR00T N1.7 | **함** | **256×256**(`N1_7_DEFAULT_IMAGE_TARGET_SIZE`), 크롭 230×230 |

즉 480×640 은 GR00T 단계에서 전부 버려진다. 320×240 은 GR00T 의 256×256 과 화소수가 비슷해 사다리가 이어진다.
Isaac 은 `--camera-res`(기본 640×480 유지 — **정적 카메라는 nvblox 입력**이라 낮추면 ESDF 가 나빠진다).

**★ 그리퍼 분포 구멍과 그 해법.** — **폐기(§29.2)**: 구멍의 원인인 `grip_approach 0.30` 이
사라져 접근·하강·해제가 전부 완전 개방이 됐다. 추가했던 RETRACT 후 완전 개방 단계도 함께 제거.
아래는 당시 기록. `grip_approach 0.30`(§26.3) 때문에 기록되는 그리퍼 채널이 `0.37~1.0`
이 되어 **완전 개방이 데이터에 없었다**. 실물은 완전히 열리므로 그대로 두면 정책이 한 번도 못 본 분포를
실물에서 만난다. → 상태머신에 **RETRACT 이후 완전 개방** 단계를 추가(그리퍼가 물체·테이블에서 멀어진
뒤라 간섭 대상이 없다). 결과 `0.0000~1.0000`. 원인 부품을 특정하지 못한 채로도 전이 영향은 제거된다.

**과수집 수정**: 디스크의 에피소드 수를 세어 목표 도달 시 중단하고 실패분만 보충한다.
성공률이 높아진 지금 `N+2` 를 일괄 요청하면 20 요청에 22개가 쌓여 데이터셋 카드가 틀려진다.

### 27.5 실측값

수집 21 ep(태스크당 7) / 32,254 프레임 / raw 906 MB → LeRobot v3.0 `(240,320,3)`.

| | 480×640 | 320×240 |
|---|---|---|
| `num_workers` | **0 만 가능** | **2 가능**(`/dev/shm` 22/64 MiB) |
| 속도 | 45 ms/sample, **디코딩 병목** | **8 step/s**, GPU 91~94% = **GPU 바운드** |
| 10만 스텝 | 10시간 이상 | **3.7시간** |

산술로는 `2워커 × prefetch 2 × 28 MiB = 112 MiB` 라 실패를 예상했으나 **틀렸다**. 재는 게 빠르다.

### 27.6 남은 것

- **정책 성능은 아직 평가하지 않았다.** 태스크당 7 ep · 2000 스텝은 관통 검증용이고 loss 0.042 는
  과적합일 가능성이 높다. 본 수집(태스크당 30~50)이 다음 단계.
- **완전 개방 간섭의 원인 부품 미상**(§26.4). 영향은 27.4 로 제거됨.
- **실물 정적 카메라 런치 없음** — D455 를 `/static_cam/*` 으로 띄우는 런치가 필요(태스크 #10).

## 28. ★ 그리퍼 규약 오판 — §26.1 철회 — 2026-09-09

**에셋의 `finger_joint` 규약은 URDF 와 같다(0 = 열림).** §26.1 이 "반대"라고 결론내고 넣은
`--gripper-invert` 는 **없던 문제를 만들었다.** 사용자가 학습 영상에서 "파지할 때 손가락이 열린 채로
이동한다"고 지적해 드러났다.

### 28.1 눈으로 본 증거

Isaac GUI 를 띄우고 그리퍼를 명령한 뒤 손목 카메라를 그대로 캡처했다.

| 명령(URDF 규약) | `/joint_states` | 실제 화면 |
|---|---|---|
| `0.0` = 열림 | 0.0000 | **닫힘** (손가락 맞닿음) |
| `0.8` = 닫힘 | 0.7999 | **열림** (블록에 닿지도 않음) |

경계 릴레이 자체는 정상 동작했다 — 문제는 릴레이가 아니라 **반전을 넣기로 한 판단**이다:

```
/isaac_joint_commands           0.8     ros2_control (URDF 닫힘)
/isaac_joint_commands_isaac_raw 0.0     반전 적용
/isaac_joint_states_isaac_raw   0.0001  에셋 실제값, 명령을 정확히 추종
/isaac_joint_states             0.7999  되돌림
```

### 28.2 왜 틀렸나 — 같은 함정에 두 번

반전의 근거는 개도 스윕의 **"패드 간격"** 이었다: `fj=0 → 0.0mm`(닫힘으로 판정),
`fj=0.8 → 84.9mm`(열림으로 판정). 그런데 그 값은 `left/right_inner_finger` 의
**링크 원점 간 거리**다.

**링크 원점은 보이는 형상이 아니다.** 같은 날 §26.2 에서 `finger_tip_link` 원점을 패드 중점으로
착각해 31 mm 를 틀렸는데, 그 교훈을 이 판정에는 적용하지 않았다. 피벗이 겹쳐도 손가락 몸통은
벌어져 있을 수 있고 실제로 그랬다.

**손목 카메라 이미지는 내내 있었다. 숫자만 보고 한 번도 눈으로 확인하지 않았다.**

### 28.3 무엇이 무효가 되나

- **`--gripper-invert` 제거 대상.** 기본값 ON 으로 들어가 있다.
- **그 위에 쌓은 값 전부 재검토**: `--grasp-close 0.35` / `--grasp-release 0.32` /
  `grip_approach 0.30` / `grip_preclose 0.40` 은 모두 반전을 전제로 정해졌다.
- **§26.3 "완전 개방 시 간섭"의 정체가 달라진다.** URDF 0.0 이 실제로는 *닫힘*이었으니,
  닫힌 그리퍼로 하강해 부딪힌 것뿐이다. 원인 부품을 못 찾은 것도 당연하다 — 그런 부품은 없었다.
  `grip_approach 0.30` 이 "해결"한 이유도 이걸로 설명된다(0.30 이 실제로는 반쯤 열린 상태).
- **`outputs/il_raw_3task_240` + `lerobot_ds_240`(21 ep) 폐기 대상.** 6/6 성공은 D5 부착이
  물체를 붙들어 준 결과이고 물리적으로 문 적이 없다. `outputs/act_2000` 체크포인트도 마찬가지.
- **§26.5 의 "어제 데이터가 가짜 파지" 진단도 재검토 필요.** 그때는 반전이 *없었으므로*
  다른 원인일 수 있다.

### 28.4 내일 할 일 (순서)

1. `--gripper-invert` 를 끄고, **개도 0.0/0.2/0.4/0.6/0.8 마다 손목 카메라를 캡처해 눈으로**
   규약을 확정한다. 숫자 프록시(패드 간격·링크 원점)는 이번 건에서 두 번 틀렸다.
2. 그 위에서 `grasp-close` / `grasp-release` / `grip_approach` / `grip_preclose` 를 다시 잡는다.
   **판정 기준은 "패드가 물체에 닿아 있는가"를 이미지로** 확인하는 것.
3. 6 사이클 재측정. **성공률만 보지 말고 파지 순간 프레임을 반드시 함께 볼 것.**
4. 통과하면 재수집 → 변환 → 학습 → 추론.

### 28.5 남는 교훈

숫자가 일관되면 맞다고 믿었다. 스윕은 단조롭고 대칭이었고 두 모델의 행정 거리도 84.8 vs 84.9 mm 로
맞아떨어졌다 — 전부 링크 원점 기준이라 **일관되게 틀린** 값이었다.
**렌더 이미지가 있는 시뮬레이터에서는, 기하 판정을 한 번은 눈으로 확인할 것.**

## 29. 그리퍼 규약 정정 실행 — §28.4 순서대로 — 2026-09-09

§28 이 남긴 4단계를 그대로 실행했다.

### 29.1 1단계 — 눈으로 규약 확정

`--gripper-invert` 를 끄고 손목 카메라를 개도별로 캡처했다. **팔을 먼저 READY 로 보내야 한다** —
기동 시 home 자세에서는 손목 카메라가 광원을 등져 전 프레임이 까맣게 나온다(첫 시도 5장 전부 폐기).

| 명령 | `/joint_states` | 실제 화면 |
|---|---|---|
| `0.0` | 0.0001 | **활짝 열림** |
| `0.4` | 0.4000 | 절반 |
| `0.8` | 0.8000 | **손가락 맞닿음 = 닫힘** |

→ **에셋 규약 = URDF 규약 = 실물 규약**(0 = 열림, 0.8 = 닫힘). §28 의 결론이 확정됐고,
반전은 처음부터 존재하지 않는 문제였다.

### 29.2 2단계 — 반전 위에 쌓은 값 되돌리기

| 항목 | 반전 전제(폐기) | 현재 | 비고 |
|---|---|---|---|
| `--gripper-invert` / `--gripper-range` / 릴레이 토픽 | 있음 | **삭제** | 인자·중계 구독/발행 전부 제거 |
| `--grasp-close` | 0.35 | **0.25** | 부착 임계 |
| `--grasp-release` | 0.32 | **0.15** | |
| `grip_approach` (sim) | 0.30 | **0.0** | 코드 기본값과 동일 → sim 보정 소멸 |
| `grip_preclose` | 0.40 | **0.28** | |
| RETRACT 후 추가 완전개방 | 있음 | **삭제** | `grip_approach 0.30` 때문에 생긴 단계였음 |

**`config/common/pick_place_sim.yaml` 의 sim 전용 보정이 0개가 됐다.** 두 개 있던 보정
(`tcp_offset`·`grip_approach`)이 모두 sim/real 차이가 아니라 **우리 버그**였기 때문이다.
파일과 런치 배선은 남겼다 — 진짜 차이는 실물 그리퍼를 돌리는 날 나올 것이고,
"지금은 차이가 없다"를 정직하게 표현하는 방법이 빈 파일이다.

부수 정리: 코드의 `weld`(용접) 표현을 전부 `grasp attach` 로 바꿨다. Isaac D5 는
`UsdPhysics.FixedJoint` 일 뿐 용접이 아니고, 사용자가 이 단어를 지적했다.

### 29.3 3단계 — 6 사이클 재측정, **파지 순간 프레임 필수 확인**

`scratchpad/grasp_watch.py`: `finger_joint` 만 보고 사이클마다 3장을 자동 저장한다
(0.70 상향 통과 → `closed_N`, 3초 뒤 → `lift_N`, 0.20 하향 통과 → `open_N`).
상태머신에 배선하지 않았으므로 상태머신이 틀려도 증거는 남는다.

**성공률만 보지 않는 이유가 §28 그 자체다** — v1 수집도 6/6 이었는데 한 번도 물지 않았다.

**결과: 6/6 성공, 배치 오차 전 사이클 0.001 m.** 그리고 성공률과 별개로 **18장(6×3) 전부를 눈으로 확인**했다.

| 프레임 | 보이는 것 |
|---|---|
| `closed_N` | 빨간 블록이 **양쪽 패드 사이에 물려** 있음 |
| `lift_N`   | 블록이 그리퍼에 붙은 채 테이블에서 떨어져 있음 |
| `open_N`   | 턱이 활짝 열리고 블록이 **초록 마커 위에** 놓임 |

v1 수집에서는 이 장면들이 없었다(턱이 열린 채 이동). 같은 6/6 이지만 내용이 다르다.

### 29.4 정리

- sim 전용 보정 0개 = **sim 과 real 이 같은 파라미터로 돈다**. IL/VLA 를 실물로 옮길 때
  "sim 에서만 되던 값"이 남아 있지 않다는 뜻이다.
- §26.3 "완전 개방 시 간섭"은 **존재하지 않는 현상**으로 확정. 닫힌 그리퍼로 하강했을 뿐이다.
- §26.5 "어제 데이터가 가짜 파지" 는 반전 도입 *이전* 이슈라 별개 원인이며, 지금은 재현되지 않는다.
- 다음: `outputs/*_240_v2` 로 재수집 → 변환 → 학습 → 추론. v1(`il_raw_3task_240`,
  `lerobot_ds_240`, `act_2000`)은 **지우지 않고 §28 의 증거로 남긴다.**

## 30. ACT 는 언어를 안 받는다 — 3태스크 혼합 학습 중단 — 2026-09-09

사용자 질문: *"지금은 ACT 를 학습하는거지? 내가 알고있는 ACT 는 txt로 명령을 받는건 아니었던 거 같아서"*.
맞는 지적이었고, 돌고 있던 학습을 중단했다.

### 30.1 확인 방법

추측하지 않고 설치된 `lerobot 0.6.1` 코드를 봤다.

```
policies/act/modeling_act.py   -> batch[OBS_IMAGES], batch[OBS_STATE], batch[OBS_ENV_STATE]
policies/act/*                 -> tokenizer/language_model 참조 0건
policies/{pi0,pi05,smolvla,groot,...}/ -> 전부 있음
```

원논문(ALOHA)의 ACT 도 태스크당 따로 학습하는 구조라 일치한다.

### 30.2 무엇이 잘못이었나

`plan_il_vla.md` §2.8 은 처음부터 "ACT는 언어 비조건부"라고 **맞게** 써 있었다.
틀린 건 문서가 아니라 실행이다 — §2.8 의 "동일 데이터로 ACT vs GR00T 비교"를
**같은 데이터셋을 ACT 에 그대로 먹인다**로 읽고 3태스크 혼합본으로 학습을 걸었다.

우리 씬은 빨강·파랑 블록과 좌·우 마커가 **매 에피소드에 모두 존재**한다. 지시문이 유일한
구분자인데 ACT 는 그걸 못 본다 → **같은 픽셀에 세 개의 다른 정답** = 모순된 지도학습.
"언어를 무시해서 성능이 낮은 정책"이 아니라 **기준선 역할을 못 하는 정책**이 나온다.

### 30.3 조치

- 20k 혼합 학습 중단(`act_v2_20k` 폐기).
- `plan_il_vla.md` §2.8 에 데이터셋 분기 규칙 추가: **ACT = 1종 필터, VLA = 3종 합본**.
- `collect_il_episodes.sh` 가 `TASKS` 환경변수로 태스크 세트를 받도록 일반화
  (1종=ACT, 3종=VLA). 스크립트를 복제하지 않았다.
- **수집한 21 에피소드는 유효하다** — `task` 문자열이 메타에 있어 필터로 단일 태스크
  데이터셋을 만들 수 있고, 합본 그대로가 VLA 용이다.

### 30.4 물체 랜덤화 확대 — 사용자 지적

*"단일 테스크로 하려면 레드 박스도 초기 위치를 랜덤하게 해주면 되겠네."*

**±25 mm → ±60 mm** (yaw 는 기존대로 전 범위). 상한을 정하는 건 물체 간 충돌이다:
red/blue 홈이 y 로 200 mm 떨어져 있고 큐브가 35 mm 이므로 ±60 mm 면 최악에도 80 mm 가 남는다.

이건 정책을 어렵게 만드는 게 아니라 **평가를 가능하게** 만드는 조치다. 물체가 거의 고정이면
ACT 가 카메라를 무시하고 궤적 하나만 외워도 loss 가 떨어져, 학습 여부를 판정할 수 없다.
3태스크 수집이 0.025 를 쓴 것은 정책에게 지나치게 관대했다.

## 31. 첫 롤아웃 — 배선은 정상, 정책이 PRE_GRASP 에서 얼어붙음 — 2026-09-09

60k 스텝 ACT(`act_red_left`, loss 0.028)를 Isaac GUI 에서 굴렸다. **파이프라인은 전 구간 정상,
정책은 태스크 미완수.**

### 31.1 첫 시도의 실패는 정책이 아니라 내 스크립트였다

사용자: *"gui 상으로 보면, ur이 멈춰있는거 같은데"*. 명령 토픽에 메시지 0건이었다.

| | 학습 데이터 50 ep | 첫 롤아웃 |
|---|---|---|
| 시작 자세 | READY `[0,-1.571,1.571,-1.571,-1.571,0]`, **편차 0.0002 rad** | `[-0.461,-1.216,1.230,-1.583,-1.571,-2.129]` |
| `wrist_3` 차이 | — | **2.13 rad (122°)** |
| 물체 | 매 에피소드 랜덤 | 홈 그대로(초기화 안 함) |

`infer_gui.sh` 가 `/scene/reset_episode` 도, `reset_pose.py ready` 도 부르지 않았다.
학습에서 한 번도 못 본 상태를 받은 정책이 얼어붙은 것. **롤아웃 하네스는 학습의 초기 조건을
재현해야 하고, 재현했는지 검증까지 해야 한다** → 스크립트에 READY 편차 검사를 넣었다(0.05 rad 초과면 경고).

**배선 무결성은 두 단계로 갈라서 확인**했다(추측 금지):
① 명령 토픽에 직접 발행 → 팔 이동 ② 어댑터를 정책 없이 단독 구동 → 팔 이동. 둘 다 정상.

### 31.2 초기 조건을 맞춘 재실행

| 구간 | 결과 |
|---|---|
| 시작 자세 | READY 편차 **0.0002 rad** |
| 정책 추론 | 66 ms/청크, `[1,50,7]` |
| 어댑터→컨트롤러 | **4,810건** 발행 |
| 컨트롤러→Isaac | 추종 오차 **0.0007 rad** |
| 카메라 | 320×240 rgb8, 손목 화면에 블록 **정중앙** |
| 태스크 | ❌ 180 s 내내 PRE_GRASP 에서 ±0.03 rad 진동, 블록 307 mm 불변 |

정책이 멈춘 자세는 학습 데이터 범위 **안**이다(예: `wrist_3` −2.80 ∈ [−2.866, 0.001]).
접근까지는 배웠고 **하강·파지를 안 한다.**

### 31.3 원인 — 데이터의 66%가 정지 프레임

```
이동량 < 1e-4 rad :  65.8%     중앙값 이동량: 0.00000 rad/step
에피소드 중반(20~80%) 구간 정지 비율: 70~89%
```

상태머신이 단계 전환마다 멈추는 시간(플래닝·그리퍼 개폐·안정화)을 30 Hz 로 그대로 기록했다.
ACT 는 L1 회귀라 **"움직이지 마"가 압도적 다수 정답**이 된다.

**★ 그래서 loss 0.028 은 성능 지표가 아니었다.** 정지 프레임만 맞춰도 loss 는 떨어진다.
`plan_il_vla.md` T4-B 에 "정지 프레임 정리"가 위험으로 이미 적혀 있었는데 수집 전에 처리하지 않았다.

### 31.4 조치 — 재수집 없이 변환만

`raw_to_lerobot.py --max-idle-run N --idle-eps E`. **삭제가 아니라 연속 정지 구간의 길이 제한**:
전부 지우면 30 Hz 등간격이 깨져 ACT 가 읽는 고정레이트 청크 구조가 무너진다.

| max_idle_run | 남는 프레임 | ep0 길이 |
|---|---|---|
| 0(기존) | 100.0% | 52.0 s |
| 2 | 38.7% | 21.7 s |
| **5(채택)** | **40.5%** | **22.7 s** |
| 20 | 47.6% | 26.2 s |

정지 판정은 `|action − state|`(기록 규약상 곧 그 프레임의 이동량)로 하며, **그리퍼만 움직이는
프레임은 정지로 치지 않는다** — 팔이 멈춘 채 그리퍼가 열리는 구간이 파지의 핵심이기 때문.

## 32. 상태머신 대기시간 제거 — 사용자 지적에서 출발 — 2026-09-09

사용자: *"정지 구간이 왜케 긴거야? moveit2 기반으로 플래닝을 한다고 해도 이정도 모션은 금방 될거 같은데"*.
맞는 직관이었다. **모션은 빠르다** — 직선 이동은 0.8~1.6 s. 느린 건 대기다.

### 32.1 한 사이클 41.3 s 의 내역 (로그 타임스탬프 실측)

| 항목 | 시간 | 성격 |
|---|---|---|
| `PRE_GRASP: did NOT settle within 3 mm` | **5.0 s** | 낭비 |
| `TRANSFER: did NOT settle within 3 mm` | **5.0 s** | 낭비 |
| PRE_GRASP 플래닝+실행 | 5.7 s | 정상 |
| READY 복귀 | 4.6 s | 태스크 밖 |
| 직선 이동 4회(GRASP/LIFT/PLACE/RETRACT) | 각 0.8~1.6 s | **빠름** |

**정착 타임아웃 10 s = 사이클의 24%.** 원인은 기다림으로 해결되지 않는 종류의 오차였다:
`topic_based` 백엔드는 위치 명령을 따라가고 남는 7 mm 는 **정상상태 오차**다. 감쇠하는 과도응답이
아니므로 5 s 를 기다리든 50 s 를 기다리든 같다. 그런데 코드는 "아직 흔들리는 중"으로 보고 기다렸다.

### 32.2 조치

| 수정 | 내용 |
|---|---|
| **평체(plateau) 조기 종료** | 오차가 `settle_plateau`(0.8 s) 동안 개선되지 않으면 종료 |
| **허용오차 2종 분리** | `settle_tol_approach` 10 mm(관절공간 웨이포인트) / `settle_tol_precise` 3 mm(직선이동 끝) |
| 그리퍼 `sleep(0.5)` | → finger_joint 도달 확인 + 스톨 감지 |
| 부착 대기 `sleep(0.8)` | → `/scene/grasp_active` **이벤트** 대기 |
| LIFT 확인 `sleep(0.7)` | → 물체 z 가 기준을 넘는 즉시 |
| 릴리스 `sleep(0.7)` | → 부착 해제 즉시 |
| 배치 판정 `sleep(0.7)` | → `wait_part_still()` 물체 정지 감지 |
| reset `sleep(1.0)` | → `wait_part_still()` |
| **READY 복귀** | `_run_cycle_inner` → `run_cycle`, 즉 **기록 구간 밖으로** |

**허용오차를 푼 게 아니라 용도별로 나눈 것**이 핵심이다. PRE_GRASP 의 7 mm 잔류는 뒤따르는
**절대좌표** 직선 하강이 0.2 mm 로 재영점한다(실측). 반면 그리퍼가 실제로 동작하는 GRASP/PLACE 는
3 mm 를 그대로 유지했다 — 그 기준은 파지가 18 mm 어긋나던 실제 버그를 잡은 장치다(§26).

**고정 대기는 짧으면 위험(오래된 물리 상태로 판정)하고 길면 정지 프레임이 된다.** 조건 자체를
기다리면 둘 다 해결되고, 검사는 그대로 남아 타임아웃 시 기존 동작으로 degrade 한다.

### 32.3 검증

3 사이클 3/3 성공, **`did NOT settle` 0건**:
```
PRE_GRASP: settled (7.2 / 7.3 / 7.6 mm)     이전: 전부 5 s 타임아웃
TRANSFER : settled (6.5 / 7.2 / 7.3 mm)     이전: 전부 5 s 타임아웃
GRASP    : settled (0.0 / 0.2 / 0.7 mm)     정밀 기준 유지
PLACE    : settled (0.1 / 0.1 / 0.4 mm)     정밀 기준 유지
```
사이클 총시간 비교는 **학습과 CPU 를 나눠 쓴 오염된 측정**이라 여기 적지 않는다. 학습 종료 후 재측정.

### 32.4 곁다리 — `/dev/shm` 의 진짜 소비자는 Isaac 이었다

`num_workers=2` 가 233 스텝에서 `unable to allocate shared memory` 로 죽었다. §24 의 그 증상인데
원인은 배치 크기가 아니었다: **Isaac + ROS 스택이 Fast-DDS 세그먼트로 64 MiB 중 ~14 MiB 를 상시 점유**한다.

Isaac 을 내리자 `/dev/shm` 22 → 3.3 MiB. 그 상태에서 워커 수를 **실측**했다(각 400 스텝):

| workers | 8 | 6 | **4** | 2 | 0 |
|---|---|---|---|---|---|
| step/s | 실패 | 실패 | **12.5** | 7.9 | 1.4 |

→ `train_thin.sh` 의 폴백 사다리를 `4 2 0` 으로. 60k 스텝이 12 h → **~70 분**.
**학습 전에는 Isaac 을 내린다** — 이게 §24 표가 상황에 따라 달라 보였던 이유다.

### 32.5 결과 — 정책이 처음으로 태스크를 해냈다 (2026-09-09)

정지 프레임을 솎아낸 데이터로 재학습(`act_red_left_thin`, 60k, 1 h 06 m, 15.16 step/s)한 뒤
같은 조건으로 롤아웃:

```
reset 배치   red@(0.601, -0.092)      ← 학습 시드(11)와 다른 시드(99)
결과         red (0.616, 0.227)       ← 좌측 마커 (0.600, 0.215)
이동         약 320 mm,  배치 오차 20 mm,  그리퍼 개방(놓음)
```
손목/정적 카메라 모두에서 블록이 마커 위에 있고 그리퍼가 물러난 것을 육안 확인.

**성공률 8/8** (`scratchpad/rollout_n.sh`, 시행마다 리셋+READY, 90 s, 판정은 상태머신과 같은
`place_tol` 35 mm + 그리퍼 개방):

| 시행 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| 배치 오차(mm) | 16 | 11 | 12 | 22 | 12 | 8 | 5 | 7 |

평균 11.6 mm. 물체 시작 위치는 매 시행 달랐다(x 0.563~0.633, y −0.155~−0.074, yaw 전 범위) —
같은 자리를 반복한 것이 아니다. **1회 성공은 성공률이 아니므로 반드시 반복 측정할 것.**

| | `act_red_left` | `act_red_left_thin` |
|---|---|---|
| 학습 데이터 정지 비율 | 65.8% | ~15% |
| 최종 loss | 0.028 | 0.028 |
| 롤아웃 | PRE_GRASP 에서 정지, 블록 **307 mm 불변** | **파지→이송→배치, 20 mm** |

**★ loss 는 성능 지표가 아니었다.** 두 정책의 최종 loss 가 **같은 0.028** 인데 결과는 정반대다.
정지 프레임이 다수인 데이터에서는 "움직이지 마"만 맞춰도 loss 가 떨어진다. 판정은 롤아웃으로 한다.

## 33. ★ 그리퍼가 물체를 뚫고 닫히고 있었다 — 사용자 지적 — 2026-09-09

사용자: *"lift_25.jpg 상으로 보면, gripper 가 블록을 집은게 아니라, gripper 는 끝까지 close 되어있고,
gripper 끝에 블록이 붙어있는 것처럼 보이는데"*. 정확한 관찰이었다.

### 33.1 무슨 일이 일어나고 있었나

`--grasp-attach` 는 `finger_joint 0.25` 에서 발동하며 **물체의 콜라이더를 끈다**(`_set_collision(False)`).
그 뒤 손가락은 저항 없이 `grip_closed 0.8`(완전 폐쇄)까지 닫히고, 블록은 FixedJoint 로 매달려 따라온다.

콜라이더를 끄는 것 자체는 필요하다 — 안 끄면 손가락이 블록을 밀어내 튕겨나간다(§26: 15회 중 2회만 성공,
블록이 테이블 건너로 비행). 문제는 **닫는 목표값이 물체 폭을 무시**한 것이다.

### 33.2 왜 문제인가 — 실물 전이

**실물 2F-85 는 물체에 걸려 멈춘다.** 코드 주석에도 그렇게 적혀 있다
(*"the 2F-85 stalls on the part, which is exactly what a successful grasp looks like"*).

| 그리퍼 채널 | v3(수정 전) |
|---|---|
| 0.00~0.05 (완전 개방) | 54.1% |
| 0.40~0.60 (**실물이 스톨할 구간**) | **1.8%** |
| 0.95~1.00 (완전 폐쇄) | 40.1% |

**명령(action)** 은 문제없다 — 실물에서 1.0 을 명령하면 걸려 멈추는 게 정상이다.
**관측(state)** 이 어긋난다: 실물은 파지 중 스톨값을 보고하는데, 정책은 그 상태를 1.8% 밖에 못 봤다.

### 33.3 스톨 개도는 공식이 아니라 실측으로

후보값 0.470 은 `gap = 84.8*(1 − j/0.8)` 에서 나왔고, 이 공식은 `inner_finger` **링크 원점** 기반이다.
링크 원점은 이미 두 번 틀렸다(§26.2 31 mm 조준 오차, §28 그리퍼 반전 오판). 그래서 실측했다.

**방법**: Isaac 을 `--grasp-attach` **없이** 기동(콜라이더 유지) → 데모를 `grip_preclose:=0.80` 으로 실행해
GRASP 자세에서 바로 완전 폐쇄 명령 → `/joint_states` 관찰.

```
명령      0.80 rad
실제 정지 0.5988 rad   ← 20 초간 고정
공식 예측 0.470 rad    ← 또 틀렸다 (세 번째)
```

0.470 을 그대로 썼다면 그리퍼가 블록에 닿기도 전에 멈춰 **파지가 아예 안 됐을 것이다.**

### 33.4 조치와 검증

`grip_closed` 0.8 → **0.599**. 기록기의 정규화 상수(0~0.8 rad → 0~1)는 그리퍼 전체 행정을 정의하는
**스키마**라 건드리지 않았다 → 파지 중 기록값이 1.0 이 아니라 0.599/0.8 = **0.749**.

**★ 함정: 코드 기본값을 바꿔도 `config/common/pick_place.yaml` 이 덮어쓴다.**
`p("grip_closed", 0.599)` 로 고쳤는데 기록값이 계속 1.0 이었다 — 런치가 얹는 yaml 에 `grip_closed: 0.8` 이
남아 있었기 때문. 같은 파일의 `grip_preclose: 0.40` 도 그대로여서, §29 에서 0.28 로 되돌린 값이
**실제로는 한 번도 적용된 적이 없었다.** → 값을 바꿨으면 소스가 아니라 **기록된 데이터로 확인할 것.**

검증(한 사이클 기록 후 raw 데이터 판독):
```
CLOSE: commanded 0.60
recorded gripper max = 0.7487   (finger_joint 0.5990 rad)   기대 0.7488
>0.95 프레임 = 0.0%   (이전 40.1%)
[cycle 1] SUCCESS (0.001 m)     파지 성공은 유지
```

## 34. 그립 깊이의 하한은 테이블이다 — 그리고 오진 두 번 — 2026-09-09

사용자: *"확실하게 하고 학습 데이터를 수집하도록 하자."* §33 에서 파지 자체는 고쳤지만,
그 데이터의 그립이 **35 mm 블록 윗면에서 3.1 mm 아래** 라는 것이 드러났다. 실물이면 미끄러진다.

### 34.1 오진 ① — "grasp_z_offset 의 근거가 무효다"

`grasp_z_offset 0.015` 의 주석은 *"블록 중심을 조준하면 손끝이 테이블에 닿는다"* 였다.
계산해 보니 패드 중심이 테이블 위 18 mm 라 여유가 충분해 보였고, **주석이 낡았다고 결론냈다. 틀렸다.**

**패드 중심만 보고 핑거의 형상을 빼먹었다.** 콜리전 메시는 `finger_tip_link` 원점 기준
**−6 ~ +51 mm**, 패드 중심은 +32 mm → **핑거 최하단이 패드 중심보다 19 mm 아래**다.
offset 0 이면 최하단이 0.199 로 테이블(0.200) **아래**다. 원래 주석의 방향이 맞았다.

### 34.2 오진 ② — "gripper() 의 평체 조기 종료가 원인이다"

깊이 스윕이 **비단조**로 나왔다(0.015 ✓, 0.010 ✓, 0.007 ✗, 0.005 ✓, 0.003 ✗, 0.000 ✗).
비단조 = 깊이 의존이 아니라고 보고, §32 에서 내가 넣은 `gripper()` 평체 종료를 의심했다
(턱이 출발하기 전에 "정체"로 판정해 반환 → 부착 대기 0.8 s 가 닫힘 전체를 감당).

그 버그는 **실재했고 고쳤다**(움직임 관측 후에만 평체 종료 허용, 상한 1.5→2.5 s).
하지만 **부착 실패의 원인은 아니었다** — 수정 후 재측정은 오히려 3/6 → **2/6**.

### 34.3 실제 원인 — 측정으로 확정

`finger_joint` 와 `/scene/grasp_active` 를 동시에 기록해 실패 케이스(offset 0.005)를 봤다:

```
명령 0.28  →  fj = 0.1032 에서 정지  (부착 임계 0.25 근처에도 못 감)
grasp_active 내내 False
```

**부착 로직이 아니라 물리적 간섭이다.** 닫히는 도중 핑거가 테이블에 걸린다. 2F-85 는 닫으면서
손끝이 안쪽·아래로 움직이므로, 정적 계산상의 여유(offset 0.005 에서 4.5 mm)를 소모한다.
→ 깊이 의존이 맞고, 스윕의 비단조는 경계 근처의 산포였다.

### 34.4 교훈

- **비단조 결과를 "깊이 무관"의 증거로 쓰면 안 된다.** 경계 근처에서는 산포가 정상이다.
- 실패의 원인은 결과(성공/실패)가 아니라 **중간 신호**(fj, grasp_active)를 봐야 나온다.
  성공/실패만 보고 두 번 연속 잘못 짚었다.
- 정적 기하 계산은 **닫히는 동안의 움직임**을 담지 못한다. 여유 4.5 mm 가 안전해 보였지만 아니었다.

## 35. ★ "측정했다"와 "맞는 것을 측정했다"는 다르다 — grip_closed 재정정 — 2026-09-09

사용자: *"finger 부분이 블럭을 약간 침투하는 느낌인데, finger 의 블럭과 닿는 부분이 아니라 바깥쪽
기준으로 닫히게 설정한게 아닌가 싶네."* 맞았다. 기준이 틀린 것은 맞고, 다만 "바깥쪽"이 아니라
**"힘 평형점"** 이었다.

### 35.1 §33 의 0.599 는 접촉점이 아니었다

§33 은 부착을 끄고 `0.8` 을 명령해 관절이 멈춘 곳(0.5988)을 `grip_closed` 로 삼았다.
**PhysX 의 스톨은 구동력과 접촉력이 균형을 이루는 지점이지 막 닿는 지점이 아니다.**
`max_effort 100` 으로 밀면 첫 접촉을 한참 지나 눌러 넣은 상태에서 멈춘다.

### 35.2 접촉 시작점 측정 (max_effort 10, 콜라이더 ON, 팔을 파지 자세에 고정)

```
cmd 0.48 -> fj 0.4801   블록 0.00mm   명령=실제, 아직 자유
cmd 0.50 -> fj 0.5000   블록 0.03mm   <- 패드가 닿기 시작
cmd 0.52 -> fj 0.5173   블록 0.08mm   실제가 명령에 뒤처짐 = 저항
cmd 0.60 -> fj 0.5406   블록 0.16mm
cmd 0.80 -> fj 0.5669   블록 0.52mm   더 안 닫힘
```

판별 근거는 블록 변위가 아니라 **`cmd` 와 `fj` 의 이탈**이다(변위 임계 0.5 mm 는 늦게 걸린다).

| 값 | 의미 |
|---|---|
| **~0.50** | 패드가 블록에 **막 닿음** |
| 0.567 | effort 10 으로 눌렀을 때 정지 |
| 0.599 | effort 100 으로 밀었을 때 정지 (§33 이 채택한 값) |

→ **`grip_closed` 0.599 → 0.52** (접촉 직후, 눌러 넣지 않음).

### 35.3 이 측정도 한 번 무효였다

1차 시도는 "0.8 까지 접촉 없음"으로 나왔다. **팔이 블록 위에 없었기 때문**이다 — 데모가 부착 실패로
중단되며 `_release_after_failure()` 가 READY 로 되돌렸고, 그 뒤에 스윕이 돌았다.
→ 파지 자세의 관절값을 실측해 팔을 그 자세로 보낸 뒤 다시 측정.

### 35.4 같은 실수를 세 번 했다

| 회차 | 무엇을 쟀다고 믿었나 | 실제로 잰 것 |
|---|---|---|
| §28 | 패드 간격 | `inner_finger` **링크 원점** 간 거리 |
| §33 | 패드-물체 접촉 개도 | 패드 간격 **공식**(역시 링크 원점) |
| §35 | 접촉 개도 | **힘 평형점**(눌러 넣은 상태) |

셋 다 "추측하지 않고 측정했다"고 적었다. 문제는 측정 여부가 아니라 **측정 대상이 질문과 일치하는가**다.
- 스톨/평형은 접촉이 아니다 — 구동력을 낮추거나, 접촉 *시작*을 별도 신호로 잡아야 한다.
- 링크 원점은 표면이 아니다.
- 그리고 **측정 전에 "지금 재는 게 정말 그 질문인가"를 한 번 적어 볼 것.**

## 36. v5 롤아웃 7/10 — 실패는 랜덤 범위의 가장자리에 몰린다 — 2026-09-09

§35 로 그립 기하를 바로잡고 v5(50 ep)를 학습해 롤아웃했다. 사용자가 GUI 로 보며
*"어떨때는 잘 하고 어떨때는 하강을 제대로 못 해서 실패한다"*, *"gripper 가 애초에 블럭이 아니라
이상한데를 집으려고 했고, 실패하는거 같아"* 라고 지적했다. **둘 다 같은 원인이었다.**

### 36.1 결과

| 시행 | 물체 (x, y) | 결과 |
|---|---|---|
| 1 | (0.612, −0.064) | SUCCESS 10 mm |
| 2 | (0.613, −0.128) | SUCCESS 11 mm |
| 3 | (0.596, −0.073) | SUCCESS 5 mm |
| 4 | (0.613, −0.125) | SUCCESS 18 mm |
| 5 | (0.638, −0.136) | SUCCESS 9 mm |
| 6 | (0.628, −0.128) | SUCCESS 24 mm |
| **7** | **(0.611, −0.050)** | **FAIL 265 mm** |
| **8** | **(0.653, −0.157)** | **FAIL 375 mm** |
| 9 | (0.616, −0.144) | SUCCESS 23 mm |
| **10** | **(0.630, −0.080)** | **FAIL 296 mm** |

**7/10.** 랜덤 범위는 y ∈ [−0.16, −0.04]. **실패 y = −0.050 / −0.157 / −0.080 — 양 끝단.**
성공은 −0.064 ~ −0.144 로 중앙부.

### 36.2 실패의 정체 — 하강이 아니라 정렬

실시간 계측(툴 TF vs 물체 GT):
```
tool=(0.590,-0.143,0.490)  red=(0.653,-0.157)  수평오차 66 mm  25 초간 정체
tool=(0.582,-0.155,0.440)  red=(0.630,-0.080)  수평오차 89 mm  20 초간 정체
```
- 명령은 계속 나간다(≈29 Hz). 정책이 죽은 게 아니라 **내려가지 않는 목표를 계속 낸다.**
- 그리퍼가 0.01~0.36 사이에서 진동한다 — 열지도 닫지도 못한다.

**툴이 물체 위가 아니라 6~9 cm 옆에 뜬다.** 그 상태는 시연에 없으므로 하강 조건이 성립하지 않는다.
겉보기 "하강 실패"의 실체는 **수평 정렬 실패**이고, 정책은 이미지에서 위치를 읽는 대신
**학습 분포의 평균으로 끌려간다** — 데이터 부족의 전형적 증상.

### 36.3 원인 — 대기 제거의 부작용

| | 프레임 | 롤아웃 |
|---|---|---|
| v3-thin | 29,728 | 8/8 |
| v5 | 21,711 | 7/10 |

§32 의 대기시간 제거는 정지 프레임(65.8%→8.4%)을 없앤 점에서 옳았지만, **프레임 절대량도 27% 줄였다.**
접근·정렬처럼 정밀도가 필요한 구간은 밀도가 아니라 **절대량**이 필요하다.
→ 100 에피소드로 증량(사용자 결정). 범위 축소는 문제를 숨기는 쪽이라 택하지 않았다.

### 36.4 하네스 버그 3건 (측정이 결과를 왜곡했다)

1. **성공 카운터 덮어쓰기** — `reset_pose` 재시도 루프에 `ok` 를 재사용해, 7 성공이 `1/10` 으로 출력됐다.
2. **물체 위치가 오래된 로그** — `"$S"/*isaac.log` 를 글롭해 **알파벳 순 마지막** 파일을 읽어 매 시행 같은 값이 찍혔다. → `ls -t | head -1`.
3. **시행 시간에 준비 시간 포함** — 45 s 예산 중 reset+READY 이동에 ~25 s 가 들어가 정책은 20 s 만 돌았다. 시행 1의 FAIL 은 이것 때문.

**그리고 나 자신이 두 번 오판했다**: 정책이 21 s 에 이미 완수하고 정지한 것을 "하강 실패"로,
직전 시행이 성공시켜 둔 장면을 "블록이 마커 위에 있다=이상"으로 읽었다.
→ **롤아웃은 끝 상태만 보지 말고 전 구간 궤적으로 판정할 것.**

## 37. 세트1 스트리밍 컨트롤러 + 내가 수집을 두 번 죽인 이야기 — 2026-09-09

사용자 질문: *"gripper 와 cam 없이 ur16e 만 omy l100 leader 와 연동해서 하면 바로 teleoperation을
해볼 수 있는건가?"*

### 37.1 답 — 실물은 추가 설정 없이 된다

| 항목 | 상태 |
|---|---|
| 그리퍼 없이 | `gripper_enable:=false` (이미 파라미터) |
| 카메라 없이 | 텔레오퍼레이션엔 불필요 |
| 관절 매핑 | IK 불필요, 관절 1:1 (§21/§22) |
| 안전장치 | 5중(기본 비활성 / 엔게이지 게이트 / 한계 클램프 / 속도 슬루 / 워치독) |
| **스트리밍 컨트롤러** | **실물은 원래 있음** |

`use_sim:=false` 는 UR 공식 `ur_control.launch.py` 로 위임되고 거기에 `forward_position_controller`
가 이미 들어 있다(mock 으로 확인: `inactive` + 토픽 존재). **처음에 "설정 추가 필요"라고 답했다가
정정했다** — 확인 전에 답했던 것.

### 37.2 sim 세트1에는 정말로 없었다 → 추가

`config/ur16e/ur16e_controllers.yaml` 에는 JTC 만 있었고 런치도 스폰하지 않았다. 세트 2/3 은
teleop 때 추가했지만 세트 1은 빠져 있었다. 그래서 **sim 에서 팔만으로 리허설**하려면 브리지가
아무도 듣지 않는 토픽에 발행하게 된다(에러 없이 팔만 안 움직임).
→ yaml 에 정의 + 런치에 `--inactive` 스포너 추가. sim/실물 양쪽 `inactive` 스폰 확인.

### 37.3 ★ 사고 1: 검증 없는 문자열 치환으로 raw 50 에피소드 삭제

100 개로 늘리려고 `APPEND=1` 을 수집 스크립트에 넣는 치환을 했는데 **패턴이 실제 줄과 달라 조용히
실패**했고, 확인하지 않고 실행했다. `collect_il_episodes.sh` 는 시작 시 `rm -rf "$OUT"` 을 하므로
기존 50 개가 지워졌다.

살아남은 것: 변환본(`lerobot_ds_red_left_v5` 96 MB)과 체크포인트 — 학습은 변환본을 쓰므로 결과는 보존.
→ **`str.replace` 뒤에는 반드시 `assert`.** 이 세션에서 조용한 치환 실패를 여러 번 냈고, 이번엔 데이터를 잃었다.
→ 그 김에 `APPEND=1` 은 제대로 넣었다(파라미터가 같을 때만 안전하다는 경고와 함께).

### 37.4 ★ 사고 2: 수집이 도는 중에 같은 머신에서 제어 스택을 띄웠다

세트1 컨트롤러를 검증하려고 `ROS_DOMAIN_ID=7` 로 mock 스택을 띄웠다. 도메인은 갈랐지만
**프로세스는 갈리지 않는다** — 테스트를 정리한 뒤에도 `ros2_control_node` / `robot_state_publisher`
가 두 세트 남았고, 수집 쪽 스택과 충돌해 **100 에피소드 수집이 0 개로 죽었다.**
CLAUDE.md 가 경고하는 "stale RSP" 함정 그대로다.

→ **수집/학습이 도는 동안에는 같은 머신에서 제어 스택을 띄우지 않는다.** 도메인 분리로는 부족하다.
검증이 필요하면 수집을 끝내고 하거나, 최소한 `ros2 control list_controllers` 로 노드가 하나뿐임을
확인한 뒤 진행한다.

## 38. 100 에피소드 → 9/10 — §36 의 진단이 맞았다 — 2026-09-09

§36 에서 v5(50 ep)가 7/10 이었고 실패가 랜덤 범위 **가장자리**에 몰렸다. 원인을 "대기 제거로
접근·정렬 구간의 **절대 프레임 수**가 줄었다"로 보고 100 에피소드로 늘렸다(사용자 결정).

### 38.1 결과

| 시행 | 물체 (x, y) | 결과 |
|---|---|---|
| 1 | (0.618, −0.088) | SUCCESS 8 mm |
| 2 | (0.647, −0.091) | SUCCESS 6 mm |
| 3 | (0.627, −0.144) | SUCCESS 5 mm |
| 4 | (0.593, −0.117) | SUCCESS **1 mm** |
| 5 | (0.640, −0.117) | FAIL 53 mm |
| 6 | (0.552, −0.086) | SUCCESS 5 mm |
| 7 | (0.556, −0.084) | SUCCESS 13 mm |
| 8 | (0.598, −0.094) | SUCCESS 20 mm |
| 9 | (0.544, −0.092) | SUCCESS 2 mm |
| 10 | (0.604, −0.102) | SUCCESS 8 mm |

**9/10.**

### 38.2 v5 와의 비교 — 실패의 *성격*이 달라졌다

| | v5 (50 ep, 21,711 프레임) | 100 ep (43,222 프레임) |
|---|---|---|
| 성공률 | 7/10 | **9/10** |
| 성공 시 평균 오차 | 13.4 mm | **7.6 mm** |
| 실패 거리 | **265 / 375 / 296 mm** | **53 mm** |
| 실패 양상 | 툴이 물체에서 6~9 cm 옆에 떠서 **정렬 실패**, 하강 못 함 | 정상 파지·이송, 마커에서 18 mm 초과 |
| 실패 위치 | y = −0.050 / −0.157 / −0.080 (**가장자리**) | y = −0.117 (**중앙부**) |

가장자리 실패가 사라졌다 — 그 구간 시연이 2배가 된 결과이고, §36 의 진단과 일치한다.
남은 1회는 "못 집음"이 아니라 "조금 빗나감"이라 문제의 종류가 다르다.

### 38.3 학습 조건 (비교가 성립하도록 스텝 고정)

| | v5 | 100 ep |
|---|---|---|
| 스텝 | 60k | 60k (고정) |
| epoch | 88.4 | 44.4 |
| 최종 loss | 0.027 | 0.029 |

**loss 는 100 ep 가 근소하게 높지만 성능은 더 좋다.** 데이터가 2배라 같은 스텝에서 epoch 이
절반이기 때문이며, §32.5 에 이어 **loss 로 정책을 비교하면 안 된다**는 사례가 하나 더 늘었다.

### 38.4 최종 파라미터 (전부 실측)

| 값 | 근거 |
|---|---|
| `grip_closed` **0.52** | 접촉 시작점 ~0.50 직후(§35). 힘 평형점 0.599 가 아님 |
| `grasp_z_offset` **0.015** | 더 깊으면 핑거가 테이블과 간섭, 사이클 3배 지연(§34) |
| `grip_preclose` 0.28 | 부착 임계 0.25 통과, 패드는 물체에서 여유 |
| 에피소드 | **100** | 50 은 가장자리에서 정렬 실패(§36) |
| 물체 랜덤화 | ±60 mm | 최대치(더 키우면 두 큐브가 겹침) |

## 39. 오늘의 마무리 + GR00T 착수 전 상태 — 2026-09-09

### 39.1 정리한 것

**`outputs/` 20 GB → 3.6 GB.** 그립 기하가 틀렸던 시절(§33/§35 이전)의 raw·체크포인트를 삭제했다.
그 데이터는 **그리퍼가 물체를 뚫고 닫히는 것**을 기록하고 있어 쓸 수 없고, 나중에 발견되면
오해를 부른다. 남긴 것:

| 자산 | 용도 |
|---|---|
| `act_red_left_100` (1.8 G) | 최종 ACT 정책, 롤아웃 9/10 |
| `il_raw_red_left_100` + `lerobot_ds_red_left_100` | 그 학습 데이터(100 ep) |
| `il_raw_wrist_only` + `lerobot_ds_wrist_only` | 카메라 1대 검증 → `act_wrist_only` **8/9** (§45.7) |
| `lerobot_ds_240_v2` | VLA 용 3태스크, 7 ep/태스크 (raw 없음) |
| `il_raw_3task_v3` + `lerobot_ds_3task_v3` | **VLA 용 3태스크 정본**, 30 ep/태스크, h264 → `groot_3task_v3_abs` **14/29** (§46) |

워크스페이스 루트에 흩어져 있던 `train_*.log` 는 `logs_archive/` 로.

### 39.2 새 문서 2개

- **`CHECKLIST.md`** — 새 PC 에서 실물까지. 설치 → HW 0개 검증 → 팔 → 리더 → 그리퍼 → 카메라.
- **`PIPELINE.md`** — IL 파이프라인 실행 명령 전부 + **단계별 합격 기준**.
  명령만 적지 않은 이유는 오늘 여러 번 겪었듯 **명령이 성공해도 결과가 쓸모없을 수 있어서**다
  (정지 프레임 과다, 그리퍼가 물체 관통, loss 는 좋은데 태스크 실패).

### 39.3 카메라 1대 — 배선은 전부 검증, 학습만 남음

| 단계 | 상태 |
|---|---|
| 수집 | ✅ `video_keys: ['video.wrist']` |
| 변환 | ✅ `observation.images.wrist` 하나만, **코드 수정 불필요** |
| 학습 | ⏸ 10K 스텝에서 중지(사용자 요청). 데이터는 준비됨 |
| 추론 CLI | ✅ `--robot.cameras_ros='{"wrist": ...}'` draccus 파싱 확인 |

**부수 효과**: 데이터셋 190 MB → 36 MB, 학습 14.7 → **25.6 step/s**, GPU 3.97 → **2.71 GB**.

**★ 실행 중인 bash 스크립트를 또 편집했다.** 파이프라인이 손상돼 3단계가 재실행되고 학습이
57K 에서 0 으로 되돌아갔다(약 35분 손실). 이 세션에서 두 번째다 — bash 는 바이트 오프셋으로
읽으므로 **실행 중 편집은 금지**. 고칠 게 있으면 멈추고 고친 뒤 다시 띄운다.

### 39.4 다음: GR00T N1.7 — 착수 전 확인된 것

**데이터는 준비돼 있다** (`outputs/lerobot_ds_240_v2`):
```
21 ep, 30,658 frames, 30 fps
observation.images.{exterior,wrist}  (240, 320, 3)
observation.state / action           float32 (7,)
tasks: red→left / blue→left / red→right   ← 언어 조건부의 전제(§2.8)
```

**환경 확인**:
| | 상태 |
|---|---|
| torch 2.11.0+cu128, CUDA | ✅ |
| lerobot 0.6.1 | ✅ |
| `lerobot.policies.groot` | ✅ 존재 |
| `accelerate` | ✅ |
| **`transformers`** | ❌ **없음** — GR00T 는 언어 인코더가 필요하다 |

→ **착수 첫 단계는 `transformers` 설치**이고, 이때 CLAUDE.md 의 격리 원칙이 걸린다:
`deps/.venv-ml` 안에서만, 기존 패키지 업그레이드 없이. `setup/requirements-ml.txt` 갱신 필요.

**카메라는 2대로 간다**(사용자 결정). 준비된 `lerobot_ds_240_v2` 가 이미 2대 구성이라
그대로 쓸 수 있고, 손목 1대 검증은 ACT 쪽 별건으로 남는다.

**미리 잡아둘 논점**:
- GR00T N1.7 은 이미지를 **256×256 으로 강제 리사이즈**한다(`N1_7_DEFAULT_IMAGE_TARGET_SIZE`,
  크롭 230×230). 우리 320×240 은 그래서 골랐던 것(§plan 2.6) — 재수집 불필요.
- ~~3B 모델 + LoRA 라 32 GB 단일 카드에서 배치가 관건.~~ **→ §40.5 에서 정정.** LoRA 가 아니라
  **동결 + 파라미터 dtype** 이고, 기본 fp32 는 **배치 크기와 무관하게** 32 GB 를 넘는다.
  ACT(80M, 2.7~4 GB)와 규모가 다르다는 것만 맞았다.
- **21 에피소드는 VLA 에 적다.** ACT 도 50 에서 7/10, 100 에서 9/10 이었다(§38).
  파이프라인 검증에는 충분하지만, 성능을 보려면 3태스크 각 50+ 재수집이 필요할 것.

---

## 40. GR00T N1.7 설계 + 게이트/VRAM 실측 — 카메라 2대 — 2026-09-10

§39 가 "GR00T 착수 전"에서 멈췄던 것을 이어, **설계를 먼저 확정하고 소스로 검증**했다.
정본 설계문서는 신규 [`ur_bringup/docs/plan_groot_n17.md`](ur_bringup/docs/plan_groot_n17.md).

### 40.1 재사용 경계 — 모델·데이터 경로에 새로 짤 코드가 **0**

`lerobot 0.6.1` 의 `policies/groot/`(4,912줄)가 정책·프로세서·액션헤드를 전부 제공하고,
데이터셋 스키마·수집/변환 스크립트·`async_inference` 서버/클라이언트·`UR16eROS` 어댑터가
**전부 그대로** 쓰인다. ACT → GR00T 는 전부 **설정 변경**이다.
운(運)이 아니라 §2.1(LeRobot 포맷)·§2.6(절대 관절 액션)을 첫날에 못 박아 둔 대가.

### 40.2 카메라 2대 — rename 없이 들어간다 (실측)

소스를 읽고 추론한 게 아니라 upstream 함수를 직접 호출해 확인했다:

```
[V1] dataset-meta visual keys       -> ['exterior', 'wrist']
[V1] checkpoint video_modality_keys -> None            (None => 데이터셋 폴백)
[V1] _ordered_image_keys(obs)       -> ['observation.images.exterior', 'observation.images.wrist']
[V1] checkpoint use_relative_action -> True
[V1] image target/crop              -> [256,256] / [230,230]
```

`embodiment_tag="new_embodiment"`(=`N1_7_EMBODIMENT_MAPPING` id **10**)면 체크포인트에
video 키가 없어 **데이터셋 메타로 폴백**하는 게 정규 경로다. 방증: 사전학습 embodiment 중
`oxe_droid_relative_eef_relative_joint` 가 **exterior+wrist 2뷰 단일팔** — 3B 가 우리와 같은
카메라 배치를 이미 대량으로 봤다.

### 40.3 ★ 함정 — `base_model_path` 에 repo id 를 주면 사이드카가 **조용히** 무시된다

`_load_n1_7_checkpoint_processor_assets()` 는 `is_raw_groot_n1_7_checkpoint()` → `Path().is_dir()`
로 판정한다. `nvidia/GR00T-N1.7-3B` 같은 **repo id 는 `None` 을 반환**한다 — 예외도 경고도 없이.

| 설정 | 체크포인트 실제값 | repo id 를 줬을 때 |
|---|---|---|
| `use_albumentations` | True | **False** |
| `state_dropout_prob` | 0.2 | **0.0** |
| `use_percentiles` | True | **False** |
| `shortest_image_edge`/`crop_fraction` | 256 / 0.95 | **None** |

가중치는 repo id 로도 정상 로드되므로 **학습은 그냥 돌아간다.** 그래서 더 위험하다 — 사전학습과
다른 전처리로 파인튜닝하는 걸 알 방법이 없다. §35 와 같은 부류.
→ **`snapshot_download()` 로 로컬 경로를 해석해 넘긴다.** 캐시가 있으면 받지 않고 경로만 준다.

### 40.4 ★ 막힘 — `nvidia/Cosmos-Reason2-2B` 는 **gated repo**

`GR00T-N1.7-3B` 자체는 gated 아님(6.5 GB, 27파일, 정상 다운로드). 그런데 학습 시작 시 401:

| 필요한 것 | 어디서 | 게이트 |
|---|---|---|
| 백본 **아키텍처 설정** | `groot_n1_7.py:_cosmos_reason2_qwen3_vl_config()` **하드코딩** | 불필요 |
| 백본 **가중치** | GR00T 체크포인트 안 `backbone.model.*` **494 텐서** | 불필요 |
| **토크나이저 + 이미지/비디오 프로세서** | `_build_n1_7_processor()` → Cosmos repo | **게이트** |

**2 GB 백본이 아니라 토크나이저 몇 MB 때문에 막힌다.** 다른 Qwen3-VL 토크나이저로 대체하는
우회는 vocab 이 어긋나면 **조용히 틀린 학습**이 되므로 하지 않았다.
→ 사용자가 라이선스 동의 + `HF_TOKEN` 발급해야 한다. `groot_smoke.sh` 가 기동 즉시 이 안내를 찍고 종료.

### 40.5 ★ VRAM — 기본 설정은 32 GB 에 **안 들어간다**. 그리고 **LoRA 가 아니다**

`GrootPolicy` 를 실제로 만들어 세어 본 값:

```
_groot_model.action_head   1621M   trainable 1621M    ← 전부 학습
_groot_model.backbone      1524M   trainable    0M    ← 전부 동결
TOTAL                      3144M   trainable 1621M  (51.5%)
```

동결 자체는 의도대로 걸린다(`tune_llm/tune_visual=False` 가 실제로 먹음). 문제는 **동결이 되는데도
학습 대상이 1.62 B** — GR00T 의 flow-matching action head 가 백본보다 크다.

`model_params_fp32=True`(기본, NVIDIA 정식 레시피) 정적 소요:
`params 11.7 + grads 6.0 + AdamW 12.1 = 29.8 GiB / 31.8 GiB` → **활성값 자리가 없다.**
**배치 크기와 무관하다** — `--batch_size=1` 이어도 안 된다.

`--policy.model_params_fp32=false` → action head 가 bf16(백본은 fp32 유지) → **≈17.8 GiB**, 여유 14 GiB.
upstream 이 제공하는 손잡이라 우리 우회가 아니다. 대신 **fp32 마스터 가중치를 포기**하므로
발산 시 첫 의심 항목.

> **§39 와 `plan_il_vla.md` §2.7/§6.2 의 "LoRA" 서술은 틀렸다.** `configuration_groot.py` 의
> `lora_*` 필드는 소스 주석이 **"never-wired"** 라고 명시한 deprecated 필드다. 메모리 관리는
> LoRA 가 아니라 **동결 + 파라미터 dtype** 으로 한다. 해당 문서들을 정정했다.

### 40.6 의존성 — `transformers` 오판 정정

파일 상단 import 만 보고 "transformers 불필요"라고 적었다가 스모크에서 즉시 틀렸다.
lerobot 의 정책 의존성은 import 문이 아니라 **`require_package()` 가드**에 있다
(`policies/groot/` 안에 6곳). → **다음엔 `grep require_package policies/<정책>/` 를 먼저 볼 것.**

정본은 개별 패키지 나열이 아니라 upstream extra: **`pip install 'lerobot[groot]'`**.
설치 전 dry-run 실측(2026-09-10): **19개 전부 신규, 기존 패키지 변경 0**,
`torch 2.11.0+cu128` / sm_120 유지. 버전: transformers 5.5.4 · diffusers 0.39.0 · peft 0.20.0 ·
timm 1.0.29 · decord 0.6.0 · dm-tree 0.1.10.

### 40.7 남은 것

| 항목 | 상태 |
|---|---|
| 카메라 2대 | ✅ 확정 |
| 상대 액션 경로 빌드 | ✅ 확정 (`relative_exclude_joints=["gripper"]`) |
| VRAM | ⚠️ `model_params_fp32=false` 필요 |
| step/s → 학습시간 (6h 게이트) | ❌ **HF 게이트에 막힘** |
| 추론 지연 (<1.33 s/청크) | 미측정 |
| 21 에피소드로 충분한가 | 미측정 — ACT 는 100 이 필요했다(§38) |

---

## 41. 텔레옵 랑데부 자세 — ROBOTIS `home` 이 우리 `ready` 로 매핑된다 — 2026-09-10

> **★ 정정 (2026-09-16, §47)**: 이 절의 전제 — SRDF `home` [0,0,+90,−90,+90,0]° 이 "리더를 그냥 내려놓는 자세" —
> 는 틀렸다. ROBOTIS 브링업은 팔로워를 `initial_positions.yaml` 의 `ready` [0,−90,+152,−62,+90,0]° 로 보내고,
> 리더도 그 형태로 놓인다. 따라서 "UR16e 쪽 초기 자세를 새로 정의할 필요가 없다"는 결론도 틀렸다.
> `ready` 는 [0,−180,+152,−152,−90,0]° 로 재정의됐고(J4 오프셋 −90° 포함), 이 절의 자세는 `ready_v1` 이다.

"UR16e 가 다른 일을 하다가 텔레옵을 시작하면 리더와 시작 자세가 크게 다를 텐데 어떻게 되는가"
라는 사용자 질문에서 출발. 현재 동작은 `/omy_bridge/enable` 이 **거부**하고 어느 관절이 몇 도
틀렸는지 알려주는 것뿐이며, 조작자가 리더를 손으로 맞추는 경로 하나만 있다.

### 41.1 결론 — 임의 자세를 쫓지 말고 **랑데부 자세**에서 만난다

`open_manipulator` 를 열어 보니 OMY SRDF 에 두 자세가 있다(`omy_f3m.srdf`):

```xml
<group_state name="init">  J1..J6 = 0, 0, 0, 0, 0, 0
<group_state name="home">  J1..J6 = 0, 0, +90°, −90°, +90°, 0     ← 손 떼도 서 있는 자세
```

이 `home` 을 우리 매핑(`sign=[1,1,1,1,−1,1]`, `offset=[0,−90°,0,0,0,0]`)에 넣으면:

```
leader home [0, 0, +90°, −90°, +90°, 0]  →  UR16e [0, −90°, +90°, −90°, −90°, 0]  =  reset_pose.py 의 ready
```

**우연이 아니다.** 양쪽 다 "팔꿈치 90° 접고 손목 정리"라는 같은 이유로 고른 자세다 —
우리 `ready` 는 특이점 회피(elbow≠0, wrist_2≠0), ROBOTIS `home` 은 리더를 내려놓을 수 있게.
→ **UR16e 쪽에 초기 자세를 새로 정의할 필요가 없다. 기존 `ready` 가 곧 랑데부다.**

반대로 L100 의 `init`(전부 0)을 쓰면 UR16e 는 `[0,−90°,0,0,0,0]` = 우리 `home` = **팔꿈치 특이점**
이고 팔이 수직으로 쭉 펴진다. **랑데부는 UR16e 쪽 제약(특이점·테이블·픽스처)을 기준으로 정해야 한다.**

### 41.2 리더는 자동으로 초기 자세로 **가지 않는다**

`omy_l100_leader_ai.launch.py` 는 `gravity_compensation_controller` + `spring_actuator_controller`
만 스폰한다. `init_position` 인자도 `arm_controller` 도 없다 —
`initial_positions.yaml` + `joint_trajectory_executor` 는 **팔로워 런치에만** 붙는다
(`config/omy_l100_follower_ai/initial_positions.yaml` 은 존재하나 리더 경로가 아니다).

즉 "L100 은 쉽게 초기 자세로 간다"는 **자동 구동이 아니라 "중력보상 덕에 사람이 쉽게 든다"**
는 의미로만 참이다. 자동화하려면 사람이 잡고 있는 팔의 컨트롤러를 position 으로 바꿔야 하므로
그 자체가 안전 사건 — 실물 확인 후 판단한다.

### 41.3 ★ L100 J2 가동범위 — 사양서와 URDF가 어긋난다

| 출처 | J2 |
|---|---|
| 공식 사양서 (§21) | **−70° ~ +100°** |
| `omy_l100_arm.urdf.xacro` | **±180°** (7관절 전부 동일값) |

URDF 는 전 관절에 ±180 을 일괄로 넣어 둔 것이라 **기구 스톱을 반영하지 않는다.**
랑데부는 J2=0° 라 어느 쪽이든 안전하지만, **브리지가 리더 쪽 한계를 신뢰할 수 없다**는 뜻이다.
(현재 브리지는 UR16e 한계 기준으로만 clamp → 안전 방향.)
→ §41 이전에 적었던 "리더 J2 때문에 UR shoulder_lift 가 −160°~+10° 로 제한된다"는
**사양서 근거이고 URDF 는 동의하지 않는다. 실측 전까지 확정된 제약이 아니다.**

### 41.4 아직 없는 것

| | 현재 | 있으면 |
|---|---|---|
| 오차 확인 | `enable` 호출해야만 보임 → 반복 호출 | 실시간 관절별 오차 발행 |
| UR16e 이동 | `reset_pose.py` = **직선 관절 보간**(충돌검사 없음) | `/omy_bridge/sync` = MoveIt 계획 |
| 리더 자동 이동 | 없음(중력보상 전용) | 실물 붙은 뒤 판단 |

두 번째가 특히 중요하다 — 직전 작업 때문에 팔이 픽스처 근처면 직선 보간은 위험하다.
**지금은 "팔 주변이 비어 있는지 눈으로 확인하고 `reset_pose.py ready`" 가 전제다**(CHECKLIST E-3).

### 41.5 실물에서 잴 것 → `CHECKLIST.md` E-1/E-2 로 반영

① 발행률·관절수·잡음 ② **rest pose 실제값** ③ **관절별 기구 가동범위(특히 J2)**
④ 트리거 실사용 구간 ⑤ **중력보상 드리프트** / ⑥ 오프셋 6개(`--mode match`) ⑦ 잔차(`verify`).

---

## 42. `/omy_bridge/sync` + 패드 바인딩 — mock 검증 완료 — 2026-09-10

§41 이 "아직 없는 것"으로 남긴 세 개 중 둘을 구현하고 mock 스택에서 검증했다.
전부 `scripts/omy_to_ur16e.py` **안**에 넣었다 — 브리지가 이미 `enabled` 상태와 매핑을 갖고
있으므로, 새 노드를 만들면 그 상태를 둘로 쪼개게 된다.

### 42.1 무엇을 만들었나

| 인터페이스 | 타입 | 하는 일 |
|---|---|---|
| `/omy_bridge/sync` | `Trigger` | MoveIt 으로 UR16e 를 `rendezvous`(기본 `ready`)로. 컨트롤러 전환 포함 |
| `/omy_bridge/engage_error` | `Float64MultiArray` (6, 5 Hz) | 매핑된 리더 − 팔로워, 관절별 [rad] |
| `/joy` 구독 | — | Options=enable · R3=sync · Create=disable |

### 42.2 설계 결정 4개와 그 이유

**① sync 는 MoveIt 으로 계획한다.** `reset_pose.py` 는 `/scaled_joint_trajectory_controller` 에
직접 쏘는 **직선 관절 보간**이라 충돌 검사가 없다. 텔레옵 직전의 팔은 **다른 작업을 하던
자리**에 있으므로(그게 §41 문제의 출발점이다) 픽스처 옆일 수 있다. `move_group` 이 없으면
서비스가 **거부하면서 수동 절차를 알려준다** — 조용히 위험한 경로로 폴백하지 않는다.

**② 즉시 반환한다.** 서비스 콜백에서 10 초짜리 팔 동작을 기다리면 단일 스레드 executor 의
100 Hz 제어 타이머가 그동안 멈춘다. 그래서 `sync_state`(`to_traj→moving→to_stream→done`)
상태기계 + `add_done_callback` 체인으로 만들고, 진행상황은 `/omy_bridge/status` 로 낸다.
패드 버튼에서 부르기에도 이 편이 맞다.

**③ 컨트롤러 전환을 서비스가 처리하고, 끝나면 streaming 으로 되돌린다.**
안 그러면 enable 이 성공해도 명령이 비활성 컨트롤러로 가서 **팔이 안 움직인다** — 증상이
"브리지 고장"과 구별되지 않는다. 같은 이유로 **`enable` 에 가드를 추가**했다:
streaming 컨트롤러가 비활성이면 거부하고 무엇을 하라고 알려준다.

> ★ 함정: STRICT 스위치는 **이미 active 인 것을 activate 하거나 inactive 인 것을 deactivate 하면
> 거부**된다. sync 직전에 어느 쪽이 active 인지는 조작자가 뭘 하다 왔느냐에 달렸으므로
> 하드코딩할 수 없다. → 1 Hz 로 `list_controllers` 를 폴링해 캐시하고, 스위치 성공 직후
> 캐시를 즉시 갱신한다(폴링 주기 안에 두 번째 스위치가 오므로).
> `switch_control_mode.py` 는 CLI 진입점으로 그대로 두고 둘을 나란히 유지한다.

**④ 움직임을 시작하는 버튼만 데드맨을 요구한다.** enable·sync 는 L1 을 눌러야 듣고,
disable 은 언제나 듣는다. 떠도는 `/joy` 하나가 16 kg 팔을 움직이면 안 되고, 반대로 멈추는
동작에 조건을 다는 것도 안 된다. 기본 인덱스는 `teleop_joy.py` 가 쓰는 0~5 를 피해 잡았다.

### 42.3 검증 (mock 스택, Isaac 없이 — `scratchpad/sync_test.sh`)

물리가 필요 없는 검증이라 mock 을 썼다. Isaac 은 기동 1분 + `/dev/shm` 14 MiB 를 쓰고
얻는 게 없다.

| # | 항목 | 결과 |
|---|---|---|
| 1 | sync: `home` → `ready`, 종료 시 STREAMING | ✅ 잔차 최대 0.5° (engage_tol 8.6° 의 1/17) |
| 2 | engage 중 sync 요청 | ✅ `disable the bridge first (arm is engaged)` |
| 3 | streaming 비활성 상태의 enable | ✅ `'forward_position_controller' is not active -- commands would go nowhere` |
| 4 | `/omy_bridge/engage_error` | ✅ `Float64MultiArray` 6개 발행 |
| 5 | 패드: 데드맨 없이 enable | ✅ 거부 + 이유 로그 / 데드맨과 함께 → `engaged` |
| 6 | 패드 disable(데드맨 없이) | ✅ `disabled` |

> 검증 순서에 함정이 있었다. `virtual_omy_leader` 는 **기동 시 1회** 팔로워에서 영점을 latch
> 한다. 그래서 "리더가 팔로워와 일치" 를 통제된 조건으로 만들려면 **브리지를 띄우기 전에
> 팔을 `ready` 로 보내고** `leader_amplitude:=0` 으로 띄워야 한다. 처음엔 순서를 반대로 해서
> enable 이 거부됐는데, 그건 코드가 아니라 하네스가 틀린 것이었다.

### 42.3-B 같은 6개를 Isaac 에서 재검증 (`scratchpad/sync_sim_test.sh`)

mock 은 명령을 그대로 상태로 되돌리므로 **MoveIt 궤적이 topic_based→Isaac 물리를 거쳐 실제로
실행되는지, 컨트롤러 전환이 `/clock` 기반 시간에서도 되는지, 팔이 랑데부에 정말 도착하는지**
를 보여주지 못한다. 그래서 세트1 · headless 로 그대로 반복했다.

| # | 결과 |
|---|---|
| 1 | ✅ `home` → 랑데부, 잔차 최대 **0.5°**, 종료 시 STREAMING |
| 2 | ✅ `disable the bridge first (arm is engaged)` |
| 3 | ✅ 거부 **사유까지** 정확 (`'forward_position_controller' is not active`) |
| 4 | ✅ 6개 발행 |
| 5 | ✅ 데드맨 없이 거부 / 함께 `engaged` |
| 6 | ✅ `disabled` |

sim 전용으로 추가 확인된 것: Isaac `/clock` 정상, **`/joint_states` 가 NaN 아님**(§1 topic_based
0.2.1 함정 회피), 기동 순서(Isaac→제어→move_group) 준수.

### 42.3-C ★ 추종 경로 회귀 — 하네스 오류를 코드 실패로 오독할 뻔했다

`_tick` 에 sync 분기를 넣었으니 **리더가 실제로 팔을 끄는 경로**가 멀쩡한지 봐야 한다.
처음엔 `ros2 param set /virtual_omy_leader amplitude 0.15` 로 가짜 리더를 흔들려 했고,
팔이 안 움직여서 실패로 보였다. 원인은 브리지가 아니라 **`virtual_omy_leader.py` L80 이
`amplitude` 를 생성자에서 1회만 읽는 것** — `param set` 은 무효였고 리더는 latch 자세를
그대로 유지했다. 즉 **팔이 안 움직인 게 정상 동작**이었다.

런치 인자로 바꿔 다시 측정(`scratchpad/track_sim_test.sh`,
`leader_amplitude:=0.05`=2.9° — `engage_tol` 8.6° 미만이라 위상 때문에 게이트가 거부하지 않는다):

```
관절별 자세 (12 s, joints [0,2] 만 구동):  shoulder_pan ±2.9°,  elbow ±2.9°  ← 진폭과 일치
worst |매핑된 리더 − 팔로워| [deg]:  +0.14  +0.01  +0.14  +0.00  +0.00  +0.00
```

**추종오차 0.14°** — §22 의 기존 실측 0.24° 보다 낫다(그때는 진폭이 더 컸다). 추종 경로 무영향 확인.

> §35.4 의 반복이다. "측정했다"가 "맞는 것을 측정했다"는 아니다 — 이번엔 **측정 도구가
> 조용히 아무것도 안 하고 있었다.** 런타임에 바꿀 수 있다고 가정한 파라미터가 실제로는
> 생성자에서만 읽히는 경우, `param set` 은 성공을 반환하고 아무 일도 일어나지 않는다.

### 42.4 부수 정정

`status` 는 engage 하는 순간 `sync_state` 를 `idle` 로 되돌린다. 안 그러면 텔레옵을 한참 하고
disable 한 뒤에도 `synced` 라고 찍혀서, 방금 랑데부에 있는 것처럼 읽힌다.

### 42.5 문서 갱신 중 발견한 실제 결함 — `ros2 param set` 안내가 틀렸다

§42.3-C 의 함정이 **브리지 자신에게도 그대로 있었고, 문서와 도구가 그 방법을 권하고 있었다.**
`omy_to_ur16e.py` 도 `g = lambda n: self.get_parameter(n).value` 를 `__init__` 에서만 부른다.

- `scripts/omy_leader_calib.py --mode match` 가 "적용 (임시)" 로 **`ros2 param set /omy_to_ur16e
  offset "[...]"` 을 출력**하고 있었다 → 성공을 반환하고 아무 일도 안 한다.
- `HARDWARE.md` §4-B④ 의 캘리브레이션 절차도 같은 명령을 쓰고 있었다.

실물 캘리브레이션은 **측정 → 적용 → 조작감 확인 → 재조정** 반복이라, 이대로 갔으면
"오프셋을 넣었는데 조작감이 그대로" 가 되고 **오프셋 값이 틀렸다고 오진**했을 것이다.
sim 에서는 드러날 수 없는 종류의 결함이다 — sim 에서는 애초에 캘리브를 안 하니까.

→ 도구 출력과 `HARDWARE.md` 를 **런치 인자 재기동**으로 고치고, 그 명령이 실재하도록
`teleop_omy.launch.py` 에 **`offset` / `sign` 런치 인자를 추가**했다(`rendezvous` 와 같은 방식).

검증(`scratchpad/argcheck.sh`) — `--show-args` 는 인자 존재만 보여주고 `ParameterValue` 타입
오류는 **런타임에만** 드러나므로 실제로 띄워서 확인했다:

```
expect offset_deg = [0.0, -90.0, 0.0, 5.7, 0.0, 11.5]
got    offset_deg=[0.0, -90.0, 0.0, 5.7, 0.0, 11.5]
       rendezvous: put the LEADER at [0.0, 0.0, 90.0, -95.7, 90.0, -11.5] deg
```

리더 자세가 새 오프셋만큼 함께 움직인 것(J4 −90→−95.7, J6 0→−11.5)이 부수 확인이다 —
기동 로그의 랑데부 안내가 하드코딩이 아니라 **매핑에서 역산**된다는 뜻.

→ §42.6 에서 구현했다.

### 42.6 `offset`/`sign` 런타임 변경 (DISABLED 일 때만) — 2026-09-10

§42.5 가 남긴 개선안. 실물 캘리브레이션은 **측정 → 적용 → 조작감 → 재조정** 반복인데,
매번 노드를 재기동하면 engage 게이트도 원점으로 돌아간다.

**허용 범위를 좁게 잡았다.**

| | |
|---|---|
| 런타임 변경 가능 | `offset`, `sign` — **DISABLED 일 때만** |
| ENGAGED 중 | **거부.** 매핑이 바뀌면 목표가 오프셋 델타만큼 점프하고 슬루가 쫓아가 팔이 움직인다 |
| sync 진행 중 | 거부 |
| **그 외 모든 파라미터** | **거부 + 재기동 명령 안내** — 조용히 무시하지 않는다 |

마지막 줄이 이번 작업의 요점이다. 원래는 **전부 조용히 성공**했고(§42.5), 그게
"값을 넣었는데 조작감이 그대로 → 오프셋이 틀렸나?" 로 나타난다. 이제는:

```
Setting parameter failed: 'max_joint_speed' is read once at construction, so setting it
here would silently do nothing. Relaunch instead: ros2 launch ur_bringup
teleop_omy.launch.py max_joint_speed:=<value>
```

**즉시 반영**: 파라미터가 바뀌면 마지막 리더 값(`leader_raw`, 신규)을 **다시 매핑**한다.
안 하면 `/omy_bridge/engage_error` 가 다음 리더 메시지까지 옛 값을 보여주고, 조작자가
그걸 보고 한 번 더 조정하게 된다. 콜백 등록은 **모든 `declare_parameter` 뒤** — rclpy 는
선언 시점에도 콜백을 부르므로, 앞에 등록하면 우리 자신의 기동값이 거부된다.

#### 검증 (mock, `scratchpad/param_test.sh` + `param_ef.py`)

| | 항목 | 결과 |
|---|---|---|
| A | DISABLED 중 offset 적용 | ✅ `engage_error` J4/J6 가 정확히 +0.09 / −0.20 이동 (즉시 재매핑 확인) |
| B | 읽기전용 파라미터 | ✅ 거부 + 재기동 명령 |
| C | 길이 5 | ✅ `needs 6 entries ... got 5` |
| D | `sign` 에 0 | ✅ `must be non-zero -- the inverse map divides by them` |
| E | ENGAGED 중 offset | ✅ 거부, 값 불변, disable 후 다시 수락 |
| F | 원자적 배치(good+bad) | ✅ 전부 거부, 정상 offset 도 미적용 |

#### ★ 내 주석이 과장이었다 — 원자성

처음엔 "`param set offset ... sign ...` 이 절반만 적용되면 안 된다" 고 썼는데, 셸 테스트에서
나쁜 `sign` 이 정상 `offset` 을 막지 못했다. 코드를 의심하기 전에 rclpy 를 봤더니 **ROS 2 의
정의된 동작**이었다 — `node.py` L764: *"called once for each parameter"*. `ros2 param set`/`load`
가 쓰는 비원자 서비스는 파라미터마다 콜백을 따로 부르고, 배치는 `set_parameters_atomically`
에서만 일어난다(F 는 그래서 rclpy 로 직접 호출해야 검증된다).

게다가 **`offset` 과 `sign` 사이엔 교차 불변식이 없다** — 각각 독립적으로 유효하고, 반쪽만
적용돼도 매핑은 유효하며 `engage_error` 에 즉시 드러난다. 즉 원래 걱정 자체가 과했다.
validate-then-apply 는 원자적 서비스에서 의미가 있고 비용이 없으니 유지하되, **주석을 실제
의미로 고쳤다.**

#### ★ 하네스 오류 3연발 — 이번 세션의 진짜 교훈

`sync` 검증부터 여기까지, **틀린 것이 코드가 아니라 테스트였던 경우가 세 번**이다:

1. §42.3 — 팔을 `ready` 로 보내기 전에 브리지를 띄워, 가짜 리더가 엉뚱한 영점을 latch → enable 거부
2. §42.3-C — `param set amplitude` 가 무효라 가짜 리더가 안 움직임 → "추종 실패" 로 오독
3. §42.6 — 직전 단계가 offset 을 `[9,9,…]` 로 만들어 둬서 engage 게이트가 거부 →
   "ENGAGED 중 거부" 케이스가 **아예 실행되지 않았는데 통과처럼 보임**

→ **테스트는 전제(precondition)를 스스로 PASS/FAIL 로 검사해야 한다.** `param_ef.py` 는
"브리지가 정말 ENGAGED 인가" 를 먼저 판정하고, 실패하면 그 뒤 결과를 아예 해석하지 않는다.

---

## 43. GR00T N1.7 — 게이트 해제 → 학습 완료 → 서비스 경로 함정 2개 (2026-09-13)

`plan_groot_n17.md` 의 검증계획 V1~V7 을 실행했다. **설계의 핵심 주장(모델·데이터 경로에 새 코드 0)은
실증됐고**, 막힌 곳은 전부 **서비스(추론 서버) 경로**였다. 학습은 한 번에 됐다.

### 43.1 HF 게이트 — 해제됨

사용자가 `nvidia/Cosmos-Reason2-2B` 라이선스 동의 + read 토큰 발급.
- 토큰 위치 **`deps/hf_cache/token`** (600). `HF_HOME` 을 워크스페이스로 두면 여기 저장되고,
  git repo(`src/`) **밖**이라 커밋될 경로가 아니며 공유 `~/.cache/huggingface` 를 오염시키지 않는다.
- **`gated=auto`** 였다 → 동의 즉시 자동 승인. NVIDIA 수동 검토 대기가 없다(`gated=manual` 과 구분).
- 검증: `whoami=iamtony-ca`, `config.json`/`tokenizer_config.json`/`preprocessor_config.json` 3개 다운로드 OK.
- Fine-grained 토큰이면 **"Read access to contents of all public gated repos"** 스코프 필수 —
  라이선스에 동의해도 이게 없으면 똑같이 401.

### 43.2 처리량 실측 — **배치가 전부, 워커는 무의미**

`lerobot_ds_240_v2`(21 ep / 30,658 프레임 / 카메라 2대 / 320×240), 절대 액션 기준:

| batch | step/s | **samples/s** | peak VRAM | peak shm |
|---|---|---|---|---|
| 8 | 2.08 | 16.6 | 25,401 MiB | 0 |
| 16 | 1.32 | 21.1 | 25,669 MiB | 0 |
| 24 | 0.926 | 22.2 | 26,141 MiB | 10 MiB |
| **32** | **0.714** | **22.9** | **27,318 MiB** | 14 MiB |
| 48 | 0.524 | 25.2 | 29,302 MiB | 21 MiB |

워커 스윕(batch 8): 2/4/6 = 2.08/1.97/1.99 step/s — **평평**. 8개는
`unable to allocate shared memory` 로 **step 24 에서 사망**(§24 ACT 와 동일 실패, 더 늦게 도달).

**★ 배치를 3배 올려도 VRAM 은 740 MiB 만 는다.** 25 GB 는 사실상 전부 모델+AdamW 상태이고
활성값은 부수적이다 → **작은 배치는 순손해**. 배치 48이 가장 빠르지만 29.3/32.6 GB 는
**공유 GPU 에서 위험**(다른 프로젝트가 상시 1.4 GB, 몇 시간 뒤 OOM 이면 런 전체 손실) → **32 채택**.

**★ 워커 평평함은 GR00T 의 성질이 아니라 이 머신의 부하다** — `nproc 20`, loadavg 16.
가용 코어가 4개 남짓이라 워커 2개가 이미 다 쓴다. 한가한 머신에서는 다르게 나온다(다른 PC 재현 시 재측정).

병목은 **AV1 소프트웨어 디코딩**이다(`data_s 0.85` vs `updt_s 0.52`, 스텝의 63%).
데이터셋 메타가 `video.codec: av1`, `video.g: 2`, `video.fast_decode: 0` — lerobot
`VideoEncoderConfig` 기본값(`vcodec="libsvtav1"`)을 그대로 받은 결과다.
**GOP 가설은 틀렸다**(이미 2라 탐색은 무료). torchcodec 도 본질이 아니다(코덱이 AV1 인 건 그대로).
→ **변환 시점에 `vcodec="h264"` 또는 `fast_decode` 로 해결 가능. 설치 0.**
단 `240_v2` 는 raw 가 없어 재변환 불가(`il_raw_red_left_100`·`il_raw_wrist_only` 만 남음).
`red_left_100` 으로 효과를 재고 `raw_to_lerobot.py` 에 인자를 추가할 것.

> **★ 정정 (§45.4, 2026-09-13)**: 위 "AV1 소프트웨어 디코딩 병목" 은 **틀렸다**. h264 로 재변환해도
> `data_s` 는 그대로였고, 분해 측정 결과 DataLoader 대기 0.000 s — `data_s` 는 전부 **메인 프로세스
> 전처리기** 시간이었다(lerobot 은 `preprocessor(batch)` 를 `data_s` 에 포함한다). 원인은 torch 기본
> 20 스레드의 과다할당 → `OMP_NUM_THREADS=8`. "워커 평평 = 머신 부하(loadavg 16)" 도 같은 오진이다
> (그 loadavg 는 우리 자신의 스레드였다). GOP·torchcodec 무관 판단만 맞았다.
(참고: torchcodec 이 못 뜨는 이유는 **FFmpeg 6 leaf 2개**(`libavdevice.so.60`, `libavfilter.so.9`)뿐이지만,
apt 전이 의존성이 20여 개(libass·libplacebo·SDL2·PulseAudio…)라 공유 컨테이너에 넣지 않기로 했다.)

### 43.3 학습 — 한 번에 됐다

```
batch 32 / workers 2 / steps 10000 / max_steps 10000 / fp32=false / 상대액션+gripper절대
loss 0.876 → 0.013   10.44 에폭   3시간 42분   rc=0   peak VRAM 27,327 MiB   peak shm 14.5 MiB
```
예측 3.7시간 = 실제 3.70시간. **lerobot 기본값 100,000 스텝이면 38.9시간**이었다.
10,000 은 NVIDIA Isaac-GR00T 레퍼런스 길이(`configuration_groot.py` 의 deprecated
`max_steps=10000`/`batch_size=32`)이고, 6시간 예산 안에 들어가는 유일한 자연스러운 값이다.

**§2.0 함정 회피가 끝까지 확인됐다** — 체크포인트에 기록된 전처리가 **체크포인트 고유값**이다:
`use_albumentations=True` / `state_dropout_prob=0.2` / `use_percentiles=True` /
`shortest_image_edge=256` / `crop_fraction=0.95`. repo id 를 줬다면 전부 조용히 바뀐다.
**카메라 2대도 실증**: `video_modality_keys = ['exterior','wrist']` — rename 불필요(§2.1 예측대로).

### 43.4 ★ 함정 — `max_steps` 는 deprecated 블록에 있는데 **실제로 쓰인다**

`configuration_groot.py` L367-368 의 `max_steps=10000` 은 *"all unused by the LeRobot N1.7
implementation"* 주석이 달린 블록에 있지만 L500 에서 쓰인다:
```python
num_warmup_steps = math.ceil(self.max_steps * self.warmup_ratio)   # --steps 가 아니다
```
실측: 4,000 스텝 런을 기본값(10000)으로 돌리면 최고 LR 이 **step 500 = 12.5% 지점**(의도는 5%).
`max_steps=4000` 을 주면 정확히 step 200 = 5.0%. 코사인 감쇠 자체는 정상(`num_training_steps`
는 외부 `--steps` 에서 온다). → **`--steps` ≠ 10000 이면 `--policy.max_steps` 를 같이 줘야 한다.** 경고 없음.

### 43.5 ★ 함정 — `relative_exclude_joints` 는 **부분 문자열** 매칭

`_infer_n1_7_action_groups` 를 직접 호출해 확인했다:
```
exclude=['gripper']  single_arm[0..5] relative=True + gripper[6] relative=False   ← 의도대로
exclude=['joint']    6축 전부 relative=False + gripper 만 relative=True           ← 완전히 뒤집힘
```
매칭이 `token == lowered or token in lowered` 다. 우리 관절 이름이 전부 `_joint` 로 끝나므로
이 워크스페이스에서 특히 위험하다. **정확히 `["gripper"]` 만 쓸 것.** 에러도 경고도 없다.

### 43.6 ★★ 함정 — `policy_device` 기본값이 `"cpu"` 이고, bf16 은 CPU 에서 죽는다

`AsyncClientConfig.policy_device` 기본값이 **`"cpu"`**. ACT 롤아웃들은 이 인자를 준 적이 없는데도
잘 돌았다(80M fp32). 그런데 `model_params_fp32=false` 체크포인트는 **bf16** 이고 CPU 에서:
```
Error in StreamActions: mixed dtype (CPU): expect parameter to have scalar type of Float
```
**최악의 실패 방식이다** — 서버가 예외를 삼키고 **관측은 계속 받는다.** 클라이언트는 크래시하지 않고
아무것도 못 받는다 → **"아무것도 안 하는 정책"으로 보인다.** 실측 0 청크 전달.
→ **`--policy_device=cuda` 필수.** 지연도 GPU 가 필요하다(V7 80 ms vs 예산 1,333 ms).

### 43.7 ★★ 함정(구조적) — **상대 액션은 `async_inference` 서버로 서비스할 수 없다**

`--policy_device=cuda` 로 고치니 추론은 나왔는데(0.12~0.58 s) 새 실패:
```
Error in StreamActions: GrootN17ActionDecodeStep cannot decode native relative actions
one step at a time. Decode the full action chunk returned by predict_action_chunk while
the matching GrootN17PackInputsStep state is still cached, ...
```
양쪽 코드를 다 읽어 확정했다:
- `policy_server._predict_action_chunk()` 는 `for i in range(chunk_size): postprocessor(action[:, i, :])`
  로 **스텝당 2-D** 를 넣는다.
- `GrootN17ActionDecodeStep` 은 `use_relative_action and ndim != 3` 이면 `NotImplementedError`.
  바로 다음 줄에 2-D 처리(`squeeze_horizon`)가 있는데 **상대일 때만 그 앞에서 거부**된다.

즉 **lerobot 0.6.1 의 async 서버는 GR00T 를 절대 액션으로만 서비스한다.** 지원 플래그는 없다.
→ `plan_groot_n17.md` §3 이 미리 등록해 둔 대비안("B 가 깨지면 A") 발동. **절대 액션으로 재학습.**

**내 스모크의 빈틈**: A/B 가 **학습**되는지만 확인하고 **서비스**되는지 확인하지 않았다.
50 스텝 절대 체크포인트로 서비스 경로를 먼저 검증했다(`probe_serve.sh`, 3분):
**38 청크 전달, 에러 0, 140~156 ms** → A 는 서비스된다. 3.7시간을 쓰기 전에 확인한 것.

### 43.8 하네스 — 전제 검사가 오기록을 처음으로 막았다

§42.6 의 교훈("테스트는 전제를 스스로 PASS/FAIL 로 검사해야 한다")이 실제로 값을 했다.
V8 1·2차 시도는 **정책 결과가 아니라 하네스 결함**이었고, 검사가 없었으면 9개 시행이 전부
"정책 실패"로 기록됐다. 넣은 검사 둘:
- **시행 전** `obs_ready.py` — 어댑터와 같은 순서·같은 2.0 s 타임아웃으로 관절+카메라 2대의
  도착률/신선도 검사. 실패 시 SKIP. (어댑터는 헤더가 아니라 **콜백 도착 시각**을 쓰므로
  sim time 문제와 혼동하면 안 된다 — 확인했다.)
- **시행 후** 전달된 청크 수. **★ 첫 버전은 틀렸다**: `Preprocessing and inference took` 를
  셌는데 이건 후처리 **이전** 로그다. §43.7 에서 추론은 성공하고 후처리가 터졌으니 카운트는
  0이 아니었고, **오기록을 막으려 만든 검사가 오기록을 통과시켰다.** →
  후처리 성공 후에만 찍히는 **`| Total time:`** 을 센다.
- `judge_task.py` 신규 — 기존 `judge_rollout.py` 는 red→left 하드코딩이라 3태스크를 채점할 수 없고
  ("파란 블록을 왼쪽"인데 빨간 게 왼쪽에 있으면 SUCCESS 오판), 실패를
  **WRONG_OBJECT / WRONG_PLACE / FAIL** 로 분리한다. 언어 실패와 파지 실패는 조치가 반대다.

### 43.9 남은 것

| 항목 | 상태 |
|---|---|
| V1 빌드·카메라 2대 / V2 VRAM / V3 처리량 / V4 shm / V5 상대빌드 | ✅ |
| 학습(상대) | ✅ 하지만 **서비스 불가**(§43.7) — 체크포인트는 `outputs/groot_240_v2_rel` 보존 |
| V7 추론 지연 | ✅ **PASS** 중앙값 80.8 ms / 최악 188 ms / 예산 1,333 ms |
| 절대 액션 서비스 경로 | ✅ 50스텝 체크포인트로 확인(38 청크) |
| **절대 액션 본학습** | ⏳ **다음 작업** — `groot_train_abs.sh`, 3.7시간 예상 |
| V8 롤아웃 3태스크 | ⏳ 본학습 후 |

**★ V8 은 성능 비교가 아니라 파이프라인 검증으로 읽어야 한다.** `240_v2` 는 태스크당 **7 ep**
(3태스크 균등, 45~52 s/ep)뿐이다. 색-위치 조합당 7 데모로는 **언어를 읽는지 vs 3개 궤적을
외웠는지 구분이 안 된다.** ACT 대비(9/10 · 7.6 mm)를 재려면 태스크당 20~30 ep 재수집이 필요하고,
그때 §43.2 의 h264 인코딩을 같이 적용하면 학습이 훨씬 싸진다.

---

## 44. GR00T N1.7 — 절대 액션 학습 + **첫 롤아웃 성공** (2026-09-13)

§43.7 의 대비안(상대 → 절대) 실행. 학습은 어제와 동일 곡선으로 한 번에 됐고, 롤아웃은 **하네스
결함 하나를 더 걷어낸 뒤** 3/8 — 이 워크스페이스에서 GR00T 가 처음으로 태스크를 완수했다.

### 44.1 학습 (절대 액션)

```
groot_train_abs.sh = groot_train.sh 에서 use_relative_actions/relative_exclude_joints 만 제거
batch 32 / workers 2 / 10k steps / max_steps 10000 / fp32=false / 로컬 스냅샷
loss 0.995 → 0.014   10.44 에폭   3시간 45분   rc=0   peak VRAM 27,485 MiB   shm 14.5 MiB
```
| step | abs(오늘) | rel(어제) |
|---|---|---|
| 1K / 3K / 5K / 10K | 0.053 / 0.027 / 0.017 / **0.014** | 0.044 / 0.024 / 0.017 / 0.013 |

두 곡선이 겹친다 → **상대→절대 전환은 학습 측면에서 대가가 없다.** CPU load 가 어제 16 → 오늘
0.75~6.5 인데 step/s 가 같다 → §43.2 의 "워커 평평함은 머신 부하 때문" 추정은 **약해지고**,
**AV1 디코딩 자체가 병목**이라는 쪽이 맞다(`data_s 0.90` > `updt_s 0.52`, 오늘도 동일).

**카메라 2대 소비 — 실측 확정.** abs 체크포인트는 `video_modality_keys: null` 로 기록됐다(rel 은
`['exterior','wrist']` 명시 — 상대 액션 자산이 데이터셋 메타를 박아 넣기 때문). §2.1 폴백이 실제로
작동하는지 전처리기에 넣어 세었다:
```
cams=[exterior,wrist]  image_grid_thw (2,3)  pixel_values (704,1536)  tokens 193
cams=[wrist]           image_grid_thw (1,3)  pixel_values (352,1536)  tokens 103
```
정확히 2배 → 둘 다 백본에 들어간다. **★ 함정**: 카메라 **1대만 줘도 에러 없이 돈다** — 모델 쪽엔
"카메라가 빠졌다"는 검사가 없다. 방어선은 `UR16eROS` 어댑터의 `no fresh image` 한 겹뿐.

### 44.2 ★★ 하네스 결함 5 — `forward_position_controller` 가 spawn 되지 않았다

오늘 1차 롤아웃: 청크 110개 전달, 판정 FAIL — 그런데 **팔이 READY 에서 0.0° 도 안 움직였다**
(40 s 샘플링: 관절·그리퍼·블록 전부 정지). `switch_control_mode.py streaming` 이 실패하고 있었다:
```
FAIL: controller 'forward_position_controller' is not loaded.
```
`bringup.sh` 는 이 컨트롤러를 spawn 하지 않는다 — 텔레옵 런치와 **`policy_inference.launch.py`** 만
한다. 어제 ACT 롤아웃 드라이버(`rollout_gui_then_n_w1.sh`)는 그 런치를 먼저 띄웠는데 `groot_v8.sh`
를 만들 때 빠뜨렸다. **그 런치의 docstring 이 이 실패를 문자 그대로 예언해 뒀다**:
*"the policy publishes into a topic nobody serves and the arm simply never moves, which looks
exactly like a broken policy."* 그리고 롤아웃 스크립트가 전환 출력을 `> /dev/null 2>&1` 로 삼켰다.
§43.8 의 두 검사(관측 신선도 / 전달된 청크 수)는 이 단계의 앞·뒤라 잡지 못했다.

→ `groot_v8.sh`: bringup 뒤 `policy_inference.launch.py` + 로드 검증.
→ `groot_rollout.sh`: 전환 출력 보존, 실패 시 **런 전체 중단**, 전환 후 상태를 awk 로 정확히
  `active` 인지 검증(`grep "active"` 는 `inactive` 에도 걸린다), 일시 타임아웃 대비 **재시도 3회**
  (실측: `list_controllers timed out` 1회 — 스택은 정상, `reset_pose` 직후 controller_manager 가 바쁨).
  되돌림(`trajectory`) 전환도 동일 처리(실패 시 `goal REJECTED by controller` → SKIP 1회 낭비).

**부수 함정 둘**: ① `ps | grep 패턴 | kill` 이 **두 번 내 셸을 죽였다** — 패턴이 같은 명령줄
(뒤쪽 `pgrep -f`, 히어독 본문)에 들어가 부모 래퍼가 매칭됐다. `bringup.sh` 주석이 경고한 그것.
정리 로직은 **파일**(`prep_streaming.sh`)로 뺀다. ② `TaskStop` 으로 `groot_v8.sh` 를 멈추면
같은 프로세스 그룹의 Isaac 이 같이 죽는다(어제는 Isaac 을 별도 호출로 띄워 살아남았음).

### 44.3 V8 결과 — **3/8 (1 SKIP)**, 절대 액션, 3태스크 교대

```
1  red→left    FAIL   d=274  grip 0.39   red 30 mm 이동, 파지 실패
2  blue→left   SKIP   (trajectory 전환 타임아웃 → READY 불가, 점수 제외)
3  red→right   ✅ 27 mm
4  red→left    FAIL   d=321  grip 0.00   red 미접촉
5  blue→left   FAIL   d=358  grip 0.49   ★ blue 를 잡아 250 mm 운반, 놓기 전 시간 초과
6  red→right   ✅ 18 mm
7  red→left    FAIL   d=315  grip 0.80   red 미접촉, 허공에서 닫힘
8  blue→left   ✅ 27 mm   ★ red(299 mm) 를 두고 blue 를 골라 왼쪽에
9  red→right   FAIL   d=533  grip 0.01   red 미접촉
```
| 태스크 | 성공 | 비고 |
|---|---|---|
| red→right | **2/3** | |
| blue→left | **1/2** | 실패 1건도 blue 를 정확히 집었다 |
| red→left | **0/3** | 셋 다 **파지 단계**에서 실패 — 목적지가 관여하기 전 |

**언어 독해 신호**: `WRONG_OBJECT` **0건**. blue 지시 2회 모두 blue 를 집었고, red 지시 6회 모두
blue 는 시작 위치(102~130 mm)에 그대로였다. 두 블록이 항상 같이 있는 씬에서 **"어떤 물체"는
8/8 맞췄다.** "어디"는 실패가 전부 파지 전이라 관측 불가(`WRONG_PLACE` 0건).
red→left 0/3 vs red→right 2/3 은 같은 물체·다른 목적지인데 파지에서 갈렸다 — n=3 이라 우연일 수
있고, 지시문에 따라 접근 궤적이 초반부터 달라지는 것일 수도 있다. 이 데이터로는 판정 불가.

**ACT 9/10 과 비교하면 안 된다**: ACT 는 red→left **100 ep**, GR00T 는 태스크당 **7 ep** 다.
같은 태스크 0/3 vs 9/10 은 데이터 14배 차이의 결과이지 모델 비교가 아니다.

### 44.4 검증 계획 최종 상태

| | 결과 |
|---|---|
| V1~V5 | ✅ (§43) |
| V7 추론 지연 | ✅ 80.8 ms / 1,333 ms |
| **V8 파이프라인** | ✅ **관측→GR00T(cuda)→절대 액션 후처리→스트리밍 컨트롤러→Isaac 루프 닫힘, 3/8 성공** |
| 성능 비교 | ⏳ 태스크당 20~30 ep 재수집 필요 (변환 기본이 h264 crf23 로 바뀜, §45.2) |

**설계 문서의 핵심 주장은 끝까지 유지됐다** — 모델·데이터 경로 신규 코드 **0**. 오늘까지 걷어낸
결함 다섯(§43.6·43.7·43.8·44.2) 은 전부 **서비스 경로의 설정·하네스** 였다.

## 45. 코덱 h264 + ★ GR00T `data_s` 의 진짜 원인은 스레드 과다할당 — 손목 ACT 파이프라인 (2026-09-13)

요청: *"2번은 안 해도 될거 같고, 4번 -> 1번 순으로 진행해줄래?"* — ② 소형 VLA 생략(`plan_il_vla.md` §2.7),
④ `raw_to_lerobot.py` h264 → ① 손목 1-카메라 ACT 전체 파이프라인.

### 45.1 ④ 변환기 — `--vcodec / --crf / --fast-decode` (신규 인코더 코드 0)

lerobot 0.6.1 의 `RGBEncoderConfig`(기본 `vcodec="libsvtav1"`, `g=2`, `crf=30`) 를 그대로
`LeRobotDataset.create(..., rgb_encoder=)` 에 넘긴다. 잘못된 코덱은 프레임을 쓰기 전에 업스트림이
`ValueError: Invalid vcodec` 로 거부(확인). **기본 `h264`, crf 는 h264 일 때 23**(아래 실측 근거).
코덱은 `meta/info.json` 에 기록되므로 기존 AV1 데이터셋(`240_v2`·`red_left_100`·`wrist_only`) 은 그대로 학습된다.

### 45.2 실측 — `il_raw_red_left_100` (100 ep, 43,222 프레임, 320×240 × 2 카메라)

| | AV1 crf30 (기존) | h264 crf30 | **h264 crf23 (채택)** |
|---|---|---|---|
| 비디오 용량 | 187 MB | 83 MB | 174 MB |
| PSNR vs 원본 JPEG (20 ep, ext/wrist) | 40.25 / 40.47 dB | 35.92 / 36.61 dB | 39.24 / 39.44 dB |
| 변환 시간 (100 ep) | – | 444 s | 448 s |
| 격리 디코딩, **1 스레드** (wrist) | 2.7 ms/프레임 | – | 3.1 ms/프레임 |
| GR00T `data_s` (batch 32 / 2 워커 / 60 스텝) | 0.80~0.86 s | 0.70~0.97 s | 0.61~0.85 s |

h264 crf30 은 4.3 dB 손실, crf23 은 AV1 과 같은 크기에 1 dB 이내 → **crf23**.
**그리고 h264 는 `data_s` 를 줄이지 못했다.** 여기서 §43.2 의 진단이 무너졌다.

### 45.3 ★★ 추적 — "26 ms/프레임" 은 측정 아티팩트였다

1. **lerobot 의 `data_s` 정의**: `lerobot_train.py` 는 `next(dl_iter)` + uint8→float + **`preprocessor(batch)`**
   를 합쳐 `dataloading_s` 로 잰다(`AverageMeter(..., reduction="max")` — 로그 구간 **최댓값**). GR00T 의
   전처리기(256×256 리사이즈·크롭·정규화·토큰화)는 **메인 프로세스**에서 돈다.
2. **격리 벤치 재검**: 같은 파일을 최소 pyav 루프로 열면 **2.2 ms**, lerobot `decode_video_frames_pyav` 는
   **44 ms**(ACT 학습과 동시 측정). cProfile: 시간의 대부분이 3×240×320 uint8 텐서의 `contiguous`(6 ms)·
   `torch.stack`(8 ms)·`.type()`(8 ms) — 마이크로초짜리 연산이다. torch 기본 스레드 = `nproc` **20** 이라
   작은 연산마다 OpenMP 병렬 구간이 열려 과다할당된다. **`torch.set_num_threads(1)` 로 2.7 ms.**
   DataLoader 워커는 원래 1 스레드로 돌기 때문에 **워커 디코딩은 처음부터 병목이 아니었다** —
   어제의 26 ms 는 메인 프로세스에서 20 스레드로 잰 값이고, 우연히 `data_s` 와 산술이 맞아 오진했다.
3. **분해 측정** (`scratchpad/bench_data_s.py`, 실제 `make_dataset`/`make_pre_post_processors`/DataLoader 재사용):

   | 메인 스레드 | loader 대기 | to_float | **preprocessor** | 합 (=data_s 평균) |
   |---|---|---|---|---|
   | 20 (기본), AV1 | **0.000 s** | 0.025 | **0.864** | 0.889 |
   | 1, AV1 | 0.001 | 0.014 | **0.316** | 0.331 |
   | 20, h264 | 0.001 | 0.020 | 0.983 | 1.004 |
   | 1, h264 | 0.000 | 0.014 | 0.313 | 0.327 |

   워커는 항상 앞서 있다(대기 0). `data_s` 는 **전부 전처리기**, 코덱 무관.

### 45.4 ★ 해결 — `OMP_NUM_THREADS=8` (설치 0, 코드 0)

`lerobot-train` 60 스텝, `red_left_100`(AV1), batch 32, 2 워커, `model_params_fp32=false`:

| `OMP_NUM_THREADS` | `data_s` | `updt_s` | 샘플/s | 10k 스텝 예상 |
|---|---|---|---|---|
| 20 (기본 = nproc) | 0.80~0.86 | 0.498 | 23~25 | 3 h 45 m (§44.1 실측) |
| 1 | 0.31 | 0.494 | 40 | 2 h 15 m |
| 4 | 0.15 | 0.497 | 49 | 1 h 50 m |
| **8** | **0.125** | 0.498 | **51** | **≈ 1 h 45 m** |
| 12 | 0.12 | 0.500 | 52 | 1 h 43 m |

**8 채택**(12 와 동률, 여유 코어를 남긴다). `updt_s` 는 불변(GPU). ACT 는 전처리가 가벼워 무관(`data_s 0.007`).
정정 목록: §43.2 "AV1 디코딩 병목" ✗, "워커 평평 = 머신 부하 loadavg 16" ✗(그 부하가 우리 스레드였다),
§43.2 의 배치 스윕 표는 전부 20 스레드 값. GOP 무관·torchcodec 무관 판단은 ✓.
`CLAUDE.md`·`PIPELINE.md` §3-B·`SETUP.md` §2-C-2·`plan_groot_n17.md` §8.6·`README.md` 를 정정했다.
**교훈**: 병목 진단은 **그 값을 만드는 코드가 무엇을 재는지** 부터 읽을 것. 산술이 맞는 것은 증거가 아니다.

### 45.5 ① 손목 1-카메라 ACT — 학습

`lerobot_ds_wrist_only`(49 ep / 20,996 프레임 / `observation.images.wrist` 만, AV1 그대로 — ACT 는
GPU 바운드라 재변환 무의미), `scratchpad/train_wrist1.sh`: 60k 스텝, batch 32, `num_workers=4`
(`UR_WS_TORCH_SHM_FIX=1`, Isaac 정지 상태), **40 m 11 s, 24.9 step/s**, loss 5.87 → **0.029**.
체크포인트 `outputs/act_wrist_only/checkpoints/{020000,040000,060000,last}`.

### 45.6 ① 롤아웃 하네스 이식

`rollout_wrist1.sh` 에 §44.2 에서 확정된 세 검사를 넣었다(없으면 "팔이 안 움직이는데 FAIL 채점" 재발):
스트리밍 전환 3회 재시도 + `list_controllers` 의 상태 열 awk `active` 확인(`grep active` 는 `inactive` 도
잡는다) / `obs_ready.py --cameras wrist=...` 사전검사 → SKIP / 서버 로그 `| Total time:` 의 **시행별 증분**
(서버는 전체 시행에 하나라 누적값을 빼야 한다) 0 이면 SKIP. `rollout_gui_then_n_w1.sh` 는 이미
`policy_inference.launch.py` 를 띄운다. 씬: 3태스크 씬(red·blue / left·right), `--randomize-radius 0.06`,
seed 151(수집 seed 111·21 과 다름), 320×240, 시행 10 × 70 s.

### 45.7 ① 롤아웃 결과 — **8/9 (1 SKIP)**, 손목 카메라 1대만으로 파이프라인 닫힘

| 시행 | 시작 위치 (x, y) | 결과 |
|---|---|---|
| 1 | 0.599, -0.117 | SUCCESS d=14 mm |
| 2 | 0.541, -0.051 | **FAIL** d=273 mm (파지 실패, 78 청크는 정상 전달) |
| 3 | 0.611, -0.109 | SUCCESS d=20 mm |
| 4 | – | **SKIP** — `/joint_states` 1.3 Hz (Isaac 순간 정체). 사전검사가 잡아 채점 제외 |
| 5~10 | 6회 | SUCCESS d=11/25/19/18/16 mm, 12 mm |

채점 시행 **8/9**, 성공 시 평균 오차 **17 mm**(11~25). 시행당 76~80 청크 전달(70 s × 30 Hz / 50 ≈ 42 관측 →
서버가 관측마다 응답, 전부 후처리 통과). 하네스 출력의 "8/10" 은 SKIP 을 분모에 넣은 값이라 8/9 가 맞다.

**의미**: 2-카메라 ACT(`act_red_left_100`, 100 ep) 9/10 과 같은 급이 **손목 카메라 1대 + 49 ep** 로 나왔다.
어댑터 `--robot.cameras_ros='{"wrist": ...}'` 1개 지정만으로 정적 카메라 없이 서비스된다(PIPELINE §4 의
"카메라 1대" 절차 실증). 단 무작위 반경은 학습 0.06 = 롤아웃 0.06 이고 seed 만 다르다. 실물 D455 정적
카메라(#10)가 늦어져도 **손목 D405 하나로 ACT 트랙은 진행 가능**하다는 근거.
④·① 완료로 `HISTORY.md` §44.4 "계획만 세우고 안 한 것" 중 남은 것: ③ 3태스크 재수집(20~30 ep/태스크, h264),
⑤ D455 정적 카메라 런치, ⑥ 리더 자동 랑데부(실물), ⑦ 실물 캘리브, ⑧ 그리퍼 에셋 재베이크.
롤아웃 프로세스는 `shutdown_rollout.sh` 로 전부 종료(GPU 641 MiB, ros2 노드 0).

### 45.8 재현 점검 — 셋업 스크립트에 GR00T 가 없었다 (2026-09-13)

요청: *"다른 pc에서도 동일하게 셋업을 해야되는데, 현재 셋업 가이드 및 스크립트가 제대로 업데이트 되어있는지
확인해줄래? … huggingface 에서 다운받아야되는건, 직접 내가 필요한걸 다운로드 받아서 필요한 디렉토리에 파일들을 옮겨놓을거야."*

**발견**: `setup.sh`·`check_env.sh`·`requirements-ml.txt` 에 GR00T 관련 항목이 **0** — `lerobot[groot]` 설치,
`deps/hf_cache`, 토크나이저 게이트 전부 `SETUP.md` 의 손 절차로만 존재했다. 환경변수(`HF_HOME`·`OMP_NUM_THREADS`
등)는 scratchpad 의 `groot_env.sh` 에만 있어 repo 에 없었다. 그리고 수동 배치 요구에 맞는 절차가 없었다:
lerobot 은 토크나이저를 **repo id 로** 연다(`processor_groot.py`: `AutoTokenizer.from_pretrained("nvidia/Cosmos-Reason2-2B")`)
→ 경로 지정이 불가능하고, 캐시 레이아웃 + `HF_HUB_OFFLINE=1` 이 유일한 오프라인 경로다.

**추가한 것** (`src/setup/`):
- `requirements-groot.txt` + `setup.sh groot` 단계(기본 제외): `pip --dry-run --report` 로 **기존 패키지 변경 시 거부**
  → 설치 → sm_120 재확인 → `deps/hf_cache` 생성 → `check_hf_cache.sh`. 이 PC 에서 실행: 변경 0, 8 초, 전부 ok.
- `ml_env.sh`: `HF_HOME`·`HF_HUB_OFFLINE=1`·`HF_HUB_ENABLE_HF_TRANSFER=0`·`WANDB_DISABLED`·`TOKENIZERS_PARALLELISM`·
  `OMP_NUM_THREADS=8`·`UR_WS_TORCH_SHM_FIX=1` 을 한곳에. PIPELINE/SETUP/CHECKLIST 의 명령이 이걸 source 한다.
- `check_hf_cache.sh`: 두 repo 의 `refs/main`·스냅샷·필수 파일 존재 + **오프라인** `snapshot_download`·토크나이저·
  이미지/비디오 전처리기 로드(vocab 151669). `check_env.sh` 에 "GR00T N1.7" 절로 편입(+ `nproc>12` 면 스레드 경고).
- `requirements-ml.lock`: `pip freeze`(299개) 참고용.

**수동 배치 검증**: 임시 `HF_HOME` 에 `refs/main` + `snapshots/<해시>/` 에 **필요 파일만**(GR00T 7개 = config·
processor_config·statistics·embodiment_id·index·safetensors×2, Cosmos 7개 = config·tokenizer·tokenizer_config·
vocab·merges·preprocessor_config·video_preprocessor_config), `blobs/` 없이, 심볼릭링크 없이 두고 `HF_HUB_OFFLINE=1`
로 `lerobot-train --policy.type=groot` 3 스텝 → **rc 0, 401/네트워크 접근 없음**. lerobot 이 GR00T 스냅샷에서 실제로
여는 파일은 grep 으로 확인(`experiment_cfg/` 등 나머지는 안 읽음). Cosmos 는 **가중치 불필요**(백본 설정은
`_cosmos_reason2_qwen3_vl_config()` 하드코딩, 가중치는 GR00T safetensors 안). 레이아웃은 `SETUP.md` §2-C-2.

**남은 재현 구멍(미해결, 사용자 결정 필요)**: 롤아웃 하네스(`groot_v8.sh`·`groot_rollout.sh`·`rollout_wrist1.sh`·
`bringup.sh`·`judge_task.py`·`judge_rollout.py`·`obs_ready.py`·`shutdown_rollout.sh`·`train_*.sh`·`collect*.sh`)가
**전부 세션 scratchpad 에만 있다**. `CLAUDE.md`·`PIPELINE.md` §4 가 `scratchpad/...` 를 참조한다. 다른 PC 에서는
존재하지 않으므로 repo(예: `ur_bringup/scripts/harness/`)로 옮겨야 한다.

### 45.9 하네스 스크립트 repo 편입 — `ur_bringup/scripts/harness/` (2026-09-13)

요청: *"하네스 스크립트들 repo로 옮기고 문서 경로 맞춰줘"*. 세션 scratchpad 의 약 170개 중 **현재 문서가 참조하고
검증된 파이프라인을 구성하는 38개**만 옮겼다(나머지는 일회성 진단 — §28~§44 의 증거로 HISTORY 에 이름만 남는다).
목록·인터프리터·로그 규칙은 `ur_bringup/scripts/harness/README.md`.

옮기며 바꾼 것(내용 변경 없음, 경로만): ① `WS=/isaac-sim/volume/ur_ws` 하드코딩 → 스크립트 위치에서 유추
(`UR_WS=` 덮어쓰기), heredoc 파이썬의 절대경로 5곳은 argv 로; ② 로그를 스크립트 옆에 쓰던 `"$S/*.log"` →
`outputs/harness_logs/`(`HARNESS_LOG=`); ③ `groot_env.sh` → `src/setup/ml_env.sh`; ④ `rollout_gui_then_n*.sh` 에
`[N] [초] [ckpt]` 인자. 함정: `set -u` 스크립트에서 `LOG=` 줄이 `WS=` 보다 먼저 들어가면 즉사 — 순서 검사로 잡음.
`CLAUDE.md`·`PIPELINE.md` §4-3·`plan_groot_n17.md` 의 `scratchpad/...` 참조를 새 경로로, `README.md`·`CHECKLIST.md` 에 위치 추가.
**검증(repo 위치에서 실행)**: `rollout_gui_then_n_w1.sh 2 60` → Isaac 기동·`bringup.sh`·`policy_inference.launch.py`·
사전검사·68 청크 전달·채점(1/2) 정상, `groot_v8.sh 1 60 <abs ckpt>` → 3태스크 3시행 전부 70~72 청크 전달·`judge_task.py`
채점(0/3 — 60 s 는 §44.3 의 90 s 보다 짧아 성능 판정 아님, 하네스 동작 확인용), `shutdown_rollout.sh` 로 GPU 640 MiB·노드 0.
로그 23개가 `outputs/harness_logs/` 에 생성됐고 소스 트리에는 아무것도 쓰이지 않았다.

## 46. GR00T N1.7 v3 — 태스크당 30 ep 재수집 → **14/29 (0.06) · 3/9 (0.025)**, 실패는 사전파지 정지 (2026-09-15)

요청: *"③ 재수집 20~30 ep/태스크(h264 + OMP_NUM_THREADS=8)로 GR00T 성능 재판정"*. §44.3 의 3/8 은 태스크당 7 ep 라
언어 독해와 궤적 암기를 구분할 수 없었다(§43.9). 이번엔 ACT 기준선(`red_left_100`: 100 ep, 반경 0.06)과 같은 씬에서
태스크당 30 ep, 전체를 `harness/groot_pipeline.sh` 한 번으로 무인 실행했다(총 **3 h 39 m**).

### 46.1 실행 — 수집 27 m · 변환 7 m · 학습 1 h 48 m · 롤아웃 1 h 13 m

| 단계 | 결과 |
|---|---|
| 수집 `collect240.sh` (N=30, RADIUS=0.06, SEED=161) | **90/90**, 실패 사이클 0, 27 m. `grasp_watch.py` 270 프레임 — 세 태스크의 lift 순간을 눈으로 확인(red·blue·red 모두 파지 중) |
| 변환 `convert240.sh` (h264 crf23 기본) | 90 ep / **38,384 프레임**(≈14 s/ep, §32 최적화 상태머신) / 태스크 3 / `lerobot_ds_3task_v3` **158 MB** |
| 학습 `groot_train_abs.sh` (절대, 10k, batch 32, `OMP_NUM_THREADS=8`) | **1 h 48 m** (1.54 step/s, `data_s` 0.129) — §45.4 예측 1.8 h 그대로. loss 0.033(2k)→**0.014**, 피크 VRAM **28.3 GB**, shm 16 MiB. 15 분 시점 ETA 1:35 로 6 h 게이트 통과 |
| 롤아웃 A `groot_v8.sh 10 90` (RADIUS=0.06, SEED=171) | **14/29**, SKIP 1(카메라 토픽 순간 0 Hz — 사전검사가 걸러냄, 직후 정상) |
| 롤아웃 B `groot_v8.sh 3 90` (RADIUS=0.025, SEED=0 = §44.3 씬) | **3/9** |

§43.2 의 "AV1 병목"이 오진이었으므로(§45.4) 이번 변환의 h264 는 속도가 아니라 크기·호환 목적이다 — 학습 시간이 준 것은
`OMP_NUM_THREADS=8` 때문이고, 그 예측(≈1.8 h)이 이번에 본학습으로 실증됐다.

### 46.2 결과 — 롤아웃 A (ACT 와 같은 씬, 태스크당 10회)

| 태스크 | 성공 | 실패 내역 |
|---|---|---|
| red→left | **8/10** | 2건 red 미접촉(288·304 mm) |
| blue→left | **5/10** | **3건 근소 미달**(38·43·61 mm — 허용 35 mm) + 2건 미접촉(143 mm = 시작 위치) |
| red→right | **1/9** | 7건 미접촉(488~566 mm) + 1건 운반 중 낙하(174 mm) |
| 합계 | **14/29 (48 %)** | `WRONG_OBJECT` **0**, `WRONG_PLACE` **0** |

성공 시 배치 오차 평균 **22.8 mm**(최대 34, ACT 7.6 mm). 롤아웃 B(§44.3 씬)는 red→left 0/3 · blue→left 1/3 ·
red→right 2/3 = **3/9** — 숫자도 태스크별 모양도 §44.3(0/3·1/2·2/3)과 같지만, A 에서 red→left 8/10 · red→right 1/9 로
**뒤집혀 있어** n=3 의 태스크별 순위는 우연이다. 두 롤아웃 합산 **17/38 (45 %)**: red→left 8/13, blue→left 6/13, red→right 3/12.

**§44.3 3/8 (38 %) → 17/38 (45 %)**: 데이터 4.3배(21→90 ep)로 성공률은 거의 안 움직였고, 언어 독해 실패는 여전히 0 이다.
**ACT 9/10 과는 여전히 비교가 안 된다** — 이번엔 데이터가 아니라 **실패 모드가 다르다**(46.3).

### 46.3 ★ 실패 모드 — 사전파지 높이에서 정지, 시작 자세와 무관

실패 15건 중 14건이 `grip < 0.1`(닫은 적 없음). 실패한 18번 시행(red→right)을 6 s 간격으로 촬영해 보니(손목·정적 카메라):
**18 s 에 red 블록 바로 위 사전파지 높이까지 정확히 내려가고**, 블록이 손목 카메라 중앙에 있는 상태로 **남은 70 s 동안 아무
것도 하지 않는다** — 최종 하강도, 닫힘도 없다. 성공한 12번은 24 s 안에 놓기까지 끝냈다. 청크는 매 시행 110~114개가
서버에서 후처리까지 성공해 전달됐으므로 서비스 경로 문제가 아니다(§43.6·§44.2 류 아님).

배제한 것 두 가지:
- **데이터가 가르친 정지가 아니다.** raw 90 ep 의 정지 프레임(관절 이동 ≤ 2 mrad/스텝) 비율 24 %, **에피소드당 최장 정지 0.8 s**
  (24~28 프레임). 70 s 정지는 데모 어디에도 없다.
- **시작 자세 탓이 아니다.** Isaac `reset_episode` 로그에서 시행별 목표 블록 (x, y, yaw)을 시행 순서(궤적 전환 직후 타임스탬프)로
  맞춰 보면 성공 14건 x 0.545~0.656(평균 0.598) / 실패 15건 x 0.544~0.653(평균 0.592), y·|yaw| 평균도 같다. 실패가
  워크스페이스 특정 영역에 몰리지 않는다.

남는 해석: 절대 액션 정책이 **사전파지 자세를 고정점**으로 삼는다 — "현재 상태 = 다음 40스텝의 목표"를 출력하면 팔이 멈추고,
멈춘 관측이 다시 같은 출력을 낳는다. 데모에서 이 지점은 상태머신이 하강 궤적을 계획하는 잠깐의 정지(≤0.8 s)와 겹치고,
파지 직전의 시각 변화(블록이 프레임을 채움)가 적어 "내려가라"와 "기다려라"가 관측상 가장 가까운 구간이다. 태스크별 편차
(A 의 red→right 1/9)는 같은 물체·같은 접근인데 지시문만 다른 경우라 언어 토큰이 이 구간의 출력에 영향을 준다는 뜻이다.
**이건 가설이다** — 확정하려면 정지 시점의 관측을 서버에 넣어 액션 청크가 현재 상태와 일치하는지 직접 봐야 한다.

blue→left 의 근소 미달 3건(38·43·61 mm)은 별개다 — 집고 옮기고 놓기까지 다 했고 정밀도만 모자란다. 성공 평균 22.8 mm 도
ACT 7.6 mm 의 3배라, GR00T 는 256×256 리사이즈(§2.3)로 손목 영상의 위치 정보가 ACT(320×240 원본)보다 거칠다.

### 46.4 하네스 — 파일 복제 없이 환경변수로 재사용, 파이프라인 1개 추가

`collect240.sh`(`N RAW RADIUS SEED TAG`) · `convert240.sh`(`RAW OUT REPO COLLECT_LOG TAG VCODEC`) · `groot_train_abs.sh`
(`DS REPO OUT TAG`) · `groot_v8.sh`(`RADIUS SEED TAG`) · `groot_rollout.sh`(`ROLL_TAG`) — **기본값은 전부 `240_v2` 재현 그대로**.
`collect240.sh` 에 `collect_100.sh` 와 같은 `grasp_watch.py` 감시를 넣었다. `groot_train_abs.sh` 헤더에 남아 있던 상대 액션
설명(스크립트는 §44.1 부터 절대)을 §43.7 근거로 고쳤다. 신규 `groot_pipeline.sh [N]`: 단계마다 산출물 검사(에피소드 수 ≥ 90 %,
`info.json` 의 에피소드 수·태스크 3·코덱 h264 일치, 체크포인트 `config.json`), 학습 15 분 시점 ETA 로 **6 h 게이트 자동 판정**,
`train_watchdog.sh`, 롤아웃 A/B 순차, `shutdown_rollout.sh`. 로그는 `outputs/harness_logs/{pipe,collect,train,rollout_v3_a,rollout_v3_b}_v3.log`.
같은 세션에서 새 것: `PIPELINE.md` §4-3 에 `scratchpad/groot_v8.sh` 참조가 §45.9 이후에도 남아 있었다(마크다운 링크가 아니라
링크 검사에 안 걸림) → 수정.

### 46.5 산출물

| 경로 | 크기 | 비고 |
|---|---|---|
| `outputs/il_raw_3task_v3` | 1.1 GB | raw 90 ep(재변환 가능 — `240_v2` 는 raw 가 없었다, §43.9) |
| `outputs/lerobot_ds_3task_v3` | 158 MB | h264 crf23, 카메라 2대 320×240, 태스크 3 |
| `outputs/groot_3task_v3_abs` | 30 GB | `checkpoints/{005000,010000,last→010000}` — 중간 체크포인트 15 GB 는 정리 후보 |
| `outputs/harness_logs/grasp_c3v3/` | 270 장 | 파지 증거 |

### 46.6 다음 갈림길 (사용자 결정)

1. **정지 가설 확정** (반나절): 실패 시행의 정지 관측을 저장해 서버에 넣고, 청크가 현재 상태와 일치하는지 본다. 맞으면
   §3 의 상대 액션이 정공법인데 서비스 불가(§43.7)라 **lerobot `GrootN17ActionDecodeStep` 의 청크 단위 후처리 구현**이
   필요하다 — 설계 원칙(§0)상 upstream 에 없는 기능이라 "별도 구현" 칸이다.
2. **데이터 쪽 손잡이** (수집 40 m + 학습 1.8 h): 사전파지 정지를 데모에서 없애거나(`--max-idle-run`, 이미 0.8 s 라 효과 의문)
   ep 수를 더 늘린다 — 21→90 에서 38→45 % 였으니 기대는 낮다.
3. **추론 손잡이** (학습 불필요): `num_inference_timesteps` 를 올려 디노이징을 늘리면 고정점 출력이 흔들릴 수 있다 —
   V7 예산(80 ms / 1,333 ms)이 16배 남아 있어 공짜다. 가장 싼 실험.

### 46.7 갈림길 3 실행 — `num_inference_timesteps` 4→16: **효과 없음, 9/27 (같은 씬 4스텝 14/29)** (2026-09-16)

요청: *"2번 num_inference_timesteps 올려서 롤아웃 다시 해줘"*. 값은 체크포인트 `config.json` 에서만 읽히고(`modeling_groot.py`
"None 이면 체크포인트 값 유지", `policy_server` 는 `from_pretrained(path)` 뿐) 서버 덮어쓰기 옵션이 없다 → 가중치·전처리기
파일을 **심볼릭링크**로 공유하고 `config.json` 만 16 으로 바꾼 변형 디렉터리
`outputs/groot_3task_v3_abs/variants/nit16/pretrained_model`(디스크 +0). 로드 확인: `policy.config` 16, `_groot_model.config` 16.
V7 지연 중앙값 **160 ms**(4스텝 80 ms — 디노이징 4배인데 전체는 2배, 백본이 절반) / 예산 1,333 ms.

롤아웃은 §46.2 A 와 **같은 시드 171**(시행별 물체 자세 동일) → 1:1 대응 비교. SKIP 3(6·21·30 번: 관절 토픽 0 Hz / 전 토픽
5 Hz / 손목 6 Hz — 전부 순간 정체, 직후 정상. 사전검사가 걸렀다).

| | 4 스텝 (§46.2) | 16 스텝 | 대응 26회 |
|---|---|---|---|
| red→left | 8/10 | **3/10** | 8 → 3 |
| blue→left | 5/10 | 3/10 | 5 → 3 |
| red→right | 1/9 | 3/7 | 1 → 3 |
| 합계 | **14/29 (48 %)** | **9/27 (33 %)** | **14 → 9** |

대응 26회의 전이: 성공→성공 5, 실패→실패 8, **성공→실패 9, 실패→성공 4**. 같은 물체 자세에서 결과가 양방향으로 뒤집혔다 —
정지가 "풀린" 것이 아니라 **어느 시행이 정지하는지가 바뀌었다**. 실패 종류도 그대로다(미접촉 9 = 사전파지 정지, 근소 미달 6,
낙하 2, 신규 1: 마커 25 mm 까지 옮기고 **안 놓음** grip 0.44). 성공 배치 평균 22.1 mm(4스텝 22.8) — 정밀도도 안 변했다.

**결론**: 디노이징 스텝은 정지의 원인이 아니다(늘려도 안 풀리고 오히려 -5). 정지는 샘플링 노이즈 수준의 무작위성으로 시행마다
켜지고 꺼지는, 정책이 사전파지 관측에서 출력하는 분포 자체의 문제다. §46.6 의 1(정지 관측을 서버에 넣어 청크 = 현재 상태인지
확정)이 남은 갈림길이고, 그 다음이 상대 액션 청크 후처리 구현이다. 변형 디렉터리는 §46.6-3 의 증거로 남긴다(링크뿐).

## 47. UR16e `ready` 재정의 초안 — OMY 팔로워의 실제 초기 자세를 매핑으로 옮긴다 (2026-09-16, GUI 검토 전)

요청: *"ur16e 의 초기 pose(ready)를 재정의하고 가자 … 실물에서 보니 OMY manipulator 의 초기 포즈는 OMY-L100 의 초기 포즈와
유사해서 처음 움직일 때 연속성이 좋고 직관적인데, 우리 UR16e 초기 포즈는 아예 다르다."* 초안을 만들고 GUI 로 함께 검토하기로.

### 47.1 왜 달랐나 — §41 의 전제가 틀렸다

§41 은 ROBOTIS **SRDF `home`**([0,0,+90,−90,+90,0]°, 상완 수직·전완 전방)이 리더의 휴식 자세라고 보고, 그 매핑값이 우리
`ready` 라서 "새 초기 자세가 필요 없다"고 결론냈다. 그러나 ROBOTIS 의 **실제 브링업**(`open_manipulator_bringup/config/
omy_f3m_follower_ai/initial_positions.yaml`, `joint_trajectory_executor`)은 팔로워를 `home`(전부 0) → **`ready`
= [0, −90, +152, −62, +90, 0]°** 로 보낸다(리더 쪽 `omy_l100_leader_ai` 에는 초기 이동이 없다 — 사람이 놓는 자세).
FK(`omy_f3m.urdf`)로 보면 이 `ready` 는 **상완이 뒤로 수평, 전완이 앞·위로 접히고, 툴이 베이스 위 z 0.39 에서 전방을
향하는** 웅크린 자세다. SRDF `home` 과는 상완 방향부터 다르다. 사용자가 본 "OMY 팔로워 초기 = L100 초기" 는 이 yaml
`ready` 이고, 우리 `ready` 는 그것의 매핑이 아니었다.

### 47.2 ★ J4 오프셋은 유도된다 — −90°

브리지(`omy_to_ur16e.py`) 주석은 "J4/J6 오프셋은 유도 불가, 튜닝 손잡이" 라 했는데 J4 는 **유도된다**. J2 처럼 두 팔의
피치 영점이 90° 다르다. `offset=[0,−90,0,−90,0,0]`, `sign=[1,1,1,1,−1,1]` 로 OMY 세 자세를 넣고 UR16e FK
(`cumotion/ur16e_2f85.urdf`)와 툴 방향(손목→툴)을 비교하면:

| OMY 자세 | OMY 툴 방향 | UR16e 매핑 | UR16e 툴 방향 |
|---|---|---|---|
| SRDF `home` [0,0,90,−90,90,0] | +X | [0,−90,90,−180,−90,0] | +X |
| bring-up `ready` [0,−90,152,−62,90,0] | +X | **[0,−180,152,−152,−90,0]** | +X |
| `init` [0,0,0,0,0,0] | −Y(측방) | [0,−90,0,−90,0,0] | +Y(측방, 손목 측방 오프셋 부호 차 §21) |

오프셋 0 이면 같은 자세에서 UR 툴이 **아래**(−Z)를 향해 베이스 옆 z 0.22 를 가리킨다 — 리더 휴식 자세에서 팔로워 그리퍼가
베이스에 처박히는 랑데부다. 브리지 주석의 "측방 오프셋 −46 vs +290 mm" 는 **위치** 차이고 피치 **방향**과는 무관하다.

### 47.3 초안 — `ready_omy` = [0, −180, +152, −152, −90, 0]°

`reset_pose.py` 에 **`ready_omy`** 로 추가(기본 `ready` 는 아직 그대로 — GUI 검토 후 교체). Isaac(headless) + 스택에서:
- MoveIt `/check_state_validity`: 구 `ready`·`ready_omy`·OMY-init 매핑 **셋 다 valid**(자기충돌·테이블 없음).
- 궤적 컨트롤러로 이동 `error_code 0`, 정적/손목 카메라 캡처(`ready_pose_draft.png`): 팔꿈치(z 0.18)가 베이스 뒤 0.48 m,
  손목 (−0.16, 0.35), 툴 (−0.03, 0.47) 전방 수평. **손목 카메라가 테이블·마커·블록을 정면에 담는다**(구 `ready` 는 바로
  아래 바닥). OMY 의 툴 (0.19, 0.39)과 같은 형태이고 0.2 m 뒤로 밀린 것은 UR16e 상완/전완 비(478/360)가 OMY(247/220)보다
  커서다.
- **실물 조건**: 베이스 뒤로 상완 길이 **≈0.55 m 이 어깨 높이(0.18 m)에서 비어 있어야** 한다. 테이블 가장자리 설치면 확인 필요.
- 바꾸면 따라 바뀌는 것: 브리지 `offset[3]=−π/2`·`rendezvous` 기본값(`omy_to_ur16e.py`·`teleop_omy.launch.py`),
  CHECKLIST E-1②/E-3 표(리더 휴식 = yaml `ready`), HARDWARE·PIPELINE·README 의 `ready` 설명, §41 정정. **기존 데이터셋
  (`red_left_100`·`wrist_only`·`3task_v3`)과 체크포인트는 구 `ready` 에서 시작**하므로 그 롤아웃은 구 자세를 써야 한다 —
  교체 시 구 자세를 `ready_v1` 로 남기고 하네스 시작 자세를 변수로 뺀다.

### 47.4 ★ 함정 2개 — 공유 머신 (이번 검증 중 발견, 하네스 수정 완료)

1. **다른 컨테이너의 ROS 2 가 같은 도메인 0 에 있다.** 사용자가 다른 컨테이너에서 `ros2 gz sim` 기반 이동로봇(nav2, `/gz_bridge`,
   `<robot name="mobile_robot">`)을 테스트 중 → 우리 `controller_manager`/`move_group` 이 **그쪽 `/robot_description`** 을
   받아 `no 'ros2_control' tag found in the URDF` / `Link 'tool0' … not known` 으로 죽는다. 우리 프로세스 목록에는 아무것도
   없어서(호스트 네트워크 DDS) `ps` 로는 안 보인다 — `ros2 node list` 에 모르는 노드가 있으면 이거다. 처방: **우리 스택을
   다른 도메인으로**(`export ROS_DOMAIN_ID=42` 뒤 Isaac 부터 기동). 하네스 14개의 `export ROS_DOMAIN_ID=0` 을
   `"${ROS_DOMAIN_ID:-0}"` 로 바꿔 환경을 물려받게 했다(기본은 여전히 0). 다른 프로젝트는 건드리지 않는다.
2. **고아 spawner 가 파일 락을 쥔다.** `controller_manager` 가 죽으면 그 spawner 들이 `~/.ros/locks/
   ros2-control-controller-spawner.lock` 을 잡은 채 남고, 이후 모든 spawner 가 `Failed to acquire lock in 20 seconds` ×5 로
   죽는다(`No controllers are currently loaded!`). `bringup.sh` 의 정리 패턴에 `controller_manager/spawner` 추가.
   부수 함정: bringup 의 "RSP 가 정확히 1개" 검사는 `ps` 문자열 매칭이라 **호출 쉘의 명령줄에 그 문자열이 있으면 오검출**된다
   (heredoc 안의 패턴 텍스트 때문에 "3 robot_state_publishers" 로 실패) — 긴 명령은 파일로 저장해 실행할 것.

### 47.5 확정 — `ready` 교체, 브리지 오프셋·랑데부·하네스 동기화, sim 3종 검증 (2026-09-16)

사용자가 Isaac GUI 로 보고 확정(*"육안으로 보았을 때는 초기 포즈가 좋아 보여. 이대로 하자."*). 바뀐 것:

| 파일 | 변경 |
|---|---|
| `isaac/common/reset_pose.py` | `ready` = [0,−180,152,−152,−90,0]°, 구 자세는 **`ready_v1`**(초안명 `ready_omy` 삭제) |
| `scripts/pick_place_demo.py` | `READY` 동일값 — 수집 에피소드 시작 = 텔레옵 랑데부 |
| `scripts/omy_to_ur16e.py`·`launch/common/teleop_omy.launch.py`·`scripts/virtual_omy_leader.py` | `offset=[0,−90°,0,−90°,0,0]`, `rendezvous` 기본 = 새 `ready`, 주석의 "J4 유도 불가" 정정 |
| `scripts/omy_leader_calib.py` | 주석 정정(J6 만 손잡이) |
| `isaac/ur16e_2f85/grasp_test.py` | 값 유지(툴 하향 자세가 필요한 테스트) — `ready_v1` 이라고 표기 |
| `harness/rollout_n.sh`·`rollout_wrist1.sh`·`groot_rollout.sh` | 시작 자세 `START_POSE`(기본 `ready_v1`); `groot_pipeline.sh` 는 `ready` 고정 |
| 문서 | CHECKLIST E-1②/E-3(기대값·표·**베이스 뒤 공간** 항목), SETUP·plan_il_vla·README·HARDWARE·harness README·CLAUDE.md, §41 정정 박스 |

**sim 검증(Isaac GUI, 도메인 42)** — 셋 다 통과:
1. `reset_pose.py ready` → error_code 0.
2. 브리지: `teleop_omy.launch.py virtual_leader:=true leader_amplitude:=0` 기동 로그가 **"put the LEADER at
   [0,−90,152,−62,90,0] deg"** 를 역산해 찍음(= OMY 브링업 `ready` 그대로) → `/omy_bridge/sync` → `synced` →
   `enable` → **`engaged`**. 새 오프셋으로 랑데부가 맞아떨어진다.
3. `pick_place_demo.launch.py cycles:=1` (red→left): READY(start) ok → **1/1 cycles succeeded** → READY(next cycle) ok.
   cuMotion 이 접힌 자세에서 사전파지까지 계획·실행한다.

**남은 것(실물에서)**: E-1② 실측(리더가 실제로 놓이는 값이 yaml `ready` 와 같은지), 베이스 뒤 0.55 m 확인, J6 오프셋
캘리브(`omy_leader_calib.py`). 기존 sim 데이터셋 3개는 `ready_v1` 시작이므로 롤아웃 시 `START_POSE=ready_v1`(기본값).

### 47.6 셋업 재점검 — `ros` 단계 추가(ROS 2 자체 설치), 문서 누락 3건 (2026-09-16)

요청: *"다른 PC 에서 셋업을 다시 해야 될 텐데, 셋업 가이드와 스크립트를 정확히 확인해줄래?"* → 이 PC 에서 `setup.sh --dry-run`
(기본 10단계) rc 0, `check_env.sh` rc 0, 하네스·setup 스크립트 문법, 파이썬 컴파일, 마크다운 링크(깨짐 0) 확인.

**빠져 있던 것 — ROS 2 Jazzy 자체.** 가이드는 "스크립트가 ROS 를 깔지 않는다, 이미지 문제" 라고만 했는데 Isaac Sim 기본
이미지에는 ROS 가 없어 새 컨테이너에서 `preflight` 가 즉시 실패한다. 사용자 결정(*"ros2 도 한번에 설치하는 것까지 하자"*)에
따라 `setup.sh` 에 **`ros` 단계**를 추가했다: `/opt/ros/jazzy` 가 **없을 때만** 공식 데비안 절차(locale → universe →
`ros2-apt-source` .deb → `ros-jazzy-desktop` + `ros-dev-tools`)를 `apt_guarded_install` 안전장치로 실행. 이 컨테이너의 설치
상태(`ros2-apt-source` 1.2.0~noble, `ros-jazzy-desktop` 0.11.0, `ros-dev-tools` 1.0.1)가 기준. `bootstrap.sh` 기본 순서는
`preflight → ros → pin → …`, `preflight` 는 `ros` 가 계획에 있으면 "ROS 없음"을 경고로 낮춘다. 검증: ROS 가 있는 이 PC 에서
"already — nothing to do", ROS 를 없는 것으로 취급한 사본으로 dry-run 시 fresh 경로(locale·universe·apt-source·guarded
install 계획)가 끝까지 출력. **실제 설치는 사용자가 만들 새 컨테이너에서 처음 검증된다.**

그 밖에: `archive/` 두 문서의 상대 링크 8개(이동 후 깨짐) 수정, `groot_rollout.sh` 주석의 옛 `groot_env.sh` 정정, 트러블슈팅 표에
§47.4 함정 2개 추가, PIPELINE·CHECKLIST 에 `START_POSE` 안내. 문서가 인용하는 `serve_policy.py`·`inference_service.py` 는
openpi·GR00T 원본 파일명이라 정상.

**Isaac Sim 6.1.0 주의(2026-09-16, 사용자 예고)**: 새 컨테이너가 6.1.0 일 수 있다. 이 워크스페이스는 6.0.1 에서만 검증됐고
(5.1.0→6.0.1 은 무수정 이식, SETUP §0), `ur16e_isaac_ros2.py` 는 `isaacsim.core.api`·`isaacsim.core.prims`·
`isaacsim.core.utils`·`isaacsim.storage.native`·`isaacsim.ros2.bridge`(OmniGraph) 를 쓴다. `preflight` 가 6.0.1 이 아니면
경고하도록 바꿨다(이전엔 6.x 면 조용히 통과). 6.1.0 에서는 §5 스모크(Isaac 기동 → `/joint_states` → MoveIt plan) 결과를
먼저 여기에 적고 나서 나머지를 진행한다.

---

## 48. 새 컨테이너(Isaac Sim 6.1.0-rc.26, ROS 없음)에서 `bootstrap.sh` 로 처음부터 재현 — 스크립트 결함 4건 수정 (2026-09-16)

요청: *"새로 컨테이너를 만들어서 CLAUDE.md 만 가져오고 git clone 만 한거야 … setup 폴더를 참고해서 개발환경 셋업을 진행해줄래.
그리고 정확한 개발환경 셋업 가이드와 스크립트를 완벽히 업데이트해줘."* / *"나중에 다른 PC 에서도 새로운 컨테이너에서 셋업을
진행할거야."* / *"groot n1.7 은 모델을 받을 때 나의 hf 권한이 필요하니깐, ACT 까지만 셋업하고 검증해줘도 돼."*

§47.6 은 "실제 설치는 새 컨테이너에서 처음 검증된다" 로 끝났다. 이번이 그 검증이다.

### 48.1 출발 상태 — Isaac Sim 기본 이미지의 실체

| 항목 | 값 |
|---|---|
| 이미지 | Isaac Sim **6.1.0-rc.26**+release.49347 (검증본 6.0.1 아님), Ubuntu 24.04.3 |
| 사용자 | `isaac-sim`(uid 1234), **NOPASSWD sudo** |
| 없는 것 | `/opt/ros`, **`python3`**, `vcs`, `colcon`, `wget`, `nvcc`, `cuobjdump` |
| apt | `ubuntu.sources` 만(universe 포함), **`apt-get update` 이력 0**(`/var/lib/apt/lists` 비어 있음) |
| GPU / 디스크 / shm | RTX 5090 sm_120, 드라이버 580.173 / 2.0 TB 여유 / **`/dev/shm` 32 GiB**(첫 머신은 64 MiB) |
| 워크스페이스 | `CLAUDE.md` + `git clone` → `src/` 만 |

### 48.2 실행 순서와 걸린 곳 — 스크립트가 fresh 머신에서 실제로 깨진 지점

`bootstrap.sh --dry-run` 은 rc 0 이었지만 계획의 절반이 "could not be simulated" 였고, 실제 실행은 아래에서 멈췄다.
전부 고쳐서 **같은 컨테이너에서 각 단계를 재실행해 통과**시켰다.

| # | 증상 | 원인 | 수정 |
|---|---|---|---|
| ① | `ros` 단계 즉사: `E: Unable to locate package software-properties-common` | apt 목록이 한 번도 갱신된 적 없는 이미지 → `apt-get install -s` 조차 해석 불가 | `lib.sh` `apt_lists_refresh`(목록 비면 `apt-get update`), `ros` 단계 맨 앞에서 호출. dry-run 은 예고만 |
| ② | `software-properties-common` 이 **11개 업그레이드**(util-linux·libsystemd0 …) 요구 → 가드 거부 | packagekit 의존. 그런데 universe 는 이미 켜져 있어 이 패키지 자체가 불필요 | universe 가 꺼져 있을 때만 설치 → 이 단계는 `locales curl` 만 (0 upgraded) |
| ③ | `ros-jazzy-desktop` 이 **14개**, 이어 `base` 의 moveit 이 **4개**(ncurses) Ubuntu 포인트 릴리스 업그레이드 요구 → 거부 | 새 이미지의 낡은 기반 라이브러리. 정당한 업그레이드지만 가드는 출처를 안 봤다 | `ALLOW_UPGRADES=ubuntu` 신설: `Inst` 줄의 출처를 파싱해 **`Ubuntu:24.04/noble-updates`·`-security` 만** 허용, 다른 출처·삭제는 여전히 거부. `bootstrap.sh --fresh` 가 이걸 켠다. 첫 파서는 apt 가 줄 끝에 붙이는 ` []`·`[amd64]` 를 못 걷어내 한 줄을 "foreign" 으로 오판 → 정규화 보강 |
| ④ | `base` 단계가 첫 `apt_guarded_install` 실패를 **성공으로 보고** | `\|\| return 1` 누락, 뒤의 realsense 줄(`\|\| warn`)이 rc 를 덮음 | `\|\| return 1` 추가 |

그 외 손본 것: `sudo env DEBIAN_FRONTEND=noninteractive apt-get`(`sudo` 가 환경을 버려 debconf 프롬프트 위험),
ros-apt-source 다운로드 `curl -sS`(진행바 소음), `preflight`/`check_env` 가 6.1.0 을 검증본으로 인정, `check_env` 에
`python3` 존재 검사, GR00T 검사가 `transformers` 를 명시 요구(§48.5), shm 우회책 메시지가 `/dev/shm` 크기를 보고 "필요 없음"
을 구분, `bootstrap.sh --with-groot`, `--help` 가 코드까지 출력하던 sed 범위 → awk.

### 48.3 설치 결과 (전부 새로 깔림)

`ros`(1446 debs, ros2-apt-source **1.3.0**, ros-dev-tools **1.0.3**) → `pin` → `repos`(핀 검증: robotiq_description 후보 0.0.1)
→ `base`(159 debs; ros2_control **4.48.0** + diagnostic_updater 4.2.7 같은 빌드 — §HARDWARE 의 ABI 함정은 업그레이드 경로
에서만 생긴다) → `cumotion`(4.6.0, **CUDA 13.2.2**, VPI 4.0.0) → `sources`(7 repo, 핀 SHA 전부 일치, nvblox_core 서브모듈)
→ `build`(ur_bringup·topic_based 0.2.1·robotiq_driver·serial, **nvblox_ros sm_120 소스빌드 2 m 19 s**; apt nvblox_node 는
sm_75 만, libcumotion 은 sm_75/86/89/120) → `leader`(dynamixel 2개 0 upgraded, 7 패키지) → `ml`(venv 7.7 GB, torch
2.11.0+cu128 archs sm_75~120, lerobot 0.6.1) → `groot`(19개 신규, transformers 5.5.4) → `verify` **environment OK**.
`requirements-ml.lock` 을 `env -i` 로 다시 뜸(옛 잠금엔 ROS site-packages 46개가 섞여 있었다; 실제 차이는 accelerate/peft/
grpcio 등 패치 버전뿐).

### 48.4 ★ Isaac Sim 6.1.0-rc.26 — 무수정 통과

`isaacsim.core.api`·`.prims`·`.utils` 가 `/isaac-sim/exts` 에서 **`/isaac-sim/extsDeprecated`** 로 옮겨졌다.
`isaacsim.core.deprecation_manager` 가 그대로 import 시켜 주므로(SimulationApp 안에서 9개 모듈 전부 import OK) 스크립트는
그대로 돈다. 다음 메이저에서 없어질 수 있는 API 라 `preflight` 는 6.0.1/6.1.0 외에는 경고한다.

| 검증 | 결과 |
|---|---|
| 세트1 mock (`use_mock_hardware`) | 15 컨트롤러 로드, JTC·JSB active, `/joint_states` 유효 |
| 세트2 headless: `/isaac_joint_states`·`/clock` → `ur16e_2f85.launch.py` → `reset_pose.py ready` → `ur16e_2f85_moveit` plan+execute | error_code 0 / **SUCCESS**, 도달 오차 0.0100 rad |
| 세트3 headless(하네스 씬, 카메라 2대 320×240) → `harness/bringup.sh`(cuMotion) → `pick_place_demo.launch.py cycles:=1` | 카메라 58 Hz, `/move_action`·gripper 액션 ok, **1/1 cycles succeeded**(part 0.001 m from marker) |
| 6.1.0 새 경고 | `pxr.Semantics is deprecated` 1건뿐. 6.0.1 의 `targetPrim is deprecated` 그대로 |

**하네스 함정(⑤)**: `bringup.sh` 의 move_group 대기 문자열 `Ready to take commands for planning group` 은 **RViz** 의
MoveGroupInterface 가 찍는다 — 이 컨테이너는 `DISPLAY=:0` 이 붙어 있어 RViz 가 떠서 우연히 통과했지만, X 없는 컨테이너면
영원히 `FAIL: move_group not ready` 다. move_group 자체의 `You can start planning now!` 도 허용하도록 수정.

### 48.5 GR00T — 스크립트 경계

`setup.sh groot` 는 통과(extra 설치·sm_120 유지·import). 모델 파일은 `nvidia/Cosmos-Reason2-2B` 게이트라 **사용자 토큰
없이는 못 받는다** → 여기서 멈추는 것이 맞다. 사용자가 할 일 7단계를 `CHECKLIST.md` **B-2** 로 정리(라이선스 동의 → read
토큰 → `hf auth login`(HF_HOME=deps/hf_cache) → 다운로드 2건 → `check_hf_cache.sh` → 20 스텝 스모크).
부수 발견: `check_env.sh` 의 GR00T 검사 `import lerobot.policies.groot.modeling_groot` 는 **extra 없이도 통과**한다
(무거운 import 가 lazy) — `verify` 가 `groot` 보다 먼저 돈 순간 "(transformers )" 로 빈 버전이 찍혔다. `transformers` 를
같이 import 하도록 고쳤다.

### 48.6 ACT 파이프라인 스모크 — 이 컨테이너에서 수집→변환→학습까지 실제 통과

사용자 지시대로 검증 경계는 ACT. 하네스 스크립트를 **그대로**(`collect_wrist1.sh 3` → `convert_wrist1.sh` → `lerobot-train`)
썼다. 도메인 42, Isaac headless.

| 단계 | 결과 |
|---|---|
| 수집 `collect_wrist1.sh 3` (Isaac 6.1.0 + `bringup.sh` + `pick_place_demo` + `il_recorder`) | 3 에피소드, 18 MB, grasp 프레임 9, rc 0 |
| 변환 `convert_wrist1.sh` (`raw_to_lerobot.py`, ML venv) | 3 ep / 1356 프레임 / 30 fps, `observation.images.wrist` (240,320,3) video, 2.5 MB |
| 학습 `lerobot-train --policy.type=act --steps=100 --batch_size=32 --num_workers=2` (Isaac 내린 뒤, `ml_env.sh`) | rc 0, `checkpoints/000100/pretrained_model/` 생성(config·safetensors·pre/postprocessor), shm 오류 0, GPU 1.6 GB |

`/dev/shm` 32 GiB 라 워커 2 로 그냥 돈다(§24 의 우회책 불필요). 100 스텝은 `log_freq` 기본 200 보다 짧아 loss 줄은 안 찍혔다 —
스모크 판정은 rc 와 체크포인트 산출물로 했다. 스모크 산출물(`outputs/act_smoke`·`il_raw_wrist_only`·`lerobot_ds_wrist_only`,
합계 0.6 GB)은 **지웠다** — 하네스의 정본 이름을 3 에피소드짜리로 선점하면 나중 `collect_wrist1.sh 50` 이 47개만 더 모은다.

### 48.6-B 가짜 리더 teleop — 6.1.0 통과 (사용자 요청으로 추가 실행)

세트2 headless → `ur16e_2f85.launch.py use_sim:=true` → `reset_pose.py ready` → `teleop_omy.launch.py use_sim_time:=true
virtual_leader:=true` → `switch_control_mode.py streaming` → `/omy_bridge/enable`. 가상 리더가 팔로워 `/joint_states` 를
역매핑해 `[0,−90,152,−62,90,0]°` 에서 출발(랑데부 일치) → `engaged` → 리더 J1 ±17°·J3 ±25° 사인 동작 25 s 동안
**추종 오차 최대 0.256°, 평균 0.199°**(3671 샘플; 기대값 §22 ≈0.24°). `disable` → trajectory 복귀 정상.
측정 함정: `/leader/joint_states` 는 **best-effort** QoS 라 기본(reliable) 구독으로는 한 샘플도 안 온다 — 측정 노드는
QoS 를 맞춰야 한다.

### 48.8 ★ ACT 롤아웃까지 닫기 — 셋업 결함 1건(로봇 플러그인 미설치) + 하네스 결함 2건 (2026-09-16, 사용자 요청)

사용자: *"ACT 도 스모크 학습하고 롤아웃까지 완료한건가?"* → §48.6 은 학습까지였다. *"롤아웃의 성공률을 보려는 목적은 아니고,
전체 파이프라인이 정상인지 보려는거야."* → 3 ep / 100 스텝 체크포인트로 `rollout_gui_then_n_w1.sh 1 60` 을 돌렸다.

| # | 증상 | 원인 | 수정 |
|---|---|---|---|
| ⑥ | `robot_client.py: error: argument --robot.type: invalid choice: 'ur16e_ros'` | **우리 LeRobot 로봇 어댑터(`ur_bringup/lerobot_robot_ur16e_ros`)가 ML venv 에 설치돼 있지 않았다.** lerobot 은 설치된 `lerobot_robot_*` 배포판을 스캔해 로봇을 등록하므로 `pip install -e` 가 필수인데, 첫 머신에서 손으로 한 뒤 **어느 문서에도 없었다**(`.gitignore` 주석뿐) | `setup.sh ml` 에 `_install_robot_plugin`(editable 설치 + 배포판 존재 검증), `check_env.sh` REQUIRED 검사 추가. 재실행으로 검증 |
| ⑦ | `rollout_wrist1.sh: line 84: 0\n0: syntax error in expression` → 트라이얼이 통째로 SKIP | `total=$(grep -c … \|\| echo 0)` — `grep -c` 는 매치 0 이면 **"0" 을 찍고 exit 1** 이라 `echo 0` 이 한 번 더 붙는다 | `total=$(grep -c …); total=${total:-0}` (`groot_rollout.sh` 의 같은 줄도) |
| ⑧ | 하네스 재기동 시 `A controller named 'scaled_joint_trajectory_controller' was already loaded` / `can not be configured from 'active' state` → `controllers never activated` | `bringup.sh` 가 패턴 kill 후 **고정 6 s** 만 기다림. 활성 컨트롤러를 가진 `ros2_control_node` 는 TERM 후 그보다 오래 살고, 새 스택의 spawner 가 **옛 controller_manager** 에 붙는다(teleop 스택 직후 수집 하네스를 띄우면 재현) | kill 대상에 `ros2 launch ur_bringup` 부모 추가, **0개가 될 때까지 최대 30 s 대기 후 KILL**, spawner 락 파일 제거 |

부수: 패턴 kill 은 **호출 셸 자신을 죽인다** — heredoc 으로 넘긴 긴 명령의 `bash -c` argv 에 패턴 문자열이 들어 있어 첫 시도가
rc 144 로 죽었다(§47.4 의 "긴 명령은 파일로" 그대로). 정리 루프는 `$$`·`$PPID` 를 제외해야 한다.

**결과(플러그인 설치 후 재실행, Isaac 6.1.0 headless, 도메인 42)** — 파이프라인 전 구간 정상:

| 구간 | 결과 |
|---|---|
| 수집 `collect_wrist1.sh 3` → 변환 → `lerobot-train` 100 스텝 | rc 0, loss 0.340, `checkpoints/last` |
| `policy_inference.launch.py` → 정책 서버 기동 → `reset_pose ready` → streaming 전환 → `obs_ready.py` | joints 41 Hz · wrist 48 Hz **fresh** |
| `robot_client --robot.type=ur16e_ros` 60 s | 서버 **68 청크** 서비스, 팔 구동(관절이 ready 에서 1.3 rad 이상 이동) |
| `judge_rollout.py` | `FAIL d=273mm grip=0.13 (too far)` — 3 ep·100 스텝 정책의 **기대된** 결과. 판정 경로 자체는 동작 |
| 클라이언트 로그 | `publisher's context is invalid` traceback 은 60 s `timeout` 종료 순간(시작 +57 s) 한 번 — SIGTERM 으로 rclpy 컨텍스트가 먼저 닫히는 종료 아티팩트 |

스모크 산출물(`act_wrist_only`·`il_raw_wrist_only`·`lerobot_ds_wrist_only`)은 다시 지웠다.

### 48.7 남은 것 / 이 컨테이너 기준 상태

- **GR00T**: `CHECKLIST.md` B-2(사용자 HF 계정 절차) 후 `check_hf_cache.sh` → §3-B 20 스텝 스모크. `check_env.sh` 의
  `deps/hf_cache ... refs/main missing` 두 줄은 그때까지 정상(REQUIRED 실패 아님, rc 0).
- **미재실행**: nvblox A/B 회피 데모(`nvblox_obstacle_demo.py`, §12), teleop 가짜 리더(§22), 실물 HW 전부. nvblox_ros 는
  sm_120 로 빌드됐고 `check_env` 통과 — 데모는 다음 사용 시.
- 문서/스크립트 반영: `SETUP.md` §0(버전표 신설)·§2-C(로봇 플러그인)·§0-B(전제·`--fresh`·1-B 재작성·소요시간)·§0-A(단계표·안전장치)·§2-C(shm 은
  `--shm-size` 문제)·§9(새 컨테이너 5행), `CHECKLIST.md` A/B/B-0/**B-2 신설**, `README.md`, `CLAUDE.md`, harness `README.md`,
  `bootstrap.sh`(`--fresh`·`--with-groot`·`--help`), `setup.sh`, `lib.sh`, `check_env.sh`, `ml_env.sh`, `requirements-ml.lock`,
  `harness/bringup.sh`. 커밋은 하지 않았다(사용자 검토용 working tree).

## 49. GELLO(`gello_software`) 대조 검토 — 리더 action 기록 결함 1건 수정 + GELLO 장점 3건 반영 (2026-09-17)

`tempp/gello_software`(wuphilipp/gello_software, HEAD `204f53a`, repo 밖 참고용 클론)를 통째로 읽고 우리
OMY 브리지·기록기·캘리브·체크리스트와 대조했다. 전제는 같다(관절 1:1 affine 매핑, IK 없음). 안전장치는 우리가
더 많다(상시 slew·clamp·워치독·데드맨·engage 게이트 vs GELLO 는 정렬 단계 속도 제한뿐). 그런데 **GELLO 가 하는
핵심 한 가지를 우리는 실제로 못 하고 있었다.**

### 49.1 결함 — `action_source:=topic` 기록이 깨져 있었다 (재현으로 확인)

`CHECKLIST.md` H·`HARDWARE.md` 4-B·`plan_il_vla.md` §3.5 가 전부 실물 수집 명령으로
`-p action_source:=topic -p action_topic:=/leader/joint_states` 를 안내했고, §20·§41·§47 은 "기록기 무수정으로
붙는다"고 적었다. **틀렸다.**

| | |
|---|---|
| 원인 | `/leader/joint_states` 의 관절 이름은 `joint1..6`/`rh_r1_joint`. 기록기는 UR 이름(`shoulder_pan_joint`…)만 찾는다 → `act_arm=None`. 이름을 맞춰도 값이 sign/offset 적용 **전** 리더 좌표라 틀린 action |
| 증상 | 기록기는 `saved ... (79 frames)` 로 **성공을 보고**하고 79 프레임 전부 `action.single_arm: null`. `raw_to_lerobot.py:226` 이 `TypeError: unsupported operand type(s) for +: 'NoneType' and 'NoneType'` 로 사망 |
| 재현 | 가짜 `/joint_states`+카메라 2개 + `virtual_omy_leader.py` + `il_recorder.py -p action_source:=topic -p action_topic:=/leader/joint_states` → start/stop → `data.json` 검사 → 변환 |
| 안 드러난 이유 | 지금까지 모든 데이터셋(§19~§46)은 `next_state` 로 모았다. 실물 리더를 붙이는 첫날 터졌을 결함 |

GELLO 가 기록하는 action 은 `agent.act(obs)` = **매핑 후 팔로워 명령**(`format_obs.save_frame` 의 `control`)이다.
우리 스키마(§2.6)의 action 도 "UR16e 목표 관절"이므로 같은 것을 브리지가 내보내면 된다.

**수정**
- `omy_to_ur16e.py`: engaged 중 매 tick **`/omy_bridge/command_joint_states`**(JointState, UR 6관절 이름 + `finger_joint`)
  발행 — 값은 이번 tick 에 실제로 보낸 명령(매핑·clamp·slew 후)과 그리퍼 목표(deadband 로 goal 을 건너뛴 tick 에도
  목표값은 기록). 리더에 트리거가 없으면 팔로워의 `finger_joint` 현재값(hold).
- `il_recorder.py`: `topic` 모드에서 action 토픽에 UR 팔 관절이 없으면 **`_ready()` 가 거부**(메시지에 올바른 토픽 안내);
  action 토픽에 `finger_joint` 만 없으면 그리퍼는 `next_state` 로 폴백(경고 1회); 저장 직전 **null action 이 하나라도
  있으면 저장 거부**(변환기에서 뒤늦게 죽는 대신 그 자리에서).
- 문서: `CHECKLIST.md` H·E-3, `HARDWARE.md` 4-B ③, `README.md`, `plan_il_vla.md` §3.5(정정 블록)·할 일 표 5행,
  `teleop_omy.launch.py` 헤더, `CLAUDE.md`.

### 49.2 GELLO 에서 가져온 것 3건

| # | GELLO | 우리(반영) |
|---|---|---|
| ① 리더 점프 가드 | 정렬 후에도 매 사이클 리더−팔로워 차 0.5 rad 초과면 `exit()` ("Action is too big") | 초안 `max_leader_jump`(절대 0.5 rad/샘플) → **§49.4 에서 오탐 확인, `max_leader_speed`(20 rad/s, Δq/Δt) 로 교체**. engaged 중 초과하면 disable, status 는 다음 enable 까지 **`leader_jump`** 로 유지(`stop_reason` — 워치독도 이제 `watchdog` 가 tick 에 덮이지 않고 남는다). 워치독은 "발행 중단"만 잡고 값 튐(케이블 접촉 불량·엔코더 wrap)은 slew 가 1 rad/s 로 충실히 따라갔던 구멍 |
| ② 팔로워→리더 자동 정렬 | 리더를 아무 자세에 두면 팔로워가 스텝당 0.05 rad × 25 스텝 접근(초기 차 0.8 rad 초과만 거부), 직선 보간·충돌검사 없음 | **`/omy_bridge/sync_to_leader`**: 목표만 "현재 리더의 매핑 자세(clamp)"로 바꾼 `/sync` — MoveIt 충돌검사·컨트롤러 전환·streaming 복귀 그대로(`_start_sync` 로 공통화). 캘리브/튜닝용. **수집은 `/sync`**(시작 자세 통일) |
| ③ 에피소드 꼬리 트리밍 | 저장 시 마지막 5 프레임 폐기, 30 프레임 미만 스킵 | `trim_tail_frames`(기본 0, **stop 시점에 읽어** `ros2 param set` 이 실제로 먹는다 — §42.3-C 의 조용한 no-op 함정 회피), `meta.json` 에 기록, `frames/*.jpg` 도 같이 삭제. §46 실패가 "정지"였고 정지 버튼을 누르는 꼬리는 항상 정지라 실험 가치 있음. 기존 데이터셋 비교를 위해 기본 0 |

반영하지 않은 것(범위 밖 또는 이미 커버): 양팔·Quest·SpaceMouse·MuJoCo·FACTR 힘반영(팔로워 토크→리더;
OMY 스택에도 없음), Dynamixel 내부 PID 가상 스프링/댐퍼(우리는 ROBOTIS `spring_actuator`+중력보상 컨트롤러가 담당,
E-1 ⑤ 드리프트 점검), baud/`latency_timer`(우리 300 Hz·`setup.sh udev`·`check_env` 이미 있음), π/2 배수
브루트포스 오프셋(우리 `--mode match` 는 연속값), 리더 EMA 필터(GELLO α=0.99 = 사실상 없음, 동일).

### 49.3 검증 (도메인 42, 다른 컨테이너 ROS 와 격리, Isaac 없이)

**T1 — 기록/가드 (가짜 팔로워: `/joint_states` 가 `/forward_position_controller/commands` 를 그대로 에코, 카메라 2개 검은 프레임)**

| 단계 | 결과 |
|---|---|
| engage 전 `start_episode` | **거부** `not ready: action topic '/omy_bridge/command_joint_states' has no UR arm joints (seen: nothing yet) ...` |
| `enable` → 명령 토픽 | 100.0 Hz |
| 2 s 기록 → `data.json` | 79 프레임, jpg 79, null 0, **max\|action[t] − state[t+1]\| = 0.0059 rad**(에코 팔로워라 action≈다음 state), `action.gripper` 채워짐 |
| 리더 샘플 1개를 166° 튀게 pub | `leader JUMPED 166.1 deg ... (> 28.6 deg) -- DISABLED`, status **`leader_jump`** 유지 → `enable` 다시 성공(게이트 통과) |
| `ros2 param set trim_tail_frames 5` 후 에피소드 2 | 104 프레임, jpg 104, meta `trim_tail_frames: 5` |
| `raw_to_lerobot.py` | `done -> t1/x (video: h264)` |

**T2 — `sync_to_leader` (set1 mock hardware + `ur16e_moveit.launch.py launch_rviz:=false` + `teleop_omy.launch.py virtual_leader:=true`)**

| 단계 | 결과 |
|---|---|
| 가상 리더가 팔로워(0°)에 homed → `reset_pose.py ready` 로 팔로워만 이동 | engage_error J2 +90°, J3 −152° … → `enable` **거부** |
| `/omy_bridge/sync_to_leader` | 6 s 뒤 `synced`, 팔로워 `[−0.3, −89.6, −0.1, −89.7, −0.2, 0.4]°` = 리더 0° 의 매핑(=offset) |
| `enable` → `disable` | `engaged` → `disabled` |
| 회귀: `/omy_bridge/sync` | `synced`, 랑데부 도달 |

산출물은 스크래치에서 지웠고 워크로드 프로세스 0. 실물 UR 에서 잴 BM(추종 지연·오버슈트, 그리퍼 반응)은
GELLO 의 `servoJ(500 Hz, lookahead 0.2 s, gain 100)` 를 기준점으로 `HARDWARE.md` 시점에 측정.

### 49.4 "동작한다"가 아니라 "이득인가" — 반증 시험 (사용자 요청)

**② 점프 가드 초안은 오탐이었다.** 절대각 0.5 rad/샘플 기준을 **벽시계** 리더(300 Hz 사인, 진폭 1 rad, 주기 2 s
= 최고 3.1 rad/s)로 돌리자 **정상 동작 중 `JUMPED 29.0 deg`** 로 끊겼다 — 리더 단독 발행률은 300 Hz(min 3 / max 4 ms)
로 멀쩡한데, 브리지+팔로워가 같이 돌 때 수신 측에서 0.16 s 갭이 생긴 것. (가상 리더 `virtual_omy_leader.py` 는 tick
누적 위상이라 SIGSTOP/CONT 해도 안 튀어서 이 오탐을 **못 보여준다** — 스톨 시험은 반드시 벽시계 리더로.)
→ **속도 기준으로 재설계**: `max_leader_speed` 20 rad/s, Δt = 수신시각 차(≥ 1 ms).

| 케이스 (벽시계 리더, `max_joint_speed` 10) | 절대각 초안 | 속도 기준 |
|---|---|---|
| 3.1 rad/s 정상 사인 | ✗ 오탐 29° | ✅ engaged 유지 |
| 0.3 s 발행 끊김(< 워치독 0.5 s) 후 재개 | (무효 시험) | ✅ engaged 유지(0.9 rad/0.3 s = 3 rad/s) |
| 글리치 1 샘플 +2 rad | — | ✅ `JUMPED 115.2 deg in 3 ms = 621 rad/s` → `leader_jump` |

**③ `sync_to_leader` 가 GELLO 보다 나은 점 = 충돌검사.** 리더를 UR `[0,−90,+170,−90,0,0]°` 에 해당하는 자세로 고정
발행(mock + MoveIt, `launch_rviz:=false`) → MoveIt `Found a contact between 'wrist_2_link' and 'upper_arm_link'`
→ `error_code=99999` → `sync_failed`, 팔로워 `[0,−90,0,−90,0,0]°` 그대로. GELLO 의 25 스텝 직선 보간은 그대로 갔을 자세.

**① 명령 action 이 `next_state` 보다 정보가 많은가 — sim 에서는 아니다.** Isaac set2 headless + 실제 제어 스택 +
가상 리더(0.25 rad, 8 s) 로 `/omy_bridge/command_joint_states` 와 `/joint_states` 를 30 Hz 로 20 s(602 샘플) 동시 샘플링:

| k (action[t] vs state[t+k]) | 0 | **1** | 2 | 3 | 5 | 10 |
|---|---|---|---|---|---|---|
| 평균 최대관절 오차 [°] | 0.122 | **0.028** | 0.160 | 0.295 | 0.574 | 1.272 |

프레임당 이동 0.139° 이므로 명령은 state 를 **정확히 1 프레임(33 ms)** 앞서고 잔차는 이동량의 20%. 즉 topic_based
sim 에서는 **리더 action ≈ next_state**. 둘이 갈라지는 조건(실물 servoj 지연, 접촉으로 팔이 막힘)은 실물에서만 잴 수
있다 — `docs/gello_comparison.md` §5 ⑤. 그러니 ① 은 "결함 수정" 이지 sim 에서 입증된 이득이 아니다.

**④ `trim_tail_frames` 는 미검증.** 이 컨테이너엔 사람 데모 raw 가 없고(스크립트 수집뿐) 가상 리더 꼬리는 무의미.
`scripts/il_tail_stats.py`(신규, `install(PROGRAMS)`) 로 꼬리 정지 프레임을 먼저 재는 절차를 `CHECKLIST.md` H 에 넣고
기본값 0 유지. 판정 전체는 **`docs/gello_comparison.md`**(GELLO 대조표·채택/미채택·실물 BM 6항목).

### 49.5 속도 가드 2차 오탐(sim 시간) · "팔로워가 느리다"의 원인 분해 · Isaac headless 는 실시간의 3.8배 (2026-09-17)

**가드 오탐 2**: Isaac(`use_sim_time:=true`) 에서 engage 직후 `leader JUMPED 1.2 deg in 1 ms = 20 rad/s` 로 끊김. 노드 시계가
`/clock` 이라 같은 틱에 도착한 두 샘플의 Δt 가 0(→ 1 ms 바닥) 이고, sim 시간 리더는 300 Hz 샘플을 **묶음으로** 보낸다.
→ Δt 는 **벽시계**(`time.monotonic`) 로, 그리고 스텝 자체가 `min_leader_jump`(0.1 rad) 를 넘어야 판정. 벽시계 리더 회귀
(fast/stall/glitch) 0/0/1 유지, Isaac 세 케이스 전부 engaged 유지.

**사용자 피드백 "실물에서 L100 보다 UR16e 가 느리게 추종"** — 속도를 정하는 곳을 분해했다(`HARDWARE.md` 4-B ⑤ 표).
브리지 `max_joint_speed`(기본 1.0, 첫 연결 0.3 권장) 가 1순위, UR16e 관절 한계 120°/s(2.09 rad/s) 가 물리 상한, 드라이버
servoj(gain 2000 / lookahead 0.03, `ur.ros2_control.xacro` 고정) 는 원인 아님. 브리지에 **`slew capped N%`** 경고(2 s 창,
30% 초과 시 10 s 스로틀) 를 넣어 실물에서 어느 것이 병목인지 바로 보이게 했다(벽시계 리더 3.1 rad/s, cap 0.3 → `100%` 경고,
cap 5.0 → 0회).

Isaac 실측(리더 0.5 rad 사인 3 s, joints 0·4, 세 캡):

| cap [rad/s] | 리더→state 최적 지연 | 평균 오차 | 최대 오차 |
|---|---|---|---|
| 0.3 | 3 프레임(100 ms) | 1.14° | 3.73° |
| 1.0 | 1 프레임(33 ms) | 0.11° | 1.06° |
| 2.0 / 3.0 | 1 프레임 | 0.27° / 0.10° | 1.37° / 1.11° |

**★ 함정(새로 확인)**: headless Isaac 은 이 컨테이너에서 **실시간의 3.78배**로 돈다(`/clock` 17.3 s / 벽시계 4.6 s). sim 시간
노드(리더·브리지) 와 벽시계 프로브가 섞이면 rad/s 절대값은 무의미하고, 300 Hz 리더 타이머도 못 따라간다(리더 peak 가 wall 기준
0.52 rad/s 로 관측). 위 표는 **상대 비교만** 유효. 절대 속도가 중요한 시험은 실물 또는 실시간 제한이 있는 Isaac 으로.

**하네스 함정 재발 3회**: 패턴 kill 스크립트(`killmine.sh`)를 그 패턴 문자열이 들어 있는 heredoc 과 **같은 `bash -c`** 에서 부르면
자기 셸을 죽인다(rc 144). 편집(heredoc)과 실행(스크립트 경로만)은 반드시 별도 호출로.

### 49.6 실물 첫 연결(다른 PC): ROBOTIS 리더 컨트롤러가 UR16e 의 `/joint_states` 를 받는다 — 래퍼 런치로 해결 (2026-09-17)

사용자가 새 PC 컨테이너(`bootstrap.sh --fresh`)에 UR16e(192.168.0.10)·L100 을 붙이고 T1(리더)·T2(UR)·T3(MoveIt)·T4(브리지)를
띄우자 T1 에 `ERROR Joint name 'joint1' not found in the first joint state message`. 원인은
`om_gravity_compensation_controller/src/gravity_compensation_controller.cpp:220` — **절대 토픽 `/joint_states`** 구독
(`follower_joint_state_sub_`, OMY-F3M 팔로워의 상태를 받아 `/collision_flag` 가 참일 때 리더를 팔로워 쪽으로 당기는 용도).
같은 도메인의 UR16e `/joint_states`(`shoulder_pan_joint`…)가 먼저 오니 인덱스 초기화가 실패하고 **UR 메시지마다** ERROR.
`use_self_collision_avoidance:=false` 라 그 데이터는 읽히지 않아 **중력보상·`/leader/joint_states` 300 Hz 는 정상** — 로그 스팸.
지금까지 안 보인 이유: sim 은 `virtual_omy_leader.py`(ROBOTIS 스택 없음), §22 의 mock 리더 시험은 UR 스택 없이 했다.

**해결**: vcs 소스는 안 건드리고 `ur_bringup/launch/common/omy_leader.launch.py` 신설 — ROBOTIS 런치를 include 하고
`SetRemap('/joint_states' → '/leader/joint_states')`. 절대 규칙이라 상대 이름(`joint_states` → `/leader/joint_states`)은 영향 없음.
mock 재현(가짜 UR `/joint_states` 50 Hz + 리더 mock, 도메인 42):

| 런치 | 4 s 동안 ERROR | `Joint index mapping initialized` | `/leader/joint_states` |
|---|---|---|---|
| `open_manipulator_bringup omy_l100_leader_ai.launch.py` | **287** | 0 | 300.0 Hz |
| `ur_bringup omy_leader.launch.py` | **0** | 1 | 300.0 Hz |

사용 명령을 전부 래퍼로 교체(`CHECKLIST` E-1, `HARDWARE` 4-B ②, `README`, `SETUP` §2-D, `setup.sh udev` 안내, 브리지·캘리브 독스트링,
`teleop_omy.launch.py` 헤더). 같은 날 함께 고친 것: `setup.sh udev`/`check_env.sh` 가 컨테이너를 감지해 호스트 절차를 안내
(컨테이너 안 udev 는 무효, `/dev/ttyUSB0` 권한은 컨테이너 재시작 전까지 `sudo chmod 666`), 실물 지연 프로브 `harness/teleop_lag_probe.py`.

**같은 날 실물에서 "속도 상한 1.0→2.0 이 체감 변화 없음"** — 브리지가 병목이 아니라는 신호. 펜던트 안전 설정을 읽으니 관절 속도는
130/190°/s(정상)인데 **Robot Limits 의 Tool Speed·Elbow Speed 가 160 mm/s**(Normal 기본 5000, Reduced 프리셋 750). 0.9 m 팔이면
base 약 10°/s 에서 이미 한계라 브리지 값과 무관하게 느렸다. `HARDWARE.md` 4-B ⑤ 표 3행에 사례로 기록. 값 결정은 안전 평가 사항이라
문서는 단계(750 → 1500 mm/s)만 제안한다. 또 `headless_mode:=true` 없이는 펜던트 프로그램이 없어 plan 만 되고 execute 가 실패했다
(`robot_program_running: false`) — `HARDWARE.md` §1 의 안내대로.

### 49.7 실물 리더에서 관절 부호 반전 관측 → 캘리브 도구 보강 + 도구의 잠복 결함 1건 (2026-09-17)

사용자가 실물 L100 텔레옵에서 **몇 관절이 UR16e 와 반대로 움직인다**고 관측. `sign [1,1,1,1,−1,1]` 은 두 URDF 의 축 방향에서
유도한 값이라 **엔코더 증가 방향**은 보장하지 못한다 — GELLO 도 조립마다 `joint_signs` 를 잰다(UR (1,1,−1,1,1,1), xArm
(1,−1,1,1,1,−1,1), YAM (1,−1,−1,−1,1,1)). 절차는 **부호 → 오프셋** 순(부호가 바뀌면 그 관절 오프셋도 바뀜), `CHECKLIST.md` E-2 에 명시.

`omy_leader_calib.py` 보강:
- **`--mode check` 에 부호 점검**: rest pose 공칭 `[0,−90,152,−62,90,0]°` 와 실측 부호 비교(J2~J5; J1/J6 은 공칭 0 이라 engage 관찰로).
- **브리지의 현재 `sign`/`offset` 을 서비스로 가져와 사용**(`/omy_to_ur16e/get_parameters`), 없을 때만 내장 기본값.
- **잠복 결함**: 도구의 `DEF_OFFSET` 이 `[0,−90,0,0,0,0]` 로 남아 있었다(§47 에서 브리지는 J4 −90° 로 바뀜). `--offset` 없이
  `verify` 하면 J4 가 **90° 초과**로 나와 "engage 거부" 를 잘못 예고했을 것 — 가상 리더로 재현(잔차 90.00°) 후 수정(live 값 사용 시 0.00°).

### 49.8 실물 L100 실측으로 매핑·랑데부 확정 — 거울 가지 채택, J5 부호 정정, `ready` 재정의 (2026-09-17, sim UR16e + 실물 L100)

사용자가 다른 PC 에서 실물 L100 ↔ **sim** UR16e 로 캘리브. 결과만 정리한다(과정: 방향 시험 → 거울 가지 발견 → 손목 판정).

**실측 rest**(손 떼도 서 있는 자세): `[−1.14, −88.42, 152.03, −69.61, 87.45, 1.58]°` — ROBOTIS 공칭 `ready` `[0,−90,152,−62,90,0]` 과
J4 7.6° 외 일치 → 엔코더 영점은 URDF 그대로, 문제는 **회전 방향**뿐.

**방향 시험(브리지 engaged, 관절 하나씩)**: 기존 `sign [1,1,1,1,−1,1]` 에서 J1~J4 같음, **J5·J6 반대**. 즉 §21 의 "J5 는 URDF 축이
+Z/−Z 라 반전" 은 **오진** — URDF 축 방향은 엔코더 증가 방향을 말해 주지 않는다(GELLO 도 조립마다 `joint_signs` 를 잰다).

**거울 가지**: 사용자가 RViz 로 실물 L100 과 같은 모양을 찾자 `[180, 0, −153, −20, 90, 0]°`. 이는 §47 랑데부 `[0,−180,152,−152,−90,0]`
의 **거울 해**(`q1+180, −180−q2, −q3, −q4, …`: 실루엣 동일, 손목 오프셋이 반대편)에 손목을 180° 뒤집은 것. L100 의 측면 오프셋이
UR16e 와 반대 부호(§21)라 이쪽이 눈에 같아 보인다. 후보 B(FK 손목)/B2(눈 손목)를 그리퍼 방향으로 판정 → **B2**, 그 위에서 J5 만 반전.

| | 최종값 (런치 기본값·`virtual_omy_leader.py`·`omy_leader_calib.py`) |
|---|---|
| sign | `[1, −1, −1, −1, +1, −1]` |
| offset | `[180, −90, 0, −90, 0, 0]°` |
| rendezvous = `reset_pose.py ready` = `pick_place_demo.py READY` | `[178.86, −1.58, −152.03, −20.39, 87.45, −1.58]°` = `[3.1217, −0.0276, −2.6534, −0.3559, 1.5263, −0.0276]` rad |

실물 확인: **6관절 방향 모두 같음, 그리퍼 방향(정면·아래) 일치, `max_joint_speed` 1.0 까지 이상 없음.** J4 의 7.6° 는 "영점 어긋남"
인지 "그렇게 쉬는 것" 인지 한 자세로는 구분 불가 → FK 유도 오프셋 −90° 유지, 랑데부를 실측 rest 의 매핑값으로. 이후 손목이 항상
7° 덜 접혀 보이면 J4 오프셋을 −82.4° 로.

**`ready` 계보**: `ready`(2026-09-17, 위) / `ready_v2`(2026-09-16 URDF 가지 §47, sim 스모크만) / `ready_v1`(그 전 데이터셋 시작 자세).
하네스 `START_POSE` 기본은 여전히 `ready_v1`(기존 체크포인트용), 새 수집은 `ready`.

**sim 회귀(도메인 42)**: ① 가짜 리더 teleop(세트2 headless, `reset_pose ready` → 가상 리더 homed → engage): 25 s 추종 **최대 0.214°,
평균 0.144°**(§48.6-B 0.256° 와 동급). ② 세트3 `pick_place_demo cycles:=1` 새 READY 에서 **1/1 SUCCESS**(part 0.001 m).
문서: `CLAUDE.md`·`CHECKLIST` E-3 표·`HARDWARE` 4-B ③·`plan_il_vla` §3.5(J5 정정, 할 일 표 2행)·브리지/`reset_pose`/데모 독스트링.
스크래치의 추종 측정기·`harness/teleop_lag_probe.py` 기본 map 도 갱신(옛 map 이면 332° 오차로 보인다 — 측정기 map 을 먼저 의심할 것).
