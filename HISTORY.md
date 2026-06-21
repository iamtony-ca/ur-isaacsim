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

> 단계별 재현 매뉴얼 전체는 **[`src/SETUP.md`](src/SETUP.md)** 참고 (apt 목록, vcs, 검증, 트러블슈팅 포함).
> 개념 Q&A (런치 구조, nav2 와의 비교 등)는 **[`src/qna.md`](src/qna.md)**.

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
| world ESDF(nvblox) 연동 | 미구현 — 다음 단계(`read_esdf_world`) |
