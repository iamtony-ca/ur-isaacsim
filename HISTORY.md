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
| **OMY leader / GELLO** | **관절 직결** → `forward_position_controller` (IK 없음) | **leader 관절값**(ALOHA 관례) |
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
3. **`omy_to_ur16e` 브리지 노드**(관절 1:1, IK 없음) 4. 속도상한+deadman 5. 기록은 `--action-source topic` 그대로

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
  **`il_recorder.py --action-source topic` 이 무수정으로 붙는다**(§18 장치무관 설계의 회수).
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
코드/스키마는 그대로다. 기록은 `il_recorder.py --action-source topic --action-topic /leader/joint_states`.

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
