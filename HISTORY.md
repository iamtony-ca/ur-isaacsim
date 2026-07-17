# HISTORY — UR16e ROS 2 + Isaac Sim 구축 이력 (누적 기록)

> 📌 이 문서는 **누적 이력/검증 로그/디버깅 교훈**을 모은 기록이다(날짜별 검증 결과, 함정, 근거 포함).
> **현재 상태 기준 요약은 [`README.md`](README.md)**, 실물 연결 절차는 [`HARDWARE.md`](HARDWARE.md).
> 새 변경/검증은 계속 이 문서에 덧붙인다. (이하 본문은 초기 구축 시점부터의 기록)
>
> ⚠️ **워크스페이스 이전(2026-07)**: §1~§13 은 원본 `ur_ws`(`/isaac-sim/ur_ws`, Isaac Sim **5.1.0**)에서의
> 이력이라 그 경로/버전 표기가 **이력 그대로** 남아 있다. 세트4(dual-tool)부터는 이 저장소를 복제한
> `ur_dualtool_ws`(`/isaac-sim/volume/ur_dualtool_ws`, Isaac Sim **6.0.1-rc.7**)에서 진행하며, 세트1~3
> 제어 스택도 6.0.1 에서 재검증됨(§14). 재현 시 옛 경로는 새 경로로 치환.

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

## 14. 세트 4 — dual-tool EOAT: STEP(CAD)→Isaac Sim 자동화 파이프라인 — 2026-07-16

`ur_ws` 를 `ur_dualtool_ws`(`/isaac-sim/volume/ur_dualtool_ws`, Isaac Sim **6.0.1-rc.7**)로 복제하고,
기구설계팀 STEP 을 받으면 **config 파라미터만으로** UR16e EOAT 조립을 sim(+MoveIt 충돌모델)으로 저작하는
재현 파이프라인을 구축. 대상 조립: `UR16e → 기구댐퍼 → OnRobot Dual Quick Changer →`
`[포트 A] HEX F/T → 2FG14 그리퍼` / `[포트 B] 어댑터 → ESTIC 스크류드라이버 (+Copick3D ×1)`.
개념/절차는 [`STEP_TO_SIM.md`](STEP_TO_SIM.md), config 스키마·미완 갭은 `ur_bringup/isaac/common/eoat/README.md`,
재현은 [`SETUP.md`](SETUP.md) §5-D.

### 아키텍처 — single source of truth
`eoat_dualtool.yaml`(부품+물성+조인트+배치) → **`eoat_model.py`**(순수-python, Isaac 무의존: config+정규화
사이드카를 하나의 link/joint 그래프로, mass/inertia/collision/frame 확정) → 여러 emitter 가 **서로 일치하는**
산출물 생성. 함정 #7(URDF↔USD 불일치)이 구조적으로 불가능.
- `build_eoat_usd.py` → `assets/ur16e_dualtool.usd` (Isaac 물리 아티큘레이션)
- `build_eoat_urdf.py` → `assets/ur16e_dualtool.urdf` (MoveIt/cuMotion 충돌모델)
- `build_eoat_moveit.py` → `urdf/ur16e_dualtool/…_eoat_macro.xacro` + `srdf/common/ur16e_dualtool.srdf.xacro`
- `build_ur16e_dualtool.py` → `assets/ur16e_dualtool_full.usd` (UR16e+EOAT 단일 아티큘레이션)

### 검증 결과 (2026-07-16, sim / 순수-python)
| 항목 | 결과 |
|---|---|
| STEP(폴더째)→USD (HOOPS 백엔드, 미터 네이티브, bbox 자동검증) 8부품 | ✅ (치수 실측 대조 통과, `STEP_TO_SIM` 표) |
| 그래프 모델: 12 링크 / mass·inertia·collision 확정, 3배치방식(tcp_pose>mount>자동스택) | ✅ |
| Isaac play — 2FG14 평행핑거가 드라이브 타깃에 **대칭 도달**(단일 DOF + mimic) | ✅ (self-collision OFF 필수) |
| 결합 `ur16e_dualtool_full.usd`: 단일 아티큘레이션 `/UR16e/root_joint`, 8-DOF, 19 body | ✅ (구조) |
| 결합 URDF `xacro …_sim.urdf.xacro \| check_urdf` = 24링크/30조인트 | ✅ (name="ur16e") |
| SRDF: ur_manipulator+gripper 그룹, 121 disable_collisions, **EOAT-vs-팔몸통 ENABLE 유지** | ✅ (충돌발굴 계약) |
| moveit_py 충돌검출: 정상 포즈 충돌無 / 접힘 포즈 실제 충돌쌍 검출 | ✅ |
| fidelity 파라미터(friction/restitution/solver iters/joint armature·friction) config 노출+DEFAULT 주석 | ✅ |
| **tool0(UR TCP)-기준 상대포즈 입력**: `tcp_pose:{xyz,rpy}` → parent-local mount 자동 역산(재합성 일치) | ✅ |

### 핵심 교훈 / 함정 (이번에 확정)
1. **STEP→USD 는 `omni.kit.converter.hoops_core` 만.** `omni.kit.converter.cad` 켜면 ODA(`libTD_DbCore.so`)
   심볼충돌로 프로세스 abort. 미터 네이티브(`dMetersPerUnit=0.001`) 아니면 reference 시 1000배 괴물(함정 #7 유형).
2. **평행 그리퍼(2FG14) = 단일 액추에이터 1-DOF + mimic** (독립 핑거 아님). finger_left 구동, finger_right mimic.
   ROS 브리지용 Isaac 자산은 `build_eoat_usd.py --physx-mimic` 로 재빌드.
3. **Isaac play 시 self-collision OFF 필수** — placeholder 핑거 박스가 몸체 메시에 박혀 드라이브가 걸림
   (finger_left 0, finger_right limit 에서 고착). `enabledSelfCollisions=False` 로 해결.
4. **copick USD 는 FOV 원뿔 포함(792mm bbox)** → density-box 시 145kg 괴물. `physics.mass`/`size`/`boundingCube`
   override 로 회피(실측 경로 시연). `_box_for` 가 link.size(원뿔 bbox) 대신 physics.size override 를 쓰도록 수정.
5. **rpy 규약 통일** — ROS(roll-X/pitch-Y/yaw-Z, R=Rz·Ry·Rx)로 eoat_model·build_eoat_usd(`_quat_from_rpy`=`_rot`)
   ·build_eoat_urdf 전부 일치 → 다축 회전에서도 USD==URDF.
6. **xacro 주석에 `--`(이중 하이픈) 금지** — XML 주석 규칙 위반으로 파싱 실패.
7. **결합 URDF `<robot name>` 은 "ur16e" 유지** — ur_moveit SRDF 이름 매칭. `$(find ur_bringup)` 는 colcon
   빌드+source 후 해석(빠른 파싱검증은 abs 경로 sed 치환).
8. **Isaac ROS2 브리지 기동 실패**(`libament_index_cpp.so: cannot open`) → Isaac 실행 전 시스템 ROS
   (`/opt/ros/jazzy/setup.bash`) source + `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 필요.
9. **시스템 python3 없음** — 순수-python 파이프라인도 `/isaac-sim/python.sh`(yaml/numpy 포함)로 실행. kit python 은 pyyaml 없음.

### GUI 튜닝 ↔ config 왕복 (rough 숫자 확정 도구)
부품별 상대자세(mount/tcp_pose/rpy)는 **전부 rough placeholder** — 개별 STEP 이 서로 다른 CAD 프레임으로
와서 부품 간 조립관계가 없기 때문. 확정 경로 2가지:
- `edit_asset.py <usd>` 로 편집가능 GUI 오픈 → 링크 프림 드래그로 맞물림 → Ctrl+S → `extract_poses.py [--tcp]`
  로 config 값(parent mount / tool0-기준 tcp_pose) 추출.
- **실 데이터**: 기구팀에 "tool0 기준 각 부품 원점 xyz[m]+rpy[deg,ROS]" 요청 → `chain[].tcp_pose:` 에 입력.

### 미완 / 다음 작업 (2026-07-17 이어서)
- **부품 상대자세 정렬**: dual_quick_changer 방향·copick 어댑터 부착(스크류드라이버 볼팅지점 지향)·전체
  translation/orientation 이 아직 안 맞음 → GUI 드래그 또는 기구팀 tool0-기준 데이터로 확정 후 `extract_poses`.
- density-box 과대질량(실 관성 오면 override), 핑거 기하 분리(분해 STEP or blob 분할), copick FOV 원뿔 제거
  재변환, 세트4 controllers.yaml/launch 는 생성됨(전체 MoveIt run-verify 대기), cuMotion XRDF 미러
  (`cumotion/gen_xrdf.py`), ROS 브리지용 asset `--physx-mimic` 재빌드. 상세는 `isaac/common/eoat/README.md §미완 갭`.


## 15. 세트 4 — 정렬 확정 · live 검증 · 정적 장애물 파이프라인 · 정합성 수정 — 2026-07-17

§14 파이프라인 위에서 부품 정렬을 GUI 로 확정하고, live(Isaac→제어→MoveIt) 로 충돌탐색+계획+실행을
end-to-end 검증했으며, 사전제공 CAD 정적 장애물 서브파이프라인을 신규 구축했다. 마지막에 Isaac↔RViz
정합성 버그 2건(EOAT 90° 회전, URDF box vs USD mesh)을 근본원인까지 파고 수정했다.

### 정렬 확정 + 조립 안정화
- **GUI 러프 정렬**: `edit_asset.py`(편집가능 오픈) → 링크 드래그 → Ctrl+S → `extract_poses.py --tcp` 로
  9개 EOAT 링크의 tool0-기준 `tcp_pose` 확정 → `eoat_dualtool.yaml`. 사용자가 눈으로 검수하며 반복.
- **copick FOV 원뿔 제거**: 기존 copick USD 는 FOV 원뿔(792mm) 포함 → 정규화 bbox 중심(=링크원점)이 몸체
  밖으로 밀려 GUI 정렬 불가·콜라이더 어긋남. 기구팀 폴더의 **본체-only STEP**(`CoPick3D 150S.STEP`)을 재변환
  (실측 142×69×184mm) → 원점=본체. `copick3d_150s_FOV.usd.bak` 백업.
- **2FG14 핑거 축 정정**: prismatic 축이 X 였으나 정규화 bbox Y=119mm≈데이터시트 폭 115mm → **열림 축은 Y**.
  `axis:[0,±1,0]`, 핑거 박스 Y로 얇은 판. (imgs/fingers_2026_07_17.png)
- **`world_fixed` 잠재버그 수정**: `build_ur16e_dualtool.py` 가 `out.RemovePrim()` 으로 EOAT standalone base
  joint 를 지우려 했으나 **reference 로 온 prim 은 RemovePrim 이 조용히 무효** → `world_fixed` 가 살아남아
  EOAT tool0 를 월드에 고정, 팔 articulation 과 충돌(발산). `SetActive(False)`+post-flatten purge 로 해결(20→19 joint).
- **HOME 포즈 baking**: 결합 USD 에 arm-up(shoulder_lift=-90°)을 기본 관절각으로 baking(DriveAPI+JointStateAPI,
  도 단위) → 어떤 로더로 열어도 바닥 충돌 없이 시작. `--no-home` opt-out. verify: arm_held=True.

### live 3-터미널 run-verify (1차 목표 end-to-end) — PASS
Isaac(headless/GUI, `--asset-path ur16e_dualtool_full.usd`, ROS 브리지=system ROS 소싱+RMW_fastrtps) →
`/isaac_joint_states` 8관절·NaN없음(0.2.1 topic_based) → `ur16e_dualtool.launch.py`(컨트롤러 3개 active) →
`ur16e_dualtool_moveit.launch.py`("You can start planning now"). **충돌탐색**(`/check_state_validity`): home valid,
접은 포즈=팔 self-collision, 특정 포즈에서 **copick·screwdriver ↔ base_link**(EOAT-vs-arm 계약 성립).
**plan+execute**(`moveit_plan_execute_demo.py`) error_code=1, 0.009rad 도달 → 루프 폐쇄.

### 그리퍼 mimic 미러(로더)
PhysxMimicJointAPI 는 Isaac 6.0.1 에서 적용은 되나 실제 커플링 안 함(finger_right 안 따라옴). URDF `<mimic>`
이 MoveIt 단일-DOF 는 보장하나 Isaac 물리는 아님 → `ur16e_isaac_ros2.py` 메인루프에 **finger_right←finger_left
미러**(가드: 양쪽 핑거 조인트 존재 시만; 세트1~3 무영향). live 확인: 양쪽 대칭 파지.

### 정적 장애물 충돌 파이프라인 (신규 — `isaac/common/obstacles/`)
사전제공 CAD(테이블·지그, **동적/perception 아님**)를 **단일 config** 로 MoveIt planning scene(sim+real 공용
충돌계층)+Isaac scene 동시 구동. `obstacles.yaml` / `prepare_obstacles.py`(STEP·STL→USD + 충돌 OBJ 3종:
mesh·convexDecomposition(CoACD)·convex) / `load_obstacles_moveit.py`(CollisionObject+ACM, `--level` 전역전환) /
`collision_report.py`(충돌쌍+접촉점 xyz+깊이) / `approach_to_collision.py`(충돌 직전정지·`--to-contact`·`--loop`) /
`ur16e_isaac_ros2.py --obstacles`. 검증: 테이블 box+CAD convex 충돌검출, ACM(base↔table) 오탐방지,
계획 자동회피(plan+execute SUCCESS; 충돌 시작자세=error_code -10). 형상 fidelity 기본 `mesh`(비과보수),
Isaac 물리는 native convexDecomposition. **핵심 원리**: 계획된 궤적은 백엔드무관 → sim=real 동일, 충돌모델만
실제와 맞추면 sim 검증이 실물로 전이. 상세 `isaac/common/obstacles/README.md`.

### 트러블슈팅 로그 (증상 → 에러/로그 → 진단 → 해결) — 향후 자가진단용

**T1. 결합본에서 팔이 발산 / `world_fixed` 가 안 지워짐**
- 증상: `verify_articulation.py ur16e_dualtool_full.usd /World/eoat/root_joint` 에서 팔이 날뛰며
  shoulder_lift q → -6~-28 rad, `arm_held=False`, `fingers reached=False`.
- 로그: `[Warning] [omni.physx.plugin] PhysicsUSD: CreateJoint - found a joint with disjointed body
  transforms, the simulation will most likely snap objects together:
  /World/eoat/wrist_3_link/eoat/tool0/world_fixed`
- 진단: `build_ur16e_dualtool.py` 로그엔 `stripped ... world_fixed` 라고 찍히는데 실제론 남아있음. 확인:
  ```python
  s=Usd.Stage.Open("ur16e_dualtool_full.usd")
  [str(p.GetPath()) for p in s.Traverse() if p.GetName()=="world_fixed"]
  #  -> ['/UR16e/wrist_3_link/eoat/tool0/world_fixed']   (살아있음!)
  sum(1 for p in s.Traverse() if p.IsA(UsdPhysics.Joint))   # -> 20 (정상 19)
  ```
  원인: `out.RemovePrim(p)` 는 **reference 로 들어온 prim 의 spec** 을 못 지움(로컬 레이어에 없음) → 조용히 무효.
  살아남은 world_fixed 가 EOAT tool0 를 월드에 고정 → 팔 articulation 과 충돌 → 스냅/발산.
- 해결: strip 루프에서 `prim.SetActive(False)` + `Flatten()` 후 `flat.RemovePrim()` purge. → Joint 20→19,
  verify `arm_held=True`. **교훈: reference prim 제거는 deactivate + post-flatten purge**.

**T2. EOAT 가 Isaac 과 RViz 에서 90° 다름 (★ 근본원인)**
- 증상: EOAT 스템(damper/dual_quick_changer)이 Isaac 뷰포트와 RViz 에서 눈으로 90° 회전 차이.
- **오진(빠지기 쉬운 함정)**: 정적 USD Xform 측정 → `wrist_3→dual_quick_changer` 가 양쪽 -90.1°X 로 "일치"처럼
  보임. **틀림!** EOAT 는 wrist_3 밑에 identity 로 authored 되고, `ATTACH_LOCAL_ROT(-90°Z)` 는 fixed joint 의
  localRot0 에만 있어 **정적 Xform(권한 있는 authored 값)엔 안 나타남**.
- 올바른 진단: **물리 재생 후** 상대회전 측정(headless):
  ```python
  sim.reset(); [sim.step(render=False) for _ in range(30)]   # fixed joint 가 바디를 스냅
  # wrist_3_link world^-1 * damper world -> axis (0,0,-1) angle 90  == -90°Z  (Isaac 실제)
  ```
  ROS(RViz)는 URDF FK(물리 스냅 없음): `ros2 run tf2_ros tf2_echo wrist_3_link tool0` → identity,
  `tf2_echo tool0 damper` → identity → ROS `wrist_3→damper` = identity.
  **차이 = -90°Z** = `build_ur16e_dualtool.py ATTACH_LOCAL_ROT_WXYZ=(0.7071,0,0,-0.7071)` (2F-85 빌드에서 상속).
  결정적 단서였던 것: T1 의 "disjointed body transforms" 경고 = 바로 이 -90°Z 스냅.
- 해결: `ATTACH_LOCAL_ROT_WXYZ=(1,0,0,0)` 항등. 근거: 이 UR16e URDF 는 TF `wrist_3→tool0`=identity 이고
  EOAT(damper)는 tool0 에 붙으므로 마운트 회전은 항등이어야 함. 재빌드 후 재측정:
  `wrist_3→damper`=identity, `wrist_3→dual_quick_changer`=-90.1°X → **둘 다 ROS TF 와 일치**, 스냅경고도 소멸.
- **교훈: Isaac↔URDF 방향 비교는 정적 Xform 이 아니라 반드시 물리-재생(sim.reset+step) pose 로**. 다른 UR
  모델로 바꿔 `wrist_3→tool0` 에 실회전이 생기면 ATTACH 를 그 값으로(코드 주석에 명시).

**T3. RViz 는 박스, Isaac 은 실제 부품 (형상 fidelity 갭, T2 와 별개)**
- 증상: 프레임은 맞는데 RViz 의 EOAT 가 박스로 보임.
- 진단: `sed -n '/dual_quick_changer/,/\/link/p' ..._eoat_macro.xacro | grep geometry`
  → `<box size="0.128373 0.09705 0.071"/>`. `build_eoat_moveit.py _box_for` 가 bbox box 로 냄(USD 는 실제 메시).
- 해결: 신규 `export_eoat_meshes.py` 로 정규화 USD → `meshes/eoat/<id>.obj`(visual) + `<id>_col.obj`(convex),
  `build_eoat_moveit.py` 는 메시 있으면 `<mesh package://ur_bringup/meshes/eoat/...>`(identity origin), 없으면 box.
  install 반영: `meshes/eoat` 를 install 에 symlink 하거나 `colcon build --packages-select ur_bringup`.
  부수효과: MoveIt 충돌도 실제 convex-hull(box 과보수 해소).

**T4. 메시 decimation 이 무효 (파일 50MB)**
- 증상: `simplify_quadric_decimation` 가 face 수 그대로(439681→439681), 예외도 없음. OBJ 22MB.
- 로그/원인: (a) `ModuleNotFoundError: No module named 'fast_simplification'` (내부에서 삼킴),
  (b) trimesh 4.11 시그니처 `(self, percent=None, face_count=None, ...)` — positional 인자가 `percent` 로 감.
- 해결: `pip install fast-simplification` + `.simplify_quadric_decimation(face_count=N)`(키워드). 50MB→5MB.

**T5. HOOPS/asset_converter USD 에서 메시 추출 0개**
- 증상: `usd_world_mesh` verts 0 → `scipy ConvexHull` `IndexError: tuple index out of range`.
  `inspect_usd.py` 도 `# total meshes: 0`.
- 진단: 기하가 **instanceable prim** 밑에 있어 `stage.Traverse()` 가 인스턴스 내부로 안 들어감.
- 해결: `stage.Traverse(Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate))`.

**T6. 2FG14 핑거가 90° 틀린 축으로 움직임**
- 증상: 핑거가 실제 열림방향과 90° 다른(깊이) 축으로 슬라이드(imgs/fingers_2026_07_17.png).
- 진단: 정규화 bbox = 75(X)×119(Y)×166(Z)mm; 데이터시트 2FG14 폭=115mm≈**Y=119** → 열림축=Y. config 는 axis=X.
- 해결: `axis:[0,±1,0]`, origin Y, 핑거 박스 Y로 얇은 판. (레퍼런스 모델 축은 그 모델 프레임 기준이라 그대로 이식 X)

**T7. copick 카메라가 붕 떠 보임 / extract 값이 0.44m 어긋남**
- 증상: `extract_poses.py --tcp` 가 copick 을 부모에서 0.44m 떨어진 것으로 보고(실제 몸체는 붙어있음).
- 진단: copick USD 에 FOV 원뿔(792mm) 포함 → `center_xy` 정규화가 **링크원점을 원뿔+몸체 bbox 중심**(몸체 밖)에
  둠. extract 는 링크원점 좌표를 보고 → 몸체와 어긋나 보임. (bbox: `normalized/copick3d_150s.json` size[0]=0.792)
- 해결: 기구팀 **본체-only STEP** 재변환(원점=몸체). vendor 가 두 STEP 을 주면 원뿔 없는 걸로.

**T8. 팔 end-effector 가 중력으로 처짐**
- 증상: 뷰어에서 EE 가 아래로 처짐, 바닥 근접.
- 진단: 뷰어(play_full.py)가 매 스텝 팔을 **현재(이미 처진) 위치로 재명령** → 처짐 누적(래칫). + Isaac 조인트는
  수동 PD(중력보상 없음). 실물은 서보 중력보상+브레이크.
- 해결: home 고정 목표로 명령(=ros2_control 이 하는 일). `play_ground.py --headless` 측정: sag=0.0°,
  최저 바디 0.985m, FLOOR CONTACT=NO. **제대로 pose 유지는 ros2_control 이 위치명령 스트리밍(cuMotion 불필요)**.

**T9. Isaac 재시작 후 제어 스택 timeout / joint_states stale**
- 증상: Isaac 재시작 후 `ros2 action send_goal .../follow_joint_trajectory` 가 timeout/Terminated,
  `ros2 topic echo /joint_states` 값이 stale(0 또는 이전 값).
- 진단: 함정 #2/#4 — Isaac 재시작 시 `/clock`·`/isaac_joint_states` 끊김 → topic_based 하드웨어/컨트롤러 재동기화
  실패. (초기 all-zeros 로 읽혀 approach_to_collision 이 "START in collision" 오판하기도)
- 해결: **Isaac 재시작 시 control+MoveIt 스택도 함께 재시작**(순서: Isaac 안정화 → control → MoveIt).

**T10. 워크로드 종료 시 `pkill -f` 가 launch 를 못 잡음**
- 증상: `pkill -f "ros2 launch ur_bringup"` 후에도 프로세스 생존.
- 원인: 실제 커맨드가 `bash -c '... eval ...ros2 launch...'` 래퍼라 패턴이 자식 python 만 매치, 래퍼는 잔존/재spawn.
- 해결: `pgrep -af` 로 PID 확인 후 `kill -9 <pid...>` 직접. (광범위 pkill 금지 — GPU/ROS 공유)

### 이번에 얻은 상위 교훈
- **Isaac↔URDF(USD↔RViz) 방향 정합은 물리-재생 pose 로 검증**. 정적 Xform 은 authored 값이라 fixed-joint 스냅을 안 보여줌.
- 새 CAD 는 단일소스 그래프라 USD↔URDF 가 **자동 일치**. 이번 90° 는 유일하게 USD 에만 있던 ATTACH 회전 때문이었고
  이제 항등이라 재발 없음. 부품별로 남는 건 "실제와 맞추는" normalize.rpy/tcp_pose 튜닝뿐(양쪽에 동일 반영).
- 통합 assembly STEP 1개 처리 OK. 단 `convex` 는 전체를 감싸 과보수 → 통합엔 `mesh`/`convexDecomposition`.
  로봇 장착 테이블만 별도 객체로 빼 ACM.

### 미완 / 다음 작업 (이후)
- 연속 충돌 모니터(실행 중 실시간 감지·로깅) — 지금은 계획시 회피 + 온디맨드 리포트(`collision_report.py --watch` 프리뷰).
- cuMotion(GPU, real) 장애물 월드 연동(world 별도 주입), 세트4 nvblox/cuMotion 경로.
- density-box 과대질량·핑거 기하 분리(여전히 placeholder box), 통합 테이블/지그 실 CAD 도착 시 config 교체.
