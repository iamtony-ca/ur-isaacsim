# SETUP — UR16e (ROS 2 Jazzy + Isaac Sim 6.0.1) 재현 매뉴얼

깨끗한 환경에서 이 워크스페이스(sim+real UR16e 제어 스택)를 **그대로 재현**하는 절차.
아키텍처/배경은 상위 [`README.md`](README.md) 참고.

---

## 0. 대상 환경 (검증된 조합)

| 항목 | 버전 |
|---|---|
| OS | Ubuntu 24.04 (noble) |
| ROS 2 | **Jazzy** (`/opt/ros/jazzy`) |
| Isaac Sim | **6.0.1** (`/isaac-sim`, 컨테이너 실측 6.0.1-rc.7; `isaacsim.ros2.bridge` 확장) |
| GPU | NVIDIA (RTX 5090 검증), 드라이버/Vulkan 정상 |
| 워크스페이스 | `/isaac-sim/volume/ur_dualtool_ws` (colcon), 코드 트리 = `src/` |

> **이 워크스페이스(`ur_dualtool_ws`)는 `ur_ws` 를 복제**해 세트4(dual-tool EOAT: OnRobot Dual Quick
> Changer 로 2FG14 그리퍼 + ESTIC 스크류드라이버 + Copick3D 카메라 동시 장착)를 **CAD→Isaac Sim
> 자동화 파이프라인**으로 저작한다. 세트1~3(팔·2F-85·D405) 재현 절차는 아래 §1~§5-C 그대로,
> 세트4 CAD→sim + MoveIt 브리지 재현은 **§5-D** 참고.

> 다른 경로를 쓰면 아래 절대경로(`/isaac-sim/volume/ur_dualtool_ws`, `/isaac-sim/python.sh`)를 본인 환경에 맞게 치환.

---

## 1. 워크스페이스 가져오기

```bash
# src 저장소를 워크스페이스의 src/ 로 (이미 있으면 생략)
mkdir -p /isaac-sim/volume/ur_dualtool_ws
cd /isaac-sim/volume/ur_dualtool_ws
# git clone <this-repo-url> src      # 신규 클론 시
```

---

## 2. apt 의존성 (바이너리, sudo 필요)

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-ur \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control-cmake \
    ros-jazzy-robotiq-description \
    ros-jazzy-xacro \
    liburdfdom-tools \
    python3-vcstool
```
- `ros-jazzy-ur` = 메타패키지 → `ur_robot_driver`, `ur_description`, `ur_moveit_config`,
  `ur_controllers`, `ur_calibration`, `ur_dashboard_msgs`, `ur_client_library` 포함.
- `ros-jazzy-xacro` = xacro→urdf 전개기. 보통 ur/moveit 의존으로 딸려오지만, **베어 ros-base 만
  깔린 PC** 에선 빠져 있어(이 워크스페이스가 그랬음) 명시 설치. 세트4 결합 URDF 전개에 필수.
- `liburdfdom-tools` = `check_urdf`(URDF 트리/충돌모델 검증). ros 접두어 없는 데비안 패키지.
  개발/검증용 — 런타임엔 불필요하지만 결합 URDF 파싱 확인에 사용.
- **세트4(dual-tool)는 추가 런타임 apt 의존이 없다** — `ur_description`+`ur_moveit_config`(위 `ros-jazzy-ur`)
  +`xacro` 만으로 충분. EOAT 링크/조인트/충돌(URDF 매크로·SRDF)은 `isaac/common/eoat/build_eoat_moveit.py`
  가 `eoat_dualtool.yaml` 에서 **생성**하므로 별도 그리퍼 description 패키지가 필요 없다(2F-85 세트와 달리
  `robotiq_description` 불요).
- `ros-jazzy-robotiq-description` = 2F-85 그리퍼 메시(RViz용) + **`ur_to_robotiq` 커플링(GRP-ES-CPL-077) 매크로/메시**.
  **그리퍼 세트(§5-B)에서만** 필요 — 팔 단독이면 생략 가능. (PickNik 카메라 브라켓 메시는 `ur_bringup/meshes/` 에 동봉.)
  (`gripper_controllers`/`position_controllers` 는 `ros-jazzy-ros2-controllers` 에 포함되어 별도 설치 불필요.)
- `ros-jazzy-topic-based-ros2-control` 은 **apt 에 없음** → §3 에서 소스로.
- (옵션, 실물 D405 카메라) `ros-jazzy-realsense2-camera ros-jazzy-librealsense2` — **sim 은 불필요**
  (Isaac 가 카메라를 렌더). 실물 단계에서만 설치. URDF 카메라 프레임은 자작 매크로라 `realsense2-description` 도 불필요.
- (옵션, 세트3 MoveIt OctoMap 충돌회피) `ros-jazzy-moveit-ros-perception` — `PointCloudOctomapUpdater` 플러그인 제공.
  `ur16e_2f85_d405_moveit.launch.py use_octomap:=true`(기본) 사용 시 **필수**. 프레임워크
  `moveit_ros_occupancy_map_monitor` 만으로는 부족. octomap 안 쓰면(`use_octomap:=false`) 불필요.
- (대안) `rosdep install --from-paths src --ignore-src -y` 로도 가능하지만, 위 명시 설치가 확실.

---

## 3. 소스 의존성 (vcstool 로 버전 고정)

`topic_based_hardware_interfaces` 는 apt 에 없고 **특정 태그(0.2.1)** 가 필요하므로 소스로 관리.

```bash
cd /isaac-sim/volume/ur_dualtool_ws
vcs import src < src/ur16e.repos        # topic_based @ 0.2.1 + ros2_robotiq_gripper + serial 클론
vcs validate src < src/ur16e.repos      # (선택) 버전 일치 확인
```
> `src/ur16e.repos` 가 다음을 고정·클론한다:
> - `topic_based_hardware_interfaces` @ **0.2.1** (sim 백엔드, **왜 0.2.1 인지는 §8 참고**)
> - `ros2_robotiq_gripper` @ `main` + `serial` @ `ros2` — **실물 2F-85 그리퍼 드라이버**용
>   (apt 에 `robotiq_driver` 없음; `robotiq_description`/`robotiq_controllers` 만 apt). 팔 단독이면 불필요.
>   소스 트리의 `robotiq_description`/`robotiq_hardware_tests` 는 `COLCON_IGNORE` 로 빌드 제외(apt 것 사용).
> 이 디렉터리들은 `src/.gitignore` 로 무시되어 src 저장소에 박히지 않고 vcs 가 관리.

---

## 4. 빌드

```bash
cd /isaac-sim/volume/ur_dualtool_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to ur_bringup \
    --cmake-args -DBUILD_TESTING=OFF
#   -DBUILD_TESTING=OFF : 0.2.1 의 ros_testing 빌드 의존(미설치) 회피
```
정상 시: `joint_state_topic_hardware_interface`, `ur_bringup` 2개 빌드 완료.

실물 그리퍼 세트를 쓸 거면 그리퍼 드라이버도 빌드 (ur_bringup 런타임 pluginlib 의존이라 별도):
```bash
colcon build --symlink-install --packages-select serial robotiq_driver robotiq_controllers \
    --cmake-args -DBUILD_TESTING=OFF
```
정상 시: `serial`, `robotiq_driver`, `robotiq_controllers` 3개 빌드 완료
(`robotiq_driver/RobotiqGripperHardwareInterface` 플러그인 + `robotiq_controllers/RobotiqActivationController`).

이후 모든 터미널에서:
```bash
source /opt/ros/jazzy/setup.bash
source /isaac-sim/volume/ur_dualtool_ws/install/setup.bash
export ROS_DOMAIN_ID=0
```

---

## 5. 시뮬레이션 구동 (Isaac Sim)

**기동 순서를 지킬 것** (§8-6). 터미널 3개:

```bash
# T1: Isaac Sim (씬 + ROS2 OmniGraph 자동 구성)
/isaac-sim/python.sh /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py          # GUI(환경/조명 포함)
#   headless 면 --headless 추가

# (T1 이 /isaac_joint_states, /clock 발행 시작할 때까지 대기)

# T2: 제어
ros2 launch ur_bringup ur16e.launch.py use_sim:=true

# T3: MoveIt2 + RViz (선택)
ros2 launch ur_bringup ur16e_moveit.launch.py use_sim:=true
```

### 검증
```bash
ros2 control list_controllers          # scaled_joint_trajectory_controller, joint_state_broadcaster = active
ros2 topic echo /joint_states --once    # 값이 유효(숫자), NaN 아님
# 프로그램 plan+execute (로봇이 목표로 이동):
python3 /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/common/moveit_plan_execute_demo.py   # -> RESULT: SUCCESS
```

---

## 5-B. 그리퍼 세트 구동 (UR16e + Robotiq 2F-85)

팔 단독 세트(§5)와 병렬. 그리퍼는 단일 `finger_joint` 만 ROS/Isaac 이 교환하고(나머지 5관절은 자동 mimic),
MoveIt 은 그리퍼 형상까지 충돌 인식. 배경/함정은 [`HISTORY.md`](HISTORY.md) §8.

```bash
# (1회) Isaac UR16e + GRP-ES-CPL-077 커플링 + 2F-85 단일 articulation USD 합성
#   --gripper-z 0.011 = 커플링 두께, --coupling-usd = 손목에 베이크할 커플링 visual (동봉)
/isaac-sim/python.sh /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/common/build_ur16e_2f85.py \
    --out /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/assets/ur16e_with_2f85.usd --gripper-z 0.011 \
    --coupling-usd /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/assets/ur_to_robotiq_coupling.usd --coupling-z 0.0

# T1: Isaac — 합성 씬 지정 (--asset-path 는 절대경로)
/isaac-sim/python.sh /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/assets/ur16e_with_2f85.usd
# T2: 제어 (팔 JTC + gripper_controller)
ros2 launch ur_bringup ur16e_2f85.launch.py
# T3: MoveIt2 + RViz — ★ 그리퍼 전용 (collision-aware SRDF). 공유 ur16e_moveit.launch.py 아님!
ros2 launch ur_bringup ur16e_2f85_moveit.launch.py
```

### 검증
```bash
ros2 control list_controllers     # scaled_joint_trajectory_controller, gripper_controller, joint_state_broadcaster = active
python3 /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/ur16e_2f85/gripper_demo.py          # open/close → goals_ok=True
python3 /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/common/moveit_plan_execute_demo.py   # 팔 plan+execute → SUCCESS (그리퍼 collision 있어도 -10 안 남)
python3 /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/ur16e_2f85/selfcollision_demo.py   # 자기충돌 포즈 → MoveGroup REJECTED 실증
```

### 5-C. 세트 3 — UR16e + 2F-85 + D405 (eye-in-hand 카메라, sim)

세트2(그리퍼) 파일은 그대로 두고 카메라를 **별도 세트(`ur16e_2f85_d405_*`)** 로 분리. 카메라는 sensor-only
(camera link + optical frames, ros2_control joint 없음). 배경/연결/캘리브는 [`HISTORY.md`](HISTORY.md) §9.

```bash
# (1회) 세트3 USD 합성 — 커플링(+7mm) + PickNik 브라켓 + gripper standoff(+18mm) 베이크
/isaac-sim/python.sh /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/common/build_ur16e_2f85.py \
    --out /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd --gripper-z 0.018 \
    --coupling-usd /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/assets/ur_to_robotiq_coupling.usd --coupling-z 0.007 \
    --camera-mount-usd /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/assets/picknik_camera_adapter.usd

# T1: Isaac — 세트3 씬 + 카메라 그래프 (--with-camera, --asset-path 절대경로)
/isaac-sim/python.sh /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd --with-camera
# T2: 제어 — d405 description(카메라 프레임이 robot_description/TF 에 포함)
ros2 launch ur_bringup ur16e_2f85_d405.launch.py
```
검증:
```bash
ros2 topic list | grep camera            # color/image_raw, depth/image_rect_raw, depth/color/points, {color,depth}/camera_info
ros2 topic hz /camera/color/image_raw    # ~수십 Hz
ros2 topic echo /camera/color/camera_info --once   # K: fx≈fy≈334, cx=320, cy=240 (640x480, HFOV≈87°)
```
> 실물 카메라는 §2 의 옵션 apt(`realsense2-camera`) 설치 후 `realsense2_camera` 노드로 (sim 과 동일 토픽명).
> sim 카메라 pose(Isaac 스크립트의 카메라 배치)와 URDF mount(`realsense_d405` origin, PickNik 브라켓 기준)는
> **한 쌍으로** 맞춘다(실물은 hand-eye 캘리브로 보정). 브라켓/시팅 상세는 [`HISTORY.md`](HISTORY.md) §9.

**depth → MoveIt OctoMap 충돌회피** (§2 의 `moveit-ros-perception` 설치 필요):
```bash
ros2 launch ur_bringup ur16e_2f85_d405_moveit.launch.py        # use_octomap:=true (기본)
# 복셀 적분 확인 (component 32 = OCTOMAP)
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 32}}" \
    | grep -E "id=|resolution=|data="
python3 /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/ur16e_2f85_d405/octomap_demo.py   # octomap 활성 plan+execute + probe
```
> perception 미설치/octomap 불필요면 `use_octomap:=false` → 세트2와 동일 collision-aware MoveIt(플러그인 불필요).

---

## 5-D. 세트4 (dual-tool EOAT) — CAD→Isaac Sim 자동화 파이프라인 + MoveIt 브리지

기구팀 STEP(요소품)을 받아 UR16e EOAT 조립을 **config 파라미터만으로** Isaac 물리 아티큘레이션 + URDF/SRDF
(MoveIt2/cuMotion 충돌모델)로 저작한다. 단일 소스 = `isaac/common/eoat/eoat_dualtool.yaml`. 개념/함정은
`STEP_TO_SIM.md`, config 스키마는 `isaac/common/eoat/README.md`.

> **추가 apt 의존 없음** — §2 의 `ros-jazzy-ur`+`ros-jazzy-moveit`+`ros-jazzy-xacro` 로 충분(세트4는
> 그리퍼 description 패키지 불요; EOAT 는 파이프라인이 생성). 순수-python 파이프라인은 시스템 python3 가
> 없으므로 **`/isaac-sim/python.sh`** 로 실행(Isaac 부트 없이 yaml 만 읽는 스크립트도 마찬가지).

```bash
cd /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup
P=/isaac-sim/python.sh ; CFG=isaac/common/eoat/eoat_dualtool.yaml

# A) STEP(폴더째) → USD (미터,Z-up; HOOPS 백엔드, bbox 자동검증)
$P isaac/common/convert_step_to_usd.py <STEP폴더> isaac/assets/cad
# B) 프레임정규화 → normalized/*.usd + *.json 사이드카(size/thickness)
$P isaac/common/eoat/normalize_part.py $CFG
# (그래프 검증) config+사이드카 → 단일 link/joint 그래프 (mass/inertia/collision 확정)
$P isaac/common/eoat/eoat_model.py $CFG
# C/D/E/F) 두 emitter (같은 그래프 → USD·URDF 일치, 함정 #7 회피)
$P isaac/common/eoat/build_eoat_usd.py  $CFG          # → assets/ur16e_dualtool.usd (Isaac 물리 아티큘레이션)
$P isaac/common/eoat/build_eoat_urdf.py $CFG          # → assets/ur16e_dualtool.urdf (EOAT-only)
# G) UR16e 결합 → 팔+EOAT 단일 아티큘레이션 USD
$P isaac/common/eoat/build_ur16e_dualtool.py          # → assets/ur16e_dualtool_full.usd
# MoveIt 브리지 산출물(그래프에서 생성): xacro 매크로 + SRDF
$P isaac/common/eoat/build_eoat_moveit.py $CFG
#   → urdf/ur16e_dualtool/ur16e_dualtool_eoat_macro.xacro , srdf/common/ur16e_dualtool.srdf.xacro

# 검증 — Isaac play(핑거 구동 대칭 도달) / 결합 구조 / 결합 URDF 파싱
$P isaac/common/eoat/verify_articulation.py isaac/assets/ur16e_dualtool.usd
$P isaac/common/eoat/verify_articulation.py isaac/assets/ur16e_dualtool_full.usd /World/eoat/root_joint
source /opt/ros/jazzy/setup.bash
xacro urdf/ur16e_dualtool/ur16e_dualtool_sim.urdf.xacro > /tmp/dualtool.urdf && check_urdf /tmp/dualtool.urdf
```

- **2FG14 핑거**는 평행 그리퍼(단일 DOF + mimic) — 구동 조인트만 ROS 와 교환, mirror 는 `<mimic>`(RSP)+
  Isaac 커플링. ROS 브리지용 Isaac 자산은 `build_eoat_usd.py --physx-mimic` 로 재빌드.
- 물성/치수는 **실측 오면 config 숫자만 교체**(density-box·placeholder). 미완 갭(과대질량·핑거 기하 분리
  ·copick FOV 원뿔·controllers/launch·cuMotion XRDF)은 `isaac/common/eoat/README.md §미완 갭` 참고.
- **배치(자세) 지정 3방식**(우선순위 `tcp_pose > mount > 자동스택`, config `chain[]`):
  ① 자동스택(+Z 면-대-면) ② `mount: {xyz,rpy}`(parent 기준, Y분기/직교포트) ③ **`tcp_pose: {xyz,rpy}`**
  (`tool0`/UR TCP 기준 절대 상대포즈 — 모델이 parent-local mount 자동 역산; `meta.tcp_offset` 로 TCP≠플랜지).
  rpy 는 ROS 규약(roll-X/pitch-Y/yaw-Z)으로 USD·URDF 통일.
- **GUI 튜닝 ↔ config 왕복**(rough 숫자 확정):
  ```bash
  $P isaac/common/eoat/edit_asset.py isaac/assets/ur16e_dualtool.usd      # 편집가능 GUI 오픈 → 링크 드래그 → Ctrl+S
  $P isaac/common/eoat/extract_poses.py isaac/assets/ur16e_dualtool.usd $CFG        # parent-local mount 로 추출
  $P isaac/common/eoat/extract_poses.py isaac/assets/ur16e_dualtool.usd $CFG --tcp  # tool0 기준 tcp_pose 로 추출
  ```
  **실 데이터 경로**: 기구팀에 "tool0 기준 각 부품 원점 xyz[m]+rpy[deg,ROS]"를 요청 → 각 `chain[]` 의
  `tcp_pose:` 에 입력하면 GUI 튜닝 없이 정밀 정렬. (드래그로 임시로 맞추면 `--tcp` 추출값을 붙여 확정.)
- **함정(재현성)**: ① STEP→USD 는 `omni.kit.converter.hoops_core` 만(‌`converter.cad` 켜면 ODA 심볼충돌
  abort). ② 미터 네이티브(`dMetersPerUnit=0.001`) 아니면 1000배. ③ Isaac play 시 **self-collision OFF 필수**
  (안 그러면 placeholder 핑거 충돌이 몸체에 박혀 드라이브가 짐). ④ 결합 URDF 는 name="ur16e" 유지(ur_moveit SRDF).
  ⑤ xacro 주석에 `--`(이중 하이픈) 금지 — XML 파싱 실패.

### 세트4 ROS 구동 (빌드 + MoveIt)

```bash
cd /isaac-sim/volume/ur_dualtool_ws
source /opt/ros/jazzy/setup.bash
vcs import src < src/ur16e.repos              # topic_based @0.2.1 (이미 있으면 "already exists" 무시)
colcon build --symlink-install --packages-up-to ur_bringup --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

# 결합 URDF/SRDF 검증(빌드 후 $(find ur_bringup) 해석)
xacro $(ros2 pkg prefix ur_bringup)/share/ur_bringup/urdf/ur16e_dualtool/ur16e_dualtool_sim.urdf.xacro \
  > /tmp/dualtool.urdf && check_urdf /tmp/dualtool.urdf
xacro $(ros2 pkg prefix ur_bringup)/share/ur_bringup/srdf/common/ur16e_dualtool.srdf.xacro name:=ur16e >/dev/null && echo "SRDF OK"

# 구동 (3 터미널): Isaac → 제어 → MoveIt
/isaac-sim/python.sh isaac/common/ur16e_isaac_ros2.py \
  --asset-path $PWD/src/ur_bringup/isaac/assets/ur16e_dualtool_full.usd     # T1 (핑거 커플링 원하면 asset 을 --physx-mimic 로 빌드)
ros2 launch ur_bringup ur16e_dualtool.launch.py                            # T2 (arm JTC + gripper_controller)
ros2 launch ur_bringup ur16e_dualtool_moveit.launch.py                     # T3 (move_group + RViz, 결합 SRDF)
```
- 파일: `urdf/ur16e_dualtool/ur16e_dualtool_sim.urdf.xacro`(최상위) + `…_eoat_macro.xacro`(생성)
  + `urdf/common/ur16e_dualtool_sim.ros2_control.xacro` + `config/common/ur16e_dualtool_controllers.yaml`
  + `srdf/common/ur16e_dualtool.srdf.xacro`(생성) + `launch/ur16e_dualtool/…{,_moveit}.launch.py`.
- **콜리전 체크(1차 목표)**: SRDF 가 EOAT-vs-팔몸통을 ENABLE 유지 → MoveIt 이 툴이 팔에 박히는 자세를
  self-collision 으로 거부. gripper 그룹은 `gripper_2fg14__finger_left_joint`(open/closed state).
- **빌드 함정**: `ros-jazzy-ros2-control-cmake` 없으면 topic_based 빌드가 `Findros2_control_cmake.cmake`
  못 찾아 실패 → §2 에 포함(bare ros-base 엔 미설치일 수 있음).
- **RViz==Isaac**: URDF 가 실제 mesh 를 참조하려면 `export_eoat_meshes.py` 실행 후 colcon build(또는
  `meshes/eoat` 심링크). 안 하면 RViz 는 박스로 보임(정합성은 문제없고 시각만 다름).

### 세트4 — 정적 장애물 충돌 + MoveJ/MoveL 데모 (sim, move_group 실행 중)

```bash
P=/isaac-sim/python.sh
# 정적 장애물(테이블·지그): config→USD/충돌OBJ→planning scene. 실물에서도 같은 노드.
$P isaac/common/obstacles/prepare_obstacles.py isaac/common/obstacles/obstacles.yaml   # CAD 있으면(box-only 면 생략)
/usr/bin/python3 isaac/common/obstacles/load_obstacles_moveit.py     # planning scene + ACM (--level mesh|convexDecomposition|convex)
/usr/bin/python3 isaac/common/obstacles/collision_report.py 0 -0.2 1.9 -1.7 -1.57 0   # 충돌쌍+접촉점 xyz+깊이
/usr/bin/python3 isaac/common/obstacles/approach_to_collision.py 0 0.2 1.9 -1.9 -1.57 0 --loop 3  # 충돌직전 왕복
$P isaac/common/ur16e_isaac_ros2.py --asset-path <abs>/ur16e_dualtool_full.usd --obstacles <abs>/obstacles.yaml  # Isaac 씬에도

# MoveJ / MoveL 등가 데모
/usr/bin/python3 isaac/common/movej_demo.py 0 0.2 0        # MoveJ(관절공간, TCP 곡선) — IK 는 MoveIt(KDL)
/usr/bin/python3 isaac/common/cartesian_demo.py 0 0.2 0    # MoveL(TCP 직선), fraction<100%면 특이점/한계
```
- **IK 는 MoveIt(KDLKinematicsPlugin)** — 하드웨어(Isaac/실물 UR)는 관절 추종만. sim=real 동일 경로.
- **CAD 파이프라인 추가 의존**(Isaac python): `/isaac-sim/python.sh -m pip install coacd fast-simplification`.

---

## 6. 실물 UR16e 구동

```bash
# 하드웨어 없이 드라이버 경로 점검(mock):
ros2 launch ur_bringup ur16e.launch.py use_sim:=false use_mock_hardware:=true

# 실물:
ros2 launch ur_bringup ur16e.launch.py use_sim:=false robot_ip:=<UR16e_IP>
ros2 launch ur_bringup ur16e_moveit.launch.py use_sim:=false
```
실물 전제조건: ① 같은 서브넷 + `ping <ip>` ② External Control URCap(또는 `headless_mode:=true` + Remote 모드)
③ `ur_calibration` 으로 기구학 추출(권장). 상세는 `src/ur_bringup/README.md`.

### 6-B. 실물 그리퍼 (Robotiq 2F-85)

그리퍼는 팔(RTDE)과 **별개 시리얼 장치**(Modbus RTU)라 **`gripper` 네임스페이스 전용 controller_manager**
로 돈다(§4 에서 `robotiq_driver` 빌드 선행). 배경/함정/연결 토폴로지표는 [`HISTORY.md`](HISTORY.md) §8.

**중요 — 2F-85 는 UR 손목 tool 커넥터에 물리고 제어 PC 에는 직접 안 붙는다.** PC 가 닿는 표준 경로는
ur_robot_driver 의 **tool communication 브리지**(UR tool RS-485 → 가상 시리얼 `/tmp/ttyUR`)다.

```bash
# 하드웨어 없이 점검(mock): 팔 mock + 그리퍼 mock + 브리지 off
ros2 launch ur_bringup ur16e_2f85_real.launch.py \
    use_mock_hardware:=true use_fake_hardware:=true use_tool_communication:=false

# A. 손목 장착(표준): 팔 + tool comm 브리지(/tmp/ttyUR) + 그리퍼 한 런치
ros2 launch ur_bringup ur16e_2f85_real.launch.py robot_ip:=<UR16e_IP>

# B. 벤치 직결(USB-RS485): 팔/그리퍼 따로
ros2 launch ur_bringup ur16e.launch.py use_sim:=false robot_ip:=<UR16e_IP>     # T1 팔
ros2 launch ur_bringup robotiq_2f85_real.launch.py com_port:=/dev/ttyUSB0      # T2 그리퍼
```
실물 전제조건: ① (A) UR tool I/O 를 RS-485/Robotiq 로 설정 + `tool_voltage:=24`, PC 가 `/tmp/ttyUR` 쓰기
권한(`dialout`); (B) 2F-85→USB-RS485→`/dev/ttyUSB0`, 컨테이너면 `--device=/dev/ttyUSB0` ② 자동 활성화는
`robotiq_activation_controller`(실물에서만 스폰)가 처리 ③ A 는 브리지가 먼저 떠야 하므로 그리퍼 기동을
`gripper_startup_delay`(기본 8s)만큼 지연한다.

#### 검증 (fake 모드)
```bash
ros2 control list_controllers -c /gripper/controller_manager   # joint_state_broadcaster, gripper_controller = active
python3 /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/ur16e_2f85/gripper_demo.py \
    --action /gripper/gripper_controller/gripper_cmd --joint-states-topic /gripper/joint_states
#   -> open/close, reached_goal=True, goals_ok=True
```

---

## 7. 종료
```bash
pkill -f ur16e_isaac_ros2.py ; pkill -f "ros2 launch ur_bringup" ; pkill -f "lib/rviz2/rviz2"
```
> 같은 머신에서 다른 GPU/ROS 작업이 돌 수 있으니 광범위한 `pkill`(`python3`, `ros2_control_node` 등) 금지,
> 위 워크로드 전용 패턴만 사용.

---

## 8. 알려진 이슈 / 버전 고정 이유 (재현성 핵심)

1. **topic_based 는 반드시 `0.2.1`.** `main`/1.0.0/1.1.0 의 신 API `set_state(name,value)` 는 apt
   `ros-jazzy` ros2_control 4.44.0 에서 exported state interface 를 갱신하지 못해 `/joint_states` 가
   전부 NaN → MoveIt 상태 없음 → RViz SIGSEGV. 0.2.1 은 classic `export_state_interfaces()` 라 정상.
2. **Isaac UR16e USD 의 articulation root = `/UR16e/root_joint`** (USD 에 default prim 없음). 스크립트가 처리.
3. **sim xacro 는 position/velocity/effort state interface 3개** 선언 (Isaac 이 effort 도 발행).
4. **sim 은 `use_sim_time:=true` + Isaac `/clock` 필요.** clock 없거나 너무 느리면 컨트롤러 활성화가
   `Switch controller timed out`. Isaac 없이 제어 스택만 테스트하려면 `use_sim_time:=false`.
5. **Isaac GUI 는 환경(조명) 로드 필수** — `--no-env` 면 광원이 없어 뷰포트가 까맣게 보임(로봇은 존재).
6. **기동 순서**: Isaac(안정화) → control(`/joint_states` 유효 확인) → move_group/RViz. Isaac 재시작 시
   move_group/RViz 도 재시작(안 그러면 과도기 NaN 캐싱 → plan `error_code -4`).

---

## 9. 빠른 트러블슈팅

| 증상 | 원인/조치 |
|---|---|
| `/joint_states` 가 전부 NaN | topic_based 가 0.2.1 아님 → §3 재확인 후 재빌드 |
| `Switch controller timed out` | Isaac `/clock` 없음/느림 → Isaac 먼저 띄우거나 `use_sim_time:=false` |
| RViz 가 떴다 바로 꺼짐(SIGSEGV) | NaN TF 렌더링 → 위 NaN 원인 해결 / 기동 순서 |
| Isaac 창이 검정 | `--no-env` 로 조명 없음 → 환경 포함으로 재기동 |
| MoveIt plan `error_code -4` | move_group 이 과도기 NaN 캐싱 → move_group/RViz 재시작 |
| `colcon build` 가 `ros_testing` 못 찾음 | `--cmake-args -DBUILD_TESTING=OFF` 추가 |
| (그리퍼) MoveIt plan `error_code -10` | URDF `<robot name>` ≠ SRDF 이름 → 그리퍼 URDF 이름이 `ur16e` 인지 확인 / 그리퍼 collision 켰으면 `ur16e_2f85_moveit.launch.py`(전용 SRDF) 사용 |
| (그리퍼) RViz 에서 그리퍼가 안 움직임 | master 조인트가 `finger_joint` 인지 확인(`/joint_states` 에 있는 이름과 일치해야 mimic 계산됨) |
| (그리퍼) `JointStateTopicSystem` plugin not found / `local_setup.bash not found` | 워크스페이스 폴더명 변경 후 topic_based 의 symlink-install 깨짐 → `rm -rf build/install 해당 패키지` 후 재빌드 |
| (그리퍼) `MoveItConfigsBuilder ... config/ur.srdf doesn't exist` | 빌더에 `.robot_description_semantic(Path("srdf")/"ur.srdf.xacro", {"name":"ur16e"})` 명시 필요 (그리퍼 moveit 런치에 이미 반영) |
| (실물 그리퍼) `The 'type' param was not defined for '...'` 로 컨트롤러 로드 실패 | `gripper` 네임스페이스 CM 인데 yaml 키가 평문 → `robotiq_2f85_real_controllers.yaml` 의 wildcard 키(`/**/controller_manager` 등) 확인 |
| (실물 그리퍼) `robotiq_driver` 플러그인 못 찾음 | §4 의 `colcon build --packages-select serial robotiq_driver robotiq_controllers` 누락 → 빌드 후 `source install/setup.bash` |
| (실물 그리퍼) 시리얼 포트 open 실패 | `/dev/ttyUSB0` 없음/권한 → 어댑터 연결, `com_port:=` 지정, `dialout` 그룹/`--device` 확인. 점검만이면 `use_fake_hardware:=true` |
| (octomap) move_group 이 `libgeometric_shapes.so.2.3.x cannot open` 로 updater 로드 실패 | `moveit-ros-perception` 만 최신이라 부분 업그레이드 ABI 불일치 → 스택 정렬: `sudo apt install -y $(dpkg -l \| awk '/^ii.*ros-jazzy-moveit/{print $2}') ros-jazzy-geometric-shapes` (전부 같은 빌드로). octomap 불필요면 `use_octomap:=false` |
| (세트2/3) Isaac 에 로봇은 안 보이고 카메라/그리퍼만 보임 | `--asset-path` 가 상대경로 → Isaac 에셋서버 기준으로 붙어 로드 실패. **절대경로**로 지정 |
| (세트2/3) Isaac EE(그리퍼) 위치가 RViz 와 어긋남 | URDF 는 커플링 +11/+18mm 인데 USD 가 옛 flush 베이크 → `build_ur16e_2f85.py` 를 `--gripper-z`/`--coupling-usd` 옵션으로 **재베이크**(§5-B/§5-C) |
| (세트3) 카메라가 브라켓에서 떠 보이거나 파고듦 | `realsense_d405` origin 의 pitch(8°)/높이(0.01847) 시팅 값 → [`HISTORY.md`](HISTORY.md) §9 참고, cradle 표면 법선과 평행+gap0 으로 맞춤 |
