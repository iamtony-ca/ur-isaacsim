# UR16e — ROS 2 Jazzy + Isaac Sim 5.1.0 (sim & real 공용 제어 스택)

**UR16e 로봇팔을 하나의 ROS 2 (Jazzy) 소프트웨어로 Isaac Sim 시뮬레이션과 실물에서 모두** 구동하는
워크스페이스. UR 공식 스택(`ur_robot_driver` + `ros2_control` + MoveIt2) 기반이며, 자체 코드는
`ur_bringup` 한 패키지에 모여 있다.

- 워크스페이스: `/isaac-sim/ur_ws` (colcon), git repo = `src/`
- 환경: Docker, ROS 2 **Jazzy**, **Isaac Sim 5.1.0**(`/isaac-sim`), GPU
- 이 문서는 **현재 상태** 기준 정리. 변경 이력·검증 로그·디버깅 교훈은 [`HISTORY.md`](HISTORY.md),
  실물 HW 연결 후 절차는 [`HARDWARE.md`](HARDWARE.md), 재현 매뉴얼은 [`SETUP.md`](SETUP.md),
  개념 Q&A 는 [`qna.md`](qna.md).

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
        │ /isaac_joint_states         │
        │ /isaac_joint_commands       │
   Isaac Sim 5.1.0              실물 UR16e
```

`use_sim` 인자 하나로 ros2_control 하드웨어 백엔드만 바뀌고, 그 위(MoveIt·컨트롤러·앱)는 동일하다.
그리퍼는 sim `topic_based` ↔ real `robotiq_driver`, 카메라는 sim Isaac ↔ real `realsense2_camera` 로
백엔드만 교체되며 **토픽·프레임은 sim/real 동일**해 상위 인식 스택은 무수정으로 재사용된다.

---

## 2. 세 세트 (독립 유지)

각 세트는 독립적으로 띄울 수 있고, 상위 세트는 하위/공유 파일을 수정하지 않고 재사용한다.

| 세트 | sim 런치 | real 런치 | 내용 |
|---|---|---|---|
| **1. UR16e 단독** | `ur16e.launch.py` (+`ur16e_moveit.launch.py`) | `ur16e.launch.py use_sim:=false` | 팔만 |
| **2. + 2F-85 그리퍼** | `ur16e_2f85.launch.py` (+`…_moveit`) | `ur16e_2f85_real.launch.py` | + GRP-ES-CPL-077 커플링 + Robotiq 2F-85, collision-aware |
| **3. + D405 카메라** | `ur16e_2f85_d405.launch.py` (+`…_moveit`) | `ur16e_2f85_d405_real.launch.py` | + PickNik 브라켓 eye-in-hand D405 + depth→OctoMap |

> `ros2 launch ur_bringup <파일명>` 은 폴더와 무관하게 이름으로 찾으므로 실행 시 하위폴더 경로 불필요.
> MoveIt 은 세트마다 전용 런치: 세트1 `ur16e_moveit`, 세트2 `ur16e_2f85_moveit`,
> 세트3 `ur16e_2f85_d405_moveit`(depth→OctoMap, `use_octomap` 기본 on).

---

## 3. 폴더 구조 (`ur_bringup/`)

카테고리(`launch`/`config`/`urdf`)는 세트별 하위폴더로, 여러 세트가 공유하는 파일은 `common/` 에 둔다.

```
ur_bringup/
├── launch/
│   ├── ur16e/            ur16e.launch.py, ur16e_moveit.launch.py
│   ├── ur16e_2f85/       ur16e_2f85[_moveit|_real].launch.py, robotiq_2f85_real.launch.py
│   └── ur16e_2f85_d405/  ur16e_2f85_d405[_moveit|_real].launch.py, d405_real.launch.py
├── config/
│   ├── common/           ur16e_2f85_controllers.yaml              (세트2·3 공유)
│   ├── ur16e/            ur16e_controllers.yaml
│   ├── ur16e_2f85/       robotiq_2f85_real_controllers.yaml
│   └── ur16e_2f85_d405/  sensors_3d.yaml, d405_real.yaml
├── urdf/
│   ├── common/           robotiq_2f85_macro.xacro, ur16e_2f85_sim.ros2_control.xacro  (세트2·3 공유)
│   ├── ur16e/            ur16e_sim.urdf.xacro, ur16e_sim.ros2_control.xacro
│   ├── ur16e_2f85/       ur16e_2f85_sim.urdf.xacro, robotiq_2f85_real.{urdf,ros2_control}.xacro
│   └── ur16e_2f85_d405/  ur16e_2f85_d405_sim.urdf.xacro, realsense_d405_macro.xacro, d405_real.urdf.xacro
├── srdf/common/          ur16e_2f85.srdf.xacro                    (세트2·3 공유)
├── isaac/
│   ├── common/           ur16e_isaac_ros2.py, moveit_plan_execute_demo.py, build_ur16e_2f85.py, convert_dae_to_usd.py
│   ├── ur16e_2f85/       gripper_demo.py, selfcollision_demo.py
│   ├── ur16e_2f85_d405/  octomap_demo.py, convert_bracket.py
│   └── assets/           합성 USD (세트 공유 버킷)
└── meshes/               PickNik 브라켓 메시 (세트3)
```

---

## 4. 설치 & 빌드

```bash
# (1) 표준 스택 — apt
sudo apt update && sudo apt install -y \
    ros-jazzy-ur ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-moveit-ros-perception          # 세트3 octomap (depth perception 플러그인)

# (2) 소스 의존성 — vcstool (topic_based[sim 백엔드], robotiq_driver+serial[실물 그리퍼])
cd /isaac-sim/ur_ws
vcs import src < src/ur16e.repos

# (3) 빌드
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to ur_bringup --cmake-args -DBUILD_TESTING=OFF
colcon build --symlink-install --packages-select serial robotiq_driver robotiq_controllers \
    --cmake-args -DBUILD_TESTING=OFF         # 실물 2F-85 그리퍼 드라이버
source install/setup.bash

# (4) 실물 RealSense D405 (세트3 real) — apt
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-librealsense2 \
    ros-jazzy-diagnostic-updater ros-jazzy-diagnostic-msgs   # diagnostic 은 realsense ABI 정합용
```

> 단계별 상세·트러블슈팅은 [`SETUP.md`](SETUP.md).
> 매 터미널 먼저: `source /opt/ros/jazzy/setup.bash && source /isaac-sim/ur_ws/install/setup.bash && export ROS_DOMAIN_ID=0`

---

## 5. 실행 — 시뮬레이션 (Isaac Sim)

기동 순서: **Isaac(안정화) → 제어 → MoveIt**. 세트별로 Isaac 에셋(`--asset-path`)만 다르다.

```bash
# ── 세트 1: UR16e 단독 ──
/isaac-sim/python.sh src/ur_bringup/isaac/common/ur16e_isaac_ros2.py           # (--headless 가능)
ros2 launch ur_bringup ur16e.launch.py use_sim:=true
ros2 launch ur_bringup ur16e_moveit.launch.py use_sim:=true
python3 src/ur_bringup/isaac/common/moveit_plan_execute_demo.py               # plan+execute 데모

# ── 세트 2: + 2F-85 그리퍼 ──
/isaac-sim/python.sh src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/ur_ws/src/ur_bringup/isaac/assets/ur16e_with_2f85.usd
ros2 launch ur_bringup ur16e_2f85.launch.py
ros2 launch ur_bringup ur16e_2f85_moveit.launch.py
python3 src/ur_bringup/isaac/ur16e_2f85/gripper_demo.py                        # 그리퍼 open/close
python3 src/ur_bringup/isaac/ur16e_2f85/selfcollision_demo.py                 # 자기충돌 거부

# ── 세트 3: + D405 카메라 ──  (--asset-path 는 반드시 절대경로)
/isaac-sim/python.sh src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/ur_ws/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd --with-camera
ros2 launch ur_bringup ur16e_2f85_d405.launch.py
ros2 launch ur_bringup ur16e_2f85_d405_moveit.launch.py                        # depth→OctoMap (기본 on)
python3 src/ur_bringup/isaac/ur16e_2f85_d405/octomap_demo.py
```

---

## 6. 실행 — 실물(real)

> 실물 HW 를 처음 연결할 때 해야 할 일(네트워크/URCap/캘리브/시리얼 권한/hand-eye 등)은
> **[`HARDWARE.md`](HARDWARE.md)** 에 정리. 아래는 SW 가 준비된 상태에서의 실행 명령.

```bash
# 세트 1: 팔만
ros2 launch ur_bringup ur16e.launch.py use_sim:=false robot_ip:=<UR16e_IP>
ros2 launch ur_bringup ur16e_moveit.launch.py use_sim:=false

# 세트 2: 팔 + 손목 2F-85 (UR tool RS-485 브리지 /tmp/ttyUR 자동)
ros2 launch ur_bringup ur16e_2f85_real.launch.py robot_ip:=<UR16e_IP>
ros2 launch ur_bringup ur16e_2f85_moveit.launch.py use_sim:=false

# 세트 3: 팔 + 2F-85 + D405 (USB3 직결)
ros2 launch ur_bringup ur16e_2f85_d405_real.launch.py robot_ip:=<UR16e_IP> \
    cam_xyz:="x y z" cam_rpy:="r p y"          # hand-eye 캘리브 결과 (기본=sim 명목)
ros2 launch ur_bringup ur16e_2f85_d405_moveit.launch.py use_sim:=false

# 하드웨어 없이 경로 점검 (mock)
ros2 launch ur_bringup ur16e_2f85_d405_real.launch.py \
    use_mock_hardware:=true use_fake_hardware:=true use_tool_communication:=false enable_camera:=false
```

---

## 7. 현재 상태

| | sim | real(SW 준비) | real(HW 검증) |
|---|---|---|---|
| 세트 1 (팔) | ✅ plan+execute | ✅ (mock 검증) | ⏳ 로봇 연결 시 |
| 세트 2 (+2F-85) | ✅ 그리퍼 개폐·자기충돌·plan+execute | ✅ `robotiq_driver` (mock 검증) | ⏳ 그리퍼 연결 시 |
| 세트 3 (+D405) | ✅ 카메라·OctoMap·plan+execute | ✅ `realsense2_camera` 노드 로드+카메라 TF | ⏳ D405 USB3 연결 시 (영상 스트림·hand-eye) |

자세한 검증 로그/날짜/근거는 [`HISTORY.md`](HISTORY.md).

---

## 8. 종료

```bash
pkill -f ur16e_isaac_ros2.py ; pkill -f "ros2 launch ur_bringup" ; pkill -f "lib/rviz2/rviz2"
```
> 같은 컨테이너에서 다른 워크스페이스가 GPU/ROS 를 공유할 수 있으므로 광범위한 `pkill`(`python3`,
> `ros2_control_node` 등)은 피하고 워크로드 전용 패턴만 사용.
