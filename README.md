# UR16e — ROS 2 Jazzy + Isaac Sim 6.0.1 (sim & real 공용 제어 스택)

**UR16e 로봇팔을 하나의 ROS 2 (Jazzy) 소프트웨어로 Isaac Sim 시뮬레이션과 실물에서 모두** 구동하는
워크스페이스. UR 공식 스택(`ur_robot_driver` + `ros2_control` + MoveIt2) 기반이며, 자체 코드는
`ur_bringup` 한 패키지에 모여 있다.

- 워크스페이스: `/isaac-sim/volume/ur_ws` (colcon), git repo = `src/`
- 환경: Docker, ROS 2 **Jazzy**, **Isaac Sim 6.0.1**(`/isaac-sim`), GPU
- 이 문서는 **현재 상태** 기준 정리. 변경 이력·검증 로그·디버깅 교훈은 [`HISTORY.md`](HISTORY.md),
  실물 HW 연결 후 절차는 [`HARDWARE.md`](HARDWARE.md), 재현 매뉴얼은 [`SETUP.md`](SETUP.md),
  개념 Q&A 는 [`qna.md`](qna.md).
- **★ 새 PC 에서 처음부터 실물까지** = [`CHECKLIST.md`](CHECKLIST.md).
  설치 → 하드웨어 0개 검증 → 팔 → 리더 → 그리퍼 → 카메라 순서를 체크박스로만 정리한 것.
  *왜* 그렇게 하는지는 SETUP/HARDWARE 로 링크가 걸려 있다.
- **★ IL 파이프라인 실행 명령** = [`PIPELINE.md`](PIPELINE.md).
  수집 → 변환 → 학습 → 추론을 재현하는 명령 전부와, 각 단계의 **합격 기준**(정지 프레임·그리퍼
  최대값·롤아웃 판정). 9/10 까지 확인한 그 명령들이다.
- **계획/설계**: **IL/VLA 정본** = [`ur_bringup/docs/plan_il_vla.md`](ur_bringup/docs/plan_il_vla.md)
  (teleoperation → LeRobot ACT → 소형 VLA → GR00T N1.7). 이 워크스페이스의 방향은 이 문서 하나로 읽으면 된다.
- **보류**: IL/VLA 전환 이전 트랙의 설계문서(`to_do.md` foundation perception,
  `LEARNING.md` RL insertion)는 [`archive/`](archive/) 로 옮겼다. 삭제가 아니라 보류이며,
  사유·살아남은 부분의 행방·복귀 시 주의사항은 [`archive/README.md`](archive/README.md).

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
   Isaac Sim 6.0.1              실물 UR16e
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
│   ├── common/           teleop_servo.launch.py, teleop_dualsense.launch.py       (teleop, 세트2·3)
│   ├── ur16e/            ur16e.launch.py, ur16e_moveit.launch.py
│   ├── ur16e_2f85/       ur16e_2f85[_moveit|_real].launch.py, robotiq_2f85_real.launch.py
│   └── ur16e_2f85_d405/  ur16e_2f85_d405[_moveit|_real].launch.py, d405_real.launch.py
├── config/
│   ├── common/           ur16e_2f85_controllers.yaml, ur16e_servo.yaml   (세트2·3 공유)
│   ├── ur16e/            ur16e_controllers.yaml
│   ├── ur16e_2f85/       robotiq_2f85_real_controllers.yaml
│   └── ur16e_2f85_d405/  sensors_3d.yaml, d405_real.yaml
├── urdf/
│   ├── common/           robotiq_2f85_macro.xacro, ur16e_2f85_sim.ros2_control.xacro  (세트2·3 공유)
│   ├── ur16e/            ur16e_sim.urdf.xacro, ur16e_sim.ros2_control.xacro
│   ├── ur16e_2f85/       ur16e_2f85_sim.urdf.xacro, robotiq_2f85_real.{urdf,ros2_control}.xacro
│   └── ur16e_2f85_d405/  ur16e_2f85_d405_sim.urdf.xacro, realsense_d405_macro.xacro, d405_real.urdf.xacro
├── srdf/common/          ur16e_2f85.srdf.xacro                    (세트2·3 공유)
├── scripts/              teleop_joy.py                            (ros2 run 실행파일)
├── isaac/
│   ├── common/           ur16e_isaac_ros2.py, moveit_plan_execute_demo.py, build_ur16e_2f85.py, convert_dae_to_usd.py,
│   │                     switch_control_mode.py, reset_pose.py
│   ├── ur16e_2f85/       gripper_demo.py, selfcollision_demo.py
│   ├── ur16e_2f85_d405/  octomap_demo.py, convert_bracket.py
│   └── assets/           합성 USD (세트 공유 버킷)
└── meshes/               PickNik 브라켓 메시 (세트3)
```

---

## 4. 설치 & 빌드

> **★ 새 PC 라면 아래를 손으로 하지 말고 두 줄로 끝낼 것** — 순서·핀·GPU 분기·검증까지 전부 들어 있다:
> ```bash
> git clone <repo> /isaac-sim/volume/ur_ws/src
> /isaac-sim/volume/ur_ws/src/setup/bootstrap.sh --dry-run   # 계획만 먼저
> /isaac-sim/volume/ur_ws/src/setup/bootstrap.sh
> ```
> 전체 순서와 전제조건은 [`SETUP.md`](SETUP.md) **§0-B**. 아래는 손으로 할 때의 참고다.

```bash
# (1) 표준 스택 — apt
sudo apt update && sudo apt install -y \
    ros-jazzy-ur ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-robotiq-description \          # 세트2/3 필수 — 없으면 URDF 가 생성조차 안 됨
    ros-jazzy-moveit-ros-perception          # 세트3 octomap (depth perception 플러그인)

# (2) 소스 의존성 — vcstool (topic_based[sim 백엔드], robotiq_driver+serial[실물 그리퍼], nvblox[소스빌드])
cd /isaac-sim/volume/ur_ws
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

# (5) cuMotion / nvblox (GPU 플래닝·실시간 회피) — SETUP.md §2-B
#   공유 머신이면 apt 핀을 먼저 넣고, 메타패키지(-examples / isaac-ros-nvblox)는 쓰지 말 것.
#   RTX 40/50(sm_89/120)이면 nvblox_ros 는 반드시 소스 빌드:
#     colcon build --packages-select nvblox_ros --cmake-args -DUSE_NATIVE_CUDA_ARCHITECTURE=1
```

> 단계별 상세·트러블슈팅은 [`SETUP.md`](SETUP.md).
> 매 터미널 먼저: `source /opt/ros/jazzy/setup.bash && source /isaac-sim/volume/ur_ws/install/setup.bash && export ROS_DOMAIN_ID=0`

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
    --asset-path /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_with_2f85.usd
ros2 launch ur_bringup ur16e_2f85.launch.py
ros2 launch ur_bringup ur16e_2f85_moveit.launch.py
python3 src/ur_bringup/isaac/ur16e_2f85/gripper_demo.py                        # 그리퍼 open/close
python3 src/ur_bringup/isaac/ur16e_2f85/selfcollision_demo.py                 # 자기충돌 거부

# ── 세트 3: + D405 카메라 ──  (--asset-path 는 반드시 절대경로)
/isaac-sim/python.sh src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd --with-camera
ros2 launch ur_bringup ur16e_2f85_d405.launch.py
ros2 launch ur_bringup ur16e_2f85_d405_moveit.launch.py                        # depth→OctoMap (기본 on)
python3 src/ur_bringup/isaac/ur16e_2f85_d405/octomap_demo.py

# ── 정적(외부) 카메라 TF ── teleop/IL 은 이것만 있으면 됨 (nvblox 불필요)
ros2 launch ur_bringup static_cam_tf.launch.py use_sim_time:=true
#   Isaac 은 --with-static-cam 으로 /static_cam/{color,depth}/* 발행 (color = IL 정책 입력)

# ── Teleoperation (MoveIt Servo) ── 세트2/3 제어 위에서. IL 데모 수집의 1단계
ros2 launch ur_bringup teleop_servo.launch.py use_sim_time:=true     # servo + 스트리밍 컨트롤러(inactive)
#   ★ 먼저 특이점 아닌 자세로: home/up/zero 는 팔꿈치 특이점이라 Servo 가 거부한다
python3 src/ur_bringup/isaac/common/switch_control_mode.py trajectory
python3 src/ur_bringup/isaac/common/reset_pose.py ready
python3 src/ur_bringup/isaac/common/switch_control_mode.py streaming
ros2 launch ur_bringup teleop_dualsense.launch.py                    # 패드 (L1=deadman 유지, L2/R2=그리퍼)
ros2 run moveit_servo servo_keyboard_input                           # 패드 없으면 키보드로 대체
python3 src/ur_bringup/isaac/common/switch_control_mode.py trajectory  # MoveIt/cuMotion 으로 복귀

# ── OMY-L100 리더 (teleop leader, 중력보상 ros2_control) ── 설치: src/setup/setup.sh leader
#    ★ 하드웨어 없이도 검증됨. 실물은 port_name:=/dev/ttyUSB0 (U2D2). 상세 SETUP.md 2-D
ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py \
    use_mock_hardware:=true use_self_collision_avoidance:=false   # ★ 후자 반드시 false
ros2 control list_controllers -c /leader/controller_manager       # ★ /leader 네임스페이스 필수
#   → /leader/joint_states (7관절), /leader/joint_trajectory (300 Hz)

# ── 리더 → UR16e 브리지 (관절 직결, IK 없음 = 특이점 제약 없음) ──
#    sim 검증됨: 전체 추종오차 0.24° (브리지 매핑 0.02°). Servo 불필요
ros2 launch ur_bringup teleop_omy.launch.py use_sim_time:=true virtual_leader:=true
#    virtual_leader:=true → 하드웨어 없이 /leader/joint_states 합성 (sim 검증용).
#    실물은 false + 위 omy_l100_leader_ai.launch.py 를 port_name:=/dev/ttyUSB0 로
python3 src/ur_bringup/isaac/common/switch_control_mode.py streaming
ros2 service call /omy_bridge/enable  std_srvs/srv/Trigger
#   ★ 리더가 로봇 현재자세와 안 맞으면 engage 거부 + 어긋난 관절을 도(deg)로 알려줌.
#     리더를 손으로 맞춘 뒤 다시 호출할 것 (이 게이트가 팔이 튀는 걸 막는다)
ros2 service call /omy_bridge/disable std_srvs/srv/Trigger
#   부호/오프셋/속도상한/clamp 전부 파라미터 — 실물 튜닝 시 코드 수정 불필요

# ── IL 데모 기록 (teleop 위에서) ──
ros2 run ur_bringup il_recorder.py --ros-args -p use_sim_time:=true \
    -p out_dir:=<데이터경로> -p task:="put the blue block in the green zone"
#   패드: Square=start / Triangle=stop+save / Cross=discard  (또는 /il/{start,stop,discard}_episode)
#   태스크 변경: ros2 param set /il_recorder task "..."   ← 3종 이상 모을 것
#   ML 환경에서 변환: python3 scripts/raw_to_lerobot.py --raw <데이터경로> --repo-id <user>/<name>

# ── cuMotion (GPU 모션플래닝, MoveIt 플러그인) ── 세트2/3 제어 위에서
ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py               # move_group + cuMotion(기본 pipeline)
python3 src/ur_bringup/isaac/common/moveit_plan_execute_demo.py                # plan+execute via cuMotion
#   설치(Isaac ROS cuMotion + CUDA13 + VPI)·함정·로봇설정은 HARDWARE.md §4 / cumotion/README.md

# ── 실시간 장애물 회피 (cuMotion + nvblox) ── 세트3, "카메라가 본 장애물을 GPU 가 실시간 회피"
#   정적 카메라(회피용) + eye-in-hand(파지용) 2대 + 데모 박스 장애물
/isaac-sim/python.sh src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd \
    --with-camera --with-static-cam --obstacle
ros2 launch ur_bringup ur16e_2f85_d405.launch.py use_sim:=true                 # 제어
ros2 launch ur_bringup ur16e_2f85_d405_nvblox.launch.py use_sim_time:=true     # segmenter + nvblox + 정적카메라 TF
ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py \
    use_sim_time:=true read_esdf_world:=true ur_only:=true                     # cuMotion 이 nvblox ESDF 읽음 + RViz 복셀
python3 src/ur_bringup/isaac/ur16e_2f85_d405/nvblox_obstacle_demo.py           # 회피 A/B 검증(FREE 성공·OBST 실패=PASS)
#   RViz: goal 마커를 장애물 너머로 → Plan → cuMotion 이 우회 → Execute. 원리·함정은 HARDWARE.md §4.

# ── pick & place (T3-D) ── 세트3 제어 + MoveIt 위에서. 검증: 6/6, 배치 오차 0.000~0.001 m
/isaac-sim/python.sh src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd \
    --scene pick_place --table --table-height 0.20 --table-pose 0.72,0.0 --table-size 0.70,0.90 \
    --object-pose 0.60,0.0,0.225 --object-size 0.035,0.035,0.035 \
    --place-pose 0.60,0.315,0.0 --object-names red,blue --place-names left,right \
    --object-spacing 0.20 --randomize-object --randomize-radius 0.025 \
    --grasp-attach --with-camera --with-static-cam
ros2 launch ur_bringup ur16e_2f85_d405.launch.py use_sim:=true                 # 제어
ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py use_sim:=true ur_only:=false
ros2 launch ur_bringup pick_place_demo.launch.py use_sim:=true cycles:=6       # 상태머신
#   ur_only:=false 필수 — true 면 move_group 이 팔만 아는 모델을 써서 gripper_frame 을 모르고,
#   MoveIt 은 모르는 링크 제약을 "이미 만족"으로 처리해 매번 SUCCESS 를 반환하며 팔이 안 움직인다.
#   그리퍼 관련 값은 스크립트/런치 기본값에 들어 있어 명령줄에 쓸 필요가 없다.
#   config/common/pick_place_sim.yaml(use_sim:=true 일 때만 로드)은 sim 전용 보정 자리인데
#   지금은 비어 있다 — 있던 두 보정이 sim/real 차이가 아니라 우리 버그였다. HISTORY.md §29.
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

# pick & place — sim 과 같은 상태머신, 백엔드만 다르다
ros2 launch ur_bringup pick_place_demo.launch.py use_sim:=false cycles:=1
#   use_sim:=false 는 config/common/pick_place_sim.yaml 을 로드하지 않는다. 그 파일은
#   현재 비어 있으므로 지금은 sim 과 real 이 **완전히 같은 파라미터**로 돈다(HISTORY.md §29).
#   배선을 남겨둔 이유는 방향이 중요해서다: sim 파일을 빠뜨리면 sim 이 시끄럽게 실패하지만,
#   sim 보정이 실물로 새면 팔이 테이블을 향해 조준된다(§26.2 의 31 mm 가 그랬다).
#   ⚠ 첫 실행은 반드시 cycles:=1 로, 속도를 낮춰(vel_scale/acc_scale) 사람이 지켜보며.
```

---

## 7. 현재 상태

| | sim | real(SW 준비) | real(HW 검증) |
|---|---|---|---|
| 세트 1 (팔) | ✅ plan+execute | ✅ (mock 검증) | ⏳ 로봇 연결 시 |
| 세트 2 (+2F-85) | ✅ 그리퍼 개폐·자기충돌·plan+execute | ✅ `robotiq_driver` (mock 검증) | ⏳ 그리퍼 연결 시 |
| 세트 3 (+D405) | ✅ 카메라·OctoMap·plan+execute | ✅ `realsense2_camera` 노드 로드+카메라 TF | ⏳ D405 USB3 연결 시 (영상 스트림·hand-eye) |
| **cuMotion (GPU 플래너)** | ✅ MoveIt 파이프라인 plan+execute (오차 0.0003 rad) | ✅ 동일 launch, `use_sim_time:=false` | ⏳ 로봇 연결 시 (실행 경로 동일) |
| **실시간 장애물 회피 (nvblox)** | ✅ 정적카메라→segmenter→nvblox ESDF→cuMotion; A/B 회피 검증 `PASS`(`nvblox_obstacle_demo.py`). **`nvblox_ros` 는 GPU arch 맞춰 소스 빌드**(SETUP.md §2-B-4) | ⏳ 정적 depth 카메라(D455 등) 추가 시 (토픽만 교체) | ⏳ 카메라 연결 시 |

위 sim 항목은 **Isaac Sim 6.0.1 / RTX 5090 에서 2026-09-06 전수 재검증**됨.
자세한 검증 로그/날짜/근거는 [`HISTORY.md`](HISTORY.md) (§12 nvblox 실시간 회피, §14 Isaac Sim 6.0.1 이식).

### IL 트랙 (ACT) — 2026-09-09

| 항목 | 상태 |
|---|---|
| 수집 (GT 상태머신) | ✅ 100/100, 사이클 11 s, 정지 프레임 8.6% |
| 변환 (LeRobot v3.0) | ✅ 43,222 프레임 @30 Hz, 320×240 ×2 cam, 후처리 불필요 |
| 학습 (ACT 60k) | ✅ 1 h 08 m, loss 0.029 |
| **롤아웃** | ✅ **9/10**, 성공 시 평균 오차 **7.6 mm** (최소 1 mm) |

정책 체크포인트 `outputs/act_red_left_100`, 데이터셋 `outputs/lerobot_ds_red_left_100`.
여기까지 오는 데 고친 것들(그리퍼 규약·대기시간·그립 기하·데이터량)은 [`HISTORY.md`](HISTORY.md)
§28~§38. **특히 §35.4 — "측정했다"와 "맞는 것을 측정했다"는 다르다.**

**★ ACT 는 지시문을 읽지 않는다**(§30). 태스크 1종당 데이터셋 1개·체크포인트 1개.
여러 태스크를 섞은 `outputs/lerobot_ds_240_v2` 는 VLA(GR00T/π) 단계용이다.

---

## 8. 종료

```bash
pkill -f ur16e_isaac_ros2.py ; pkill -f "ros2 launch ur_bringup" ; pkill -f "lib/rviz2/rviz2"
```
> 같은 컨테이너에서 다른 워크스페이스가 GPU/ROS 를 공유할 수 있으므로 광범위한 `pkill`(`python3`,
> `ros2_control_node` 등)은 피하고 워크로드 전용 패턴만 사용.
