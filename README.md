# UR16e — ROS 2 Jazzy + Isaac Sim 6.0.1 / 6.1.0 (sim & real 공용 제어 스택)

**UR16e 로봇팔을 하나의 ROS 2 (Jazzy) 소프트웨어로 Isaac Sim 시뮬레이션과 실물에서 모두** 구동하는
워크스페이스. UR 공식 스택(`ur_robot_driver` + `ros2_control` + MoveIt2) 기반이며, 자체 코드는
`ur_bringup` 한 패키지에 모여 있다.

- 워크스페이스: `/isaac-sim/volume/ur_ws` (colcon), git repo = `src/`
- 환경: Docker, ROS 2 **Jazzy**, **Isaac Sim 6.0.1 / 6.1.0**(`/isaac-sim`), GPU(RTX 5090 sm_120).
  새 컨테이너 재현은 `src/setup/bootstrap.sh --fresh` 한 줄 — 2026-09-16 Isaac 6.1.0 기본 이미지에서 처음부터 검증([`HISTORY.md`](HISTORY.md) §48).
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

> **★ 새 PC 라면 아래를 손으로 하지 말고 스크립트로 끝낼 것** — 순서·핀·GPU 분기·검증까지 전부 들어 있다:
> ```bash
> # (0) src/ 를 /isaac-sim/volume/ur_ws/src 에 둔다 — 기본 이미지엔 git 이 없으니 호스트 쪽 마운트 경로에 clone/복사
> git clone <repo> <호스트의 볼륨 경로>/ur_ws/src
> # (1) root 레이어 (호스트에서, 1회) — 기본 Isaac 6.1.0 이미지엔 sudo·git·gpg·locales·GUI(Vulkan/GL) 라이브러리가 없다
> #     ★ 전용 컨테이너 전제: 기반 패키지 약 10 개(libc6·mesa·curl …)도 업그레이드한다 — 설치 전에 apt 시뮬레이션을 찍는다
> docker exec -u 0 -it <container> bash /isaac-sim/volume/ur_ws/src/setup/container_prereqs.sh --sudo-user isaac-sim
> # (2) 컨테이너 안에서 (일반 사용자)
> /isaac-sim/volume/ur_ws/src/setup/bootstrap.sh --dry-run   # 계획만 먼저
> /isaac-sim/volume/ur_ws/src/setup/bootstrap.sh --fresh     # 전용 새 컨테이너 (ROS 없는 기본 이미지는 이것)
> # /isaac-sim/volume/ur_ws/src/setup/bootstrap.sh           # 공유 컨테이너 (기존 패키지 업그레이드 거부)
> ```
> 전체 순서와 전제조건은 [`SETUP.md`](SETUP.md) **§0-B**(2026-10-21 순정 `nvcr.io/nvidia/isaac-sim:6.1.0` 에서 호스트 검증, `HISTORY.md` §57). 아래는 손으로 할 때의 참고다.

```bash
# (1) 표준 스택 — apt
#   robotiq-description = 세트2/3 필수(없으면 URDF 가 생성조차 안 됨),
#   moveit-ros-perception = 세트3 octomap (depth perception 플러그인)
sudo apt update && sudo apt install -y \
    ros-jazzy-ur ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-robotiq-description \
    ros-jazzy-moveit-ros-perception

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
> 매 터미널 먼저: `source /opt/ros/jazzy/setup.bash && source /isaac-sim/volume/ur_ws/install/setup.bash && export ROS_DOMAIN_ID=42`
> (도메인은 0 이 아니고 **같은 호스트의 컨테이너마다 다른 값**, **모든 터미널에서 같게**. 42 = 이 워크스페이스(호스트의 첫 컨테이너) —
> 같은 호스트에 두 번째 컨테이너면 다른 값으로. 도메인이 겹치면 그쪽 `/robot_description` 을 받아 `controller_manager` 가 죽는다.
> 하네스는 환경값을 물려받고 안 정하면 0)

---

## 5. 실행 — 시뮬레이션 (Isaac Sim)

기동 순서: **Isaac(안정화) → 제어 → MoveIt**. 세트별로 Isaac 에셋(`--asset-path`)만 다르다.
Isaac 스크립트는 **실행 중 NVIDIA 에셋 서버(네트워크)가 필요**하다 — 시작 시 asset root 를 못 찾으면 `--no-env` 여도 종료한다
(룸 환경·세트1 기본 UR16e USD 가 거기서 온다. 세트2/3 USD 는 로컬).
디스플레이 없는(headless) 컨테이너면 Isaac 은 `--headless`, MoveIt 런치(`*_moveit.launch.py`, 기본 RViz 켜짐)는 `launch_rviz:=false`.

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
python3 src/ur_bringup/isaac/common/reset_pose.py ready      # = 텔레옵 랑데부 = 데이터 시작 자세 (2026-09-17 실측, §49.8; 구 자세 ready_v2/ready_v1)
python3 src/ur_bringup/isaac/common/switch_control_mode.py streaming
ros2 launch ur_bringup teleop_dualsense.launch.py                    # 패드 (L1=deadman 유지, L2/R2=그리퍼)
ros2 run moveit_servo servo_keyboard_input                           # 패드 없으면 키보드로 대체
python3 src/ur_bringup/isaac/common/switch_control_mode.py trajectory  # MoveIt/cuMotion 으로 복귀

# ── OMY-L100 리더 (teleop leader, 중력보상 ros2_control) ── 설치: src/setup/setup.sh leader
#    ★ 하드웨어 없이도 검증됨. 실물은 port_name:=/dev/ttyUSB0 (U2D2). 상세 SETUP.md 2-D
ros2 launch ur_bringup omy_leader.launch.py \
    use_mock_hardware:=true use_self_collision_avoidance:=false   # ★ 후자 반드시 false
ros2 control list_controllers -c /leader/controller_manager       # ★ /leader 네임스페이스 필수
#   → /leader/joint_states (7관절), /leader/joint_trajectory (300 Hz)

# ── 리더 → UR16e 브리지 (관절 직결, IK 없음 = 특이점 제약 없음) ──
#    sim 검증됨: 추종오차 0.116°(2026-10-21). Servo 불필요.  pad:=true 면 게임패드로도 조작
ros2 launch ur_bringup teleop_omy.launch.py use_sim_time:=true virtual_leader:=true pad:=true
#    virtual_leader:=true → 하드웨어 없이 /leader/joint_states 합성 (sim 검증용).
#    실물은 false + 위 omy_leader.launch.py(ROBOTIS 런치 래퍼) 를 port_name:=/dev/ttyUSB0 로

# ── 랑데부: 임의 자세에서 engage 하지 않는다 ──
#    리더를 rest pose(손 떼도 서 있는 자세)로 내려놓고 → UR16e 를 MoveIt 으로 이동
ros2 service call /omy_bridge/sync    std_srvs/srv/Trigger   # 충돌 검사됨. move_group 필요
ros2 topic echo   /omy_bridge/status                         # sync:moving → synced
ros2 topic echo   /omy_bridge/engage_error                   # [rad] 관절별 오차 (5 Hz)
ros2 service call /omy_bridge/enable  std_srvs/srv/Trigger
ros2 service call /omy_bridge/sync_to_leader std_srvs/srv/Trigger   # 대안: UR16e 를 리더의 현재 자세로 (캘리브용)
# IL 기록의 action 토픽 = /omy_bridge/command_joint_states (UR 이름, 매핑 후 명령). /leader/joint_states 아님
#   ★ 리더가 로봇 현재자세와 안 맞으면 engage 거부 + 어긋난 관절을 도(deg)로 알려줌.
#     engage_error 를 띄워 두면 호출 없이 보면서 맞출 수 있다 (이 게이트가 팔이 튀는 걸 막는다)
#   ★ streaming 컨트롤러가 비활성이면 enable 도 거부 — "engage 됐는데 안 움직임" 방지
ros2 service call /omy_bridge/disable std_srvs/srv/Trigger
#   패드: Options=enable / R3=sync (데드맨 L1 필요) · Create=disable (불필요)
#   부호/오프셋/속도상한/clamp/랑데부 전부 런치 인자 — 실물 튜닝 시 코드 수정 불필요
#   ★ 팔로워가 리더보다 느리면 브리지 로그 `slew capped N%` → max_joint_speed(기본 1.0, 실물 0.3→2.0) 가 병목. HARDWARE.md 4-B ⑤
#   ★ 글리치 가드: 리더 샘플 간 속도 > max_leader_speed(20 rad/s) 이고 스텝 > min_leader_jump(0.1 rad) 면 disable, status leader_jump
#   ★ offset/sign 은 DISABLED 일 때 ros2 param set 으로 즉시 변경된다(캘리브 반복용).
#     ENGAGED 중이면 거부(팔이 움직인다). 나머지 파라미터는 param set 이 조용히 무시되지
#     않고 거부되며 재기동 명령을 알려준다. param set 값은 노드와 함께 사라진다.

# ── IL 데모 기록 (teleop 위에서) ── physical 씬(--grasp-mode physical, 수집 기본) 기준
ros2 run ur_bringup il_recorder.py --ros-args -p use_sim_time:=true \
    -p out_dir:=<데이터경로> -p task:="put the blue block in the green zone" \
    -p grasp_mode:=physical -p gripper_action_topic:=/isaac_joint_commands
#   ★ physical 씬에서 패드/Servo 텔레옵을 기록할 때 gripper_action_topic 을 빼면 기록기가 시작을 거부한다(/il/status 에 이유) —
#     그대로 기록하면 action.gripper 가 jaw 정지값(0.506)이라 정책이 쥐지 않는다(HISTORY.md §53.6·§53.7). 하네스 collect_il_episodes.sh 는
#     GRASP_MODE=physical 이면 두 파라미터를 자동으로 붙인다. 용접 씬(--grasp-attach)이면 grasp_mode:=attach, gripper_action_topic 생략
#   리더 action 을 기록하려면: -p action_source:=topic -p action_topic:=/omy_bridge/command_joint_states (engaged 후 시작)
#     — 이 토픽의 finger_joint(그리퍼 명령)를 그대로 쓰므로 gripper_action_topic 은 필요 없다
#   꼬리 정지 프레임 폐기: -p trim_tail_frames:=N (기본 0; 먼저 ros2 run ur_bringup il_tail_stats.py <raw_dir> 로 잰다)
#   패드: Square=start / Triangle=stop+save / Cross=discard  (또는 /il/{start,stop,discard}_episode)
#   태스크 변경: ros2 param set /il_recorder task "..."   ← 3종 이상 모을 것
#   변환은 ML venv 로 (워크스페이스 루트에서, 시스템 python3 엔 lerobot 이 없다 — PIPELINE.md §2):
#     deps/.venv-ml/bin/python src/ur_bringup/scripts/raw_to_lerobot.py --raw <데이터경로> --repo-id <user>/<name> --root <출력경로>

# ── cuMotion (GPU 모션플래닝, MoveIt 플러그인) ── 세트2/3 제어 위에서
ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py               # move_group + cuMotion(기본 pipeline)
python3 src/ur_bringup/isaac/common/moveit_plan_execute_demo.py                # plan+execute via cuMotion
#   설치(Isaac ROS cuMotion + CUDA13 + VPI)·함정·로봇설정은 HARDWARE.md §4 / cumotion/README.md
#   로봇 충돌 구(XRDF) 재생성·검사 — 2026-10-16 재생성(메시를 덮는 world 구 + self 구, HISTORY.md §51):
/isaac-sim/python.sh src/ur_bringup/cumotion/gen_xrdf.py && /isaac-sim/python.sh src/ur_bringup/cumotion/check_xrdf.py   # PASS
#   다른 XRDF 로 A/B: ...cumotion_moveit.launch.py xrdf:=/abs/path.xrdf  (플래너만; nvblox 의 segmenter 는 배포 파일 고정, 빈 값 금지)

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
#   데모는 계획 전에 ESDF 에 장애물이 실제로 들어올 때까지 기다린다(최대 180 s, 안 들어오면 `ESDF NOT READY` rc 3 — 판정 안 함).
#   대조: cuMotion 을 read_esdf_world:=false 로 다시 띄우고 `nvblox_obstacle_demo.py --control`(OBST 도 성공=PASS)
#   2026-10-20 재생성 XRDF + Isaac 6.1.0 으로 재실행: PASS, ESDF 끈 대조에선 OBST 도 성공(HISTORY.md §54.4)
#   RViz: goal 마커를 장애물 너머로 → Plan → cuMotion 이 우회 → Execute. 원리·함정은 HARDWARE.md §4.

# ── pick & place (T3-D) ── 세트3 제어 + MoveIt 위에서. 검증: attach 6/6(배치 오차 0.000~0.001 m) · physical 20/20(HISTORY.md §53)
/isaac-sim/python.sh src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd \
    --scene pick_place --table --table-height 0.20 --table-pose 0.72,0.0 --table-size 0.70,0.90 \
    --object-pose 0.60,0.0,0.225 --object-size 0.035,0.035,0.035 \
    --place-pose 0.60,0.315,0.0 --object-names red,blue --place-names left,right \
    --object-spacing 0.20 --randomize-object --randomize-radius 0.025 \
    --grasp-mode physical --with-camera --with-static-cam
#   --grasp-mode physical = 용접 없이 실제 접촉으로 파지(soft 그리퍼 프로파일, HISTORY.md §53). 2026-10-19 부터 수집 기본.
#     상태머신엔 extra_params:=$(ros2 pkg prefix ur_bringup)/share/ur_bringup/config/common/pick_place_physical.yaml
#     (구) --grasp-attach = 용접. 그 시대 데이터·체크포인트 재현 때만(+ 상태머신 기본값, 하네스 GRASP_MODE=attach)
#     ★ 리더/패드 텔레옵도 physical 권장 — 원래 게인은 닫히는 순간 손목을 2~3 rad/s 로 걷어찬다(§53.1)
ros2 launch ur_bringup ur16e_2f85_d405.launch.py use_sim:=true                 # 제어
ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py use_sim:=true ur_only:=false
ros2 launch ur_bringup pick_place_demo.launch.py use_sim:=true cycles:=6 \
    extra_params:=$(ros2 pkg prefix ur_bringup)/share/ur_bringup/config/common/pick_place_physical.yaml   # 상태머신 (physical)
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
ros2 launch ur_bringup ur16e_2f85_moveit.launch.py use_sim_time:=false   # ★ 실물엔 /clock 이 없다 (기본 true)

# 세트 3: 팔 + 2F-85 + D405 (USB3 직결)
ros2 launch ur_bringup ur16e_2f85_d405_real.launch.py robot_ip:=<UR16e_IP> \
    cam_xyz:="x y z" cam_rpy:="r p y"          # hand-eye 캘리브 결과 (기본=sim 명목)
ros2 launch ur_bringup ur16e_2f85_d405_moveit.launch.py use_sim_time:=false

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

위 sim 항목은 **Isaac Sim 6.0.1 / RTX 5090 에서 2026-09-06 전수 재검증**됨. **Isaac Sim 6.1.0-rc.26** 에서는
2026-09-16 새 컨테이너에서 세트1 mock, 세트2 plan+execute, 세트3 카메라 2대 + cuMotion `pick_place_demo` 1/1 SUCCESS
를 확인(§48). nvblox A/B 회피 데모는 2026-10-20 6.1.0 + 새 XRDF 로 재실행해 PASS(§54.4).
자세한 검증 로그/날짜/근거는 [`HISTORY.md`](HISTORY.md) (§12 nvblox 실시간 회피, §14 Isaac Sim 6.0.1 이식, §48 6.1.0 재현).

### IL 트랙 (ACT) — 2026-09-09

| 항목 | 상태 |
|---|---|
| 수집 (GT 상태머신) | ✅ 100/100, 에피소드 14.3 s(중앙값), 정지 프레임 8.6% (2026-10-20 재측정) |
| 변환 (LeRobot v3.0) | ✅ 43,222 프레임 @30 Hz, 320×240 ×2 cam, 후처리 불필요 |
| 학습 (ACT 60k) | ✅ 1 h 08 m, loss 0.029 |
| **롤아웃** | ✅ **9/10**, 성공 시 평균 오차 **7.6 mm** (최소 1 mm) |
| **손목 카메라 1대 ACT** (2026-09-13) | ✅ 49 ep, 60k 스텝 **40 m**, 롤아웃 **8/9**(1 SKIP = 관측 정체, 채점 제외), 평균 오차 17 mm — 정적 카메라 없이 서비스됨 ([`HISTORY.md`](HISTORY.md) §45.7) |
| **물리 접촉 파지로 재수집한 ACT** (2026-10-20) | ✅ 수집 100/100(정지 10.0 %) → 60k 학습 1 h 06 m → 롤아웃 **8/10**(시드 131)·**9/10**(새 시드 171, 영상) — 용접 없이 마찰로 쥠. 용접 데이터 체크포인트는 같은 씬에서 0/10. **200k 까지 이어 학습 → 19/20**(9/10·10/10), 성공 오차 12 → 6 mm ([`HISTORY.md`](HISTORY.md) §53.6·§54.5·§54.6) |
| **무작위 범위 확장 ACT — 최종** (2026-10-22) | ✅ 기존 100 ep(±60 mm) + x 바깥 띠 60–100 mm 90 ep(시드 301, 30 % 는 yaw 를 접기 경계 ±10° 에서) = **190 ep · 94,574 프레임** → 200k 학습 ≈ 4 h 23 m(≈ 12.7 step/s, loss 0.013, 100k 에서 중단 후 `--resume`) → 기존 범위 **19/20**(시드 131·171), 넓은 범위(x ±100 mm) **20/20**(시드 211·251; 기존 100-ep 200k 는 13/20 — 실패 7 회 전부 ±60 mm 밖 조준 실패), 성공 오차 4–6 mm. 140k 30/30, 100k 27/30. yaw 추종은 여전히 약함(기울기 −0.62…+0.05, 시연 −1.00) ([`HISTORY.md`](HISTORY.md) §58) |
| 변환 코덱 | 기본 **h264 crf23**(`--vcodec`). AV1 과 같은 크기·1 dB 이내. 기존 AV1 데이터셋은 그대로 사용 (§45.2) |

정책 체크포인트 `outputs/act_red_left_190_phys/checkpoints/200000`(2 cam, physical, 190 ep, **최종·권장** — 기존 범위 19/20 · 넓은 범위 20/20, §58.7) · `outputs/act_red_left_100_phys_long/checkpoints/200000`(2 cam, physical, 100 ep — 기존 범위 19/20, 넓은 범위 13/20) · `outputs/act_red_left_100_phys`(2 cam, physical 60k — 하네스 기본) · `outputs/act_red_left_100`(2 cam, 용접) · `outputs/act_wrist_only`(손목 1 cam, 용접), 데이터셋 `outputs/lerobot_ds_red_left_190_phys`(190 ep, raw `il_raw_red_left_190_phys` = `_100_phys` + `_band_phys` 심볼릭 링크) · `outputs/lerobot_ds_red_left_100_phys` · `lerobot_ds_red_left_100` · `lerobot_ds_wrist_only`.
`outputs/` 는 git 에 없다 — 새 PC 로 가져갈 것은 추론만이면 `outputs/act_red_left_190_phys/checkpoints/200000/pretrained_model`
(≈198 MB) 하나. 이어 학습까지 하려면 `training_state`(≈394 MB) + 데이터셋 `outputs/lerobot_ds_red_left_190_phys`(382 MB)도 필요하고,
`train_config.json` 엔 절대경로(`/isaac-sim/volume/ur_ws/...`)가, `checkpoints/last` 는 심볼릭링크가 들어 있다. 절차는
[`CHECKLIST.md`](CHECKLIST.md) B-3(옛 PC 의 학습 산출물 옮기기), 200k 이어 학습 명령은 [`PIPELINE.md`](PIPELINE.md) §3-1(100-ep 판). 190-ep 판의 중단 후 재개는 `outputs/harness_logs/resume_train_190.sh`(LeRobot `--resume`) — **기존 체크포인트에 `train_100.sh` 를 다시 돌리면 시작 시 `rm -rf "$CK"` 로 지워진다**(§58.6).
수집·변환·학습·롤아웃 자동화 스크립트는 [`ur_bringup/scripts/harness/`](ur_bringup/scripts/harness/README.md)(ACT `pipeline_*.sh`, GR00T `groot_pipeline.sh` = 수집→변환→학습→롤아웃 무인).
다른 데이터셋·범위는 환경변수 덮어쓰기로(기본 동작 불변, §58.2): `collect_100.sh` `OUT/SEED/TAG/RAND`, `convert_100.sh` `RAW/DS/REPO/TAG`, `train_100.sh` `DS/REPO/CK/STEPS/SAVE_FREQ/TAG`, `rollout_gui_then_n.sh` `RAND/SEED` + 3번째 인자 = 체크포인트. 덮어쓰기를 주면 삭제 대상인 `OUT`/`DS`/`CK` 가 필수다(없으면 rc 2 — 기본값이 원본 `*_100_phys` 라 빠뜨리면 원본이 지워진다). Isaac 인자 `--randomize-radius-x/-y`·`--randomize-inner-x`·`--randomize-yaw-edge-frac/-deg`.
여기까지 오는 데 고친 것들(그리퍼 규약·대기시간·그립 기하·데이터량)은 [`HISTORY.md`](HISTORY.md)
§28~§38. **특히 §35.4 — "측정했다"와 "맞는 것을 측정했다"는 다르다.**

**★ ACT 는 지시문을 읽지 않는다**(§30). 태스크 1종당 데이터셋 1개·체크포인트 1개.
여러 태스크를 섞은 `outputs/lerobot_ds_3task_v3`(30 ep/태스크, 현재 정본) · `lerobot_ds_240_v2`(7 ep/태스크) 는 VLA(GR00T/π) 단계용이다.

### VLA 트랙 (GR00T N1.7, 카메라 2대) — sim 파이프라인 검증 완료 2026-09-13, 30 ep/태스크 재판정 2026-09-15

설계 정본 [`ur_bringup/docs/plan_groot_n17.md`](ur_bringup/docs/plan_groot_n17.md).
GELLO 대조·실물 BM 항목: [`ur_bringup/docs/gello_comparison.md`](ur_bringup/docs/gello_comparison.md).
**모델·데이터 경로에 새로 짤 코드가 0** — ACT → GR00T 는 전부 설정 변경이다.

| 항목 | 상태 |
|---|---|
| 재사용 경계 확정 | ✅ 데이터셋 스키마·수집/변환·`async_inference`·`UR16eROS` 전부 그대로 — **끝까지 신규 코드 0** |
| 의존성 `lerobot[groot]` | ✅ 19개 신규, 기존 변경 0, torch 2.11.0+cu128/sm_120 유지 |
| 베이스 가중치 6.5 GB + HF 게이트 | ✅ `deps/hf_cache`(토큰도 여기, git 밖). `Cosmos-Reason2-2B` 는 `gated=auto` — 동의 즉시 열림 |
| **카메라 2대** | ✅ rename 없이. 전처리 출력 실측: 2대 → `image_grid_thw (2,3)`, 픽셀·토큰 정확히 2배 |
| **VRAM / 처리량** | ✅ `fp32=false` + **batch 32** = 27.3 GB. **`OMP_NUM_THREADS=8`** 로 23→51 샘플/s(2배, §45.4 — `data_s` 는 디코딩이 아니라 메인 프로세스 전처리기의 스레드 과다할당이었다). 워커 8개는 shm 사망 |
| **학습 (6h 게이트)** | ✅ 10k 스텝 **1h 48m**(`OMP_NUM_THREADS=8`, §46.1; 이전 3h 45m), loss →0.014 (lerobot 기본 100k 는 안 쓴다) |
| 상대 액션 | ⚠️ 학습은 되지만 **`async_inference` 서버가 서비스 못 함**(스텝당 후처리 vs 청크 디코드) → **절대 액션 채택** |
| **추론 지연 (V7)** | ✅ 청크 40액션 **80.8 ms** (예산 1,333 ms, 16.5배 여유) |
| **롤아웃 (V8)** | ✅ 루프 닫힘 — 7 ep/태스크 **3/8**(§44.3) → **30 ep/태스크 14/29**(ACT 씬 반경 0.06, §46.2), `WRONG_OBJECT`·`WRONG_PLACE` 0 |
| 성능 비교 | ⚠️ **ACT 9/10 에 못 미친다(48 %)** — 데이터 부족이 아니라 **실패 모드**: 사전파지 높이까지 정확히 가서 멈춤(14/15 건, 시작 자세 무관). 성공 배치 오차 22.8 mm(ACT 7.6). `num_inference_timesteps` 4→16 은 **효과 없음**(9/27, §46.7). 갈림길 §46.6 |

체크포인트 `outputs/groot_3task_v3_abs`(현재 정본, 절대) · `outputs/groot_240_v2_abs`(7 ep 판) · `outputs/groot_240_v2_rel`(상대, 학습만).
함정은 [`HISTORY.md`](HISTORY.md) §40(설계 3개)·§43(학습 4개)·§44(서비스 2개). **특히 §43.6·§43.7·§44.2 —
셋 다 "정책이 아무것도 안 함"으로 보이는 조용한 하네스 실패**였고, 이제 하네스가 전제를 검사한다.

### 텔레옵 랑데부 + sync/패드 — 2026-09-10

UR16e 가 다른 작업을 하다 텔레옵으로 넘어올 때 시작 자세가 어긋나는 문제. **랑데부 = `reset_pose.py ready`
= 실물 L100 이 손 떼도 서 있는 자세(실측 [−1.1,−88.4,152.0,−69.6,87.5,1.6]°)를 실측 매핑 `sign [1,−1,−1,−1,1,−1]`,
`offset [180,−90,0,−90,0,0]°` 로 옮긴 [178.9°, −1.6°, −152.0°, −20.4°, 87.5°, −1.6°]**(2026-09-17 확정, `HISTORY.md` §49.8 —
URDF 유도 매핑은 J5 방향이 틀렸고 사용자가 고른 거울 가지가 L100 과 같은 모양; 2026-09-16 URDF 가지는 `ready_v2`,
그 전 SRDF `home` 매핑은 `ready_v1`). 리더는 자동 구동되지 않고(중력보상 전용) 사람이 내려놓는다.

| 추가된 것 | 상태 |
|---|---|
| `/omy_bridge/sync` — MoveIt 으로 UR16e 를 랑데부로(컨트롤러 전환 포함, streaming 으로 복귀) | ✅ mock + Isaac sim, 잔차 0.5° |
| `/omy_bridge/engage_error` — 관절별 오차 6개 @5 Hz | ✅ |
| `/omy_bridge/sync_to_leader` — UR16e 를 리더의 매핑 자세로(MoveIt, GELLO 방식) | ✅ mock + MoveIt 2026-09-17 (§49) |
| `/omy_bridge/command_joint_states` — 매핑·clamp·slew 후 명령(JointState, UR 이름 + `finger_joint`) = IL action | ✅ 기록→변환 round-trip (§49) |
| `max_leader_speed` — engaged 중 리더 샘플 간 속도가 비정상(기본 20 rad/s)이면 자동 disable, status `leader_jump` | ✅ 오탐 2종 없음·글리치 차단 (§49.4) |
| 패드 바인딩 — Options=enable · R3=sync(데드맨 필요) · Create=disable | ✅ |
| `enable` 가드 — streaming 비활성이면 거부 | ✅ |
| 리더 추종 회귀 | ✅ 오차 **0.14°** (기존 0.24°) → 2026-10-21 순정 이미지 호스트 재현 **0.116°** |

[`HISTORY.md`](HISTORY.md) §41·§42, 실물 측정 항목은 [`CHECKLIST.md`](CHECKLIST.md) E-1~E-3.

---

## 8. 종료

```bash
bash /isaac-sim/volume/ur_ws/src/ur_bringup/scripts/harness/shutdown_rollout.sh   # 마지막 줄 CLEAN 확인 / 학습·변환까지: shutdown_all.sh
```
> 실행 파일 경로로만 고른다(`$WS/install/**`, `src/ur_bringup/{scripts,isaac}/**`, `deps/.venv-ml`; `/opt/ros/jazzy/lib/<pkg>/<exe>` 와 `ros2 launch ur_bringup` 은 환경에 이 워크스페이스 install 이 source 된 경우만) — 같은 컨테이너의 다른 워크스페이스는 건드리지 않는다. 끝나면 남은 프로세스 + `ros2 node list` 로 `CLEAN` / `LEFT: …`(rc 1). 상세 `ur_bringup/scripts/harness/README.md` '종료'.
> 예전 3줄 `pkill -f`(Isaac·런치 부모·RViz)는 런치 부모보다 오래 사는 ros2_control_node·move_group·cuMotion 컨테이너·OMY 브리지를 남겼다.
