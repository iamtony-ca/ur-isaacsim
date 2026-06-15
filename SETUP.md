# SETUP — UR16e (ROS 2 Jazzy + Isaac Sim 5.1.0) 재현 매뉴얼

깨끗한 환경에서 이 워크스페이스(sim+real UR16e 제어 스택)를 **그대로 재현**하는 절차.
아키텍처/배경은 상위 [`README.md`](../README.md) 참고.

---

## 0. 대상 환경 (검증된 조합)

| 항목 | 버전 |
|---|---|
| OS | Ubuntu 24.04 (noble) |
| ROS 2 | **Jazzy** (`/opt/ros/jazzy`) |
| Isaac Sim | **5.1.0** (`/isaac-sim`, `isaacsim.ros2.bridge` 확장) |
| GPU | NVIDIA (RTX 5090 검증), 드라이버/Vulkan 정상 |
| 워크스페이스 | `/isaac-sim/ur_ws` (colcon), git repo = `src/` |

> 다른 경로를 쓰면 아래 절대경로(`/isaac-sim/ur_ws`, `/isaac-sim/python.sh`)를 본인 환경에 맞게 치환.

---

## 1. 워크스페이스 가져오기

```bash
# src 저장소를 워크스페이스의 src/ 로 (이미 있으면 생략)
mkdir -p /isaac-sim/ur_ws
cd /isaac-sim/ur_ws
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
    python3-vcstool
```
- `ros-jazzy-ur` = 메타패키지 → `ur_robot_driver`, `ur_description`, `ur_moveit_config`,
  `ur_controllers`, `ur_calibration`, `ur_dashboard_msgs`, `ur_client_library` 포함.
- `ros-jazzy-topic-based-ros2-control` 은 **apt 에 없음** → §3 에서 소스로.
- (대안) `rosdep install --from-paths src --ignore-src -y` 로도 가능하지만, 위 명시 설치가 확실.

---

## 3. 소스 의존성 (vcstool 로 버전 고정)

`topic_based_hardware_interfaces` 는 apt 에 없고 **특정 태그(0.2.1)** 가 필요하므로 소스로 관리.

```bash
cd /isaac-sim/ur_ws
vcs import src < src/ur16e.repos        # topic_based_hardware_interfaces @ 0.2.1 클론
vcs validate src < src/ur16e.repos      # (선택) 버전 일치 확인
```
> `src/ur16e.repos` 가 버전을 `0.2.1` 로 고정한다. **왜 0.2.1 인지는 §8 참고.**
> 이 디렉터리는 `src/.gitignore` 로 무시되어 src 저장소에 박히지 않고 vcs 가 관리.

---

## 4. 빌드

```bash
cd /isaac-sim/ur_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to ur_bringup \
    --cmake-args -DBUILD_TESTING=OFF
#   -DBUILD_TESTING=OFF : 0.2.1 의 ros_testing 빌드 의존(미설치) 회피
```
정상 시: `joint_state_topic_hardware_interface`, `ur_bringup` 2개 빌드 완료.

이후 모든 터미널에서:
```bash
source /opt/ros/jazzy/setup.bash
source /isaac-sim/ur_ws/install/setup.bash
export ROS_DOMAIN_ID=0
```

---

## 5. 시뮬레이션 구동 (Isaac Sim)

**기동 순서를 지킬 것** (§8-6). 터미널 3개:

```bash
# T1: Isaac Sim (씬 + ROS2 OmniGraph 자동 구성)
/isaac-sim/python.sh /isaac-sim/ur_ws/src/ur_bringup/isaac/ur16e_isaac_ros2.py          # GUI(환경/조명 포함)
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
python3 /isaac-sim/ur_ws/src/ur_bringup/isaac/moveit_plan_execute_demo.py   # -> RESULT: SUCCESS
```

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
