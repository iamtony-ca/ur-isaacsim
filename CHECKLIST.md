# CHECKLIST — 새 PC 에서 실물 UR16e + OMY-L100 까지

**이 문서는 "무엇을 어떤 순서로" 만 담는다.** 각 항목의 *이유*와 상세는
[`SETUP.md`](SETUP.md)(소프트웨어) / [`HARDWARE.md`](HARDWARE.md)(하드웨어)에 있고, 링크로 건다.

원칙 두 가지:
- **하드웨어를 꽂기 전에 소프트웨어를 끝낸다.** A~C 는 장치 0개로 전부 검증된다.
- **한 번에 장치 하나씩 붙인다.** 팔 → 리더 → (그리퍼) → (카메라). 두 개를 동시에 붙이면
  둘 중 어느 쪽이 문제인지 가릴 수 없다.

---

## A. 준비물 확인 (설치 시작 전)

| | 항목 | 확인 방법 | 없으면 |
|---|---|---|---|
| ☐ | Isaac Sim **6.0.1** 컨테이너 | `/isaac-sim/python.sh` 존재 | 컨테이너부터. 이 워크스페이스는 컨테이너 밖에서 돌지 않는다 |
| ☐ | ROS 2 **Jazzy** | `/opt/ros/jazzy` 존재 | **스크립트가 설치하지 않는다.** 베이스 이미지를 바꿔야 한다 |
| ☐ | NVIDIA GPU + 드라이버 | `nvidia-smi` | — |
| ☐ | 디스크 **30 GiB** 이상 | `df -h` | ML venv 만 약 8 GB |
| ☐ | `git` `curl` `sudo` | — | — |

컨테이너에 필요한 것은 셋뿐: **GPU 전달**(`--gpus all`), **볼륨**(`/isaac-sim/volume`),
**GUI 를 볼 거면 X 소켓**(`-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`).

---

## B. 소프트웨어 설치 (하드웨어 0개)

```bash
mkdir -p /isaac-sim/volume/ur_ws
git clone <이 저장소> /isaac-sim/volume/ur_ws/src        # ★ repo = src/ 다

/isaac-sim/volume/ur_ws/src/setup/bootstrap.sh --dry-run  # ☐ 먼저 계획만 본다
/isaac-sim/volume/ur_ws/src/setup/bootstrap.sh            # ☐ 실행
```

- ☐ `--dry-run` 출력에 **"기존 패키지 업그레이드/삭제"가 없는지** 확인.
  있으면 멈춘다 — 이 머신의 다른 프로젝트를 깨뜨린다는 뜻이다.
- ☐ 옵션: `--no-ml`(torch/lerobot 8 GB 생략, sim+teleop 만 할 때) /
  `--with-udev`(실물 OMY-L100 이 있을 때만)

돌아가는 순서: `preflight → pin → repos → base → cumotion → sources → build → leader → ml → verify`
(`pin` 이 `repos` 보다 먼저인 이유는 [`SETUP.md`](SETUP.md) §0-B — NVIDIA 레포가 ROS 패키지를 덮어쓴다.)

- ☐ 언제든 재점검: `src/setup/check_env.sh` (읽기 전용)

---

## C. 소프트웨어 검증 (아직 하드웨어 0개)

세 터미널 모두 먼저:
```bash
source /opt/ros/jazzy/setup.bash && source /isaac-sim/volume/ur_ws/install/setup.bash
export ROS_DOMAIN_ID=0
```

- ☐ **C-1 sim pick&place** — Isaac → 제어 → MoveIt → 데모. [`README.md`](README.md) §5
- ☐ **C-2 가짜 리더 teleop** — 리더 없이 브리지 검증:
  ```bash
  ros2 launch ur_bringup teleop_omy.launch.py use_sim_time:=true virtual_leader:=true
  python3 src/ur_bringup/isaac/common/switch_control_mode.py streaming
  ros2 service call /omy_bridge/enable std_srvs/srv/Trigger
  ```
  ✅ 팔이 가짜 리더를 따라가면 성공 (기대 추종오차 ≈ 0.24°).
- ☐ **C-3 mock 실물 경로** — 실물 배선 전에 런치가 뜨는지만:
  ```bash
  ros2 launch ur_bringup ur16e.launch.py use_sim:=false use_mock_hardware:=true
  ```

> **여기까지 통과하면 소프트웨어는 끝이다.** 이후 실패는 전부 배선/설정 문제로 좁혀진다.

---

## D. 실물 ① UR16e 팔 — 이더넷

> 팔만으로 teleoperation 까지 갈 수 있다. 그리퍼·카메라는 나중에 붙여도 된다.

### 배선
- ☐ UR16e 컨트롤박스 ↔ 제어 PC **같은 서브넷 LAN**. 스위치/무선 경유는 RTDE 지터를 만든다
- ☐ `ping <UR16e_IP>` 성공
- ☐ PolyScope 설정 → 네트워크에서 IP 확인

### 펜던트
- ☐ **External Control URCap** 설치 + 프로그램에 추가 (호스트 IP = **제어 PC**, 포트 50002)
  - 펜던트 없이 하려면 런치에 `headless_mode:=true`
- ☐ (권장) 기구학 캘리브레이션 — 정확한 TCP 를 원하면:
  ```bash
  ros2 launch ur_calibration calibration_correction.launch.py \
      robot_ip:=<UR16e_IP> target_filename:="$HOME/ur16e_calib.yaml"
  ```
  결과를 런치의 `kinematics_params_file:=` 로 넘긴다

### 검증
```bash
ros2 launch ur_bringup ur16e.launch.py use_sim:=false robot_ip:=<UR16e_IP>
# 펜던트에서 External Control 프로그램 ▶ 실행 (headless_mode 면 생략)
ros2 control list_controllers          # ☐ scaled_joint_trajectory_controller = active
ros2 topic echo /joint_states --once   # ☐ 6관절 position 유효 (NaN 아님)
ros2 launch ur_bringup ur16e_moveit.launch.py use_sim:=false   # ☐ RViz plan→execute 시 실제 이동
```

상세: [`HARDWARE.md`](HARDWARE.md) §1

---

## E. 실물 ② OMY-L100 리더 — USB (U2D2)

### 배선
- ☐ **U2D2 → PC USB** (`/dev/ttyUSB0`, FTDI)
- ☐ 리더에 **12 VDC 별도 전원** — USB 로는 전원이 안 된다
- ☐ UR16e·그리퍼와 **전기적으로 무관**하다. 팔로워 배선은 그대로 둔다

### udev 규칙 (1회, sudo) — ★ 건너뛰지 말 것
```bash
/isaac-sim/volume/ur_ws/src/setup/setup.sh udev
# 또는 처음부터: bootstrap.sh --with-udev
```
- ☐ 확인: `cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer` → **1**

> FTDI 기본 지연은 **16 ms**. 규칙 없이 쓰면 4 Mbps 다이나믹셀 동기읽기가 실질 **60 Hz** 로 떨어져
> 리더가 못 쓸 물건이 된다. 이 한 줄이 300 Hz 와 60 Hz 를 가른다.

### 리더 단독 확인 (UR16e 없이)
```bash
ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py \
    port_name:=/dev/ttyUSB0 use_self_collision_avoidance:=false
ros2 control list_controllers -c /leader/controller_manager   # ★ /leader 네임스페이스
ros2 topic echo /leader/joint_states --once
```
- ☐ 컨트롤러 4개 전부 `active`
- ☐ 손으로 리더를 움직이면 `/leader/joint_states` 가 따라 변한다

### UR16e 에 연결 (팔만, 그리퍼 없이 가능)
```bash
# 1) 팔                      2) 리더 (위)              3) 스트리밍 모드로 전환
python3 src/ur_bringup/isaac/common/switch_control_mode.py streaming
# 4) 브리지 — 그리퍼가 없으면 gripper_enable:=false
ros2 run ur_bringup omy_to_ur16e.py --ros-args -p gripper_enable:=false
# 5) 리더를 UR16e 의 "현재 자세"에 맞춘 뒤에만 활성화된다
ros2 service call /omy_bridge/enable std_srvs/srv/Trigger
```

- ☐ `enable` 이 **거부되면 리더 자세를 더 맞춘다.** 무리해서 우회하지 않는다 —
  이 게이트가 없으면 16 kg 급 팔이 한 제어주기에 리더 자세로 튄다
- ☐ 첫 시도는 `max_joint_speed` 를 낮춰서, 짧은 동작으로

> ⚠️ **`omy_to_ur16e.py` 는 실물 미검증이다** (sim + mock 까지만). 특히 **J4/J6 오프셋은 유도
> 불가능한 값**이라 실물에서 조작감을 보며 튜닝해야 한다 — L100 의 손목 오프셋 −46 mm 대
> UR16e +290.7 mm 로 크기도 부호도 다르기 때문. 전부 ROS 파라미터라 코드 수정은 필요 없다.

상세: [`HARDWARE.md`](HARDWARE.md) §4-B

---

## F. 실물 ③ Robotiq 2F-85 그리퍼 (선택)

- ☐ 손목 커플링(GRP-ES-CPL-077) 경유 UR tool RS-485
- ☐ 펜던트에서 tool communication 활성
- ☐ `sudo usermod -aG dialout $USER` 후 **재로그인** (`/tmp/ttyUR` 쓰기 권한)
- ☐ 드라이버 별도 빌드:
  ```bash
  colcon build --symlink-install --packages-select serial robotiq_driver robotiq_controllers \
      --cmake-args -DBUILD_TESTING=OFF
  ```
- ☐ `ros2 launch ur_bringup ur16e_2f85_real.launch.py robot_ip:=<IP>`

상세: [`HARDWARE.md`](HARDWARE.md) §2

---

## G. 실물 ④ RealSense D405 (선택)

- ☐ **PC USB3 직결** (파란색/SS 포트). UR 을 거치지 않는다 — 영상 대역폭 때문
- ☐ `realsense-viewer` 또는 `ros2 launch ur_bringup d405_real.launch.py` 로 스트림 확인
- ☐ hand-eye 캘리브레이션 → 결과를 `cam_xyz:=` `cam_rpy:=` 로 전달

상세: [`HARDWARE.md`](HARDWARE.md) §3

---

## H. 데이터 수집 → 학습 → 추론 (실물)

- ☐ 리더로 시연하며 기록:
  ```bash
  ros2 run ur_bringup il_recorder.py --ros-args \
      -p out_dir:=<경로> -p task:="<지시문>" \
      -p action_source:=topic -p action_topic:=/leader/joint_states
  ```
  카메라가 1대뿐이면 `-p cameras.exterior:=none`
- ☐ 변환: `deps/.venv-ml/bin/python src/ur_bringup/scripts/raw_to_lerobot.py --raw <경로> --repo-id <id> --root <출력>`
- ☐ 학습: `deps/.venv-ml/bin/lerobot-train --policy.type=act --policy.push_to_hub=false ...`
- ☐ 추론: [`launch/common/policy_inference.launch.py`](ur_bringup/launch/common/policy_inference.launch.py) 참고

> **ACT 는 지시문을 읽지 않는다.** 태스크 1종당 데이터셋 1개, 체크포인트 1개.
> 여러 태스크를 섞은 데이터셋은 VLA(GR00T/π) 단계용이다 — [`HISTORY.md`](HISTORY.md) §30.

---

## 자주 걸리는 것

| 증상 | 원인 | 확인 |
|---|---|---|
| 팔이 안 움직이는데 에러도 없다 | 스트리밍/궤적 컨트롤러가 **상호배타** | `ros2 control list_controllers` 로 어느 쪽이 active 인지 |
| 리더가 느리다 | udev 규칙 미적용 | `latency_timer` 가 1 인지 |
| MoveIt 이 매번 SUCCESS 인데 안 움직인다 | cuMotion 런치에 `ur_only:=false` 누락 | 모르는 링크 제약을 "이미 만족"으로 처리한다 |
| apt 가 ROS 패키지를 덮어썼다 | `pin` 을 `repos` 뒤에 했다 | `/etc/apt/preferences.d/99-nvidia-isolate.pref` |
