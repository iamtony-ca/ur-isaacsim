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

> ★ **컨테이너를 다른 프로젝트와 공유한다면 `--dry-run` 출력을 그냥 넘기지 말고
> 아래 `B-0. 공유 컨테이너 격리 확인` 을 먼저 볼 것.**

- ☐ 옵션: `--no-ml`(torch/lerobot 8 GB 생략, sim+teleop 만 할 때) /
  `--with-udev`(실물 OMY-L100 이 있을 때만)

돌아가는 순서: `preflight → pin → repos → base → cumotion → sources → build → leader → ml → verify`
(`pin` 이 `repos` 보다 먼저인 이유는 [`SETUP.md`](SETUP.md) §0-B — NVIDIA 레포가 ROS 패키지를 덮어쓴다.)

- ☐ 언제든 재점검: `src/setup/check_env.sh` (읽기 전용)

### ★ B-0. 공유 컨테이너 격리 확인 — `--dry-run` 출력에서 볼 것

**이 설치는 "영향 0" 이 아니다.** 워크스페이스 밖에 쓰는 곳이 4군데 있고, 그게 의도한 것인지
사람이 확인하고 넘어가야 한다. 컨테이너에 다른 프로젝트가 있으면 **B 실행 전에** 이 절을 먼저 본다.

**워크스페이스 안에만 있는 것** (지우면 완전히 되돌아감 — 확인 불필요):
`$WS/install`(colcon 오버레이, `/opt/ros` 에 안 씀) · `$WS/src`(vcs 소스) ·
`$WS/deps/.venv-ml`(torch·lerobot) · `$WS/deps/hf_cache`(GR00T 6.5 GB)

**컨테이너 전역에 쓰는 것 — 4가지 전부 확인할 것**:

- ☐ `/etc/apt/preferences.d/99-nvidia-isolate.pref` — `pin` 단계.
      이미 있으면 건드리지 않는다. 다른 프로젝트가 NVIDIA 레포 패키지를 **일부러** 쓰고 있다면
      Pin-Priority 100 이 그걸 막을 수 있으니 그 프로젝트 담당자와 확인
- ☐ `/etc/apt/sources.list.d/*.list` + GPG 키 — `repos` 단계 (Isaac ROS / CUDA / VPI)
- ☐ **apt 패키지 시스템 설치** — `base`·`cumotion` 단계. ★ 아래가 핵심
- ☐ `/etc/udev/rules.d/99-open-manipulator-cdc.rules` — `udev` 단계.
      **기본 실행에서 제외돼 있다**(`DEFAULT_STAGES` 에 없음). U2D2 실물이 붙었을 때만 E단계에서

**apt 판정 — `--dry-run` 출력에서 이 줄들을 본다**:

| 출력 | 뜻 | 조치 |
|---|---|---|
| `ok pure addition (0 upgraded, 0 removed)` | 순수 추가 | ☐ 그대로 진행 |
| `warn would UPGRADE existing packages:` | 기존 패키지가 바뀐다 | ☐ **멈춘다** — 목록을 읽는다 |
| `warn would REMOVE packages:` | 삭제된다 | ☐ **멈춘다** |
| `FAIL refusing: this would change packages other workspaces may depend on.` | 가드가 막았다 | ☐ 정상 동작이다 |

- ☐ **`ALLOW_UPGRADES=1` 을 반사적으로 붙이지 않는다.** 이 플래그는 가드를 끄는 것이지
      안전하게 만드는 게 아니다. 목록을 읽고 "이 패키지가 바뀌어도 되는가"를 판단한 뒤에만 쓴다.
- ☐ 새 컨테이너면 오히려 거부가 뜰 수 있다 — 가드는 *"이미 깔린 게 바뀌는가"* 를 보므로,
      다른 버전이 선점돼 있으면 여기서 순수 추가였던 게 거기선 업그레이드가 된다. **버전 차이지 오류가 아니다.**

**알려진 예외 — 이건 진짜로 다른 ws 에 영향이 간다**:

- ☐ cuMotion/realsense 가 `diagnostic_updater` 4.2.7 을 끌어오면 구버전 ros2_control(4.44)이
      `undefined symbol: diagnostic_updater::Updater` 로 죽는다 → **ros2_control 스택 4.45.2 동반 업그레이드**가
      필요하다. `setup.sh` 는 이걸 **자동화하지 않고 가드에 걸려 멈춘다**(사람이 결정하라는 설계).
      배경은 [`CLAUDE.md`](../CLAUDE.md) cuMotion 함정 ① / [`HARDWARE.md`](HARDWARE.md) §4.

> **가장 확실한 답: 가능하면 전용 컨테이너를 쓴다.** 다른 프로젝트가 없으면 위 4가지가 충돌할
> 대상 자체가 없다. 공유가 불가피할 때 이 스크립트가 하는 일은 *"안전하게 만드는 것"* 이 아니라
> **"위험한 순간에 멈추고 사람에게 묻는 것"** 이다.

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

### E-1. 리더 단독 측정 (UR16e 없이) — ★ 여기서 5가지를 잰다

**순서가 중요하다.** E-1 이 틀리면 E-2 의 캘리브레이션은 틀린 값을 정밀하게 재는 일이 된다.

```bash
ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py \
    port_name:=/dev/ttyUSB0 use_self_collision_avoidance:=false
ros2 control list_controllers -c /leader/controller_manager   # ★ /leader 네임스페이스
ros2 run ur_bringup omy_leader_calib.py --mode check
```

- ☐ 컨트롤러 4개 전부 `active`
- ☐ 손으로 리더를 움직이면 `/leader/joint_states` 가 따라 변한다
- ☐ **① 살아있는가** — 발행 **~300 Hz**, `joint1..6` + `rh_r1_joint` **7개 전부** 존재,
      관절별 표준편차 **< 0.01 rad**(`--mode check` 가 `★ 잡음 큼` 을 찍으면 배선/전원부터)
- ☐ **② ★ 물리적 rest pose 의 실제값** — 리더를 **손 떼도 서 있는 자세**로 놓고 `--mode check`
      의 관절별 현재값을 읽어 적는다.
      기대값 `[0°, 0°, +90°, −90°, +90°, 0°]` (ROBOTIS SRDF `home`, `omy_f3m.srdf`)
      → 이 값을 매핑에 넣으면 UR16e `ready` 가 나온다. **이게 랑데부 자세다**(E-3).
      기대값과 다르면 **읽은 값이 정본** — 그걸로 UR16e 쪽을 역산하고, 그 자세가 elbow≈0
      (특이점)이나 테이블 충돌에 걸리지 않는지 확인한다.
      실측값: `[____, ____, ____, ____, ____, ____]`
- ☐ **③ ★ 관절별 기구 가동범위** — 각 관절을 손으로 **기구 스톱까지** 천천히 돌려 min/max 기록.
      **J2 가 핵심**: 사양서는 **−70°~+100°**, URDF(`omy_l100_arm.urdf.xacro`)는 **±180°** 로 서로 다르다.
      URDF 는 7관절에 ±180 을 일괄로 넣어둔 것이라 **기구 스톱을 반영하지 않는다 → 실측이 정본.**
      이 값이 "리더로 도달할 수 없는 UR16e 자세"의 범위를 정한다.
      J2 실측: `____ ~ ____`
- ☐ **④ ★ 그리퍼 트리거 실사용 구간** — 완전히 놓았을 때 / 완전히 쥐었을 때 `rh_r1_joint` 값.
      현재 기본값 `gripper_in_open=0.0`, `gripper_in_closed=-1.0` 은 **추정치**다.
      실측: 열림 `____` / 닫힘 `____` → 브리지 파라미터에 반영
- ☐ **⑤ ★ 중력보상 드리프트** — 임의 자세에 놓고 손을 뗀 뒤 **10초** 관절값 변화.
      `engage_tol`(8.6°)보다 **훨씬** 작아야 한다. 크면 게이트를 통과시켜 놓고 리더가 흘러내려
      팔이 따라간다 → 게이트 값이 아니라 **중력보상 튜닝**을 먼저 고친다

### E-2. 오프셋 캘리브레이션 (둘 다 켜고, **브리지는 disabled 유지**)

**E-1 ②가 끝나야 시작할 수 있다.** 이 단계 전에는 `enable` 이 **정상적으로 계속 거부한다** —
고장이 아니라 순서 문제다.

```bash
python3 src/ur_bringup/isaac/common/switch_control_mode.py trajectory
python3 src/ur_bringup/isaac/common/reset_pose.py ready      # UR16e 를 랑데부로
# 리더를 E-1 ②의 rest pose 로 놓고 가만히 둔 채로
ros2 run ur_bringup omy_leader_calib.py --mode match          # → offset 6개 출력

# 적용 — 브리지가 DISABLED 면 바로 먹는다 (재기동 불필요)
ros2 service call /omy_bridge/disable std_srvs/srv/Trigger
ros2 param set /omy_to_ur16e offset "[<출력값 6개>]"
ros2 run ur_bringup omy_leader_calib.py --mode verify
#   → 자세 바꿔가며 반복
```

- ☐ `--mode match` 가 출력한 **오프셋 6개**를 기록. 이 값이 **다이나믹셀 엔코더 영점 + J4/J6
      손목 오프셋**을 한 번에 흡수한다 — J4/J6 는 유도 가능한 값이 없어 **이 측정 말고 얻을 방법이 없다**
- ☐ `--mode verify` 잔차가 `engage_tol` 8.6° 보다 **확실히** 작다
- ☐ **자세를 바꿔가며 match/verify 를 2~3회 반복.** 한 자세에서만 맞는 오프셋은 오프셋이 아니라
      그 자세의 우연이다
- ☐ **ENGAGED 중에는 `param set` 이 거부된다** — 매핑이 바뀌면 팔이 움직이기 때문. 먼저 disable.
      런타임 변경이 되는 건 **`offset`/`sign` 뿐**이고, 나머지는 거부하면서 재기동 명령을 알려준다
      (원래는 조용히 무시됐다 — `HISTORY.md` §42.5·§42.6)
- ☐ **`param set` 값은 노드와 함께 사라진다.** 최종값을 **양쪽에** 반영:
      `teleop_omy.launch.py` 기본값 **+ `virtual_omy_leader.py`**
      (후자를 빼먹으면 sim 회귀의 engage 게이트가 거부하기 시작한다)

### E-3. UR16e 에 연결 — 랑데부에서 engage (팔만, 그리퍼 없이 가능)

**임의 자세에서 시작하지 않는다.** 둘 다 정해진 랑데부 자세로 간 뒤 붙인다.

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| **OMY L100** (그냥 내려놓는 자세) | 0° | 0° | +90° | −90° | +90° | 0° |
| **UR16e** `reset_pose.py ready` | 0° | −90° | +90° | −90° | −90° | 0° |

```bash
# 1) 팔 + 리더 런치는 위에서 이미 떠 있음.  pad:=true 면 게임패드로도 조작 가능
ros2 launch ur_bringup teleop_omy.launch.py use_sim_time:=false pad:=true

# 2) 리더를 rest pose 에 내려놓고 → UR16e 를 랑데부로 (MoveIt, 충돌 검사됨)
ros2 service call /omy_bridge/sync std_srvs/srv/Trigger
ros2 topic echo /omy_bridge/status          # sync:moving → synced

# 3) 얼마나 어긋났는지 보면서 리더를 맞춘 뒤
ros2 topic echo /omy_bridge/engage_error    # [rad] 관절별, 매핑된 리더 − 팔로워
ros2 service call /omy_bridge/enable std_srvs/srv/Trigger
```

- ☐ `/omy_bridge/sync` 가 `synced` 로 끝나고 팔이 랑데부 자세에 있다.
      **sync 는 move_group 이 필요하다**(`ur16e_moveit.launch.py`). 없으면 서비스가 그 사실과
      수동 절차를 알려주고 거부한다
- ☐ **수동 절차를 쓸 때만**: `reset_pose.py ready` 는 MoveIt 이 아니라 **직선 관절 보간**이라
      충돌 검사가 없다 → **팔 주변이 비어 있는지 눈으로 확인**하고 실행
- ☐ `enable` 이 **거부되면 리더 자세를 더 맞춘다.** 무리해서 우회하지 않는다 —
  이 게이트가 없으면 16 kg 급 팔이 한 제어주기에 리더 자세로 튄다
- ☐ 거부 메시지가 **어느 관절이 몇 도 틀렸는지** 알려준다. `/omy_bridge/engage_error` 를 띄워 두면
      호출 없이 실시간으로 보면서 맞출 수 있다
- ☐ 첫 시도는 `max_joint_speed` 를 낮춰서, 짧은 동작으로
- ☐ 끝낼 때는 `/omy_bridge/disable` — 팔은 그 자리에 정지. 재연결 시 게이트가 처음부터 다시 걸린다

**게임패드** (`pad:=true`, 손이 리더에 있어 터미널을 못 쓸 때):

| 버튼 | 동작 | 데드맨 |
|---|---|---|
| Options (9) | `/omy_bridge/enable` | **필요** |
| R3 (12) | `/omy_bridge/sync` | **필요** |
| Create/Share (8) | `/omy_bridge/disable` | 불필요 |

- ☐ 데드맨(L1, 4번) 없이 Options/R3 를 누르면 **거부되고 로그에 이유가 찍힌다.** 정상이다 —
      떠도는 `/joy` 메시지 하나가 16 kg 팔을 움직이면 안 된다. 멈추는 동작만 데드맨이 없다
- ☐ 패드 인덱스는 전부 파라미터다. 다른 패드면 `ros2 topic echo /joy` 로 확인해 덮어쓴다

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
