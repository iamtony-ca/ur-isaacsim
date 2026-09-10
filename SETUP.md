# SETUP — UR16e (ROS 2 Jazzy + Isaac Sim 6.0.1) 재현 매뉴얼

깨끗한 환경에서 이 워크스페이스(sim+real UR16e 제어 스택)를 **그대로 재현**하는 절차.
아키텍처/배경은 상위 [`README.md`](README.md) 참고.

---

## 0. 대상 환경 (검증된 조합)

| 항목 | 버전 |
|---|---|
| OS | Ubuntu 24.04 (noble) |
| ROS 2 | **Jazzy** (`/opt/ros/jazzy`) |
| Isaac Sim | **6.0.1** (`/isaac-sim`, `isaacsim.ros2.bridge` 확장) — 5.1.0 에서 이식, 스크립트 무수정 |
| GPU | NVIDIA **RTX 5090 (sm_120 / Blackwell)**, 드라이버 580.x |
| 워크스페이스 | `/isaac-sim/volume/ur_ws` (colcon), git repo = `src/` |

> 다른 경로를 쓰면 아래 절대경로(`/isaac-sim/volume/ur_ws`, `/isaac-sim/python.sh`)를 본인 환경에 맞게 치환.
> 문서 곳곳에 남아 있는 `/isaac-sim/volume/ur_ws` 는 같은 워크스페이스의 옛 경로 표기다.

> **Isaac Sim 5.1.0 → 6.0.1 이식**: `ur_bringup` 의 Isaac 스크립트/USD 에셋은 **수정 없이 그대로 동작**한다
> (`isaacsim.core.api`·`isaacsim.core.nodes`·`isaacsim.ros2.bridge` OmniGraph 노드 이름 전부 유지).
> 6.0.1 에서 새로 뜨는 경고 `[ROS2 Publish Joint State] Reading from targetPrim is deprecated` 는
> **동작에 영향 없음**(권고사항). 자세한 이식 검증 로그는 [`HISTORY.md`](HISTORY.md) §14.

> **GPU 아키텍처 주의 (RTX 50 시리즈)**: apt 로 받는 NVIDIA 바이너리 중 **`nvblox_node` 는 sm_75(Turing)
> 전용으로만 컴파일**되어 있고 PTX 폴백도 없어, sm_120 인 RTX 5090 에서는 실행 즉시
> `cudaErrorInvalidDevice: invalid device ordinal` 로 죽는다 → **소스 빌드 필요(§2-B-4)**.
> cuMotion(`libcumotion.so`)은 sm_75/86/89/**120** 을 모두 포함하므로 apt 그대로 쓴다.

---

## 0-B. ★ 다른 PC 에서 처음부터 — 전체 순서

**한 줄 요약: 컨테이너 띄우고 → clone → `bootstrap.sh --dry-run` → `bootstrap.sh`.**
아래 §0-A 는 단계별 세부, §1~§4 는 손으로 하는 법이다. 처음이면 이 절만 따라가면 된다.

### 전제 (스크립트의 `preflight` 가 자동 확인한다)
| 항목 | 필요값 | 없으면 |
|---|---|---|
| Isaac Sim 컨테이너 | **6.0.1**, `/isaac-sim/python.sh` 존재 | 컨테이너 밖이면 즉시 중단 |
| ROS 2 | **Jazzy** (`/opt/ros/jazzy`) | **스크립트가 설치하지 않는다** — 베이스 이미지 선택 문제이고, 남의 머신에 ROS 배포판을 몰래 까는 건 이 워크스페이스의 격리 원칙 위반 |
| GPU | NVIDIA + 드라이버 | sm_89/120(RTX 40/50)이면 **nvblox 소스빌드로 자동 전환** |
| 디스크 | ≥ 30 GiB | ML venv 만 약 8 GB |
| 기타 | `git` `curl` `sudo` | — |

### 1) 컨테이너
지금 이 머신과 **같은 방식으로** Isaac Sim 6.0.1 컨테이너를 띄운다. 필요한 것은 세 가지뿐:
**GPU 전달**(`--gpus all`), **워크스페이스 볼륨**(`/isaac-sim/volume` 에 마운트),
**GUI 를 볼 거면 X 소켓**(`-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`).
GUI 없이 `--headless` 로만 쓸 거면 X 는 생략해도 된다.

### 2) 클론
```bash
mkdir -p /isaac-sim/volume/ur_ws
git clone <이 저장소> /isaac-sim/volume/ur_ws/src      # ★ repo = src/ 다
```
> 다른 경로에 두려면 그대로 두면 된다 — `bootstrap.sh` 가 **자기 위치에서 WS 를 역산**한다.

### 3) 설치 — 두 줄
```bash
/isaac-sim/volume/ur_ws/src/setup/bootstrap.sh --dry-run   # ★ 먼저 계획만 본다
/isaac-sim/volume/ur_ws/src/setup/bootstrap.sh             # 실행
```
`preflight → pin → repos → base → cumotion → sources → build → leader → ml → verify` 를 순서대로 돈다.

| 옵션 | 용도 |
|---|---|
| `--dry-run` | 아무것도 바꾸지 않고 계획만. **처음엔 반드시 이것부터** |
| `--no-ml` | torch/lerobot venv(약 8 GB) 생략. sim + teleop 만 할 거면 불필요 |
| `--with-udev` | U2D2 udev 규칙까지 설치. **실물 OMY-L100 이 있을 때만** (유일하게 `/etc/udev` 에 쓴다) |

> **★ `pin` 이 `repos` 보다 먼저인 이유**: NVIDIA 레포는 ROS 패키지의 상위 버전을 갖고 있어서
> (`robotiq_description` 0.0.1 → **9.0.1**) 핀 없이 레포부터 추가하면 다음 apt 때 조용히 덮어쓴다.
> `bootstrap.sh` 는 이 순서를 강제하고, `repos` 단계가 **핀이 실제로 먹는지 검증**한 뒤 진행한다.

### 4) 검증 (하드웨어 0개로 가능)
설치가 끝나면 `bootstrap.sh` 가 아래 순서를 화면에 다시 찍어준다.
```bash
source /opt/ros/jazzy/setup.bash && source /isaac-sim/volume/ur_ws/install/setup.bash && export ROS_DOMAIN_ID=0
# 터미널 1  Isaac  (디스플레이 없으면 --headless)
/isaac-sim/python.sh src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_with_2f85.usd
# 터미널 2  제어
ros2 launch ur_bringup ur16e_2f85.launch.py use_sim:=true
python3 src/ur_bringup/isaac/common/reset_pose.py ready
# 터미널 3  teleop (가짜 리더로 리더 없이)
ros2 launch ur_bringup teleop_omy.launch.py use_sim_time:=true virtual_leader:=true
python3 src/ur_bringup/isaac/common/switch_control_mode.py streaming
ros2 service call /omy_bridge/enable std_srvs/srv/Trigger
```
✅ **팔이 가짜 리더를 따라 움직이면 성공.** 기대 추종오차 ≈ 0.24°(`HISTORY.md` §22).
언제든 재점검: `src/setup/check_env.sh` (읽기 전용).

### 5) 실물
UR16e / 2F-85 / D405 → [`HARDWARE.md`](HARDWARE.md) §1·§2·§3.
**OMY-L100 리더 → `HARDWARE.md` §4-B** (먼저 `bootstrap.sh --with-udev`).

---

## 0-A. 자동 설치 — `setup/` 스크립트 (권장)

아래 §1~§4 를 손으로 따라가는 대신 **스크립트로 재현**할 수 있다.
처음부터 한 번에 하려면 **§0-B 의 `bootstrap.sh`** 를 쓰고, 여기는 단계별로 손볼 때 본다.

```bash
cd <ws>/src/setup
./setup.sh --list        # 단계 목록
./setup.sh --dry-run     # 아무것도 바꾸지 않고 "무엇을 할지"만 출력  ← 먼저 이걸로 확인
./setup.sh               # 전체 실행 (preflight → pin → repos → base → cumotion → sources
                         #            → build → leader → ml → verify).  udev 는 제외됨(실물 전용)
./setup.sh base build    # 특정 단계만
./check_env.sh           # 언제든 환경 점검 (읽기 전용)
```

| 단계 | 하는 일 |
|---|---|
| `preflight` | **읽기 전용 사전점검** — Isaac/ROS/GPU/디스크/도구. 아무것도 깔기 전에 먼저 걸러낸다 |
| `pin` | **NVIDIA 레포 격리 핀을 먼저** 넣는다 (§2-B-1) |
| `repos` | Isaac ROS / CUDA / VPI 레포 + 키 추가, **핀이 실제로 먹는지 검증** |
| `base` | UR·MoveIt·ros2_control·robotiq_description·moveit_servo·joy 등 |
| `cumotion` | cuMotion + nvblox **개별 패키지만** (메타패키지 금지) |
| `sources` | `vcs import` + **nvblox_core 서브모듈 init** |
| `build` | colcon 빌드. **GPU arch 를 감지해 nvblox 소스빌드 필요 여부를 자동 판단** |
| `leader` | **OMY-L100 teleop 리더 스택** (apt 2개·업그레이드 0, 7패키지 빌드, 나머지 COLCON_IGNORE) |
| `ml` | IL/VLA용 격리 venv (torch sm_120 + lerobot). 약 8 GB |
| `verify` | `check_env.sh` |
| `udev` | **U2D2 udev 규칙 — 실물 리더 전용.** 유일하게 `/etc/udev` 에 쓰므로 **기본 실행에서 제외**되어 있고 명시해야 돈다 |

**★ 공유 머신 안전장치 (스크립트에 내장)**
- 모든 apt 설치를 **먼저 시뮬레이션**하고 영향도를 출력한다.
- **기존 패키지를 업그레이드/삭제하게 되면 거부한다**(`ALLOW_UPGRADES=1` 로만 강제 가능).
  실측: `ros-jazzy-isaac-ros-nvblox` 를 넣으려 하면 python3.12 7개 업그레이드를 감지해 **거부**한다.
- 소스빌드 산출물은 전부 워크스페이스 `install/` 오버레이 → 워크스페이스를 지우면 원상복구된다.

`check_env.sh` 가 잡아주는 것: apt 핀 무력화, **GPU arch ↔ nvblox 바이너리 불일치**,
topic_based 버전 오류, ros2_control ABI 불일치, 메타패키지 오설치, 워크스페이스 설정 누락.

> `WS=/다른/경로 ./setup.sh` 로 워크스페이스 위치를 바꿀 수 있다.

---

## 1. 워크스페이스 가져오기

```bash
# src 저장소를 워크스페이스의 src/ 로 (이미 있으면 생략)
mkdir -p /isaac-sim/volume/ur_ws
cd /isaac-sim/volume/ur_ws
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
    python3-vcstool
```
- `ros-jazzy-ur` = 메타패키지 → `ur_robot_driver`, `ur_description`, `ur_moveit_config`,
  `ur_controllers`, `ur_calibration`, `ur_dashboard_msgs`, `ur_client_library` 포함.
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

## 2-B. cuMotion + nvblox (GPU 플래닝 / 실시간 장애물 회피)

세트2/3 의 GPU 플래닝과 실시간 회피용. **팔 단독(세트1)·기본 MoveIt(OMPL)만 쓸 거면 통째로 생략 가능.**

> **이 컨테이너/PC 를 다른 프로젝트와 공유한다면 반드시 §2-B-1 의 핀부터 넣고 시작할 것.**
> NVIDIA 레포는 ROS 공식 패키지를 **더 높은 버전으로 덮어쓴다**(실측: `robotiq_description` 0.0.1→**9.0.1**,
> `moveit_task_constructor_core`→**99.99.0**). 핀 없이 레포만 추가해도 이후 다른 프로젝트가 `apt upgrade`
> 하는 순간 스택이 갈린다.

### 2-B-1. NVIDIA 레포 3개 추가 (핀을 **먼저**)

```bash
# (0) 핀 먼저 — NVIDIA 레포는 "다른 데 없는 패키지"만 제공하도록 격리
sudo tee /etc/apt/preferences.d/99-nvidia-isolate.pref > /dev/null <<'EOF'
Package: *
Pin: origin isaac.download.nvidia.com
Pin-Priority: 100

Package: *
Pin: origin developer.download.nvidia.com
Pin-Priority: 100

Package: *
Pin: origin repo.download.nvidia.com
Pin-Priority: 100
EOF
# Pin-Priority 100 = "이미 설치된 패키지는 이 레포 버전으로 업그레이드하지 않는다".
# 아직 설치 안 된 패키지(cuMotion/nvblox/CUDA/VPI)는 정상적으로 설치된다.

# (1) Isaac ROS release-4
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
  | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-isaac-ros.gpg
echo 'deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4 noble main' \
  | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list

# (2) CUDA (isaac_ros_common 이 cuda-toolkit 을 하드 의존)
curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/3bf863cc.pub \
  | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-cuda.gpg
echo 'deb [signed-by=/usr/share/keyrings/nvidia-cuda.gpg] https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/ /' \
  | sudo tee /etc/apt/sources.list.d/nvidia-cuda.list

# (3) VPI 4 (libnvvpi4 — Jetson OTA x86_64, r38.2)
curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
  | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-jetson.gpg
echo 'deb [signed-by=/usr/share/keyrings/nvidia-jetson.gpg] https://repo.download.nvidia.com/jetson/x86_64/noble r38.2 main' \
  | sudo tee /etc/apt/sources.list.d/nvidia-vpi.list

sudo apt update
# 핀 검증 — Candidate 가 ROS 쪽(0.0.1)으로 유지되어야 정상
apt-cache policy ros-jazzy-robotiq-description
```

### 2-B-2. 설치 전 영향도 확인 (습관화)

```bash
apt-get install -s <패키지들> | grep "upgraded,"
apt-get install -s <패키지들> | grep '^Inst' | grep -E '\[[0-9]'   # ← 여기 출력이 있으면 기존 패키지가 바뀐다는 뜻
```
아래 §2-B-3 조합은 **`0 upgraded ... 0 to remove`** 여야 정상(실측: 90 newly installed, 0 upgraded).

### 2-B-3. 설치 (메타패키지 금지 — 꼭 이 목록만)

```bash
sudo apt install -y \
    ros-jazzy-isaac-ros-cumotion \
    ros-jazzy-isaac-ros-cumotion-moveit \
    ros-jazzy-isaac-ros-cumotion-robot-description \
    ros-jazzy-isaac-ros-cumotion-robot-segmenter \
    ros-jazzy-nvblox-ros ros-jazzy-nvblox-msgs ros-jazzy-nvblox-rviz-plugin
```
- **`ros-jazzy-isaac-ros-cumotion-examples` 를 넣지 말 것** — moveit2-tutorials/kinova 를 끌어오면서
  `robotiq_description` 을 NVIDIA 레포 9.0.1 로 바꾸려 든다(핀이 막지만 굳이 건드릴 이유가 없음).
- **`ros-jazzy-isaac-ros-nvblox`(메타) 도 넣지 말 것** — `nvblox_examples_bringup` → triton / tensor_rt /
  visual_slam / detectnet / unet + 모델설치 패키지까지 딸려와 **432개 설치 + python3.12 컨테이너 전역 업그레이드**가
  일어난다(실측). 우리가 그중 실제로 쓰는 건 `nvblox_base.yaml` **한 개**뿐이라,
  `ur_bringup/config/ur16e_2f85_d405/vendor/nvblox_base.yaml` 로 **vendoring** 해 두었다.
  (갱신법은 그 파일 헤더 주석 참고. 런치는 apt 판이 있으면 그걸, 없으면 vendored 판을 쓴다.)

### 2-B-4. nvblox 는 소스 빌드 (RTX 40/50 시리즈 = sm_89/120 필수)

apt `nvblox_node` 는 **sm_75 전용 + PTX 없음** → RTX 5090(sm_120)에서 기동 즉시 SIGABRT
(`thrust ... parallel_for failed: cudaErrorInvalidDevice: invalid device ordinal`).
확인법:
```bash
/usr/local/cuda-13.2/bin/cuobjdump --list-elf /opt/ros/jazzy/lib/nvblox_ros/nvblox_node | grep -oE 'sm_[0-9]+' | sort -u
nvidia-smi --query-gpu=name,compute_cap --format=csv     # 5090 -> 12.0 (= sm_120)
```
GPU 의 compute_cap 이 위 목록에 있으면 apt 판 그대로 써도 된다. 없으면 워크스페이스에 소스 빌드:

```bash
cd /isaac-sim/volume/ur_ws
vcs import src < src/ur16e.repos          # isaac_ros_nvblox @ release-4.6 포함
cd src/isaac_ros_nvblox && git submodule update --init --recursive --depth 1 && cd ../..
#   ^ nvblox_core(= github.com/nvidia-isaac/nvblox)가 서브모듈이라 반드시 init 필요

source /opt/ros/jazzy/setup.bash
export CUDACXX=/usr/local/cuda-13.2/bin/nvcc PATH=/usr/local/cuda-13.2/bin:$PATH
colcon build --symlink-install --packages-select nvblox_ros \
    --cmake-args -DUSE_NATIVE_CUDA_ARCHITECTURE=1 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```
- `USE_NATIVE_CUDA_ARCHITECTURE=1` = 이 머신의 GPU arch 하나만 컴파일(=sm_120). 빌드도 빨라진다.
- 빌드 의존성(`libgoogle-glog-dev libgflags-dev libgtest-dev libbenchmark-dev libsqlite3-dev`)은
  §2-B-3 설치 시 함께 들어온다. 없으면 `sudo apt install -y` 로 추가.
- `nvblox_msgs`/`nvblox_ros_common`/`nvblox_rviz_plugin` 은 **apt 것을 그대로 쓴다**(소스 트리의 동명 패키지는
  `COLCON_IGNORE` 처리). 워크스페이스 `install/` 이 오버레이라 `nvblox_ros` 만 우리 빌드가 이긴다
  (colcon 이 띄우는 "overriding package" 경고는 **의도된 것**).
- 검증: `cuobjdump --list-elf install/nvblox_ros/lib/nvblox_ros/nvblox_node | grep -oE 'sm_[0-9]+' | sort -u`

---

## 2-C. ML 환경 (IL/VLA 학습 + LeRobot 변환)

**IL/VLA 트랙에서만 필요.** 제어·teleop·데모 기록(ROS 쪽)은 이것 없이 전부 동작한다.

```bash
cd <ws>/src/setup
./setup.sh ml                                     # deps/.venv-ml 생성 + torch + lerobot
TORCH_INDEX=https://download.pytorch.org/whl/cu130 ./setup.sh ml   # CUDA 인덱스 교체
./check_env.sh                                    # sm_120 및 격리 검증
```

### ★ 격리 — 반드시 워크스페이스 로컬 venv
```
deps/.venv-ml/          ← 여기에만 설치. `rm -rf` 하면 완전 원복
```
- **시스템 python 에 torch 를 깔지 않는다** — 이 컨테이너는 다른 프로젝트와 공유된다.
- **Isaac 의 python 에도 깔지 않는다** — Isaac 스택을 오염시킨다.
- `check_env.sh` 가 두 곳 모두 깨끗한지 **실제로 확인**한다(누출 감지).
- `deps/` 는 git repo(`src/`) 밖이라 자동으로 버전관리 제외.

### ★ 함정 — `python3-venv` 를 apt 로 깔지 말 것
`python3 -m venv` 는 되지만 이 이미지에는 `ensurepip` 이 없어 venv 안에 pip 이 없다.
그렇다고 `apt install python3-venv python3-pip` 을 하면 **python3.12 를 시스템 전역으로 업그레이드**한다
(실측: 7개 패키지). 우리 격리 규칙 위반이고 `apt_guarded_install` 이 거부할 대상이다.
→ **venv 안에만 pip 을 부트스트랩**한다(`setup.sh` 가 자동 처리):
```bash
curl -fsSL https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
deps/.venv-ml/bin/python /tmp/get-pip.py
```

### ★ torch 는 반드시 CUDA 인덱스에서 (sm_120)
기본 PyPI `pip install torch` 는 **sm_120(Blackwell/RTX 50) 커널을 보장하지 않는다.**
없으면 GPU 는 보이고 텐서도 올라가는데 **커널이 안 도는** — `HISTORY.md` §14 의 nvblox 와 똑같은 실패다.
```bash
deps/.venv-ml/bin/python -m pip install \
    --index-url https://download.pytorch.org/whl/cu128 torch torchvision
```
| 인덱스 | torch (2026-09 기준) |
|---|---|
| `cu128` | 2.11.0 |
| `cu130` | 2.14.0 (설치된 CUDA 툴킷 13.2 와 정렬) |

**검증(설치 직후 반드시)**:
```bash
deps/.venv-ml/bin/python -c "import torch; print(torch.cuda.get_arch_list())"   # sm_120 포함?
nvidia-smi --query-gpu=compute_cap --format=csv                                  # 5090 -> 12.0
```
`setup.sh ml` 은 이 검사를 **torch 설치 직후와 lerobot 설치 후 두 번** 한다 —
나중 의존성이 torch 를 조용히 다른 빌드로 바꿔치기할 수 있기 때문이다.

### ffmpeg 은 apt 대신 `imageio-ffmpeg`
LeRobot 은 mp4 인코딩에 ffmpeg 이 필요한데 시스템에 없다. apt 설치는 전역 변경이므로
**venv 안에 바이너리를 동봉하는 `imageio-ffmpeg`** 를 쓴다(`requirements-ml.txt`).

### ★ extra 는 `[dataset,training]` 둘 다 필요
둘이 서로 다른 절반을 담당한다:
- `[dataset]` — `lerobot.datasets` 임포트 자체. 없으면 `raw_to_lerobot.py` 가 안 돈다.
- `[training]` — `lerobot-train` 실행. 없으면 **즉시** `'accelerate' is required but not installed`.

`[dataset]` 만 있어도 **데이터 변환은 멀쩡히 되기 때문에**, 이 누락은 첫 실제 학습을 돌릴 때까지
드러나지 않는다(실제로 그랬다 — `HISTORY.md` §24). `check_env.sh` 가 이제 `accelerate` 를 따로 검사한다.

### ★★ `/dev/shm` 이 64 MiB — DataLoader 워커가 간헐적으로 죽는다
이 컨테이너의 `/dev/shm` 은 Docker 기본값 **64 MiB** 다. PyTorch 기본 공유전략
(`file_descriptor`)은 DataLoader 워커→학습루프 배치 전달에 `/dev/shm` 을 쓰는데,
**ACT 배치 하나가 8 × 카메라2 × 3×480×640 float32 ≈ 59 MiB** 라 한 배치도 겨우 들어간다.

```
RuntimeError: unable to allocate shared memory (shm) for file <...>: Resource temporarily unavailable (11)
RuntimeError: DataLoader worker (pid ...) exited unexpectedly
```

**실측: 동일 조건 3회 중 2회 크래시.** 결정적이 아니라 **간헐적**이라 더 나쁘다 —
스모크 테스트는 통과해 놓고 몇 시간짜리 학습 중간에 죽는다.

`/dev/shm` 을 키우려면 컨테이너를 다시 만들어야 하는데, **이 머신은 다른 프로젝트와 공유**라 불가.
그래서 공유전략을 `file_system`(temp dir 사용)으로 바꾼다. `setup.sh ml` 이 자동 설치한다.

```bash
# 학습 실행 시 환경변수 하나만 붙이면 된다
UR_WS_TORCH_SHM_FIX=1 deps/.venv-ml/bin/lerobot-train ... --num_workers=4
```

| | data_s | smp/s | 200 스텝 | 3회 중 |
|---|---|---|---|---|
| 수정 없음, `num_workers=4` | 0.001 | 161 | 16 s | **2회 크래시** |
| `num_workers=0` (회피) | 0.13 | 47 | 40 s | 3회 OK |
| `UR_WS_TORCH_SHM_FIX=1`, `num_workers=4` | 0.001 | 161 | 16 s | **3회 OK** |

loss 는 세 경우 모두 동일(step 200 에서 3.483/3.484) — **속도만 2.5배**, 결과는 안 바뀐다.

> ### ★ 2026-09-08 추가 — 이 표는 **0단계 공개 데이터셋** 기준이다
> 우리 자체 데이터셋(**480×640 카메라 2대**)에서는 `UR_WS_TORCH_SHM_FIX=1 + num_workers=4` 가
> **8 스텝 만에 죽는다**. 우회책이 고장난 게 아니라, 이 문제는 *전략*이 아니라 *용량*이기 때문이다 —
> `file_descriptor` 든 `file_system` 이든 둘 다 `/dev/shm` 에 할당한다.
>
> 미리 계산할 수 있다. float32 로 디코딩된 480×640×3 이미지 = **3.5 MiB**:
>
> | | 배치당 |
> |---|---|
> | 카메라 1대, batch 8 | 28 MiB |
> | **카메라 2대, batch 8** | **56 MiB** |
> | 카메라 2대, batch 32 | 225 MiB |
>
> `/dev/shm` 은 64 MiB(가용 ~57 MiB)다. 즉 **배치 하나로 이미 꽉 찬다.** 워커는 기본
> `prefetch_factor=2` 로 앞서 읽으므로 `워커수 × 2 × 배치바이트` 가 필요하고, 4 워커면 450 MiB —
> 애초에 불가능하다.
>
> **판단 기준**: `워커수 × 2 × (카메라수 × 3.5 MiB × batch) < 50 MiB` 이면 워커를 써도 되고,
> 아니면 `--num_workers=0`. 워커가 없으면 공유 메모리를 **아예 쓰지 않는다**.
>
> 실측(카메라 2대, `num_workers=0`): batch 8 → 0.355 s/step, batch 32 → 1.44 s/step, GPU 6.3/32 GB.
> 샘플당 시간이 같다(≈45 ms) — 즉 **GPU 가 아니라 디코딩이 병목**이다. 대규모 학습이 느리면
> 배치를 키우지 말고 **데이터셋 해상도를 줄여라**(240×320 이면 배치당 바이트가 1/4 이라 워커도
> 다시 쓸 수 있다). 단 해상도는 스키마라서 **수집 전에** 정해야 한다.

**함정 두 개를 지나야 여기 도달한다** (둘 다 조용히 실패한다):
1. **부모에만 걸면 안 된다.** lerobot 은 `dataloader_multiprocessing_context = "spawn"` 을
   **명시적으로** 박아 뒀다(`configs/train.py`, 시스템 기본은 `fork`). 부모의
   `set_sharing_strategy()` 는 spawn 된 워커에 상속되지 않는다.
2. **venv 의 `sitecustomize.py` 는 안 먹는다.** `/usr/lib/python3.12/sitecustomize.py` 가 이미 있고
   stdlib 경로가 site-packages 보다 **앞서서**, venv 쪽은 임포트조차 안 된다.
   → **`.pth` 파일**을 쓴다. `site` 가 모든 인터프리터(=spawn 된 워커 포함)에서 실행하고 이름 충돌도 없다.
   환경변수로 게이팅해서 평소 venv python 기동 비용은 0.

설치물(둘 다 venv 안, `rm -rf deps/.venv-ml` 로 완전 원복):
```
deps/.venv-ml/lib/python3.12/site-packages/ur_ws_shm_fix.pth   ← site 가 실행하는 한 줄
deps/.venv-ml/lib/python3.12/site-packages/ur_ws_shm_fix.py    ← 실제 전략 설정
```
`check_env.sh` 는 **파일 존재가 아니라 실제 전략값**을 확인한다 — 위 함정 2 때문에
"설치된 것처럼 보이지만 안 도는" 상태가 실제로 있었다.

### 사용법
ROS 쉘에 **source 하지 말 것**(ROS 파이썬 환경을 오염시킨다). 인터프리터를 직접 지정한다:
```bash
deps/.venv-ml/bin/python src/ur_bringup/scripts/raw_to_lerobot.py --raw <dir> --repo-id <user>/<name>
```

### 2-C-2. GR00T N1.7 (VLA) 추가 설치 — 2026-09-10

ACT 만 할 거면 필요 없다. GR00T 단계에 들어갈 때만. 설계 정본은
[`ur_bringup/docs/plan_groot_n17.md`](ur_bringup/docs/plan_groot_n17.md).

**정본은 개별 패키지 나열이 아니라 upstream extra 다** — lerobot 은 정책별 의존성을 extra 로
분리하고 런타임에 `require_package(..., extra="groot")` 로 막는다:

```bash
# 1) 반드시 dry-run 으로 기존 패키지 변경 여부 확인 (§0-A 와 같은 습관)
deps/.venv-ml/bin/pip install --dry-run --report /tmp/plan.json 'lerobot[groot]'
# 2) 순수 추가면 설치
deps/.venv-ml/bin/pip install 'lerobot[groot]'
# 3) ★ sm_120 이 살아있는지 즉시 확인
deps/.venv-ml/bin/python -c "import torch;print(torch.__version__, torch.cuda.get_device_capability(0))"
#    -> 2.11.0+cu128 (12, 0)
```

2026-09-10 실측: **19개 전부 신규, 기존 패키지 변경 0**. `transformers 5.5.4` · `diffusers 0.39.0` ·
`peft 0.20.0` · `timm 1.0.29` · `decord 0.6.0` · `dm-tree 0.1.10`.
**유일한 진짜 위험은 torch 교체**다 — PyPI 기본 인덱스 torch 로 갈아치워지면 sm_120 이 깨진다
(§2-C 함정 ②, §14 nvblox 와 같은 부류). 3번이 그걸 잡는다.

> 파일 상단 import 만 보고 "transformers 불필요"라고 판단했다가 즉시 틀렸다.
> **lerobot 의 정책 의존성은 import 문이 아니라 `require_package` 가드에 있다** —
> 다음에는 `grep require_package deps/.venv-ml/.../policies/<정책>/` 를 먼저 볼 것.

**HF 캐시 격리** — 공유 `~/.cache/huggingface` 를 오염시키지 않는다:

```bash
export HF_HOME=/isaac-sim/volume/ur_ws/deps/hf_cache   # GR00T 6.5 GB 가 여기로
export WANDB_DISABLED=true                              # GrootConfig.report_to 기본값이 wandb
export TOKENIZERS_PARALLELISM=false
```

**★ 게이트 — 사용자 계정 조치가 필요하다.** `nvidia/GR00T-N1.7-3B` 는 gated 가 아니지만,
백본 **토크나이저**가 gated `nvidia/Cosmos-Reason2-2B` 에 있어 학습 시작 시 401 로 죽는다.
(백본 *가중치* 494 텐서와 아키텍처 설정은 로컬에 다 있다 — 몇 MB 짜리 토크나이저 때문에 막힌다.)

1. https://huggingface.co/nvidia/Cosmos-Reason2-2B 라이선스 동의
2. https://huggingface.co/settings/tokens read 토큰 발급 → `export HF_TOKEN=hf_...`

다른 Qwen3-VL 토크나이저로 대체하는 우회는 **하지 않는다** — vocab 이 어긋나면 조용히 틀린 학습이 된다.

**★ 학습 명령 — 두 인자가 기본값이면 안 된다**:

```bash
BASE=$(deps/.venv-ml/bin/python -c \
  "from huggingface_hub import snapshot_download; print(snapshot_download('nvidia/GR00T-N1.7-3B'))")

deps/.venv-ml/bin/lerobot-train \
  --policy.type=groot --policy.push_to_hub=false --wandb.enable=false \
  --policy.base_model_path="$BASE" \          # ★ repo id 금지 (아래)
  --policy.model_params_fp32=false \          # ★ 기본 fp32 는 32 GB 초과 (아래)
  --dataset.repo_id=<...> --dataset.root=<...>
```

- **`base_model_path` 는 로컬 디렉터리여야 한다.** `_load_n1_7_checkpoint_processor_assets()` 가
  `Path().is_dir()` 로 판정해서 **repo id 면 `None` 을 반환**하고, albumentations·state dropout·
  percentile 정규화·크롭 기하가 **경고 없이** lerobot 기본값으로 바뀐다. 가중치는 정상 로드되어
  **학습이 그냥 돌아가므로** 알아챌 방법이 없다.
- **기본 `model_params_fp32=true` 는 32 GB 에 안 들어간다.** LoRA 가 아니라 동결 방식이고
  (`lora_*` 는 never-wired), action head **1,621 M** 이 동결 백본(1,524 M)보다 크다:
  `params 11.7 + grads 6.0 + AdamW 12.1 = 29.8 GiB / 31.8 GiB` → **배치 크기와 무관하게** 실패.
  `false` 로 두면 ≈17.8 GiB.

---

## 2-D. OMY-L100 teleop 리더 (IL 시연 데이터 수집용)

ROBOTIS AI **OMY-L100** 을 UR16e 의 teleop 리더로 쓰기 위한 스택. **팔로워(UR16e) 쪽은 건드리지 않는다.**
설계 근거와 UR16e 관절 매핑은 `ur_bringup/docs/plan_il_vla.md` §3.5.

**왜 ROS 스택인가** — L100 은 수동 암이 아니라 **중력보상되는 능동 리더**다
(`gravity_compensation_controller` + `spring_actuator_controller`, 300 Hz effort).
ROS 없이 다이나믹셀을 직접 읽는 `lerobot_teleoperator_omy` 로도 관절값은 얻지만 **중력보상이 없어**
1.46 kg 암을 에피소드 내내 사람이 들고 있어야 하고, 그 피로가 데이터 품질로 직결된다.
이 스택은 `/leader/joint_states` 를 네이티브로 발행해서 **`il_recorder.py --action-source topic` 이 무수정으로 붙는다.**

```bash
cd /isaac-sim/volume/ur_ws
src/setup/setup.sh --dry-run leader     # 먼저 계획만 확인
src/setup/setup.sh leader               # apt 2개 + COLCON_IGNORE 6개 + 빌드 7개
```

### 영향도 (2026-09-06 실측)
apt 는 **2개뿐이고 업그레이드 0건**이다 — 컨트롤러가 요구하는 `control_toolbox`/`kdl_parser`/
`generate_parameter_library`/`rsl`/`tl_expected`/`backward_ros`/`angles`/`realtime_tools` 는
이미 맞는 버전으로 설치되어 있다.
```
0 upgraded, 2 newly installed, 0 to remove
  ros-jazzy-dynamixel-sdk  ros-jazzy-dynamixel-interfaces      # 둘 다 공식 ROS 레포
```

### 빌드하는 것 / 안 하는 것
`open_manipulator` 는 13개 패키지인데 **7개만** 빌드하고 나머지는 `COLCON_IGNORE`:

| 빌드 | 제외 (COLCON_IGNORE) |
|---|---|
| `open_manipulator_description` (URDF/메시) | `open_manipulator` (메타) |
| `open_manipulator_bringup` (리더 런치/설정) | `open_manipulator_gui` (**Qt 끌어옴**) |
| `om_gravity_compensation_controller` | `open_manipulator_moveit_config` |
| `om_spring_actuator_controller` | `open_manipulator_collision` |
| `om_joint_trajectory_command_broadcaster` | `open_manipulator_playground` |
| `dynamixel_hardware_interface`, `robotis_interfaces` | `open_manipulator_teleop` |

> **★ 함정: `open_manipulator_bringup` 은 `gz_ros2_control`·`ros_gz_*` 를 의존으로 선언한다.**
> 그래도 **Gazebo 는 설치되지 않고 필요도 없다** — 이 패키지는 `ament_python` 이라 colcon 이 그 의존을
> 해결하지 않기 때문이다. **`rosdep install` 을 돌리면 Gazebo 스택이 통째로 딸려온다 — 돌리지 말 것.**
> `check_env.sh` 가 `ros_gz` 유입 여부를 감시한다. 그런데도 bringup 을 빌드해야 하는 이유는
> `omy_l100.urdf.xacro` 가 `$(find open_manipulator_bringup)` 을 참조하기 때문이다.

> **★ 함정: 평면 `omy_l100.urdf` 를 쓰지 말 것.** `ros2_control` 블록이 **없어서**(0줄) 리더가 안 뜬다.
> 반드시 `omy_l100.urdf.xacro` 경로로 갈 것.

### 하드웨어 없이 검증 (U2D2/L100 미연결 상태에서 가능)
```bash
ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py \
    use_mock_hardware:=true use_self_collision_avoidance:=false
ros2 control list_controllers -c /leader/controller_manager   # ★ 네임스페이스 /leader
ros2 topic hz /leader/joint_trajectory                        # 300 Hz
```
기대 결과 — 컨트롤러 4개 전부 `active`:
`gravity_compensation_controller` / `spring_actuator_controller` /
`joint_state_broadcaster` / `joint_trajectory_command_broadcaster`

> **★ 함정 ①**: `use_self_collision_avoidance` 기본값이 `true` 인데 그 노드는
> `open_manipulator_collision`(우리가 COLCON_IGNORE 한 패키지)에 있다 → **반드시 `false`**.
> **★ 함정 ②**: 모든 것이 **`/leader` 네임스페이스** 아래다. `ros2 control list_controllers` 를
> 그냥 치면 `/controller_manager` 를 찾다가 무한 대기한다. `-c /leader/controller_manager` 필수.
> **★ 함정 ③**: ros2_control 4.45 의 controller_manager 는 `robot_description` **파라미터가 아니라
> `/robot_description` 토픽**을 구독한다. `ros2_control_node` 를 손으로 띄우면
> `Waiting for data on 'robot_description' topic` 에서 멈춘다 — `robot_state_publisher` 가 같이 떠야 한다
> (공식 런치는 이미 그렇게 되어 있다).

### UR16e 로 연결 — `omy_to_ur16e` 브리지 (sim 검증 완료, 추종오차 0.24°)
```bash
# 하드웨어 없이 sim 으로 먼저 돌려볼 것 (virtual_leader 가 /leader/joint_states 를 합성)
ros2 launch ur_bringup teleop_omy.launch.py use_sim_time:=true virtual_leader:=true
python3 src/ur_bringup/isaac/common/switch_control_mode.py streaming   # 팔로워 스트리밍 모드
ros2 service call /omy_bridge/enable  std_srvs/srv/Trigger
```
> **★ mock 리더로는 팔을 못 움직인다.** `mock_components/GenericSystem` 은 *짝이 맞는* 인터페이스만
> 미러링하는데 L100 은 **effort 명령 / position 은 state 전용**이라 위치가 0 으로 고정된다.
> 컨트롤러가 뜨는지 확인하는 용도로는 유효하지만, 구동 검증에는 `virtual_leader:=true` 를 쓸 것.

> **★ 측정 전에 발행자 수를 확인할 것** — `ros2 topic info /leader/joint_states | grep Publisher`.
> `pkill -f "<런치파일명>"` 은 `ros2 launch` **부모만** 죽이고 자식 노드는 살아남아서, 다시 런치하면
> 리더 2개가 서로 다른 위상을 같은 토픽에 쏜다(실제로 겪음 — `HISTORY.md` §22 함정 1).
> **★ engage 게이트**: 매핑된 리더 자세가 UR 현재 자세와 `engage_tol`(0.15 rad) 이내가 아니면
> **거부하고 어긋난 관절을 도(deg)로 알려준다.** 리더를 손으로 맞춘 뒤 다시 호출할 것.
> 이 게이트가 없으면 enable 순간 팔이 차이만큼 튄다.
>
> 부호/오프셋/속도상한/clamp 는 **전부 ROS 파라미터**다. 실물 튜닝은 파라미터로만 하고
> `sign`/`offset` 기본값(J5 반전, J2 −90°)은 URDF 실측값이니 근거 없이 바꾸지 말 것.

### 실물 연결 시
`port_name:=/dev/ttyUSB0`(U2D2), 4 Mbps. udev 규칙은 `open_manipulator_bringup/open-manipulator-cdc.rules`.
남은 캘리브레이션(손목 J4/J6 오프셋, 엔코더 영점)은 `plan_il_vla.md` §3.5 표 참조.

**★ 시작 자세는 랑데부에서 맞춘다 (2026-09-10)** — 임의 자세에서 engage 하지 않는다.
ROBOTIS OMY SRDF 의 `home`(손 떼도 서 있는 자세)이 우리 매핑을 통과하면 정확히 `ready` 가 된다:

```
leader [0, 0, +90°, −90°, +90°, 0]  →  UR16e [0, −90°, +90°, −90°, −90°, 0] = reset_pose.py ready
```

리더는 **자동으로 그 자세에 가지 않는다** — 리더 런치는 중력보상 컨트롤러만 스폰하고
`init_position`/`arm_controller` 가 없다(그건 팔로워 런치에만 붙는다). 사람이 내려놓는다.
실물 측정 항목(rest pose 실측·기구 가동범위·중력보상 드리프트)은 `CHECKLIST.md` E-1~E-3,
배경은 `HISTORY.md` §41.

#### `/omy_bridge/sync` — UR16e 를 랑데부로 (mock + Isaac sim 검증 완료 2026-09-10)

```bash
ros2 service call /omy_bridge/sync   std_srvs/srv/Trigger   # 리더를 먼저 내려놓고
ros2 topic echo   /omy_bridge/status                        # sync:moving → synced
ros2 topic echo   /omy_bridge/engage_error                  # [rad] 관절별 오차, 5 Hz
ros2 service call /omy_bridge/enable std_srvs/srv/Trigger
```

- **MoveIt 으로 계획한다**(`/move_action`, `plan_only=false`). 직전 작업 때문에 팔이 픽스처
  근처일 수 있어서 `reset_pose.py` 식 직선 관절 보간은 안전하지 않다. `move_group` 이 없으면
  서비스가 **거부하면서 수동 절차를 알려준다** — 그때는 팔 주변을 눈으로 확인하고
  `switch_control_mode.py trajectory` + `reset_pose.py ready` + `switch_control_mode.py streaming`.
- 컨트롤러 전환(streaming↔trajectory)을 **서비스 안에서 처리**하고, 끝나면 **streaming 으로 되돌려
  둔다.** 안 그러면 enable 이 성공해도 명령이 갈 곳이 없어 "팔이 안 움직인다"로 보인다.
- **즉시 반환한다**(수락 여부만). 서비스 콜백에서 10 초짜리 팔 동작을 기다리면 단일 스레드
  executor 의 100 Hz 제어 타이머가 멈춘다. 진행상황은 `/omy_bridge/status`.
- 목표는 `rendezvous` 파라미터(기본 = `ready`). **에피소드 리셋 자세와 함께**가 아니면 바꾸지 말 것 —
  텔레옵은 시연 데이터가 시작하는 자세에서 시작해야 한다.
- `enable` 은 **streaming 컨트롤러가 비활성이면 거부**한다(위와 같은 무증상 실패 방지).

#### 게임패드로 enable/disable/sync — `pad:=true`

손이 리더에 있으면 `ros2 service call` 을 칠 수 없다. `teleop_omy.launch.py pad:=true` 가
`joy_node` 를 띄우고, **브리지가 직접 `/joy` 를 읽는다**(teleop_joy 는 Servo/직교 경로라 여기선
쓰지 않는다 — 아무도 듣지 않는 twist 를 발행하게 된다).

| 버튼 (DualSense 기본) | 동작 | 데드맨(L1) |
|---|---|---|
| Options (9) | enable | **필요** |
| R3 (12) | sync | **필요** |
| Create/Share (8) | disable | 불필요 |

**움직임을 시작하는 버튼만 데드맨을 요구한다.** 멈추는 버튼은 언제나 눌린다. 떠도는 `/joy`
메시지 하나가 16 kg 팔을 움직이면 안 되기 때문이다. 인덱스는 전부 파라미터
(`button_bridge_enable` 등)이고, 0~5 는 `teleop_joy.py` 가 쓰므로 기본값을 그 위로 잡아 두었다.

> **★ 함정**: `--symlink-install` 은 **원본 파일의 실행 비트를 그대로 쓴다.** `install(PROGRAMS)` 를
> 걸어도 `chmod +x scripts/*.py` 를 안 하면 `ros2 run` 이 `No executable found` 로 실패한다.

---

## 3. 소스 의존성 (vcstool 로 버전 고정)

`topic_based_hardware_interfaces` 는 apt 에 없고 **특정 태그(0.2.1)** 가 필요하므로 소스로 관리.

```bash
cd /isaac-sim/volume/ur_ws
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
cd /isaac-sim/volume/ur_ws
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
source /isaac-sim/volume/ur_ws/install/setup.bash
export ROS_DOMAIN_ID=0
```

---

## 5. 시뮬레이션 구동 (Isaac Sim)

**기동 순서를 지킬 것** (§8-6). 터미널 3개:

```bash
# T1: Isaac Sim (씬 + ROS2 OmniGraph 자동 구성)
/isaac-sim/python.sh /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py          # GUI(환경/조명 포함)
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
python3 /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/common/moveit_plan_execute_demo.py   # -> RESULT: SUCCESS
```

---

## 5-B. 그리퍼 세트 구동 (UR16e + Robotiq 2F-85)

팔 단독 세트(§5)와 병렬. 그리퍼는 단일 `finger_joint` 만 ROS/Isaac 이 교환하고(나머지 5관절은 자동 mimic),
MoveIt 은 그리퍼 형상까지 충돌 인식. 배경/함정은 [`HISTORY.md`](HISTORY.md) §8.

```bash
# (1회) Isaac UR16e + GRP-ES-CPL-077 커플링 + 2F-85 단일 articulation USD 합성
#   --gripper-z 0.011 = 커플링 두께, --coupling-usd = 손목에 베이크할 커플링 visual (동봉)
/isaac-sim/python.sh /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/common/build_ur16e_2f85.py \
    --out /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_with_2f85.usd --gripper-z 0.011 \
    --coupling-usd /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur_to_robotiq_coupling.usd --coupling-z 0.0

# T1: Isaac — 합성 씬 지정 (--asset-path 는 절대경로)
/isaac-sim/python.sh /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_with_2f85.usd
# T2: 제어 (팔 JTC + gripper_controller)
ros2 launch ur_bringup ur16e_2f85.launch.py
# T3: MoveIt2 + RViz — ★ 그리퍼 전용 (collision-aware SRDF). 공유 ur16e_moveit.launch.py 아님!
ros2 launch ur_bringup ur16e_2f85_moveit.launch.py
```

### 검증
```bash
ros2 control list_controllers     # scaled_joint_trajectory_controller, gripper_controller, joint_state_broadcaster = active
python3 /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/ur16e_2f85/gripper_demo.py          # open/close → goals_ok=True
python3 /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/common/moveit_plan_execute_demo.py   # 팔 plan+execute → SUCCESS (그리퍼 collision 있어도 -10 안 남)
python3 /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/ur16e_2f85/selfcollision_demo.py   # 자기충돌 포즈 → MoveGroup REJECTED 실증
```

### 5-C. 세트 3 — UR16e + 2F-85 + D405 (eye-in-hand 카메라, sim)

세트2(그리퍼) 파일은 그대로 두고 카메라를 **별도 세트(`ur16e_2f85_d405_*`)** 로 분리. 카메라는 sensor-only
(camera link + optical frames, ros2_control joint 없음). 배경/연결/캘리브는 [`HISTORY.md`](HISTORY.md) §9.

```bash
# (1회) 세트3 USD 합성 — 커플링(+7mm) + PickNik 브라켓 + gripper standoff(+18mm) 베이크
/isaac-sim/python.sh /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/common/build_ur16e_2f85.py \
    --out /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd --gripper-z 0.018 \
    --coupling-usd /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur_to_robotiq_coupling.usd --coupling-z 0.007 \
    --camera-mount-usd /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/picknik_camera_adapter.usd

# T1: Isaac — 세트3 씬 + 카메라 그래프 (--with-camera, --asset-path 절대경로)
/isaac-sim/python.sh /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd --with-camera
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
python3 /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/ur16e_2f85_d405/octomap_demo.py   # octomap 활성 plan+execute + probe
```
> perception 미설치/octomap 불필요면 `use_octomap:=false` → 세트2와 동일 collision-aware MoveIt(플러그인 불필요).

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
python3 /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/ur16e_2f85/gripper_demo.py \
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
| (nvblox) `nvblox_node` 가 기동 즉시 SIGABRT, `cudaErrorInvalidDevice: invalid device ordinal` | apt 바이너리가 이 GPU arch 미지원(sm_75 전용) → §2-B-4 소스 빌드. `cuobjdump --list-elf`/`nvidia-smi --query-gpu=compute_cap` 대조 |
| (nvblox) home 포함 **모든** 시작자세가 `world collision detected` / `INVALID_INITIAL_CSPACE_POSITION` | 미관측 복셀이 장애물로 취급됨. `nvblox_cumotion.yaml` 의 `esdf_and_gradients_unobserved_value: 1000.0` 확인(기본 -1000). 노드 재기동 필요(`ros2 param set` 무효) |
| (nvblox) 팔 잔상이 ESDF 에 남아 시작자세 충돌 | `ros2 launch ... ur16e_2f85_d405_nvblox.launch.py segmenter_buffer:=0.25` 로 마스킹 여유 확대 |
| (nvblox) 런치가 `nvblox_examples_bringup` 못 찾음 | 정상 — 그 패키지는 일부러 설치하지 않고 `config/ur16e_2f85_d405/vendor/nvblox_base.yaml` 을 쓴다 |
| apt 로 뭔가 깔았더니 다른 워크스페이스가 깨짐 | NVIDIA 레포가 ROS 패키지를 덮어씀 → §2-B-1 핀 확인. `apt-cache policy <pkg>` 로 Candidate 출처 점검 |
| (세트3) 카메라가 브라켓에서 떠 보이거나 파고듦 | `realsense_d405` origin 의 pitch(8°)/높이(0.01847) 시팅 값 → [`HISTORY.md`](HISTORY.md) §9 참고, cradle 표면 법선과 평행+gap0 으로 맞춤 |
