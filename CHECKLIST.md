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
| ☐ | Isaac Sim **6.0.1 또는 6.1.0** 컨테이너 | `/isaac-sim/python.sh` 존재, `cat /isaac-sim/VERSION` | 컨테이너부터(A-1). 이 워크스페이스는 컨테이너 밖에서 돌지 않는다. 이 머신은 `6.1.0-rc.26+release.49347` — **같은 6.1.0 이미지**를 쓴다. 다른 버전이면 `preflight` 가 경고한다. 6.1.0 의 다른 빌드는 경고만 찍고 통과하니(FAIL 아님) 직접 [`SETUP.md`](SETUP.md) §5 스모크부터 |
| ☐ | ROS 2 **Jazzy** | `/opt/ros/jazzy` 존재 | **없어도 된다** — `bootstrap.sh` 의 `ros` 단계가 공식 절차로 설치한다(Isaac 기본 이미지엔 ROS 도 `python3` 도 없다). 새 컨테이너는 `--fresh` 필요([`SETUP.md`](SETUP.md) §0-B 1-B) |
| ☐ | NVIDIA GPU + 드라이버 **≥ 580** | `nvidia-smi` (이 머신 580.178.04, RTX 5090) | 호스트 드라이버부터 올린다 — CUDA 13.2(cuMotion 의존)가 580 계열을 요구한다 |
| ☐ | 디스크 — 컨테이너 루트 fs **20 GiB** + 볼륨 **30 GiB**(같은 디스크면 합계 50 GiB) | `df -h / /isaac-sim/volume` | apt 는 볼륨이 아니라 **컨테이너 루트 fs** 에 쓴다: 2026-10-21 순정 이미지 실측 apt ≈ 11 GB(CUDA 4.8 + Nsight 2.4 + ROS 2.8), 레이어 +17 GB. 볼륨: ML venv 7.7~7.9 GB + pip 캐시 1.4 GB + build 0.6 GB (+ GR00T 모델 6.5 GB) + 데이터. `preflight` 가 둘 다 본다(WARN) — [`SETUP.md`](SETUP.md) §0-B 전제 |
| ☐ | `/dev/shm` **1 GiB 이상** | `df -h /dev/shm` | 64 MiB 면 학습 워커가 죽는다 — 컨테이너를 `--shm-size=8g` 로 다시 만들거나 [`SETUP.md`](SETUP.md) §2-C 우회책 |
| ☐ | `sudo`(NOPASSWD) `git` `gpg` `curl` locale | `command -v sudo git gpg curl` | **stock Isaac 이미지엔 `sudo`·`git`·`gpg` 가 없다** → A-1 ③ 의 root 단계(`container_prereqs.sh`). 없으면 `preflight` 가 FAIL 하며 그 명령을 찍는다 |
| ☐ | 인터넷 — 설치 때 + **Isaac 실행 때도** | — | Isaac 스크립트가 기동 시 Isaac 에셋 서버(NVIDIA 클라우드)를 찾고, 못 찾으면 `Could not find Isaac Sim assets folder` 로 **종료**한다(배경 방 `Simple_Room`·세트1 로봇 USD 가 거기 있다). 오프라인/프록시 PC 면 먼저 확인 |
| ☐ | `ROS_DOMAIN_ID` 정하기 | `ros2 node list` 에 모르는 노드가 없는가 | **0 이 아니고 같은 호스트의 컨테이너마다 다른 값**, 한 셋업의 **모든 터미널에서 같은 값**. 이 워크스페이스(호스트의 첫 컨테이너)는 `42` — 같은 호스트에 두 번째 컨테이너를 만들면 다른 값(예: 43)으로, 아래 예시의 `42` 도 전부 그 값으로 읽는다 |

### A-1. 컨테이너 만들기 (호스트에서) — ★ stock 이미지에는 root 단계가 하나 더 필요하다

2026-09-16 "새 컨테이너" 검증([`HISTORY.md`](HISTORY.md) §48)은 `sudo`·`git`·`gnupg2`·locale·GUI 라이브러리 + NOPASSWD sudoers 의
root 레이어가 이미 있는 이미지였다(§55.1). 그 레이어를 지금은 `src/setup/container_prereqs.sh` 가 깐다.
**순정 이미지 경로는 2026-10-21 에 검증됐다**([`HISTORY.md`](HISTORY.md) §57): 호스트에서 `nvcr.io/nvidia/isaac-sim:6.1.0` 새 컨테이너 +
작업 트리 `rsync` 복사, headless 로 이 문서의 A-1 → B → C-1~C-4 통과(`bootstrap.sh --fresh` 34 분 59 초, 재실행 17 초).
첫 시도는 apt 가드 파서 결함으로 `ros` 단계에서 멈췄고 고쳤다. `git clone` 경로는 사용자 커밋 뒤 다시 확인할 것 — 결과는 `HISTORY.md` 에.

```bash
# ① 호스트에서 src 를 볼륨 자리에 둔다 (컨테이너엔 아직 git 이 없다)
VOL=/path/to/new_volume                           # ★ 바꿀 것: 새 볼륨의 호스트 경로. 기존 워크스페이스 볼륨을 가리키지 말 것
                                                  #   (아래 setfacl -R 이 그 볼륨 전체에 돈다)
mkdir -p "$VOL/ur_ws"
git clone <이 저장소> "$VOL/ur_ws/src"            # 또는 옛 PC 에서 rsync -a (★ clone 은 커밋된 것만 온다 — 옛 PC 에서 git status 확인)
# 컨테이너 사용자(isaac-sim, uid 1234)와 호스트 사용자 둘 다 — 기본 ACL(d:)이라 컨테이너가 만든 build/install/deps 도
# 호스트에서 읽고 지울 수 있다(없으면 B-0 의 되돌리기 rm -rf 가 호스트에서 실패). 이미 쓰던 볼륨이면 다시 돌려도 된다
sudo setfacl -R -m u:1234:rwX,u:$(id -u):rwX,d:u:1234:rwX,d:u:$(id -u):rwX "$VOL"

# ② 컨테이너 생성. nvcr.io/nvidia/isaac-sim:6.1.0 = 이 머신과 같은 6.1.0-rc.26+release.49347 (2026-10-21; 안에서 cat /isaac-sim/VERSION 확인)
docker run -d -it --name ur_ws --entrypoint bash -u 1234:1234 \
  --gpus all --network host --shm-size=8g \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -v "$VOL":/isaac-sim/volume \
  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  nvcr.io/nvidia/isaac-sim:6.1.0

# ③ root 단계 (1회, 멱등) — sudo·git·gnupg2·locales·GUI(mesa/vulkan/xauth) + NOPASSWD sudoers
docker exec -u 0 -it ur_ws bash /isaac-sim/volume/ur_ws/src/setup/container_prereqs.sh --sudo-user isaac-sim
docker exec -it ur_ws bash                        # 이후는 전부 컨테이너 안, isaac-sim 사용자로
git config --global --add safe.directory /isaac-sim/volume/ur_ws/src   # (컨테이너 안) ① 을 호스트에서 clone·rsync 했으면
```

- ☐ `--network host` — 실물 UR16e(RTDE)·ROS 2 DDS 가 호스트 네트워크를 그대로 쓴다([`HARDWARE.md`](HARDWARE.md) 4-B ⑥). sim 만이면 생략 가능
- ☐ `-e DISPLAY -v /tmp/.X11-unix…` 줄은 **GUI 를 볼 때만**(호스트에서 `xhost +local:` 필요할 수 있음). 전부 `--headless` 면 그 줄을 빼도 된다 — 2026-09-16 검증과 하네스는 전부 headless
- ☐ **③ 은 전용 컨테이너 전제다** — 빠진 패키지를 깔면서 이미지의 기반 패키지 **약 10 개**(`libc6`·mesa·`curl` …, Ubuntu 출처)를
      **업그레이드한다**. 설치 직전에 apt 시뮬레이션(새 패키지 수 + 업그레이드 전부와 출처)을 찍고, Ubuntu 가 아닌 출처 업그레이드는 경고,
      삭제는 거부한다(`ALLOW_UPGRADES=1` 이면 강제). 확인 없이 진행한다
- ☐ `container_prereqs.sh --dry-run` 으로 먼저 목록만 볼 수 있다(root 불필요) — 영향도는 apt 목록이 있을 때만 나온다(순정 이미지는
      목록이 비어 있어 실제 실행이 설치 직전에 찍는다). 실제 실행은 root 가 아니면 거부한다
- ☐ 호스트에서 clone·`rsync` 한 `src/` 는 소유자가 호스트 사용자라 컨테이너 안 `git status` 가 `detected dubious ownership` 으로 거부된다 →
      위 `safe.directory` 한 줄(설치 스크립트는 상관없다)
- ☐ 이 머신의 실제 컨테이너는 템플릿과 조금 다르다(`--privileged`, 호스트 IPC, Isaac 캐시 마운트 6 개 — 캐시가 없으면 컨테이너를
      새로 만들 때마다 셰이더 재컴파일 3~4 분). 목록은 [`SETUP.md`](SETUP.md) §0-B 1
- ☐ 워크스페이스 루트 `CLAUDE.md`(Claude Code 에이전트 가이드)는 **git 에 없다**(`src/.gitignore`). 새 PC 에서도 에이전트를 쓸 거면
      옛 PC 의 `/isaac-sim/volume/ur_ws/CLAUDE.md` 를 같은 자리에 손으로 복사한다. 사람이 볼 내용은 전부 `src/` 문서에 있다

---

## B. 소프트웨어 설치 (하드웨어 0개)

- ☐ **ROS 2 Jazzy** — `ls /opt/ros/jazzy/setup.bash`. 없으면(Isaac Sim 기본 이미지) `bootstrap.sh` 의 `ros` 단계가
      공식 절차로 `ros-jazzy-desktop` + `ros-dev-tools` 를 설치한다([`SETUP.md`](SETUP.md) §0-B 1-B). 있으면 건드리지 않는다.

```bash
# src 는 A-1 ① 에서 이미 /isaac-sim/volume/ur_ws/src 에 있다 (★ repo = src/ 다)
/isaac-sim/volume/ur_ws/src/setup/bootstrap.sh --dry-run  # ☐ 먼저 계획만 본다
/isaac-sim/volume/ur_ws/src/setup/bootstrap.sh --fresh    # ☐ 실행 — 이 워크스페이스 전용 새 컨테이너
/isaac-sim/volume/ur_ws/src/setup/bootstrap.sh            # ☐ 실행 — 다른 프로젝트와 공유하는 컨테이너
```

> ★ **컨테이너를 다른 프로젝트와 공유한다면 `--dry-run` 출력을 그냥 넘기지 말고
> 아래 `B-0. 공유 컨테이너 격리 확인` 을 먼저 볼 것.** 새 컨테이너의 `--dry-run` 에서
> `base`·`cumotion`·`verify` 가 "could not be simulated"(끝에 `not simulated: …`)로 뜨는 건 정상이다(apt 목록·레포가 아직 없다).
> `ros` 는 할 일(`would …`)만 찍는다.

- ☐ `preflight`(읽기 전용, `--dry-run` 에서도 실제로 돈다)가 보는 것: Isaac + 버전 · `/opt/ros/jazzy`(또는 `ros` 단계 예정) ·
      GPU sm + 드라이버 ≥ R580 · `/dev/shm`(1 GiB 미만 WARN) · 암호 없는 `sudo`·`git`·`curl`·`gpg`·locales · vcstool ·
      빈 디스크(루트 fs + 워크스페이스) · 설치용 호스트 HTTP 도달(Ubuntu·ROS·GitHub·NVIDIA 3·gitlab·PyTorch·PyPI·pypa) · `src/ur16e.repos`.
      **FAIL 은 Isaac 없음 · ROS 없는데 `ros` 단계도 없음 · 준비물 없음 · `ur16e.repos` 없음뿐**, 나머지는 WARN — WARN 도 읽고 넘어간다

- ☐ 옵션: `--fresh`(**새 컨테이너 필수** — Ubuntu 공식 출처의 기반 라이브러리 업그레이드를 받아들인다, 그게 없으면 `ros` 단계가 가드에서 멈춘다.
  개수는 Ubuntu 업데이트에 따라 늘어난다 — 2026-09-16 14+4, 2026-10-21 23+4. 볼 것은 개수가 아니라 가드가 나열한 줄이 **전부
  `Ubuntu:24.04/noble-updates`·`-security` 출처인가**다; `--fresh` 는 그것만 받는다) /
  `--no-ml`(torch/lerobot 8 GB 생략, sim+teleop 만 할 때) / `--with-groot`(GR00T extra 까지; 모델은 B-2) /
  `--with-udev`(실물 OMY-L100 이 있을 때만)

돌아가는 순서: `preflight → ros → pin → repos → base → cumotion → sources → build → leader → ml → verify`
(`ros` 는 `/opt/ros/jazzy` 없을 때만. `pin` 이 `repos` 보다 먼저인 이유는 [`SETUP.md`](SETUP.md) §0-B — NVIDIA 레포가 ROS 패키지를 덮어쓴다.)
실측 소요: 전체 약 35~40 분(2026-10-21 순정 이미지 **34 분 59 초**, 재실행 17 초 — 네트워크 좌우). 끝나면 `check_env.sh` 가 `environment OK` 여야 한다.
WARN 은 통과에 영향이 없다 — 새로 깐 머신이면 apt 표류 WARN 한 줄이 언제나 뜬다(목록은 `check_env.sh --verbose`).

- ☐ 호스트 CPU 를 다른 컨테이너와 나눠 쓰면 빌드 병렬도는 `BUILD_JOBS`(기본 **2** — colcon `--parallel-workers` 와 `MAKEFLAGS=-j`).
      전용 PC 면 `BUILD_JOBS=8 …/bootstrap.sh --fresh` 처럼 올려도 된다
- ☐ 버전 고정: ML venv 는 `setup/requirements-ml.lock` 을 pip constraints 로 쓰고 torch 는 `2.11.0` 고정(풀려면 `ML_UNLOCKED=1`).
      apt 는 고정이 아니라 **비교**다 — `check_env.sh` 가 `setup/apt-versions.lock`(이 머신 스냅샷)과 다른 버전 개수를 WARN 한 줄로 알린다(`--verbose` 로 나열).
      다르다고 실패는 아니지만, 이상하면 그 목록부터 본다([`SETUP.md`](SETUP.md) §0-A)
- ☐ 언제든 재점검: `src/setup/check_env.sh` (읽기 전용)

### ★ B-0. 공유 컨테이너 격리 확인 — `--dry-run` 출력에서 볼 것

**이 설치는 "영향 0" 이 아니다.** 워크스페이스 밖에 쓰는 곳이 아래 목록이고, 그게 의도한 것인지
사람이 확인하고 넘어가야 한다. 컨테이너에 다른 프로젝트가 있으면 **B 실행 전에** 이 절을 먼저 본다.

**워크스페이스 안에만 있는 것** (지우면 완전히 되돌아감 — 확인 불필요):
`$WS/build`·`$WS/install`(colcon 오버레이, `/opt/ros` 에 안 씀) · `$WS/src`(vcs 소스) ·
`$WS/deps/.venv-ml`(torch·lerobot) · `$WS/deps/pip_cache`(≈ 1.4 GB) · `$WS/deps/torch_home`(ACT 백본 resnet18 45 MB) ·
`$WS/deps/hf_cache`(GR00T 6.5 GB + Cosmos 토크나이저 11 MB — 직접 받아 넣으려면 [`SETUP.md`](SETUP.md) §2-C-2 "수동 배치" 레이아웃, 검증 `src/setup/check_hf_cache.sh`).
`setup.sh ml` 이 venv 에 `ur_ws_cache_paths.pth` 를 넣어 venv 프로세스의 `PIP_CACHE_DIR`·`TORCH_HOME`·`HF_HOME` 기본값이 이 `deps/` 들이다
(`ml_env.sh` 도 같은 값). **되돌리기** `rm -rf $WS/build $WS/install $WS/deps` — 호스트에서 지우려면 A-1 ① 의 ACL 에 호스트 사용자 항목이 있어야 한다. `setup.sh groot` 가 `deps/hf_cache` 를 `chmod 700`(HF 토큰 보호)으로 만들어 ACL mask 가 `---` 이므로, 그 뒤엔 **컨테이너 안에서** 지우거나 먼저 `chmod -R g+rwX deps/hf_cache`.

**컨테이너 전역에 쓰는 것 — 전부 확인할 것**:

- ☐ (A-1 ③ root 단계) `container_prereqs.sh` — 빠진 패키지 + 기반 패키지 약 10 개 업그레이드(전용 컨테이너 전제), `/etc/sudoers.d/<사용자>`(`--sudo-user`)
- ☐ `ros` 단계(`/opt/ros/jazzy` 가 없을 때만) — `/etc/apt/sources.list.d/ros2.sources` + `/usr/share/keyrings/ros2-archive-keyring.gpg` +
      `/usr/share/ros-apt-source/`(`ros2-apt-source` 1.3.0 .deb, sha256 검사), locale(`en_US.UTF-8` 이 없을 때만: `locale-gen` →
      `/usr/lib/locale/locale-archive`, `update-locale` → `/etc/default/locale` 의 `LANG`/`LC_ALL=en_US.UTF-8`)
- ☐ `/etc/apt/preferences.d/99-nvidia-isolate.pref` — `pin` 단계.
      이미 있으면 건드리지 않는다. 다른 프로젝트가 NVIDIA 레포 패키지를 **일부러** 쓰고 있다면
      Pin-Priority 100 이 그걸 막을 수 있으니 그 프로젝트 담당자와 확인
- ☐ `/etc/apt/sources.list.d/nvidia-{isaac-ros,cuda,vpi}.list` + `/usr/share/keyrings/nvidia-{isaac-ros,cuda,vpi}.gpg` — `repos` 단계, 그리고 `/var/lib/apt/lists`
- ☐ **apt 패키지 시스템 설치** — `ros`·`base`·`cumotion` 단계(`/opt/ros/jazzy`·`/usr/local/cuda-13.x`·Nsight·`/usr`, 루트 fs +17 GB). ★ 아래가 핵심
- ☐ `~/.cmake/packages/Eigen3/<md5>` — `nvblox_ros` 소스빌드가 쓴다(Eigen ExternalProject 의 `export(PACKAGE)`, colcon 줄로 못 끈다).
      `build` 단계가 끝에 `$WS/build` 를 가리키는 항목을 지운다. 손으로 `colcon build --packages-select nvblox_ros` 하면 다시 생기고, 그 파일 `rm` 이 되돌리기
- ☐ 실행 중: `~/.ros`(로그, `~/.ros/locks`), `~/.cache/matplotlib`(시스템 python3 matplotlib — `base_placement`)
- ☐ `/etc/udev/rules.d/99-open-manipulator-cdc.rules` — `udev` 단계.
      **기본 실행에서 제외돼 있다**(`DEFAULT_STAGES` 에 없음). U2D2 실물이 붙었을 때만 E단계에서

더 이상 안 쓰는 곳: `~/.cache/pip`·`~/.cache/torch`·`~/.cache/huggingface`(`setup.sh ml` 이후엔 위 `deps/` 로 간다), `/tmp`(.deb·임시 파일을
남기지 않는다). 2026-10-21 이전에 설치한 머신은 `src/setup/setup.sh ml` 을 다시 돌린다(그 전까지 `check_env.sh` WARN 2 건) — 그 뒤 옛
`~/.cache/{pip,torch,huggingface}` 는 다른 데서 안 쓰면 손으로 지워도 된다.

**apt 판정 — `--dry-run` 출력에서 이 줄들을 본다**:

| 출력 | 뜻 | 조치 |
|---|---|---|
| `ok pure addition (0 upgraded, 0 removed)` | 순수 추가 | ☐ 그대로 진행 |
| `warn would UPGRADE existing packages:` | 기존 패키지가 바뀐다 | ☐ **멈춘다** — 목록을 읽는다 |
| `warn would REMOVE packages:` | 삭제된다 | ☐ **멈춘다** |
| `FAIL refusing: this would change packages other workspaces may depend on.` | 가드가 막았다 | ☐ 정상 동작이다 |

- ☐ **`ALLOW_UPGRADES=1` 을 반사적으로 붙이지 않는다.** 이 플래그는 가드를 끄는 것이지
      안전하게 만드는 게 아니다. 목록을 읽고 "이 패키지가 바뀌어도 되는가"를 판단한 뒤에만 쓴다.
      중간 단계 **`ALLOW_UPGRADES=ubuntu`**(= `--fresh`)는 출력의 출처가 전부 `Ubuntu:24.04/noble-updates`·`-security` 일 때만
      통과시킨다 — 새 컨테이너의 낡은 기반 라이브러리(`util-linux`·`libsystemd0`·`ncurses` …)가 바로 그 경우다.
- ☐ 새 컨테이너면 오히려 거부가 뜰 수 있다 — 가드는 *"이미 깔린 게 바뀌는가"* 를 보므로,
      다른 버전이 선점돼 있으면 여기서 순수 추가였던 게 거기선 업그레이드가 된다. **버전 차이지 오류가 아니다.**

**알려진 예외 — 이건 진짜로 다른 ws 에 영향이 간다**:

- ☐ cuMotion/realsense 가 `diagnostic_updater` 4.2.7 을 끌어오면 구버전 ros2_control(4.44)이
      `undefined symbol: diagnostic_updater::Updater` 로 죽는다 → **ros2_control 스택 4.45.2 동반 업그레이드**가
      필요하다. `setup.sh` 는 이걸 **자동화하지 않고 가드에 걸려 멈춘다**(사람이 결정하라는 설계).
      배경은 [`HARDWARE.md`](HARDWARE.md) §4 의 ②(부분 업그레이드 ABI 깨짐).
      **새 컨테이너에는 해당 없음** — 처음부터 깔면 ros2_control 4.48.0 과 diagnostic_updater 4.2.7 이 같은 빌드로 들어온다(2026-09-16 실측).

> **가장 확실한 답: 가능하면 전용 컨테이너를 쓴다.** 다른 프로젝트가 없으면 위 항목들이 충돌할
> 대상 자체가 없다. 공유가 불가피할 때 이 스크립트가 하는 일은 *"안전하게 만드는 것"* 이 아니라
> **"위험한 순간에 멈추고 사람에게 묻는 것"** 이다.

### B-2. GR00T N1.7 — 스크립트가 못 하는, **사용자 계정이 필요한** 부분

`bootstrap.sh` 는 ACT 까지 완결한다(ML venv, sm_120 torch, lerobot). GR00T 는 **모델 파일이 HF 게이트**라
사용자가 아래를 직접 해야 한다. 상세·함정은 [`SETUP.md`](SETUP.md) §2-C-2, 학습 명령은 [`PIPELINE.md`](PIPELINE.md) §3-B.

- ☐ **B-2-1 extra 설치** — `src/setup/setup.sh groot` (또는 처음부터 `bootstrap.sh --fresh --with-groot`).
      기존 패키지 변경 0 이어야 통과한다(torch 교체 = sm_120 상실 방지). 끝에 "model files NOT in place yet" 경고가 뜨는 게 정상.
- ☐ **B-2-2 라이선스 동의(브라우저)** — <https://huggingface.co/nvidia/Cosmos-Reason2-2B> (gated=auto, 동의 즉시 열림).
      `nvidia/GR00T-N1.7-3B` 자체는 게이트가 아니지만 **백본 토크나이저**가 Cosmos 쪽에 있어 이게 없으면 학습 시작 시 401.
- ☐ **B-2-3 read 토큰** — <https://huggingface.co/settings/tokens>. Fine-grained 면 **"Read access to contents of all public gated repos you can access"** 스코프 필수.
- ☐ **B-2-4 토큰을 워크스페이스 안에** (공유 `~/.cache/huggingface` 오염 금지):
  ```bash
  cd /isaac-sim/volume/ur_ws && source src/setup/ml_env.sh      # HF_HOME=deps/hf_cache
  deps/.venv-ml/bin/hf auth login                                # → deps/hf_cache/token
  ```
- ☐ **B-2-5 다운로드** (총 6.5 GB + 11 MB; `HF_HUB_OFFLINE=0` 은 다운로드 때만):
  ```bash
  HF_HUB_OFFLINE=0 deps/.venv-ml/bin/hf download nvidia/GR00T-N1.7-3B
  HF_HUB_OFFLINE=0 deps/.venv-ml/bin/hf download nvidia/Cosmos-Reason2-2B \
      --include config.json --include 'tokenizer*' --include vocab.json --include merges.txt \
      --include '*preprocessor_config.json'
  ```
  ★ `--include` 는 **패턴마다 하나씩** 붙인다. `--include 'a' 'b' 'c'` 로 쓰면 hf 1.31 은 `'b' 'c'` 를 파일명 인자로 받아
  `Ignoring --include since filenames have been explicitly set` 경고와 함께 **`config.json` 을 빼고** 받는다(→ B-2-6 FAIL).
  다른 PC 에 이미 있으면 `cp -rL <그 PC>/deps/hf_cache/hub/models--nvidia--* deps/hf_cache/hub/` 로 복사해도 된다(레이아웃은 SETUP §2-C-2).
- ☐ **B-2-6 검증** — `src/setup/check_hf_cache.sh` 가 필수 파일(Cosmos 는 `config.json` `tokenizer.json` `tokenizer_config.json`
      `vocab.json` `merges.txt` `preprocessor_config.json` `video_preprocessor_config.json`)을 전부 `ok` 로 찍고
      `OFFLINE_OK`(토크나이저 vocab 151669)까지 rc 0. 이후 학습·추론은 토큰 없이 오프라인으로 돈다(`ml_env.sh` 가 `HF_HUB_OFFLINE=1`).
- ☐ **B-2-7 학습 스모크 → C-4c 로 옮겼다.** 학습에는 LeRobot 데이터셋이 있어야 하는데 B 단계에는 아직 없다.

### B-3. (선택) 옛 PC 의 학습 산출물 옮기기

`outputs/`(체크포인트·데이터셋)는 git 에도 설치 스크립트에도 없다. 다시 수집·학습하지 않고 **현재 최고 정책**(ACT physical
190 ep 200k, sim 기존 범위 19/20 · 넓은 범위 20/20 — `HISTORY.md` §58.7)을 새 PC 에서 롤아웃하려면 손으로 옮긴다 — 명령·크기·이어 학습 시 고칠 것은 [`SETUP.md`](SETUP.md) §0-C.

- ☐ 추론만: `outputs/act_red_left_190_phys/checkpoints/200000/pretrained_model`(≈198 MB) 하나 (100 ep 판 `act_red_left_100_phys_long/checkpoints/200000/pretrained_model` 도 같은 크기 — 비교용)
- ☐ 이어 학습까지: 같은 체크포인트의 `training_state` + `checkpoints/last` 심볼릭링크 + 데이터셋 `outputs/lerobot_ds_red_left_190_phys`(382 MB; 100 ep 판은 `lerobot_ds_red_left_100_phys` 202 MB). 이어 학습은 [`PIPELINE.md`](PIPELINE.md) §3-1·§3-2 의 `--resume` — `train_100.sh` 는 시작 시 `CK` 를 `rm -rf` 한다.
      `train_config.json` 에 절대경로(`/isaac-sim/volume/ur_ws/...`)가 박혀 있다 — **같은 경로로** 옮기면 고칠 것이 없다
- ☐ **옮기기 전에 옛 컨테이너 안에서 `chmod -R g+rwX outputs/act_red_left_190_phys`(옮기는 체크포인트마다 — `act_red_left_100_phys_long` 도 같다)** — `model.safetensors` 는 0600 으로 저장돼 ACL 마스크가
      `---` 라 호스트 사용자 ACL 이 있어도 호스트에서 못 읽는다(호스트끼리 `rsync -a` 실패). 다른 사용자까지 읽게 하려면 `a+rX`
- ☐ 복사는 `rsync -a`/`tar` 로(심볼릭링크 보존). GR00T 체크포인트(`groot_3task_v3_abs` 등)는 이 머신에도 없다 — 옮길 것 없음

---

## C. 소프트웨어 검증 (아직 하드웨어 0개)

세 터미널 모두 먼저:
```bash
source /opt/ros/jazzy/setup.bash && source /isaac-sim/volume/ur_ws/install/setup.bash
export ROS_DOMAIN_ID=42      # A 에서 정한 값 — 0 이 아니고 이 호스트의 다른 컨테이너와 다른 값, 모든 터미널에서 같게
```
(42 는 이 워크스페이스(호스트의 첫 컨테이너) 값이다 — 같은 호스트의 다른 컨테이너와 도메인이 겹치면 그쪽 `/robot_description` 을 받아
`controller_manager` 가 죽는다, 아래 "자주 걸리는 것". 두 번째 컨테이너면 다른 값으로. 하네스는 환경의 값을 물려받고 안 정하면 0 + 경고.)

C 의 항목은 **하나씩** 한다 — 다음 항목 전에 앞 항목의 Isaac·런치를 `bash src/ur_bringup/scripts/harness/shutdown_rollout.sh` 로 내리고
마지막 줄 `CLEAN` 을 확인한다(`LEFT: …` 면 남은 것이 찍히고 rc 1). 디스플레이 없는(headless) 컨테이너면 Isaac 은 `--headless`,
MoveIt 런치(`*_moveit.launch.py`)는 `launch_rviz:=false`.

- ☐ **C-1 sim pick&place** — Isaac → 제어 → MoveIt → 데모. [`README.md`](README.md) §5
- ☐ **C-2 가짜 리더 teleop** — 리더 없이 브리지 검증. 먼저 C-1 을 내리고, 세트 2 Isaac + 제어를 띄워 `ready` 로 보낸다
      ([`SETUP.md`](SETUP.md) §0-B 4 의 터미널 1·2 — MoveIt 불필요):
  ```bash
  /isaac-sim/python.sh src/ur_bringup/isaac/common/ur16e_isaac_ros2.py --headless \
      --asset-path /isaac-sim/volume/ur_ws/src/ur_bringup/isaac/assets/ur16e_with_2f85.usd     # 터미널 1
  ros2 launch ur_bringup ur16e_2f85.launch.py use_sim:=true                                     # 터미널 2
  python3 src/ur_bringup/isaac/common/reset_pose.py ready                                       # error_code 0 확인
  # 터미널 3
  ros2 launch ur_bringup teleop_omy.launch.py use_sim_time:=true virtual_leader:=true
  python3 src/ur_bringup/isaac/common/switch_control_mode.py streaming
  ros2 service call /omy_bridge/enable std_srvs/srv/Trigger
  ```
  ✅ 팔이 가짜 리더를 따라가면 성공 (기대 추종오차 1° 의 몇 분의 1 이하 — 2026-10-21 실측 0.116°, `/omy_bridge/engage_error`).
- ☐ **C-3 mock 실물 경로** — 실물 배선 전에 런치가 뜨는지만. **C-2 의 Isaac·제어·브리지를 먼저 내린다**(`shutdown_rollout.sh` → `CLEAN`;
      같은 도메인에 sim `controller_manager` 가 남아 있으면 이름이 겹친다). Isaac 불필요:
  ```bash
  ros2 launch ur_bringup ur16e.launch.py use_sim:=false use_mock_hardware:=true
  ros2 control list_controllers      # 다른 터미널 — 컨트롤러 목록이 나오면 합격. 끝나면 Ctrl-C 후 shutdown_rollout.sh
  ```
- ☐ **C-4 (선택, ML 을 깔았으면 권장) sim IL 스모크** — C-1~C-3 은 ML 쪽(venv·`ur16e_ros` 플러그인·정책 서버·채점)을
      한 번도 실행하지 않는다. 하나만 고르면 된다. 롤아웃 하네스 `rollout_gui_then_n.sh` 는 Isaac 을 **스스로 띄우고 기존
      `ur16e_isaac_ros2.py` 를 죽이므로** 그 전에 C-1~C-3 의 Isaac·런치를 내린다 — `bash src/ur_bringup/scripts/harness/shutdown_rollout.sh`
      (마지막 줄 `CLEAN` 확인 — `LEFT: …` 면 남은 것이 찍히고 rc 1). 이 스크립트는 Isaac·런치 부모와 그 노드 전부(ros2_control_node·
      move_group·cuMotion 컨테이너·OMY 브리지·ur_robot_driver 보조 노드 …)를 **실행 파일 경로 + 이 워크스페이스 install 이 source 된 환경**으로만
      골라 내리고 우리 도메인의 ROS 데몬만 멈춘다 — 다른 워크스페이스의 같은 바이너리는 'not ours' 로 보고만 하므로 공유 컨테이너에서도 쓸 수 있다.
      `--dry-run` 은 대상만 보여 준다. 끝난 뒤에도 같은 방법으로 정리한다.
  - **C-4a — 옮겨 온 체크포인트 롤아웃 1회**(B-3 을 했을 때, ≈10 분 — 셰이더 캐시가 비면 Isaac 첫 기동에 3~4 분):
    ```bash
    cd /isaac-sim/volume/ur_ws && export ROS_DOMAIN_ID=42      # GRASP_MODE 기본 physical, 시작 자세 ready
    bash src/ur_bringup/scripts/harness/rollout_gui_then_n.sh 1 70 \
        $PWD/outputs/act_red_left_190_phys/checkpoints/200000/pretrained_model
    ```
    인자 = `[횟수] [초] [체크포인트]`. 세 번째를 빼면 기본값 `outputs/act_red_left_100_phys/checkpoints/last`(60k, 새 PC 엔 없음)를 찾아
    Isaac·스택을 다 띄운 **뒤에** `FAIL: no checkpoint` 로 끝난다. ✅ 마지막 줄이 `== 1/1 rollouts succeeded`(기존 범위 19/20 · 넓은 범위 20/20 정책이라 대개 성공; 0/1 이어도 `start=… FAIL`
    줄이 찍혔으면 파이프라인은 동작한 것 — `FAIL: isaac`/`FAIL: bringup`/`FAIL: policy server`/`SKIP` 이면 설치 문제). 로그 `outputs/harness_logs/r100_*`
  - **C-4b — 짧은 전체 흐름**(체크포인트 없이, ≈30 분): [`PIPELINE.md`](PIPELINE.md) §1~§3 을 작은 숫자로.
    ① §1-1 Isaac + §1-2 제어·MoveIt 을 띄우고 §0 대로 `cd /isaac-sim/volume/ur_ws` 한 쉘에서
    `TASKS="red|left|put the red block on the left marker" src/ur_bringup/scripts/collect_il_episodes.sh 3 outputs/il_raw_smoke_phys`
    ② §2 변환을 `--raw outputs/il_raw_smoke_phys --repo-id tony/ur16e_smoke_phys --root outputs/lerobot_ds_smoke_phys` 로 → `meta/info.json` 존재
    ③ `shutdown_rollout.sh` 로 Isaac·런치를 내리고 `source src/setup/ml_env.sh` 후 §3 ACT 명령을 `--steps=200 --batch_size=8 --num_workers=2` 와 위 데이터셋,
    `--output_dir=outputs/act_smoke_phys` 로 → rc 0 + `outputs/act_smoke_phys/checkpoints/last/pretrained_model/config.json`
    ④ `bash src/ur_bringup/scripts/harness/rollout_gui_then_n.sh 1 70 $PWD/outputs/act_smoke_phys/checkpoints/last/pretrained_model` — 200 스텝 정책이라 **성공은 기대하지
    않는다**, `start=… FAIL …` 채점 줄까지 가면 합격. 스모크 산출물(`outputs/*smoke*`)은 끝나면 지워도 된다
  - **C-4c — GR00T 학습 스모크**(B-2 를 했을 때만, 옛 B-2-7): C-4b 의 데이터셋(`--dataset.repo_id=tony/ur16e_smoke_phys --dataset.root=outputs/lerobot_ds_smoke_phys`)으로 [`PIPELINE.md`](PIPELINE.md) §3-B
    명령에 `--steps=20 --policy.max_steps=20 --save_freq=20 --output_dir=outputs/groot_smoke` → rc 0
    (`base_model_path` 는 로컬 경로, `model_params_fp32=false`, `OMP_NUM_THREADS=8` — 셋 다 기본값이면 안 된다).
    스키마가 같은 데이터셋이라 그대로 쓴다(이 조합 자체는 아직 돌려 보지 않았다 — 검증은 3태스크 데이터셋으로 했다)

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
ros2 launch ur_bringup omy_leader.launch.py \
    port_name:=/dev/ttyUSB0 use_self_collision_avoidance:=false
ros2 control list_controllers -c /leader/controller_manager   # ★ /leader 네임스페이스
ros2 run ur_bringup omy_leader_calib.py --mode check
```

- ☐ 컨트롤러 4개 전부 `active`
- ☐ T1 에 `Joint name 'joint1' not found in the first joint state message` 가 **반복되면** ROBOTIS 런치를 직접 띄운 것
      (UR16e 의 `/joint_states` 를 받음). 무해하지만 `ur_bringup omy_leader.launch.py`(리매핑 래퍼)로 띄우면 사라진다(`HARDWARE.md` 4-B ②)
- ☐ 손으로 리더를 움직이면 `/leader/joint_states` 가 따라 변한다
- ☐ **① 살아있는가** — 발행 **~300 Hz**, `joint1..6` + `rh_r1_joint` **7개 전부** 존재,
      관절별 표준편차 **< 0.01 rad**(`--mode check` 가 `★ 잡음 큼` 을 찍으면 배선/전원부터)
- ☐ **② ★ 물리적 rest pose 의 실제값** — 리더를 **손 떼도 서 있는 자세**로 놓고 `--mode check`
      의 관절별 현재값을 읽어 적는다.
      기대값 `[0°, −90°, +152°, −62°, +90°, 0°]` (ROBOTIS 브링업 `ready`,
      `open_manipulator_bringup/config/omy_f3m_follower_ai/initial_positions.yaml` — 상완 뒤로 수평,
      전완 앞·위로 접힘. SRDF `home` 이 **아니다**, `HISTORY.md` §47)
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

### E-2. 부호 → 오프셋 캘리브레이션 (둘 다 켜고, **브리지는 disabled 유지**)

- ☐ **부호(sign)가 먼저다.** 기본값 `sign [1,−1,−1,−1,1,−1]` / `offset [180,−90,0,−90,0,0]°` 는 **2026-09-17 실물 1호기 실측**
      (`HISTORY.md` §49.8; URDF 유도값은 J5 방향이 틀렸고 거울 가지로 교체됨). 다른 개체의 L100 이면 다시 확인한다 — URDF 는
      엔코더 증가 방향을 말해 주지 않는다(GELLO 도 조립마다 `joint_signs` 를 잰다: UR (1,1,−1,1,1,1)).
      ① `--mode check` 의 "부호 점검" 줄(rest pose 공칭 `[0,−90,152,−62,90,0]°` 와 실측 부호 비교, J2~J5)
      ② J1/J6 과 확정은 `max_joint_speed:=0.5` 로 engage 해 관절 하나씩 조금 움직여 방향 비교
      ③ 반대면 disable 후 `ros2 param set /omy_to_ur16e sign "[...]"` → **그 다음에** 아래 오프셋 match
      (부호가 바뀌면 그 관절 오프셋도 바뀐다 — 순서를 바꾸면 match 가 틀린 값을 정밀하게 잰다)

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
| **OMY L100** 실측 rest (2026-09-17 1호기; ROBOTIS 공칭 `ready` [0,−90,152,−62,90,0]) | −1.1° | −88.4° | +152.0° | −69.6° | +87.5° | +1.6° |
| **UR16e** `reset_pose.py ready` (2026-09-17, `HISTORY.md` §49.8; 매핑 sign [1,−1,−1,−1,1,−1], offset [180,−90,0,−90,0,0]) | 178.9° | −1.6° | −152.0° | −20.4° | 87.5° | −1.6° |
| (구) `ready_v2` = 2026-09-16 URDF 가지 (§47) | 0° | −180° | +152° | −152° | −90° | 0° |

- ☐ **★ 베이스 뒤 공간**: UR16e `ready` 는 상완이 **뒤로 수평**(팔꿈치가 베이스 뒤 0.48 m, 높이 0.18 m)이다.
      베이스 뒤로 **0.55 m 가 어깨 높이에서 비어 있는지** 먼저 확인. 부족하면 J2 를 세우고(예 −150°) 브리지
      `offset[1]` 도 같은 양만큼 바꾼다 — 자세와 오프셋은 항상 같이 움직인다.

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
- ☐ (캘리브/튜닝 중 대안) **`/omy_bridge/sync_to_leader`** — 리더를 편한 아무 자세에 두고 부르면 UR16e 가 **리더의
      매핑 자세로**(MoveIt, 충돌 검사) 온다. GELLO 의 "팔로워가 리더에게 간다" 방식. 데이터 수집은 시작 자세 통일을 위해 `/sync`.
- ☐ engaged 중 리더 샘플 간 속도가 `max_leader_speed`(기본 20 rad/s; 사람 ≤ 5, 엔코더 wrap 수백) 를 넘으면 브리지가
      스스로 끊고 status 가 `leader_jump` 로 남는다(케이블/엔코더 글리치 가드). 다시 `enable` 하면 engage 게이트가 판단한다.
      절대각 기준이 아닌 이유(0.16 s 통신 끊김에 오탐)는 `docs/gello_comparison.md` §3-②.
- ☐ 팔이 리더보다 느리면 브리지 로그의 **`slew capped N%`** 를 본다 — 나오면 `max_joint_speed` 가 병목(재기동으로 올림, UR16e 는
      2.0 rad/s 이하), 안 나오면 UR 안전 설정/하드웨어 한계. 표: `HARDWARE.md` 4-B ⑤.
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
      -p action_source:=topic -p action_topic:=/omy_bridge/command_joint_states
  ```
  카메라가 1대뿐이면 `-p cameras.exterior:=none`
  - ★ **action 토픽은 브리지의 `/omy_bridge/command_joint_states`**(UR 관절 이름, 매핑·clamp·slew 후 명령 + `finger_joint`
    목표)이지 `/leader/joint_states`(이름이 `joint1..6`, 매핑 전 값)가 **아니다**. 후자를 주면 기록기는 시작을 거부한다
    (`action topic ... has no UR arm joints`). 2026-09-17 이전 문서는 후자를 안내했다 — 결함, `HISTORY.md` §49.
  - 기록은 브리지가 **engaged** 상태여야 시작된다(그때만 명령 토픽이 나온다). 꼬리 정지 프레임을 버리려면
    `-p trim_tail_frames:=5`(GELLO 관례, 기본 0; 실행 중 `ros2 param set` 도 된다) — 켜기 전에
    `ros2 run ur_bringup il_tail_stats.py <raw_dir>` 로 꼬리 정지 프레임 중앙값을 먼저 잰다(≈0 이면 무의미).
- ☐ 변환: `deps/.venv-ml/bin/python src/ur_bringup/scripts/raw_to_lerobot.py --raw <경로> --repo-id <id> --root <출력>`
- ☐ 학습 (ACT): `deps/.venv-ml/bin/lerobot-train --policy.type=act --policy.push_to_hub=false ...`
- ☐ GR00T 준비: `src/setup/setup.sh groot`(lerobot[groot] + `deps/hf_cache`) → 모델 파일 배치(다운로드 또는 복사,
  [`SETUP.md`](SETUP.md) §2-C-2) → `src/setup/check_hf_cache.sh` 가 전부 `ok`
- ☐ 모든 ML 명령 앞에 `source src/setup/ml_env.sh`(HF 캐시 경로·오프라인·`OMP_NUM_THREADS=8`·shm 픽스)
- ☐ 롤아웃 시작 자세: 하네스의 `START_POSE` 는 `GRASP_MODE` 를 따른다 — physical(기본) → `ready`, attach → `ready_v1`
      (2026-09-16 이전 용접 시대 데이터셋, [`HISTORY.md`](HISTORY.md) §47.5). 보통 따로 정할 필요 없다 — `START_POSE=` 는
      그 규칙에서 벗어나는 데이터셋일 때만. 정책은 시연이 시작한 자세에서만 동작한다.
- ☐ 수집→변환→학습→롤아웃 **자동화 스크립트는 `src/ur_bringup/scripts/harness/`**(README 에 목록·인터프리터·로그 위치).
  로그는 `outputs/harness_logs/`(`HARNESS_LOG=` 로 변경), 워크스페이스 경로는 위치에서 자동 유추(`UR_WS=` 로 덮어쓰기)
- ☐ 학습 (GR00T N1.7): [`PIPELINE.md`](PIPELINE.md) §3-B 의 명령 **그대로** — `base_model_path` 는 로컬 경로,
  `fp32=false`, `--steps` 와 `--policy.max_steps` 를 같이. 예상 ≈1.8 h / 10k 스텝 (RTX 5090; `OMP_NUM_THREADS` 기본값이면 3.7 h, §45.4)
- ☐ 추론: [`launch/common/policy_inference.launch.py`](ur_bringup/launch/common/policy_inference.launch.py) **반드시** 먼저
  (스트리밍 컨트롤러 spawn) → `switch_control_mode.py streaming` → `list_controllers` 에서 `active` 확인
- ☐ GR00T 추론은 `--policy_device=cuda`, `--actions_per_chunk=40`, 카메라 2대(기본값) — [`PIPELINE.md`](PIPELINE.md) §4-3

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
| `controller_manager` 가 `no 'ros2_control' tag found in the URDF` 로 죽는다 / move_group 이 `Link 'tool0' … not known` | **같은 호스트의 다른 컨테이너**가 도메인 0 에서 `/robot_description` 을 발행(예: gz sim 이동로봇) | `ros2 node list` 에 모르는 노드가 있으면 이것(`shutdown_rollout.sh` 가 우리 프로세스 없이 `LEFT: nodes(domain N)` 만 찍을 때도). Isaac 부터 이 호스트에서 겹치지 않는 0 아닌 `ROS_DOMAIN_ID`(이 워크스페이스 42, 두 번째 컨테이너면 다른 값)로 띄운다(하네스는 환경값을 따름, [`HISTORY.md`](HISTORY.md) §47.4) |
| spawner 가 전부 `Failed to acquire lock in 20 seconds` ×5 로 죽고 `No controllers are currently loaded!` | 죽은 controller_manager 의 **고아 spawner** 가 `~/.ros/locks/` 파일 락을 쥠 | `pgrep -f controller_manager/spawner` 로 찾아 종료. `harness/bringup.sh` 는 자동 정리 |
