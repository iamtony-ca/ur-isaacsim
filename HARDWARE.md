# HARDWARE.md — 실물 HW 연결 후 할 일 (bring-up 런북)

이 문서는 **실물 하드웨어를 처음 연결할 때 해야 할 일**만 모았다. SW 는 이미 "연결하면 바로 동작"
하도록 준비돼 있고(아래 §0 체크리스트), 각 장치마다 *물리 연결 → 설정 → 실행 → 검증* 순서로 정리한다.

- 구성/아키텍처: [`README.md`](README.md) · 재현 매뉴얼: [`SETUP.md`](SETUP.md) · 배경/이력: [`HISTORY.md`](HISTORY.md)
- 매 터미널 먼저:
  `source /opt/ros/jazzy/setup.bash && source /isaac-sim/ur_ws/install/setup.bash && export ROS_DOMAIN_ID=0`

---

## 0. SW 준비 상태 (HW 없이 이미 완료) ✅

| 구성요소 | SW | 비고 |
|---|---|---|
| 팔 (UR16e) | `ur_robot_driver` (apt) | `use_sim:=false` 경로 mock 검증 완료 |
| 그리퍼 (2F-85) | `robotiq_driver`/`robotiq_controllers`/`serial` (vcs+빌드됨) | `use_fake_hardware:=true` mock 검증 완료 |
| 카메라 (D405) | `realsense2_camera` (apt), 노드 로드 확인 | 카메라 TF RSP 발행 확인 (장치만 없음) |

→ HW 가 없을 때 추가로 빌드/설치할 것은 없다. 아래는 **장치를 꽂은 뒤** 할 일.

빈손 점검(언제든): `ros2 launch ur_bringup ur16e_2f85_d405_real.launch.py use_mock_hardware:=true use_fake_hardware:=true use_tool_communication:=false enable_camera:=false`

---

## 1. UR16e 팔 (RTDE / External Control)

### 물리/네트워크
1. UR16e 컨트롤박스를 제어 PC 와 같은 서브넷 LAN 에 연결. `ping <UR16e_IP>` 확인.
2. PolyScope 에서 IP 확인(설정 → 네트워크).

### 펜던트 설정
3. **External Control URCap** 설치 + 프로그램에 추가(호스트 IP = 제어 PC, 포트 50002 기본).
   - 또는 펜던트 없이: 런치에 `headless_mode:=true` (PolyScope 프로그램 불필요).
4. (권장) `ur_calibration` 으로 로봇 고유 기구학 추출 → 정확한 TCP:
   ```bash
   ros2 launch ur_calibration calibration_correction.launch.py \
       robot_ip:=<UR16e_IP> target_filename:="$HOME/ur16e_calib.yaml"
   ```
   추출한 yaml 을 런치 `kinematics_params_file:=` 로 넘기면 반영.

### 실행 & 검증
```bash
ros2 launch ur_bringup ur16e.launch.py use_sim:=false robot_ip:=<UR16e_IP>
# (펜던트에서 External Control 프로그램 ▶ 실행, 또는 headless_mode:=true)
ros2 control list_controllers          # scaled_joint_trajectory_controller = active
ros2 topic echo /joint_states --once   # 6관절 position 유효(NaN 아님)
ros2 launch ur_bringup ur16e_moveit.launch.py use_sim:=false   # MoveIt+RViz, plan+execute
```
- ✅ 기준: 컨트롤러 active, `/joint_states` 유효, RViz 에서 plan→execute 시 실제 팔 이동.
- ⚠️ `use_sim_time` 은 real 에서 false(기본). sim `/clock` 없음.

---

## 2. Robotiq 2F-85 그리퍼 (손목 장착, RS-485)

2F-85 는 PC 가 아니라 **UR 손목 tool 커넥터**에 물려 24V + RS-485(Modbus RTU, 115200 8N1)를 받는다.
PC 는 `ur_robot_driver` 의 tool-communication 브리지(UR tool 시리얼 → TCP 54321 → 가상 시리얼 `/tmp/ttyUR`)
로만 닿는다.

### 물리/펜던트 설정
1. 그리퍼를 GRP-ES-CPL-077 커플링으로 손목에 체결(기계+전기 커넥터).
2. PolyScope **설치 → General → Tool I/O**:
   - Tool Output Voltage = **24 V**
   - Tool Communication Interface = **활성(RS-485)**, Baud 115200, Parity None, Stop 1, RX/TX idle 기본.
   - Controlled by = **User**(URCap 이 tool I/O 를 잡지 않도록).
3. 제어 PC 사용자가 `/tmp/ttyUR` 쓰기 가능해야 함: `sudo usermod -aG dialout $USER` (재로그인).

### 실행 & 검증 (손목 장착 표준)
```bash
ros2 launch ur_bringup ur16e_2f85_real.launch.py robot_ip:=<UR16e_IP>
#   내부: ur_control(use_tool_communication:=true, tool_voltage:=24) → /tmp/ttyUR 생성,
#         gripper_startup_delay(기본 8s) 뒤 robotiq_driver(com_port:=/tmp/ttyUR) 기동.
ros2 control list_controllers -c /gripper/controller_manager   # gripper_controller, robotiq_activation_controller = active
python3 src/ur_bringup/isaac/ur16e_2f85/gripper_demo.py \
    --action /gripper/gripper_controller/gripper_cmd --joint-states-topic /gripper/joint_states
```
- ✅ 기준: `GripperCommand` open/close(0↔0.6) `reached_goal=True`, 실제 그리퍼 개폐.
- e-stop 후 재활성화: `ros2 service call /gripper/robotiq_activation_controller/reactivate_gripper ...`
- **대안(벤치 직결, USB-RS485)**: 그리퍼를 PC USB-RS485 어댑터에 직접 연결한 경우
  `ros2 launch ur_bringup robotiq_2f85_real.launch.py com_port:=/dev/ttyUSB0` (팔은 별도 `ur16e.launch.py`).

---

## 3. RealSense D405 카메라 (eye-in-hand, USB3 직결)

그리퍼와 정반대로 카메라는 **UR 을 거치지 않고 제어 PC 에 USB3-C 직결**한다(영상 대역폭). 케이블은
팔을 따라 정리하고 손목 회전분 서비스 루프 확보.

### 물리/인식
1. D405 를 PickNik 브라켓 cradle 에 안착, USB3 포트(파란색/SS)에 연결.
2. 인식 확인:
   ```bash
   rs-enumerate-devices -s          # 시리얼/펌웨어 표시되면 OK
   rs-enumerate-devices -c          # 지원 stream profile 목록 (yaml 프로파일 조정용)
   ```
   - 안 보이면: USB2 포트/케이블 의심, `dmesg | tail`, udev 규칙(`librealsense2` 설치 시 포함).

### 실행 & 검증 (카메라 단독)
```bash
ros2 launch ur_bringup d405_real.launch.py
ros2 topic list | grep /camera        # color/image_raw, depth/image_rect_raw, depth/color/points, */camera_info
ros2 topic hz /camera/color/image_raw
ros2 topic echo /camera/color/camera_info --once   # K(fx,fy,cx,cy) 확인
```
- **★ 토픽명 확인(중요)**: 본 런치는 `/camera/<topic>` 을 노린다. realsense-ros 버전에 따라
  `/camera/camera/<topic>` 으로 **중첩**될 수 있다(`camera_name` 이 `camera_namespace` 아래로).
  중첩되면 sim 토픽명과 어긋나 인식 스택이 안 붙으므로, `launch/ur16e_2f85_d405/d405_real.launch.py`
  의 realsense 노드에 `camera_name:=''` 를 주거나 remap 으로 sim 토픽명(`/camera/color/image_raw` 등)에
  맞춘다. → 맞춘 뒤 이 문서/README 의 토픽명 주의를 "확정"으로 갱신.
- ✅ 기준: 위 5개 토픽이 sim 과 동일 이름/인코딩(rgb8, 32FC1, XYZRGB)으로 발행, `camera_*_optical_frame`.

### hand-eye 캘리브레이션 (tool0 → camera_link)
sim 은 USD 카메라가 ground-truth 라 캘리브 불필요. 실물은 브라켓 명목 마운트(기본값)를 **hand-eye 로 보정**한다.
1. 캘리브 도구 설치(택1): `easy_handeye2` 또는 MoveIt Hand-Eye Calibration(rviz 플러그인) + ArUco/charuco 보드.
2. eye-in-hand 모드로 팔을 여러 자세로 움직이며 보드를 관측 → `tool0 → camera_link` 변환 추출.
3. 결과를 런치 인자로 반영(영구화하려면 `urdf/ur16e_2f85_d405/d405_real.urdf.xacro` 의
   `cam_xyz`/`cam_rpy` 기본값을 교체):
   ```bash
   ros2 launch ur_bringup d405_real.launch.py cam_xyz:="x y z" cam_rpy:="r p y"
   ```
   - 기본값(미보정) = sim 명목: `cam_xyz:="0 -0.067 0.01847"`, `cam_rpy:="0 -1.4311700 1.5707963"`
     (= 0, -π/2+8°, π/2). 검증: `ros2 run tf2_ros tf2_echo tool0 camera_link`.

### 전체 결합 + OctoMap
```bash
ros2 launch ur_bringup ur16e_2f85_d405_real.launch.py robot_ip:=<UR16e_IP> cam_xyz:="..." cam_rpy:="..."
ros2 launch ur_bringup ur16e_2f85_d405_moveit.launch.py use_sim:=false       # depth→OctoMap (sim 과 동일 토픽)
# planning scene 에 OctoMap(component 32) 채워지는지:
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 32}}"
```
- ✅ 기준: `/camera/depth/color/points` → OctoMap 적분, 카메라가 보는 장애물을 MoveIt 이 회피.

---

## 4. cuMotion (GPU 모션플래닝, MoveIt 플러그인)

NVIDIA Isaac ROS cuMotion 을 MoveIt **planning pipeline** 으로 붙여 GPU 로 플래닝하고,
실행은 기존 `scaled_joint_trajectory_controller`(sim/real 공용)로 한다. **sim 에서 plan+execute 검증 완료**
(오차 0.0003 rad). HW 와 무관한 SW 설정이라 실물 팔에도 그대로 적용된다.

### 설치 (apt) — 전제 레포 3개 + cuMotion 패키지
```bash
# (a) Isaac ROS 4.x (Jazzy/Noble) 레포
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg
echo 'deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4 noble main' | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list
# (b) CUDA 13 레포 (cuda-toolkit-13-0 — isaac_ros_common 하드 의존, 수 GB)
cd /tmp && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
# (c) VPI 4 레포 (libnvvpi4 — NVIDIA Jetson OTA x86_64, r38.2 고정)
curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-jetson.gpg
echo 'deb [signed-by=/usr/share/keyrings/nvidia-jetson.gpg] https://repo.download.nvidia.com/jetson/x86_64/noble r38.2 main' | sudo tee /etc/apt/sources.list.d/nvidia-vpi.list
sudo apt update
sudo apt install -y ros-jazzy-isaac-ros-cumotion ros-jazzy-isaac-ros-cumotion-moveit \
                    ros-jazzy-isaac-ros-cumotion-examples ros-jazzy-isaac-ros-cumotion-robot-description
```
cuMotion **엔진은 deb 에 번들**(`libcumotion_impl.so`)이라 런타임 추가설치 불필요. CUDA 드라이버(580, CUDA13 호환)는 이미 있음.

### UR16e 로봇 설정 (XRDF) — 일회성
cuMotion 은 URDF + XRDF(충돌 sphere) 필요. UR16e 용은 standalone cuMotion 엔진 휠로 sphere 를 생성해
`ur_bringup/cumotion/ur16e_2f85.{urdf,xrdf}` 로 vendored 되어 있다. 재생성/배경은 [`../ur_bringup/cumotion/README.md`](../ur_bringup/cumotion/README.md).

### 실행
```bash
ros2 launch ur_bringup ur16e_2f85_d405.launch.py                         # (또는 ur16e_2f85 / *_real) 제어
ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py         # move_group + cuMotion(기본 pipeline) + RViz
#   real 은 use_sim_time:=false
python3 src/ur_bringup/isaac/common/moveit_plan_execute_demo.py          # 프로그램 plan+execute (cuMotion)
# RViz: Goal State <random valid>/마커 → Plan → Execute (planner=isaac_ros_cumotion 기본)
```
- **`ur_only`(기본 true)**: RViz 인터랙티브 plan 을 위해 move_group/RViz 를 **UR 팔 6관절 모델**로 띄움
  (cuMotion 플러그인이 start_state 를 cspace 로 필터 안 해서, RViz 가 보내는 full 12관절 start 가 `[12]≠[6]` 로 거부됨).
  cuMotion 은 **그리퍼 충돌(XRDF sphere)까지 포함해 플래닝**함 — `ur_only` 는 그 RViz 의 그리퍼 *표시*만 끔.
  `ur_only:=false` 면 풀 그리퍼 모델이지만 RViz 인터랙티브는 안 되고 프로그램 경로(빈 start_state)만 됨.
- **pose 초기화/복구**: 반복 plan 으로 손목이 ±2π 벗어나면 MoveIt 이 "start state out of bounds" 로 모든 plan 거부
  → `python3 src/ur_bringup/isaac/common/reset_pose.py home`(컨트롤러 직접, 범위 밖에서도 복구; `up`/`zero` 가능).
  RViz 의 named state(home/up)는 범위 안일 때 동작.
### world(장애물) 실시간 회피 — nvblox + robot_segmenter (sim 검증됨)

cuMotion `read_esdf_world:=true` 면 **nvblox** 가 만든 3D ESDF(`/nvblox_node/get_esdf_and_gradient`)를
매 plan 마다 읽어 카메라가 보는 장애물을 GPU 로 회피한다. 기본은 off(자기충돌+planning scene 만).

**구성(파이프라인)**: `정적 카메라 depth → robot_segmenter(로봇 마스킹) → nvblox(3D ESDF, base_link) → cuMotion`
- 런치: `ur16e_2f85_d405_nvblox.launch.py`(segmenter + nvblox + 정적카메라 TF 한 번에), 설정: `config/ur16e_2f85_d405/nvblox_cumotion.yaml`.

```bash
# sim: Isaac 에 카메라 2대(파지용 eye-in-hand + 회피용 정적) + 데모 박스 장애물
/isaac-sim/python.sh src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path .../isaac/assets/ur16e_2f85_d405.usd --with-camera --with-static-cam --obstacle
ros2 launch ur_bringup ur16e_2f85_d405.launch.py use_sim:=true          # 제어
ros2 launch ur_bringup ur16e_2f85_d405_nvblox.launch.py use_sim_time:=true   # segmenter+nvblox+TF
ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py \
    use_sim_time:=true read_esdf_world:=true ur_only:=true              # cuMotion이 ESDF 읽음 + RViz 복셀
```
검증: `Updated ESDF grid successfully` + plan `success: true`(18pt). nvblox 가 `Successfully wrote requested ESDF`.
장애물 매핑 확인: home 시작자세는 충돌 없음(plan 성공)이고, 장애물 안으로 가는 goal 은
`target c-space position ... is invalid` 로 거부됨(=장애물이 월드 모델에 반영). RViz 에서 goal 마커를
장애물 너머로 → Plan → 우회 궤적 → Execute 로 Isaac 에서 직접 회피 관찰.

**데모 옵션**: `--obstacle`(박스, `--obstacle-pose`/`--obstacle-size` 로 조정). Isaac 은 시작 시 **home 자세**로
초기화(USD 기본 전관절0=팔 수평 대신 팔 위로) — 안 하면 수평 팔이 낮은 장애물에 닿아 보임. 데모 박스는 수평·home
양쪽 팔과 안 닿도록 바닥 z≈0.35 로 띄움.

**★ 핵심 교훈 — eye-in-hand 가 아니라 정적(외부) 카메라로 매핑한다.** 손목장착 D405 는 시야의 대부분이
로봇 자신이고 팔과 함께 움직여 TSDF 가 로봇으로 오염됨 → 시작자세가 "world collision" 으로 거부된다
(`Invalid c-space position: world collision detected`). NVIDIA 매니퓰레이션 레퍼런스처럼 **워크스페이스를
내려다보는 정적 카메라**로 매핑하면 깔끔하다. eye-in-hand D405 는 파지/비전 전용으로 둔다.
nvblox 로 매핑할 카메라는 `depth_image:=`/`depth_info:=` 로 교체 가능(기본=정적카메라 `/static_cam/depth/*`).

**함정**: ① nvblox `global_frame` 은 cuMotion 의 로봇 base 프레임(XRDF `set_base_frame`=`base_link`)과 **반드시 일치**
해야 함 — 다르면 `Requested ... in base_link frame but nvblox is mapping in world frame. Sending empty grid`
→ cuMotion `World update failed`. (yaml 에 `global_frame: base_link` 반영) ② segmenter 가 로봇을 못 빼면 잔상이
ESDF 에 남아 시작자세 충돌 → `additional_buffer_distance` 키움(0.12). ③ 정적카메라 TF(`base_link→static_cam_depth_optical_frame`)
는 런치가 발행하며 **Isaac 카메라 포즈(`--static-cam-xyz/-target`)와 좌표가 일치**해야 함(런치 `SCAM_TF` 와 동기).
④ RViz 복셀 표시는 **`tsdf_layer`**(~15Hz) 사용 — `static_esdf_pointcloud`/`static_occupancy_grid` 는 미발행이라 안 보임.
RViz 설정은 `read_esdf_world:=true` 면 `config/ur16e_2f85_d405/cumotion_nvblox.rviz` 로 자동 전환됨.
⑤ 노드가 많아 **Fast DDS 공유메모리(SHM) 포트 포화**(`fastrtps_port ... open_and_lock_file failed`) 가능 — plan/execute 는
정상이나 신규 CLI/RViz 구독이 불안정하면 전체를 **UDP 전송**(`export FASTDDS_BUILTIN_TRANSPORTS=UDPv4`)으로 재기동.
- **실물**: 정적 depth 카메라(D455 등 워크스페이스 오버룩)를 추가하고 `depth_image:=/<그 카메라>/depth/...`,
  TF 는 그 카메라 실제 장착위치로(`static_cam_tf:=false` 후 별도 발행 또는 `SCAM_TF` 수정).

### ★ 설치 시 겪는 함정 (이번에 확정)
1. **`cuda-toolkit-13-0` / `libnvvpi4` "not installable"** → (b)CUDA·(c)VPI 레포 누락. 위 3개 레포 다 추가하면 `gxf-isaac-*`/`nitros` 까지 연쇄 해결.
2. **부분 업그레이드 ABI 깨짐**: cuMotion/realsense 가 `diagnostic_updater` **4.2.7** 을 끌어오면, 구버전(4.44) `controller_manager` 가 `undefined symbol: diagnostic_updater::Updater` 로 죽어 **모든 ros2_control 이 마비**된다. → ros2_control 스택을 **같이** 4.45.2 로 올린다(실제 패키지 직접 지정, 메타패키지로는 안 올라감):
   `sudo apt install -y ros-jazzy-controller-manager ros-jazzy-controller-interface ros-jazzy-hardware-interface ros-jazzy-controller-manager-msgs ros-jazzy-joint-trajectory-controller ros-jazzy-joint-state-broadcaster ros-jazzy-position-controllers`
3. **컨트롤러가 cuMotion 궤적 goal 거부**: `Velocity of last trajectory point ... is not zero`. cuMotion 종단 잔여속도(~1e-3) 때문. → JTC 에 `allow_nonzero_velocity_at_trajectory_end: true`(우리 컨트롤러 yaml 에 반영됨).

## 5. 알아두면 좋은 함정 (실물 고유)

| 증상 | 원인 / 대처 |
|---|---|
| 컨트롤러 `Switch controller timed out` | (sim) `/clock` 없음 → real 은 `use_sim_time:=false`(기본). |
| 그리퍼가 안 열림/안 잡힘 | tool I/O 24V·RS-485 설정 누락, `/tmp/ttyUR` 권한(dialout), `gripper_startup_delay` 부족. |
| realsense/cuMotion `undefined symbol: diagnostic_updater::Updater` | `diagnostic_updater` 4.2.7 ↔ 구버전 ros2_control/realsense ABI 불일치 → §4-②(ros2_control 스택 동반 업그레이드). |
| 카메라 토픽이 `/camera/camera/...` | realsense-ros 네임스페이스 중첩 → `camera_name:=''` 또는 remap (§3). |
| RViz↔실물 카메라 위치 어긋남 | hand-eye 미보정 → §3 캘리브 후 `cam_xyz`/`cam_rpy` 반영. |
| `No RealSense devices were found!` | 장치 미연결/USB2 포트 → SS(USB3) 포트, 케이블 교체, `rs-enumerate-devices`. |
| cuMotion 궤적 실행 거부(goal rejected) | 종단 잔여속도 → `allow_nonzero_velocity_at_trajectory_end: true` (§4-③). |

> 더 많은 디버깅 이력/근거: [`HISTORY.md`](HISTORY.md) (특히 §6 함정, §8 그리퍼, §9 카메라).
