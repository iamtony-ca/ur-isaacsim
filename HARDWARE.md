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

## 4. 알아두면 좋은 함정 (실물 고유)

| 증상 | 원인 / 대처 |
|---|---|
| 컨트롤러 `Switch controller timed out` | (sim) `/clock` 없음 → real 은 `use_sim_time:=false`(기본). |
| 그리퍼가 안 열림/안 잡힘 | tool I/O 24V·RS-485 설정 누락, `/tmp/ttyUR` 권한(dialout), `gripper_startup_delay` 부족. |
| realsense 노드 `undefined symbol: diagnostic_updater::Updater` (SIGABRT) | realsense2_camera ↔ 구버전 `diagnostic_updater` ABI 불일치 → `ros-jazzy-diagnostic-updater`·`-msgs` 동반 업그레이드. |
| 카메라 토픽이 `/camera/camera/...` | realsense-ros 네임스페이스 중첩 → `camera_name:=''` 또는 remap (§3). |
| RViz↔실물 카메라 위치 어긋남 | hand-eye 미보정 → §3 캘리브 후 `cam_xyz`/`cam_rpy` 반영. |
| `No RealSense devices were found!` | 장치 미연결/USB2 포트 → SS(USB3) 포트, 케이블 교체, `rs-enumerate-devices`. |

> 더 많은 디버깅 이력/근거: [`HISTORY.md`](HISTORY.md) (특히 §6 함정, §8 그리퍼, §9 카메라).
