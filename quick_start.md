# Quick Start — UR16e Dual-tool EOAT: CAD 요청부터 충돌 체크까지

기구팀에 CAD 를 요청하는 것부터 → STEP 인수 점검 → USD 변환/조립 → MoveIt 브리지 생성 →
**Isaac Sim + ros2_control + MoveIt2 + RViz2** 를 모두 띄워 **충돌 체크**(빈 EOAT self·장애물 + **파지한 휠**)까지,
한 번에 따라 할 수 있는 복붙용 튜토리얼입니다. 명령어는 그대로 붙여넣으면 동작하며, 각 단계마다 **어떤 출력이 나와야 정상인지**
예시로 적어두었습니다.

> **개념/함정 상세**는 `STEP_TO_SIM.md`(7단계 A~G), `ur_bringup/isaac/common/eoat/README.md`(config 스키마),
> `HISTORY.md`(누적 이력)를 참고하세요. 이 문서는 "일단 돌려보는" 최단 경로입니다.
>
> **전제**: Isaac Sim 6.0.1 컨테이너, ROS 2 Jazzy, GPU. 워크스페이스 = `/isaac-sim/volume/ur_dualtool_ws`.
> 경로 표기는 두 종류입니다 — **CAD 파이프라인**은 `cd src/` 기준 상대경로(Isaac python, ROS 불필요),
> **ROS 실행**은 워크스페이스 루트 기준(ROS 소싱 필요).

---

## 0. 공통 준비 (터미널마다 1회)

```bash
# ── CAD 파이프라인용 (ROS 불필요, Isaac bundled python 만 사용) ──
cd /isaac-sim/volume/ur_dualtool_ws/src
export P=/isaac-sim/python.sh                       # Isaac python 런처
export CFG=isaac/common/eoat/eoat_dualtool.yaml     # 전체 dual-tool config (또는 eoat_gripper_branch.yaml)

# ── ROS 실행용 (제어/MoveIt/RViz/데모 터미널마다) ──
source /opt/ros/jazzy/setup.bash
source /isaac-sim/volume/ur_dualtool_ws/install/setup.bash
export ROS_DOMAIN_ID=0
```

> `$P`(Isaac python) 는 **공유 설치**입니다. 패키지 추가 설치는 신중히 (`pip install ... --no-deps` 권장).
> CAD 파이프라인 스크립트는 ROS 를 소싱하면 오히려 충돌할 수 있으니 **별도 터미널**에서 도세요.

---

## 1. 기구팀에 CAD 요청하기

인수 요청서를 그대로 보냅니다. 핵심만 요약하면:

- **포맷**: STEP (`.step`), **AP242 우선**(안 되면 AP214), **mm**, **Z-up**, **솔리드(B-rep)** — STL/네이티브(x_t·sat·par) 금지.
- **분할(4그룹)**: ① 댐퍼+Y툴체인저(조립), ② F/T 센서, ③ 그리퍼는 **base/left finger/right finger 분해**, ④ copick+스크류드라이버+어댑터(조립).
- **원점 규약**(가능하면): 원점 = 장착면, **+Z = 스택(부착) 방향**, x/y = 장착축 중심. → 안 되면 최소한 **축 방향·클로킹만** 맞춰달라고 요청.
- **물성표**(별도, 나중에 와도 됨): 질량(kg)·COM(x,y,z)·관성텐서(Ixx…Iyz) + 기준 프레임/단위.

```bash
# 요청서 문서 위치 (이 파일을 그대로 전달)
less isaac/common/eoat/CAD_DELIVERY_REQUEST.md
```

> **왜 STEP·mm·Z-up 인가**: STEP 은 정확 B-rep(테셀레이션은 우리가 변환 시 결정), STL 은 단위 없음·고정밀도 불가.
> mm·Z-up 은 변환 스크립트 기본값과 일치해 스케일 오류(1000배 괴물)를 예방합니다.

---

## 2. STEP 인수 점검 (변환 전, pure python)

받은 STEP 이 요청대로인지 **변환 전에** 원본 텍스트로 점검합니다. Isaac 불필요.

```bash
# 폴더째 점검 (+ 기대 파일 목록으로 누락/추가 확인 가능)
python3 isaac/common/check_step_delivery.py isaac/assets/cad
python3 isaac/common/check_step_delivery.py isaac/assets/cad \
    --expect Dual_Quick_Changer_v3.STEP,EH2-H0025-SC.STEP    # manifest 검사(선택)
```

**정상/문제 출력 예시** — PASS 는 통과, WARN/ERROR 는 조치 필요:

```
=== Dual_Quick_Changer_v3.STEP ===
    schema=AP214  units=mm  solids=12  size=3.4MB
    [PASS] AP214 — acceptable (AP242 preferred if the CAD tool offers it)
    [PASS] units = mm
    [INFO] 12 solid bodies (assembly/multi-body) — fine; merging toward fewer lowers load cost

=== CoPick3D 150S.STEP ===
    schema=AP203  units=mm  solids=153  size=76.6MB
    [WARN] AP203 is old — request AP242/AP214 export          ← 재추출 요청
    [PASS] units = mm
    [WARN] 153 solid bodies > 50 — heavily fragmented; ask to merge (no accuracy loss)  ← 병합 요청

==== checked N STEP file(s): 0 error, 2 with warning(s) ====
```

**확인 포인트**: `units=mm` 여야 하고(아니면 스케일 오류), `schema` 는 AP242/AP214 면 통과,
`solids` 는 그리퍼처럼 분해가 필요한 부품 외에는 적을수록 좋음. **ERROR(솔리드 0=서피스만)** 가 뜨면
그 파일은 못 씁니다 → 재요청. WARN 은 이상적이진 않지만 진행은 가능합니다.

> **자동으로 못 잡는 것**(GUI/데이터시트 필요): 축의 실제 +Z 방향, 클로킹, 핑거 개폐 상태, defeaturing 여부.

---

## 3. STEP → USD 변환

```bash
# 폴더째 변환 (assets/cad/*.usd 생성). 미터 네이티브 + Z-up 고정, 변환 후 bbox(mm) 출력.
$P isaac/common/convert_step_to_usd.py isaac/assets/cad isaac/assets/cad
```

**정상 출력 예시** — 각 부품의 **bbox(mm)** 가 나옵니다. 실측/도면과 대조하세요:

```
[convert] Dual_Quick_Changer_v3.STEP -> dual_quick_changer.usd
          bbox = 78.0 x 78.0 x 34.5 mm      ← 실측과 비슷해야 정상
[convert] EH2-H0025-SC.STEP -> eh2_h0025_sc.usd
          bbox = 63.0 x 63.0 x 25.0 mm
[done] converted 6 file(s)
```

**확인 포인트**: bbox 가 실측 대비 **1000배**면 단위 오류(mm↔m), **25.4배**면 inch 혼입입니다.
정상이면 다음 단계로.

> ★ STEP/IGES/JT 는 HOOPS 백엔드로만 들어옵니다. **`omni.kit.converter.cad` 확장은 켜지 말 것**(ODA 심볼 충돌로 abort).
> 스크립트에 이미 반영돼 있으니 그대로 쓰면 됩니다.

---

## 4. 변환 후 CAD 점검 (USD)

변환된 USD 를 기하 측면에서 점검합니다(스케일·바디수·watertight·밀도·원점).

```bash
# 폴더째 요약
$P isaac/common/validate_cad.py isaac/assets/cad
# config 의 expected_size_mm 와 자동 대조(스케일 검증 강화)
$P isaac/common/validate_cad.py isaac/assets/cad --config $CFG
```

**정상/문제 출력 예시**:

```
=== dual_quick_changer.usd ===
    [PASS] scale: bbox 78.0x78.0x34.5 mm ~ expected 78x78x35 (Δ<5%)
    [PASS] geometry: 1 mesh body
    [PASS] watertight: 0 boundary edges
    [PASS] density: 1.2k tris (< 200k cap)
    [PASS] origin: bbox min-Z = 0.3 mm (passthrough-ready)

=== copick3d_150s.usd ===
    [WARN] density: 812k tris > 200k — heavy; consider merge/decimate
    [WARN] geometry: 235 mesh bodies (assembly — expected for a merged part?)
```

**확인 포인트**: `scale` PASS(치수 일치), `origin` 이 min-Z≈0 이면 `passthrough:true` 로 무튜닝 조립 가능.
WARN(과밀·다바디)은 진행은 되지만 병합/decimate 를 고려. `expected_size_mm` 를 config 에 적어두면 스케일을 자동 판정합니다.

---

## 5. config 작성/튜닝 (`eoat_*.yaml`)

**코드 수정 없이 이 YAML 숫자만** 바꿔서 조립을 만듭니다. 핵심 필드:

```yaml
meta:
  name: ur16e_dualtool
  root_link: tool0                       # 체인의 뿌리 = UR TCP(tool0)
  cad_dir: ../../assets/cad
  out_assembly: ../../assets/ur16e_dualtool.usd
  out_urdf:     ../../assets/ur16e_dualtool.urdf
  defaults:
    density: 2000                        # physics 없는 부품은 이 밀도로 bbox-box 근사(mass/inertia)
    collision: convexHull                # convexHull(기본) | convexDecomposition | mesh | boundingCube | none

parts:                                   # 부품별 프레임 정규화 (장착면->원점, 장착축->+Z)
  dual_quick_changer:
    cad: dual_quick_changer.usd          # assets/cad 안의 소스 USD
    normalize: {rpy_deg: [0, 0, 0], seat: zmin, center_xy: true}  # 축을 +Z 로 돌리고 zmin 면을 원점에
    # normalize: {passthrough: true}     # (대안) CAD 원점을 그대로 신뢰 (규약대로 왔을 때)
    physics: {collision: convexDecomposition}   # (선택) 부품별 override
  2fg14:                                 # ★ 유일한 "1 STEP ≠ 1 링크" — 몸체 + 핑거 2개(prismatic+mimic)
    cad: 2fg14.usd
    normalize: {rpy_deg: [-90, 0, 0], seat: zmin, center_xy: true}
    sublinks:                            # 커스텀 핑거 CAD (휠 Ø125 감싸 잡는 L-브래킷)
      finger_left:  {cad: finger_left,  physics: {collision: convexDecomposition, friction: 1.0}}
      finger_right: {cad: finger_right, physics: {collision: convexDecomposition, friction: 1.0}}
    joints:                              # 평행 그리퍼 = 1축 구동 + mimic (열림축=Y, q=0 open→q↑ close)
      - {name: finger_left_joint,  child: finger_left,  type: prismatic,
         origin_xyz: [0,  0.03105, 0.1144], axis: [0, -1, 0], limit: {lower: 0, upper: 0.025, effort: 140, velocity: 0.45}}
      - {name: finger_right_joint, child: finger_right, type: prismatic,
         origin_xyz: [0, -0.03105, 0.1144], axis: [0,  1, 0], limit: {lower: 0, upper: 0.025, effort: 140, velocity: 0.45},
         mimic: finger_left_joint}       # 오른쪽은 왼쪽을 대칭으로 따라감

chain:                                   # 위→아래 결합 순서 (아래는 tcp_pose 방식, 중간 부품은 …로 축약)
  - {id: damper,             part: damper,             parent: tool0,   joint: fixed,
     tcp_pose: {xyz: [0, 0, 0], rpy: [0, 0, 0]}}
  - {id: dual_quick_changer, part: dual_quick_changer, parent: damper,  joint: fixed,   # 여기서 Y분기(두 가지가 이 parent)
     tcp_pose: {xyz: [0, -0.035, 0.071], rpy: [-90, 0, 0]}}
  # 포트 A: … hex_qc → gripper_2fg14
  - {id: gripper_2fg14,      part: 2fg14,              parent: hex_qc,  joint: fixed,
     tcp_pose: {xyz: [-0.091, -0.002, 0.112], rpy: [-59.3, 5.6, 94.8]}}
  # 포트 B: … qc_tool_side_B → adapter → screwdriver (+ camera_adapter → copick)
  - {id: screwdriver,        part: screwdriver_estic,  parent: adapter, joint: fixed,
     tcp_pose: {xyz: [0.088, -0.003, 0.112], rpy: [-58.7, 20.4, -81.5]}}
```

**파라미터 요령**:
- **원점 정규화** — 규약대로 왔으면 `passthrough:true`(무튜닝). 제각각이면 `rpy_deg`+`seat`+`center_xy` 로 재원점(GUI 확인 필요). `expected_size_mm` 를 적어두면 `validate_cad` 가 스케일 자동 대조.
- **위치 지정 3방법**(우선순위 `tcp_pose > mount > 자동스택`): ① (기본) parent 윗면에 +Z 면-대-면 스택(`gap`) ② `mount:{xyz,rpy}` = parent 프레임 기준 명시 자세(직교 포트·Y분기) ③ **`tcp_pose:{xyz,rpy}` = root_link(tool0/TCP) 기준 절대 상대포즈** — **현재 dual-tool config 가 쓰는 방식**. 기구팀 TCP 데이터를 그대로 넣고, GUI 튜닝 후 `extract_poses.py --tcp` 로 재추출해 붙임.
- **2FG14 = 유일한 articulated 부품**: `sublinks`(핑거 CAD) + `joints`(prismatic + `mimic`). 평행 그리퍼는 **1축 구동 + mimic** (왼쪽 구동, 오른쪽 대칭). 열림축=Y, `q=0`=open→`q=0.025`=close.
- **`physics.collision`** 단일 knob 이 USD 근사와 URDF 충돌메시를 **함께** 결정: `convexHull`(=`convex`, 가벼움·기본), `convexDecomposition`(CoACD 조각, 오목 추종), `mesh`(실메시 decimate, 파지/삽입 정밀).
- **숫자는 전부 rough** — 치수는 headless 로 검증되지만 **방향/자세 최종 확인은 Isaac GUI 필수**.

---

## 6. 정규화 → 조립 USD → UR16e 결합

```bash
$P isaac/common/eoat/normalize_part.py $CFG        # 장착면→원점, 축→+Z (assets/cad/normalized/*.usd)
$P isaac/common/eoat/build_eoat_usd.py $CFG        # EOAT 단일 조립 USD (assets/ur16e_dualtool.usd)
$P isaac/common/eoat/build_ur16e_dualtool.py       # + UR16e 결합본 (assets/ur16e_dualtool_full.usd, HOME baked)
```

**정상 출력 예시**:

```
# normalize_part.py — 각 부품 장착면/두께 확인
[normalize] dual_quick_changer: base_z=0.000  thickness_z=0.0345  size=(0.078,0.078,0.0345)
[normalize] gripper_2fg14:      base_z=0.000  thickness_z=0.1420  ...

# build_eoat_usd.py — 스택 높이/링크 위치
[eoat] damper            +Z @ 0.000
[eoat] dual_quick_changer +Z @ 0.020
[eoat] gripper_2fg14      +Z @ 0.055
[eoat] total stack height = 0.197 m, 12 bodies

# build_ur16e_dualtool.py
[full] single articulation /UR16e/root_joint, 8 DOF (6 arm + 2 finger), 19 bodies -> ur16e_dualtool_full.usd
```

**확인 포인트**: `base_z=0.000`(장착면이 원점에 옴), 스택 높이/방향이 상식적이면 통과. 방향이 틀리면
(예: 그리퍼가 팔쪽을 봄) config 의 `rpy_deg` 를 90° 단위로 고치고 재실행.

> (선택) 물리 재생 검증: `$P isaac/common/eoat/verify_articulation.py isaac/assets/ur16e_dualtool.usd` — 단일 articulation 파싱 + DOF 유한(NaN 없음) + 2FG14 핑거가 평행 mimic 으로 **양 턱 대칭 이동**하면 정상.

---

## 7. MoveIt 브리지 생성 (URDF + 실메시 + SRDF)

Isaac USD 와 **같은 단일 소스**에서 MoveIt2 충돌모델(URDF/SRDF)과 RViz 표시용 실메시를 뽑습니다.

```bash
$P isaac/common/eoat/build_eoat_urdf.py $CFG       # URDF (충돌모델) — assets/ur16e_dualtool.urdf
$P isaac/common/eoat/export_eoat_meshes.py $CFG    # ★ 부품 실메시 -> meshes/eoat/ (RViz==Isaac, box 대신)
$P isaac/common/eoat/build_eoat_moveit.py $CFG     # xacro 매크로 + SRDF (mesh 있으면 <mesh> 참조)
```

**정상 출력 예시** (export_eoat_meshes.py — 시각/충돌 메시 쌍):

```
[export-mesh] gripper_2fg14: 2fg14.usd -> gripper_2fg14.obj (9832f, from 41k) + gripper_2fg14_col.obj [convexDecomposition] (312f)
[export-mesh] dual_quick_changer: ... -> dual_quick_changer.obj + dual_quick_changer_col.obj [convexHull] (...)
[export-mesh] exported 8 part mesh(es) -> .../meshes/eoat
```

**확인 포인트**: `_col.obj` 뒤 대괄호(`[convexHull]`/`[convexDecomposition]`/`[meshSimplification]`)가
config 의 `physics.collision` 과 일치해야 합니다(USD↔URDF 일관성). 실패 시 box 폴백으로 떨어지니 로그 확인.

> ★ **함정 #7**: URDF/USD 충돌모델이 어긋나면 RViz↔Isaac 자세가 달라집니다. 이 파이프라인은 단일 소스라 자동 일치.
> `convexDecomposition` 은 CoACD 가 필요합니다(현 Isaac python 엔 이미 설치됨; 다른 PC 재현 시 `pip install coacd`, 없으면 convex hull 로 자동 폴백).

---

## 8. (선택) 정적 장애물 준비 — 테이블·지그

충돌 체크 대상에 환경 장애물을 넣고 싶으면:

```bash
# CAD -> USD + 충돌 OBJ 3종 생성
$P isaac/common/obstacles/prepare_obstacles.py isaac/common/obstacles/obstacles.yaml
```
장애물을 MoveIt planning scene 에 올리는 것은 **11단계(스택 기동 후)** 에서 합니다.

---

## 9. 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd /isaac-sim/volume/ur_dualtool_ws
vcs import src < src/ur16e.repos          # 최초 1회 (topic_based @0.2.1 등)
colcon build --symlink-install --packages-up-to ur_bringup --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

**확인 포인트**: `Summary: N packages finished` 에 실패 0. `export_eoat_meshes` 산출물을 install 에 반영하려면
`ur_bringup` 을 다시 빌드해야 RViz 가 실메시를 봅니다.

> ★ `-DBUILD_TESTING=OFF` 필수(0.2.1 의 ros_testing 의존 회피). `topic_based` 는 **태그 0.2.1** 고정(그 외 버전은 /joint_states 전부 NaN → RViz SIGSEGV).

---

## 10. 실행 — 3 터미널 (Isaac → 제어 → MoveIt+RViz)

**반드시 이 순서**로. 각 터미널은 0번의 ROS 소싱을 먼저 하세요.

### 터미널 A — Isaac Sim (dual-tool 씬)

```bash
/isaac-sim/python.sh /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
    --asset-path /isaac-sim/volume/ur_dualtool_ws/src/ur_bringup/isaac/assets/ur16e_dualtool_full.usd
```
**확인**: GUI 뷰포트에 UR16e+EOAT 가 조명과 함께 보임. 터미널에 `Publishing /isaac_joint_states`,
`/clock` 발행 로그. → **안정될 때까지 기다렸다가** 다음 터미널.

> ★ `--asset-path` 는 **절대경로** 필수(상대경로는 에셋서버 기준으로 붙어 로봇이 안 뜸).
> 뷰포트가 까맣게만 보이면 환경(조명) 미로드 — `--no-env` 를 빼세요.
> **초기(HOME) 자세 변경**: `isaac/common/home_pose.py` 의 `HOME_DEG` 6줄만 수정(단일소스 — Isaac·RViz·reset_pose 모두 반영). 값 바꾸면 `build_ur16e_dualtool.py` 재실행해 USD 재베이크. RViz 는 `/joint_states` 를 미러하므로 별도 설정 없음.

### 터미널 B — ros2_control (topic_based ↔ Isaac)

```bash
ros2 launch ur_bringup ur16e_dualtool.launch.py
```
**확인**: 컨트롤러 스폰 성공 로그(`Configured and activated scaled_joint_trajectory_controller`,
`gripper_controller`). 별도 터미널에서 검증:
```bash
ros2 topic echo /joint_states --once     # 6 관절 + 핑거, position 이 NaN 이 아니어야 정상
```

> ★ `/joint_states` 가 **유효한 숫자**인지 반드시 확인 후 MoveIt 기동. NaN 이면 Isaac/순서 문제.
> Isaac 을 재시작했으면 아래 MoveIt/RViz 도 재시작(과도기 NaN 캐싱 → plan error_code -4).

### 터미널 C — MoveIt2 + RViz2

```bash
ros2 launch ur_bringup ur16e_dualtool_moveit.launch.py launch_rviz:=true
```
**확인**: RViz 에 로봇+EOAT 실메시 표시, MotionPlanning 패널 로드. `move_group` 로그에
`You can start planning now!`. RViz 에서 목표를 끌어 **Plan & Execute** 하면 Isaac 팔이 따라 움직이면 성공.

> 기본 `launch_rviz:=false` 이므로 RViz 를 원하면 `:=true` 를 붙이세요.
> `use_sim_time` 은 sim 이라 true 기본(Isaac `/clock` 사용). Isaac 없이 제어스택만 테스트하려면 B/C 에서 `use_sim_time:=false`.

---

## 11. 충돌 체크

MoveIt 의 `/check_state_validity`(self-collision + planning scene)를 쓰므로 **sim/real 동일**하게 동작합니다.
좌표는 planning frame = `world` = `base_link`(항등)이라 접촉점 xyz 가 **UR base 기준**으로 나옵니다.

새 터미널(0번 ROS 소싱)에서:

### (a) 장애물을 planning scene 에 올리기 (8단계에서 준비했다면)

```bash
/usr/bin/python3 isaac/common/obstacles/load_obstacles_moveit.py    # MoveIt scene + ACM (sim+real 공통)
```
**확인**: `Added N collision object(s)` 로그. RViz Scene 에 장애물 표시.

### (b) 현재/특정 자세의 충돌 지점 보고

```bash
cd /isaac-sim/volume/ur_dualtool_ws/src
/usr/bin/python3 ur_bringup/isaac/common/obstacles/collision_report.py                       # 현재 /joint_states
/usr/bin/python3 ur_bringup/isaac/common/obstacles/collision_report.py 0 -0.2 1.9 -1.7 -1.57 0   # 특정 6관절(rad)
/usr/bin/python3 ur_bringup/isaac/common/obstacles/collision_report.py --watch                # 연속(0.5s)
```

**충돌 없음 / 충돌 발생 출력 예시**:

```
# 충돌 없음
[collision] state VALID — no collision.

# 충돌 발생 — 어느 쌍이, 어디서(planning frame xyz), 얼마나 깊이 파고들었는지
[collision] state INVALID — 2 contact(s):
  #1  gripper_2fg14 <-> table_top     xyz=( 0.412, -0.020,  0.198)  depth=0.007 m
  #2  wrist_3_link  <-> screwdriver   xyz=( 0.030,  0.100,  0.640)  depth=0.003 m
```

**확인 포인트**: `VALID` 면 충돌 없음. `INVALID` 면 쌍(link↔obstacle/self)·접촉점(UR base 기준 xyz)·침투깊이가 나옵니다.

### (c) 충돌 직전까지 실제로 이동 (충돌 지점 눈으로 확인)

```bash
# 목표로 가되 첫 충돌 직전 자세에서 정지 → 무엇이 어디서 닿는지 확인
/usr/bin/python3 ur_bringup/isaac/common/obstacles/approach_to_collision.py 0 -0.2 1.9 -1.7 -1.57 0
/usr/bin/python3 ur_bringup/isaac/common/obstacles/approach_to_collision.py 0 -0.2 1.9 -1.7 -1.57 0 --to-contact  # 접촉 자세까지 진입
/usr/bin/python3 ur_bringup/isaac/common/obstacles/approach_to_collision.py 0 -0.2 1.9 -1.7 -1.57 0 --no-exec     # 이동 없이 보고만
```
**확인**: 경로가 깨끗하면 목표까지 이동. 충돌이 있으면 **직전 자유 자세에서 정지**하고 첫 충돌 접촉점을 보고. Isaac 팔이 그 자세에서 멈춥니다.

### (d) 충돌 회피 plan+execute 데모 (MoveIt OMPL)

```bash
python3 ur_bringup/isaac/common/moveit_plan_execute_demo.py     # 관절 목표, 충돌 회피 계획+실행
python3 ur_bringup/isaac/common/cartesian_demo.py               # MoveL 직선(경로 전체 충돌검사)
python3 ur_bringup/isaac/common/movej_demo.py                   # MoveJ 곡선(IK 목표만 검사)
```
**확인**: `moveit_plan_execute_demo` 는 계획 성공 시 실행 후 목표 도달을 /joint_states 로 검증(도달 로그).
계획이 충돌로 실패하면 `error_code` 음수 로그 → 목표 자세가 충돌하거나 경로가 막힌 것.

> **데모별 충돌검사 범위**: `moveit_plan_execute_demo`=OMPL 전 경로 충돌검사(MoveJ 완전판),
> `cartesian_demo`(MoveL)=`/compute_cartesian_path avoid_collisions=true` 로 전 경로 검사,
> `movej_demo`(MoveJ)=IK 목표만 검사(직선보간은 통과). 목적에 맞게 고르세요.

### (e) 파지한 휠 충돌 체크 — gripped vs not-gripped (manip/)

휠을 로봇 articulation 에 **합치지 않고** MoveIt **AttachedCollisionObject** 로 그리퍼에 "붙여서" 충돌 쿼리에
포함시킵니다. 한 번 붙이면 위 (b)~(d) 의 모든 도구(collision_report/approach/plan)가 **파지한 휠까지 자동으로
검사**합니다(코드 무수정, sim==real). 파지 대상 2상태(정지) — 실시간 pick/place 는 phase-1(§8.4)입니다.

```bash
cd /isaac-sim/volume/ur_dualtool_ws/src

# (1회, GUI) 파지 자세 튜닝 — 휠을 핑거 사이에 맞추면 터미널에 --grasp-xyz/-rpy 값이 실시간 출력
/isaac-sim/python.sh ur_bringup/isaac/common/manip/place_wheel_gui.py

# gripped   : 휠을 그리퍼에 붙임(원기둥 프록시 Ø125×20, 기본 grasp=GUI 튜닝값) → 이후 (b)~(d)가 휠까지 검사
/usr/bin/python3 ur_bringup/isaac/common/manip/attach_wheel.py
# not-gripped: 휠 제거(붙였던 걸 world 에서도 함께 제거)
/usr/bin/python3 ur_bringup/isaac/common/manip/attach_wheel.py --detach
# (실메시로 붙이려면) --shape mesh --mesh <abs>/wheel.obj
```

**파지가 진짜 검사되는지 판별**(프로브 박스 = 휠 트레드 rim 에 놓는 검증용 작은 박스):

```bash
/usr/bin/python3 ur_bringup/isaac/common/manip/probe_box.py            # 박스 투입(휠 rim, 라이브 TF 계산)
/usr/bin/python3 ur_bringup/isaac/common/manip/probe_box.py --remove   # 박스 제거
```

**검증 시퀀스**: ① not-gripped + 박스 → `collision_report.py` = **VALID** → ② gripped + 박스 → **INVALID**
(`probe_box ↔ wheel`). 두 상태 차이가 **오직 파지한 휠뿐**이므로 = 휠이 충돌검사에 정상 참여함이 증명.

**확인 포인트**: attach 후 (b) 가 휠 접촉을 잡으면 정상. `touch_links`(그리퍼·양 핑거)는 파지 접촉이라
무시되고 **그 외(팔·world) 충돌만** 잡힙니다.

> ★ 함정 1 — `/check_state_validity` 는 요청 RobotState 의 **`is_diff=True`** 여야 붙인 휠이 유지됨(기본 False 면
> 씬의 attached object 가 드롭돼 **항상 VALID**). collision_report/approach 에 반영됨.
> ★ 함정 2 — MoveIt detach 는 휠을 **world 로 되돌리므로**, `attach_wheel.py --detach` 가 world CollisionObject
> 제거까지 함께 보냄(안 그러면 not-gripped 인데 유령 휠이 남음).

---

## 12. 정리 (종료)

GPU/ROS 를 다른 워크스페이스와 공유하므로 **광범위한 pkill 금지**. 이 워크로드 패턴만:

```bash
pkill -f ur16e_isaac_ros2.py ; pkill -f "ros2 launch ur_bringup" ; pkill -f "lib/rviz2/rviz2"
```

---

## 부록 — config 숫자만 교체해 재빌드 (실제 조립 STEP 이 왔을 때)

구조(스크립트·트리·조인트)는 그대로 두고 **config 숫자만** 실제 값으로 바꾼 뒤 6~9단계를 재실행:

```bash
# (2~4단계로 새 STEP 인수/변환/점검 후)
$P isaac/common/eoat/normalize_part.py $CFG
$P isaac/common/eoat/build_eoat_usd.py $CFG
$P isaac/common/eoat/build_ur16e_dualtool.py
$P isaac/common/eoat/build_eoat_urdf.py $CFG
$P isaac/common/eoat/export_eoat_meshes.py $CFG
$P isaac/common/eoat/build_eoat_moveit.py $CFG
cd /isaac-sim/volume/ur_dualtool_ws && colcon build --symlink-install --packages-select ur_bringup --cmake-args -DBUILD_TESTING=OFF
```

## 부록 — 자주 겪는 함정 요약

| 증상 | 원인/조치 |
|---|---|
| bbox 가 1000배 | mm↔m 단위. STEP 헤더 mm 확인 / 변환 스크립트 미터 네이티브 유지 |
| `/joint_states` 전부 NaN → RViz SIGSEGV | topic_based 가 0.2.1 아님. `ur16e.repos` 로 태그 고정 |
| `Switch controller timed out` | sim 인데 `/clock` 없음. Isaac 먼저, `use_sim_time:=true` |
| plan `error_code -4` | Isaac 재시작 후 move_group/RViz 미재시작(NaN 캐싱). C 터미널 재시작 |
| 로봇 안 뜨고 카메라/그리퍼만 | `--asset-path` 상대경로. **절대경로**로 |
| 뷰포트 까맣게만 | 환경(조명) 미로드. `--no-env` 제거 |
| RViz↔Isaac 자세 어긋남 | URDF/USD 충돌모델 불일치. 7단계 재생성(단일 소스) |
| EOAT 방향 90° 틀림 | config `rpy_deg` 90° 조정 후 재빌드. Isaac↔URDF 비교는 **물리-재생 pose** 로 |
| 파지한 휠이 항상 VALID | `/check_state_validity` 요청 state 에 `is_diff=True` 누락(attached object 드롭). 11-(e) |
| not-gripped 인데 유령 휠 충돌 | detach 가 휠을 world 로 되돌림. `attach_wheel.py --detach`(world 제거 포함) 사용 |
