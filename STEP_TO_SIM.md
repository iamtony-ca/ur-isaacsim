# STEP(CAD) → Isaac Sim 온전한 파이프라인

기구설계팀에서 **요소품별 STEP**(또는 JT/IGES)을 받았을 때, UR16e EOAT(End-Of-Arm Tooling)
조립체를 Isaac Sim + ROS2/MoveIt2 sim 환경으로 **스스로** 꾸리기 위한 재현 가능한 절차.

> 대상 조립체(dual-tool): `UR16e → 기구댐퍼 → OnRobot Dual Quick Changer →`
> `[포트 A] HEX F/T → 2FG14(+custom finger)` / `[포트 B] 어댑터 → ESTIC 스크류드라이버 (+Copick3D ×1)`.
> Dual Quick Changer 는 두 툴을 **동시 장착**하므로 단일 articulation + Y분기 트리 = 신규 "세트 4".
> Copick3D 는 **1대**(어댑터 측면에 장착, 스크류드라이버 볼팅 지점을 바라봄).

---

## 핵심 개념 — STEP 은 "지오메트리"만 준다

STEP/JT/IGES 는 **형상(B-rep)** 뿐이다. 로봇 sim 에 필요한 **링크 프레임·조인트·관성·충돌·articulation·ROS**
는 전부 우리가 **저작(author)** 해야 한다. 그래서 온전한 sim = 아래 7단계이고, STEP 이 채워주는 건 **A 하나뿐**이다.

| 단계 | 산출물 | STEP 이 주나? |
|---|---|---|
| **A. 지오메트리 변환** | 부품별 USD 메시(미터, Z-up) | ✅ (형상만) |
| **B. 프레임 정규화** | 각 링크 원점을 조인트 위치로 | ❌ 저작 |
| **C. 충돌 형상** | convex/저폴리 collision | ❌ 저작(A 에서 단순화) |
| **D. 질량·관성** | mass/COM/inertia | ❌ SolidEdge 물성 or 근사 |
| **E. 운동학(URDF/xacro)** | 링크+조인트(고정/회전/직동) 매크로 | ❌ 저작 |
| **F. 단일 articulation USD bake** | UR16e+부품 참조, 조인트, 루트 1개 | ❌ 저작(`build_*` 스크립트) |
| **G. 검증** | RViz↔Isaac EE 일치, MoveIt plan | ❌ |

> **B~G 는 이제 자동화 완료** — "❌ 저작"이 코드로 손볼 일이 아니라 **config(`eoat_dualtool.yaml`) 한 줄**로
> 바뀐다. 아래 "단계 B~G — config-driven 단일 소스 파이프라인" 참고.

---

## 단계 A — STEP → USD (검증됨, 자동화 완료)

### 왜 asset_converter 가 아니라 HOOPS 인가 (함정)
- `omni.kit.asset_converter`(= `convert_dae_to_usd.py`)는 **STEP 을 거부**한다
  (`OmniConverterStatus.UNSUPPORTED_IMPORT_FORMAT`). 메시(.dae/.stl/.obj)용이다.
- CAD B-rep 은 **HOOPS Exchange 백엔드**(`omni.kit.converter.hoops_core`)로만 들어온다.
- **★ 함정: `omni.kit.converter.cad` 확장을 켜지 말 것.** ODA(Teigha, DWG/DGN) 라이브러리
  `libTD_DbCore.so` 를 끌어와 in-process 심볼 충돌로 죽는다
  (`undefined symbol: _ZN13OdConstStringC1EPKw`, 프로세스 abort). **`hoops_core` 만** 켠다.
- **★ 함정: 반드시 미터 네이티브로.** `dMetersPerUnit=1.0` 로 출력해야 `metersPerUnit=1.0` +
  좌표=미터(0.0749…) 가 되어 미터 씬에 1:1 로 들어온다. 안 그러면 mm 스테이지가 되고
  reference 시 metersPerUnit 이 무시되어 **1000배 괴물**이 된다(RViz↔Isaac 어긋남, 함정 #7 유형).
- 출력은 **Z-up**(`iUpAxis=2`) — Isaac/USD 관례.

### 사용법 (`isaac/common/convert_step_to_usd.py`)
```bash
# 단품
/isaac-sim/python.sh src/ur_bringup/isaac/common/convert_step_to_usd.py \
    부품.STEP  src/ur_bringup/isaac/assets/cad/부품.usd

# 폴더째 (한 번의 Isaac 세션으로 전부 변환 — "폴더 받으면 스스로" 진입점)
/isaac-sim/python.sh src/ur_bringup/isaac/common/convert_step_to_usd.py \
    <step_들어있는_폴더>  src/ur_bringup/isaac/assets/cad
# opts: --up X|Y|Z(기본 Z)  --lod 0..4(기본 2, 클수록 촘촘)  --keep-hidden  --no-materials
```
변환 후 결과 USD 의 **bounding box(mm)** 를 자동 출력한다 → 실물 치수와 대조해 스케일을 즉시 검증.

### 검증 로그 (Isaac 6.0.1-rc.7, 실 STEP)
| 부품 USD | meshes | 실측(변환 결과) | 비고 |
|---|---|---|---|
| 2fg14 | 90 | 74.9 × 166.4 × 118.6 mm | OnRobot 2FG14 그리퍼 |
| hex_qc | 22 | 88.2 × 50.0 × 101.2 mm | HEX F/T + QC |
| hex_backplate | 1 | 69.1 × 13.5 × 68.5 mm | HEX 백플레이트 |
| adapter_t | 1 | 100.0 × 21.5 × 100.0 mm | Adapter_T |
| dual_quick_changer | 26 | 128.4 × 97.1 × 71.0 mm | Dual Quick Changer |
| quick_changer_tool_side | 14 | 100.4 × 28.4 × 100.4 mm | QC 툴사이드 |
| screwdriver_estic | 4 | 58.0 × 69.4 × 226.2 mm | ESTIC EH2-H0025-SC |
| copick3d_150s | 43 | 792 × 255 × 360 mm | ⚠️ **FOV 원뿔 포함** — 몸체만 쓰려면 76MB 원본 변환 or 원뿔 프림 제거 |

> 소스 STEP: `oht_bolting/docs/onrobot/*` + 사용자 추가분(`assets/cad/*.STEP`). 결과: `isaac/assets/cad/*.usd`.

### 플레이스홀더 STEP 생성 (`make_placeholder_step.py`, cadquery)
실물 CAD 가 없는 임의 부품(원기둥 댐퍼, Copick 카메라 어댑터 플레이트)은 **cadquery(OpenCASCADE)로
진짜 B-rep STEP 을 생성** → 벤더 부품과 **동일한 STEP→USD 경로**를 타고, 기구팀에 넘기거나 실 CAD 로 교체 가능.
- Isaac 컨테이너엔 CAD 커널이 없으므로 **격리 venv** 에 cadquery 설치(헤드리스, X11 불필요; gmsh 는 libXcursor
  의존으로 실패). 설치/실행법은 `isaac/common/eoat/requirements-cad.txt`.
  ```bash
  /isaac-sim/kit/python/bin/python3 -m venv .venv-cad
  .venv-cad/bin/python -m pip install -r isaac/common/eoat/requirements-cad.txt
  .venv-cad/bin/python isaac/common/eoat/make_placeholder_step.py isaac/assets/cad   # STEP 생성(mm)
  /isaac-sim/python.sh isaac/common/convert_step_to_usd.py isaac/assets/cad/<part>.step isaac/assets/cad/<part>.usd
  ```
| 부품 STEP→USD | 형상 | 치수 | 파라미터 |
|---|---|---|---|
| damper | 원기둥(+Z, base@0) | Ø63 × 25 mm | `--damper-dia --damper-h` |
| camera_adapter | 박스 플레이트 | 120 × 60 × 12 mm | `--adapter 120x60x12` |

> (구식 대안 `make_placeholder_usd.py` = CAD 커널 없이 USD 직접 저작. cadquery 가 있으면 STEP 경로가 우선.)

---

## 단계 B~G — config-driven 단일 소스 파이프라인 (구현 완료)

**핵심 아키텍처 — single source of truth.** STEP 은 A(형상)만 준다. B~G(프레임·충돌·관성·운동학·
articulation·MoveIt)는 전부 **`isaac/common/eoat/eoat_dualtool.yaml` 하나**에 선언하고,
순수-python 그래프 모델 `eoat_model.py` 가 이를 **하나의 link/joint 그래프**로 해석한 뒤, 여러 emitter 가
서로 일치하는 산출물(Isaac USD·URDF·SRDF)을 만든다 → **함정 #7(URDF↔USD 불일치)이 구조적으로 불가능**.
실 CAD/물성이 오면 **코드 수정 없이 config 숫자만 교체**.

```
eoat_dualtool.yaml ─▶ eoat_model.py (graph: mass/inertia/collision/frames 확정)
                          ├─▶ build_eoat_usd.py   → ur16e_dualtool.usd      (Isaac 물리 아티큘레이션)
                          ├─▶ build_eoat_urdf.py  → ur16e_dualtool.urdf     (MoveIt/cuMotion 충돌모델)
                          ├─▶ build_eoat_moveit.py→ *_eoat_macro.xacro + *.srdf.xacro
                          └─▶ build_ur16e_dualtool.py → ur16e_dualtool_full.usd (UR16e+EOAT 단일 아티큘레이션)
```

### B. 프레임 정규화 — `normalize_part.py <cfg>`
각 부품 USD 를 **장착면→원점, 장착축→+Z** 로 정규화(`assets/cad/normalized/*.usd`) + `*.json` 사이드카
(size/thickness/base_z/bbox)를 남겨 그래프가 면-대-면 스택 높이를 자동 계산. config `parts.<>.normalize:
{rpy_deg, seat, center_xy}`.

### C. 충돌 형상 — config `collision:`
`convexHull`(기본) | `convexDecomposition`(오목형상, 예 2FG14 몸체) | `boundingCube`(예 copick, FOV 원뿔
제외용) | `none`. 부품별 `parts.<>.physics.collision` 로 override.

### D. 질량·관성 — density-box 근사 or 실측 override
`meta.defaults.density`(kg/m³)로 bbox-box 관성 근사가 기본. **실측 오면 `parts.<>.physics` 에
`mass`/`com`/`inertia` 명시 → 근사를 덮어씀**(copick 은 mass 1.2 로 시연). USD emitter 는 전 관성텐서를
`eigh` 대각화해 `principalAxes`+`diagonalInertia` 로 저작. ⚠️ 현재 density-box 는 질량을 과대평가
(gripper 4kg vs 실 1.45kg) — 실 관성 오기 전까지의 placeholder(`README.md §미완 갭`).

### E. 운동학 — 3가지 배치 방식 (우선순위: tcp_pose > mount > 자동스택)
`chain[]` 에 부품을 순서대로 나열. 각 항목의 위치 지정:
1. **자동스택**(기본): parent 윗면에 +Z 면-대-면(정규화 사이드카의 thickness 사용) — 직렬 스택.
2. **`mount: {xyz, rpy}`**: parent 프레임 기준 명시 자세 — Y분기/직교 포트(예 스크류드라이버 가지 90° 꺾음).
3. **★`tcp_pose: {xyz, rpy}`**: `root_link`(tool0/UR TCP) 기준 **절대 상대포즈**. 기구설계팀이 TCP 기준으로
   준 데이터를 그대로 입력 → 모델이 부모의 tool0-포즈와 합성해 **parent-local mount 를 자동 역산**.
   `meta.tcp_offset` 로 TCP≠플랜지 오프셋 처리. rpy 는 **ROS 규약**(roll-X/pitch-Y/yaw-Z, R=Rz·Ry·Rx)로
   eoat_model·build_eoat_usd·build_eoat_urdf 전부 통일 → 다축 회전도 USD==URDF.
조인트: `joint: fixed`(스택 결합) 또는 구동 조인트(2FG14 핑거 prismatic 1-DOF + mimic — `parts.2fg14.joints`).

### F. 단일 articulation bake + MoveIt 브리지
- `build_eoat_usd.py` → EOAT 단독 물리 아티큘레이션(RigidBody/Mass/Collision/Joint/Drive/재질/ArticulationRoot).
- `build_ur16e_dualtool.py` → UR16e USD 에 EOAT 를 tool0 에 고정결합, EOAT 의 ArticulationRoot/world_fixed 제거
  → **팔+EOAT 단일 아티큘레이션**(루트 `/UR16e/root_joint`, 8-DOF = 6팔 + 2핑거). HOME 포즈 baked.
  ★ **`ATTACH_LOCAL_ROT`=항등 유지** — 이 UR 은 TF `wrist_3→tool0`=항등이라 -90°Z 주면 물리-재생 시 EOAT 가
  90° 스냅돼 RViz 와 어긋남(HISTORY §15 T2). `world_fixed` 는 reference prim 이라 RemovePrim 무효 →
  `SetActive(False)`+post-flatten purge 로 제거(안 하면 tool0 를 월드에 고정해 팔 발산).
- `build_eoat_urdf.py` + `export_eoat_meshes.py` + `build_eoat_moveit.py` → URDF/xacro/SRDF. **export_eoat_meshes**
  가 부품 실제 mesh 를 `meshes/eoat/` 로 뽑아 URDF 가 box 대신 `<mesh>` 참조 → **RViz==Isaac**(안 하면 RViz 는 박스).
- ★ Isaac play 시 **self-collision OFF 필수**(placeholder 핑거 박스가 몸체에 박혀 드라이브가 걸림).

### G. 검증
- `verify_articulation.py <usd> [root] [--zero-gravity]` — play-test(핑거 대칭 도달, arm_held).
- `xacro …_sim.urdf.xacro | check_urdf` — 결합 트리(24링크/30조인트) 파싱, `robot name="ur16e"` 유지.
- SRDF 는 **EOAT-vs-팔몸통 충돌을 ENABLE 유지**(collision-discovery contract) → MoveIt 이 툴이 팔에 박히는
  자세를 self-collision 으로 거부(1차 목표 = 충돌 point 발굴/확인).
- **live 3터미널**(Isaac→제어→MoveIt) 검증됨: `/check_state_validity` 충돌탐색(self+EOAT-vs-팔+장애물),
  plan+execute, MoveJ/MoveL(`movej_demo.py`·`cartesian_demo.py`) — HISTORY §15.
- **RViz 자세=Isaac 자세 확인은 물리-재생 pose 로**(정적 Xform 은 fixed-joint 스냅을 안 보여줌, HISTORY §15 T2).

### 정적 장애물 충돌 (사전제공 CAD, sim+real 공용)
테이블·지그 등 알려진 장애물을 STEP/STL 로 받아 **MoveIt planning scene**(sim=real 충돌계층)+Isaac scene 에
올려 self+장애물 충돌을 계획시 자동회피/체크. 같은 단일-소스 철학. 상세 `isaac/common/obstacles/README.md`.

### GUI 튜닝 ↔ config 왕복 (숫자 확정 도구)
숫자(mount/tcp_pose/rpy)는 rough placeholder라 **Isaac GUI 에서 눈으로 맞추는 단계**가 필요하다:
- `edit_asset.py <usd>` — 조립 USD 를 편집가능 루트로 GUI 오픈(돔라이트 세션레이어 추가). 링크 프림을
  기즈모로 드래그해 부품을 맞물린 뒤 **Ctrl+S** 로 원본 USD 에 저장.
- `extract_poses.py <tuned.usd> <cfg>` — 튜닝된 각 링크의 **parent-local** 자세를 `mount:` 값으로 출력.
  `--tcp` 를 주면 **tool0 기준** `tcp_pose:` 로 출력 → config 에 붙여넣기.
- **실 데이터 경로**: 기구팀에게 "**UR tool0(플랜지) 기준, 각 부품 원점의 xyz[m] + rpy[deg, ROS RPY]**" 로
  요청하면 각 `chain[]` 의 `tcp_pose:` 에 그대로 입력 → 정밀 정렬이 한 번에. (컨트롤러 TCP 기준이면
  그 오프셋만 `meta.tcp_offset`.)

---

## 재현성 (다른 PC)
- 필요한 것: Isaac Sim **6.0.1** 컨테이너(이 확장들 포함): `omni.kit.converter.hoops_core`,
  `omni.kit.converter.common`. (HOOPS Exchange 는 Isaac 번들에 포함 — 별도 설치 불필요.)
- 이 파이프라인 스크립트는 순수 headless, GPU 렌더 불필요. STEP 폴더만 주면 A 단계는 완전 자동.
- 5.1→6.0.1 이식 검증 완료(스크립트 API·기존 bake USD 전부 6.0.1 통과).
