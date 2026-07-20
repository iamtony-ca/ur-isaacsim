# EOAT config-driven CAD→sim 파이프라인

STEP(요소품) → 정규화 → 단일 조립 USD 를 **YAML 하나로** 굴리는 템플릿. 실제 STEP 을 받으면
**코드 수정 없이 config 파라미터 튜닝만**으로 sim 조립을 만든다.

## 3단계 (전부 Isaac bundled python)
```bash
CFG=isaac/common/eoat/eoat_gripper_branch.yaml
P=/isaac-sim/python.sh

# 0) ★ 원본 STEP 인수 점검 (변환 전, pure python·Isaac 불필요) — 포맷/AP스키마/단위(mm)/솔리드수/faceted.
#    요청서(CAD_DELIVERY_REQUEST.md)의 "소스측" 항목 검사. validate_cad(변환 후)와 상보적.
python3 isaac/common/check_step_delivery.py <STEP폴더> [--expect a.step,b.step]

# 1) STEP -> USD (미터, Z-up). 폴더째도 가능. (assets/cad/*.usd 생성)
$P isaac/common/convert_step_to_usd.py <STEP폴더> isaac/assets/cad

# 1.5) ★ CAD 인수 점검 (변환 직후) — 스케일/단위·bodies·watertight·밀도·원점 자동 리포트.
#      실측/도면과 대조할 게 있으면 config 에 parts.<>.expected_size_mm:[x,y,z](mm) 적어두면 자동 스케일 검증.
$P isaac/common/validate_cad.py isaac/assets/cad            # 폴더째, PASS/WARN 요약
$P isaac/common/validate_cad.py isaac/assets/cad --config $CFG    # expected_size_mm 대조

# 2) 프레임 정규화: 장착면->원점, 장착축->+Z (assets/cad/normalized/*.usd)
$P isaac/common/eoat/normalize_part.py $CFG            # [part ...] 로 일부만도 가능

# 3) 조립: chain 대로 tcp_pose/mount/스택 -> EOAT 단일 조립 USD
$P isaac/common/eoat/build_eoat_usd.py $CFG

# 4) UR16e 결합본 (_full.usd, HOME 포즈 baked). ★ ATTACH_LOCAL_ROT=항등 유지 —
#    이 UR16e URDF 는 TF wrist_3->tool0=항등이라, -90°Z 를 주면 물리-재생 시 EOAT 가 90° 스냅돼
#    RViz 와 어긋난다(HISTORY §15 T2). Isaac↔URDF 방향 비교는 반드시 물리-재생 pose 로.
$P isaac/common/eoat/build_ur16e_dualtool.py

# 5) MoveIt 브리지: URDF + xacro 매크로 + SRDF (충돌모델 = USD 와 단일소스)
$P isaac/common/eoat/build_eoat_urdf.py $CFG
$P isaac/common/eoat/export_eoat_meshes.py $CFG    # ★ 부품 실제 mesh -> meshes/eoat/ (RViz==Isaac; box 대신)
$P isaac/common/eoat/build_eoat_moveit.py $CFG     # mesh 있으면 <mesh> 참조, 없으면 box(placeholder 핑거)
```
> 추가 파이썬 의존(Isaac python): `pip install coacd fast-simplification` (convexDecomp / mesh decimate).
> `export_eoat_meshes` 산출물을 install 에 반영하려면 `colcon build --packages-select ur_bringup`(또는 `meshes/eoat` 심링크).

## config 스키마 (`eoat_*.yaml`)
- **`parts.<name>`**
  - `cad`: `assets/cad/` 안의 소스 USD 파일명
  - `expected_size_mm`(선택): `[x,y,z]` 도면 실측 치수(mm). 있으면 `validate_cad.py --config` 가
    변환된 bbox 와 자동 대조 → 스케일/단위 오류(1000배 등) 경고. 없으면 사람이 눈으로 대조.
  - `physics.collision`(선택, 부품별): `convex`(=`convexHull`, 기본) | `convexDecomposition` | `mesh` | `boundingCube` | `none`. **단일 knob 이
    USD 근사와 export 되는 `_col.obj` 를 함께 결정**(장애물 3레벨과 동형). `convex`=단일 hull(가벼움·오목메움),
    `convexDecomposition`=CoACD 조각(오목추종, `pip install coacd` 필요·없으면 hull 폴백),
    `mesh`=decimate 실메시(정밀·파지/삽입 접촉용). 시각 `<id>.obj` 는 항상 실메시.
  - `normalize.passthrough`: **CAD 원점을 그대로 신뢰**(rotate/seat/center 전부 skip, 항등 래퍼).
    기구팀이 아래 "원점 규약"대로 저작해 주면 이걸 켜서 GUI 튜닝 없이 바로 조립. base_z≠0 이면
    (장착면이 원점에 없으면) 자동 경고. `rpy_deg`/`seat`/`center_xy` 를 덮어씀.
  - `normalize.rpy_deg`: 장착축을 +Z 로 돌리는 회전(도, XYZ). 부품 CAD 프레임마다 다름.
  - `normalize.seat`: `zmin|zmax|none` — 회전 후 어느 bbox 면을 z=0 에 앉힐지(=장착면).
  - `normalize.center_xy`: 회전 후 x,y 중심을 tool 축에 정렬.

  **원점 두 갈래** — (A) 원점 제각각으로 오면 `rpy_deg`+`seat`+`center_xy` 로 **기하에서 재원점**
  (GUI 확인 필요). (B) 기구팀에 원점을 지정 요청할 수 있으면 아래 **규약**대로 받아 `passthrough:true` 로
  **그대로 사용**(무튜닝). 부품마다 A/B 혼용 가능.

  **★ 원점 규약(passthrough 전제, 기구팀에 요청):**
  - 원점 = **장착면**(부모와 맞닿는 면) 위, - **+Z = 스택 방향**(장착면에서 다음 부품 쪽 바깥),
  - **x/y = 장착축 중심**, - **Z-up, mm**.
  - Z가 미세하게 어긋날 우려가 있으면 완전 passthrough 대신 `rpy_deg:[0,0,0], center_xy:false, seat:zmin`
    (방향·XY는 신뢰, 장착면만 z=0 스냅)로 부분 신뢰.
- **`chain`** (list, 위→아래 순): `{id, part, parent, joint, gap?, mount?}`
  - `id`: 링크 고유이름(생략 시 `part`). **같은 부품 다중 인스턴스**는 id 로 구분(예: 카메라 여러 대).
  - `part`: 기하 소스(`parts.<part>`).
  - `parent`: `root_link`(tool0) 또는 앞서 나온 **id**. **Y분기**는 두 링크가 같은 parent 를 쓰면 됨.
  - `joint`: `fixed|revolute|prismatic` (구동부는 다음 단계에서 axis/limit 부여).
  - `gap`: (기본 스택 모드) 결합면 여유(+Z, m).
  - `mount`: `{xyz:[m], rpy:[deg]}` — 주면 parent 프레임 기준 **명시 자세**(Y분기·직교 포트). 없으면 parent
    윗면에 +Z 면-대-면 자동 스택. 한 가지의 첫 부품만 mount 로 꺾으면 이후는 자동 스택이 그 방향을 따라감.

## 새 부품/새 가지 추가 절차 (파라미터만)
1. STEP 을 1단계로 변환 → `assets/cad/<name>.usd`.
2. `parts.<name>` 추가(`cad` + `normalize` 초기 추정).
3. `chain` 에 한 줄 추가(parent 지정; 스크류드라이버 가지는 `parent: dual_quick_changer`).
4. 2·3단계 재실행. Isaac GUI 로 열어 rpy/seat/gap 미세튜닝 후 재빌드.

## rpy/seat 초기 추정 근거 & 튜닝
- `normalize_part.py` 는 각 부품의 `base_z`(0 이어야 함)·`thickness_z`·`size` 를 출력 → 장착면이
  바닥(z=0)에 왔는지, 두께가 맞는지 즉시 확인.
- `build_eoat_usd.py` 는 각 링크의 +Z 위치와 전체 스택 높이·bbox 를 출력.
- 방향이 틀리면(예: 그리퍼가 팔쪽을 봄) `rpy_deg` 를 90° 단위로 조정, 장착면이 위/아래 바뀌면
  `seat` 를 zmin↔zmax 교체. **숫자는 Isaac GUI 확인이 최종** (headless 는 치수만 검증 가능).

## 실제 전체 조립 STEP 을 받으면 (정밀화) — `extract_poses.py`
개별 부품 + **전체 조립 STEP** 을 함께 받으면(권장): 개별 STEP = 부품별 깨끗한 기하·물성,
조립 STEP = 부품 간 상대 pose. 조립 STEP 을 변환·정렬한 USD 에서 자세를 뽑아 config 숫자만 덮어쓴다.
**구조(스크립트·트리·조인트)는 그대로**, 방향/자세 눈대중이 사라진다.
```bash
# 조립 USD 를 열고(또는 GUI 로 링크 정렬 후 저장) 자세를 config 형식으로 추출:
$P isaac/common/eoat/extract_poses.py <assembly.usd> $CFG          # 부모기준 mount 값
$P isaac/common/eoat/extract_poses.py <assembly.usd> $CFG --tcp    # tool0(TCP) 기준 tcp_pose 값
```
- **`--tcp`** = 각 링크를 root_link(tool0) 기준으로 출력 → 조립본이 tool0/루트 프림을 포함하면 전체
  EOAT↔TCP 앵커까지 확정(별도 상대좌표 표 불필요). 부모기준(mount)은 기본 모드.
- **조립 STEP 필수 3조건**(CAD_DELIVERY_REQUEST.md §1-A): ① 개별과 **동일 부품·좌표계·이름**,
  ② mm/Z-up/AP242/솔리드 동일, ③ **tool0 앵커 포함**. 이 3개가 어긋나면 부품 매칭(geometric
  registration)을 수동으로 해야 해 자동 붙여넣기가 깨진다.
- 매칭 기준 = 프림 이름(=chain id). 조립본의 부품 이름을 개별 파일명과 맞춰 받으면 무손실.

## config 예시
- `eoat_gripper_branch.yaml`: 포트 A 만 (tool0→damper→dual_quick_changer→hex_qc→2fg14). 학습용 최소 예.
- `eoat_dualtool.yaml`: **전체 dual-tool** (포트 A 그리퍼 + 포트 B 스크류드라이버+카메라). Y분기·mount·다중인스턴스 사용.

## 현재 상태 (단일 소스 파이프라인 완성)
`eoat_dualtool.yaml` → `eoat_model.py`(순수 python 그래프) → 두 emitter, 전부 검증됨:
- `build_eoat_usd.py` → Isaac 물리 아티큘레이션 USD (`assets/ur16e_dualtool.usd`). `verify_articulation.py` 로
  play 검증: 12 body, 2FG14 핑거(평행 mimic) 드라이브 목표 **대칭 도달**. self-collision OFF 필수.
- `build_eoat_urdf.py` → URDF (`assets/ur16e_dualtool.urdf`), MoveIt2/cuMotion 충돌모델. 트리·mimic·axis 검증.
- `build_ur16e_dualtool.py` → **UR16e 결합** (`assets/ur16e_dualtool_full.usd`): 단일 아티큘레이션
  `/UR16e/root_joint`, 8 DOF(6축+핑거2), 19 body. (동적검증은 ROS 컨트롤러 경로.)
- fidelity 전부 파라미터화(DEFAULT+주석): 마찰재질(핑거 μ=1.0)·관성 대각화·조인트 armature/friction/velocity·솔버 반복수.

## ⚠️ 미완 갭 (나중에 확인/보강 — 실데이터·다음단계)
1. **밀도-box 과대질량** — bbox×밀도 근사가 과대(gripper 4kg vs 실 1.45kg, EOAT 합 ~11kg → 결합 sim 불안정).
   해결: (실측 관성 오면) `parts.<>.physics.{mass,com,inertia}` override. (반자동 개선안) `eoat_model._resolve_physics`
   의 부피를 bbox 대신 **정규화 USD 의 실제 mesh 볼륨**으로 근사하도록 보강 → 질량 자동 현실화.
2. **핑거 기하 분리** — 지금 `2fg14` 핑거는 placeholder box(구조만 완비). 실 파지 전에 분리 필요:
   (a) 기구팀 **분해 STEP**(finger_left/right/base) → `parts.2fg14.sublinks.<>.cad` 로 즉시 매핑(권장·최소작업).
   (b) 단일 `2fg14.usd`(부품프림 존재, 이름=CAD번호)를 base/left/right 로 분류·분할하는 스크립트 신규 작성
       (X좌표 부호 등 휴리스틱 + GUI 확인). → `sublinks.<>.geom_prim` 로 연결.
3. **copick FOV 원뿔**(A단계) — 몸체 bbox 왜곡. 원뿔 프림 제거 필터 or 원본 재변환. 지금은 `physics.{mass,size}` override 로 우회.
4. **MoveIt/cuMotion 브리지**(진행중) — URDF 를 xacro 매크로로 UR16e description 에 결합 + SRDF(self-collision·group)
   + cuMotion XRDF(collision sphere). 세트3 패턴(`srdf/common/`, `cumotion/gen_xrdf.py`) 미러링.
