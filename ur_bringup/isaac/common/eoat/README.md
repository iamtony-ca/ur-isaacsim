# EOAT config-driven CAD→sim 파이프라인

STEP(요소품) → 정규화 → 단일 조립 USD 를 **YAML 하나로** 굴리는 템플릿. 실제 STEP 을 받으면
**코드 수정 없이 config 파라미터 튜닝만**으로 sim 조립을 만든다.

## 3단계 (전부 Isaac bundled python)
```bash
CFG=isaac/common/eoat/eoat_gripper_branch.yaml
P=/isaac-sim/python.sh

# 1) STEP -> USD (미터, Z-up). 폴더째도 가능. (assets/cad/*.usd 생성)
$P isaac/common/convert_step_to_usd.py <STEP폴더> isaac/assets/cad

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
  - `normalize.rpy_deg`: 장착축을 +Z 로 돌리는 회전(도, XYZ). 부품 CAD 프레임마다 다름.
  - `normalize.seat`: `zmin|zmax|none` — 회전 후 어느 bbox 면을 z=0 에 앉힐지(=장착면).
  - `normalize.center_xy`: 회전 후 x,y 중심을 tool 축에 정렬.
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

## 실제 전체 조립 STEP 을 받으면 (정밀화)
개별 부품 대신 **전체 조립 STEP** 이 오면: 변환 후 부품별 world transform 을 추출해 `parts.*.normalize`
와 `chain.*`(gap/상대자세)를 그 값으로 덮어쓴다. **구조(스크립트·트리·조인트)는 그대로**, 숫자만 교체.
(추출 스크립트는 조립 STEP 도착 시 작성 — 부품 매칭 기준 필요.)

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
