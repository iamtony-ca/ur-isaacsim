# 정적 장애물 충돌 파이프라인 (`obstacles/`)

사전 제공된 CAD 정적 장애물(테이블·지그·펜스 등, **동적/perception 아님**)을 환경에
통으로 올려 로봇의 **self + 장애물 충돌**을 계획 단계에서 자동 회피/체크한다.
EOAT 파이프라인과 같은 **단일 소스 config** 철학: `obstacles.yaml` 하나가
MoveIt planning scene(sim+real 공용 충돌계층) 과 Isaac scene(sim 시각/콜라이더)을 동시에 구동.

## sim ↔ real 원리
실제 "충돌 체크"는 **MoveIt planning scene**(백엔드 무관)이 담당 → sim(Isaac)이든
real(RTDE)이든 **동일한 collision-free 궤적**을 계획한다. 장애물 pose 는 `fixed_frame`
(로봇 base 고정) 기준이라 sim == real. Isaac 로딩(`--obstacles`)은 sim 시각/물리용일 뿐.
전제: 충돌 모델(로봇/EOAT 기하 = 함정 #7 URDF==USD, + 장애물 pose)이 실제와 일치 +
tracking 오차는 collision padding 이 흡수 → sim 검증이 실물로 전이.

## config (`obstacles.yaml`)
```yaml
meta:
  fixed_frame: world           # 장애물 pose 기준(로봇 base 고정). MoveIt planning frame 과 일치.
  cad_dir: ../../assets/cad/obstacles       # 기구팀 STEP/STL 원본
  out_usd_dir: ../../assets/obstacles       # 변환 USD(Isaac) + 콜라이더 OBJ(MoveIt)
  defaults: {collision: mesh, lod: 1}   # 기본 mesh(정확·비과보수)
obstacles:
  - id: table                  # box 예: CAD 없이 primitive
    collision: box
    size: [1.2, 0.8, 0.72]
    pose: {xyz: [0.15, 0, -0.36], rpy: [0,0,0]}
    allowed_collisions: [base_link, base_link_inertia]   # 로봇이 얹힌 구조 → ACM(오탐 방지)
  - id: jig_a                  # CAD 예: 기구팀 STEP/STL
    cad: jig_a.step            # .step/.stp/.igs/.jt(HOOPS) 또는 .stl/.obj(mesh) — 어느 레벨이든 입력은 동일
    collision: mesh            # mesh | convexDecomposition | convex | box  (per-obstacle 선택)
    pose: {xyz: [0.5, 0.25, 0.1], rpy: [0,0,0]}
```
- `allowed_collisions` = 이 장애물과 충돌을 허용할 로봇 링크(테이블처럼 로봇이 닿아있는 구조의 자기충돌 오탐 방지).

## 충돌 형상 4-레벨 (입력 포맷과 독립)
입력 파일(STEP/STL/…)과 충돌 표현은 **별개의 축**이다 — 어느 레벨을 골라도 입력은 같은 STEP/STL 하나.
`prepare_obstacles.py` 는 CAD 마다 mesh/convexDecomposition/convex 를 **모두** 산출하므로 재변환 없이 전환.

| level | 뜻 | 실제 CAD 대비 | 용도 |
|---|---|---|---|
| `mesh` ★기본 | 실제 tessellated 표면(decimate 로 삼각형 수만 축소) | = 실제, **안 부풀림** | 빽빽한 환경·정확 필요 |
| `convexDecomposition` | CoACD 볼록조각 병합 | ≈ 실제(오목 따라감) | 정확+경량 중간 |
| `convex` | 단일 볼록껍질 | **오목부 메꿔 커짐**(보수적) | 이미 볼록한 부품만 |
| `box` | 바운딩 박스 | 훨씬 큼 | 테이블·벽 등 단순형 |

- STEP 은 B-rep(곡면)이라 변환 시 `lod`(0~4)로 tessellate 세밀도 제어; STL 은 export 시 이미 삼각형화됨. 둘 다 mesh 로 쓸 수 있음.
- **점진 점검**: `load_obstacles_moveit.py --level mesh|convexDecomposition|convex` 로 전역 전환해 여유(margin) 가늠.
- MoveIt/FCL 은 이 OBJ 메시로 체크; **Isaac 물리 콜라이더는 항상 native convexDecomposition**(경량, RL 부담 회피)로 독립.
- 여전히 타이트하면 MoveIt `default_robot_padding`(안전여유 inflation)을 줄이는 별도 노브도 있음.
- (convexDecomposition 은 CoACD 필요: `/isaac-sim/python.sh -m pip install coacd`, 설치됨)

## 실행 (from `isaac/common/`)
```bash
P=/isaac-sim/python.sh ; CFG=obstacles/obstacles.yaml
# 1) (CAD 장애물만) STEP/STL → USD + 충돌 OBJ 3종(mesh / convexDecomposition / convex)
$P obstacles/prepare_obstacles.py $CFG           # box-only 면 생략 가능. 3레벨 모두 산출

# 2) MoveIt planning scene 에 장애물 + ACM 올리기 (move_group 실행 중; sim+real 공용)
#    ROS 소싱 필요: source /opt/ros/jazzy + install/setup.bash + ROS_DOMAIN_ID
/usr/bin/python3 obstacles/load_obstacles_moveit.py $CFG
/usr/bin/python3 obstacles/load_obstacles_moveit.py $CFG --level convexDecomposition  # 전역 레벨 전환(점진 점검)

# 3) 충돌지점 보고 (현재 상태 또는 특정 6관절 상태)
/usr/bin/python3 obstacles/collision_report.py                 # 현재 /joint_states
/usr/bin/python3 obstacles/collision_report.py 0 -0.2 1.9 -1.7 -1.57 0   # 특정 상태 → 접촉점 xyz+깊이

# 4) (sim 시각화) Isaac scene 에도 같은 장애물 배치
$P ur16e_isaac_ros2.py --asset-path <abs>/assets/ur16e_dualtool_full.usd --obstacles obstacles/obstacles.yaml
```
로드 후 move_group 은 계획 시 이 장애물을 자동 회피(collision-free 궤적). 시작/목표가 충돌이면 계획 거부(START/GOAL_STATE_IN_COLLISION).

## 기구팀 CAD 오면
`cad_dir` 에 파일 넣고 `obstacles.yaml` 에 한 줄(`id/cad/collision/pose`) 추가 → `prepare_obstacles.py` → `load_obstacles_moveit.py`.
STEP·STL 둘 다 동일 경로(STEP=HOOPS, STL=asset_converter)이고 **입력 포맷과 collision 레벨은 무관** — 부품마다
`collision:` 로 mesh(정확)/convexDecomposition/convex/box 를 골라 정확도·무게를 조절. 현 테이블은 Isaac 기본 asset(box);
기구팀 실제 테이블/지그 CAD 오면 `table` 항목을 `cad: table.step, collision: mesh` 로 교체만.

## 미완/다음
- 연속 충돌 모니터(실행 중 실시간 감지) — 지금은 계획시 회피 + 상태 보고까지. `collision_report.py --watch` 가 경량 프리뷰.
- cuMotion(GPU, real) 장애물 월드 연동 — cuMotion 은 world 를 별도로 먹으므로 후속.
