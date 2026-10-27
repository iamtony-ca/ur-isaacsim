# base_placement — 협소 셀에서 UR16e 베이스 위치 선정

고정된 TCP 목표 자세 여러 개가 주어졌을 때, **협소한 셀 안에서 UR16e 베이스를 어디에(바닥·받침대·천장·벽) 두면
모든 목표에 자연스럽게(재구성 없이, 특이점·관절한계·벽에서 떨어져서, 짧은 사이클로) 도달하는지**를 찾는 도구와
그 기술검토입니다.

> **상태 (2026-10-16 검증, 2회차)**: `self_test.py` 회귀 30항목 PASS(FK·IK 3e-10 m, 약 2 s). 예제 셀 기본 실행 53 s
> (단일 코어), 경량 4.2 s. **MoveIt 실가동 검증**: 세트2 sim 스택(Isaac 없이) + move_group 에서 `--pick all` 2회 —
> 상위 10개의 목표 자세 60개 모두 메시 충돌 없음, 10개 중 9개가 목표 간 계획까지 **두 번 모두** 통과(이동당 요청 최대 3번),
> 도구의 `recommended`(천장) 통과, 점수 1등 `best`(받침대 15 cm)는 선반 근처 두 이동에서 두 번 모두 계획 실패.
> 그래서 **결과는 반드시 `moveit_validate.py --pick all` 로 확정**합니다(§5).
> ROS 패키지가 아니며(`COLCON_IGNORE`), colcon 빌드 대상이 아닙니다. 아직 `ur_bringup` 에 반영하는 단계(§6)는 없습니다.
>
> **재조정 완료 (2026-10-20, `HISTORY.md` §51.6·§54.2)**: cuMotion XRDF 재생성(메시를 **덮는** 구) 뒤 기본 여유값을
> **`env_margin` 0 mm · `self_margin` −25 mm** 로 바꿨습니다(코드와 예제 YAML). 근거: 덮는 구가 이미 메시 + XRDF 버퍼 5 mm 를
> 감싸므로 환경 여유는 더 필요 없고, 이웃 링크의 덮는 구는 서로 겹치므로 자기충돌 여유는 음수(무작위 자세 MoveIt 대비 놓침 11·
> 오경보 17). env 5 mm 면 받침대 15 cm 후보가, 10 mm 면 30 cm 도 전부 사라집니다. 예제 셀 기본 실행 **66 s**(옛 53 s).
> **MoveIt `--pick all` 3회**(세트2, Isaac 없이): 상위 10개 × 목표 6개 = 60 자세 **매번 전부 메시 충돌 없음** — 실패는 모두
> 선반 근처 계획(bench_regrasp→shelf_place→tray_pick). 3회 모두 통과: #3 받침대 15 cm (0.41, 0.41) · #4·#5 천장 (0.86, 0.35/0.29)
> · #6 받침대 15 cm (0.42, 0.36) · #10 ymin 벽. 도구의 `recommended`(#8 천장 0.91, 0.30, 강건 최고)는 **1/3** 만 통과, 점수 1등
> (#1 받침대 15 cm 0.44, 0.56)은 0/3 — 옛 XRDF 때와 같은 양상이라 **결정은 여전히 MoveIt 확정으로** 합니다.
> 위 "상태" 문단의 53 s·통과 수치는 옛 XRDF 기준입니다. 옛 결과를 재현하려면 git 의 이전 `ur16e_2f85.xrdf` 를 따로 꺼내고
> `ur16e_kin.py` 51행의 `XRDF` 상수를 그 경로로, 여유값을 env 20 mm · self +10 mm 로 되돌리세요
> (런타임 대입 `ur16e_kin.XRDF = …` 은 기본 인자로 묶여 반영되지 않음; CLI 옵션 없음).

---

## 1. 기술검토 요약

- **방법은 있고, 잘 정립된 문제입니다**(robot/base placement). 구조는 항상 같습니다:
  **후보 베이스 하나를 평가하는 함수** + **후보 공간을 뒤지는 탐색**.
- 협소 셀 + 목표 5~20개 + UR16e 라면 **"해석적 IK 로 전 후보를 전수 평가"** 가 가장 맞습니다.
  - UR 은 닫힌 형태 IK(최대 8해)가 있어 IK 는 후보당 약 20 µs, 충돌까지 포함한 평가는 약 1~2 ms
    → 수천~수만 후보를 단일 코어로 초~분 단위에 처리합니다.
  - 역도달성 맵(Reuleaux 등)은 **환경 충돌을 보지 않아** 협소 셀에선 1차 필터 이상이 못 됩니다.
  - 연속 최적화(B\*, 사이클타임 최적화 논문들)는 더 정교하지만 구현·라이선스 부담이 큽니다(§3).
- **"자연스러운 모션"의 핵심 조건은 하나의 IK 가지(branch)로 모든 목표에 도달하는 것**입니다. UR 의 8개 해는
  어깨/팔꿈치/손목 플래그로 나뉘고, 각 플래그는 특이점에서만 바뀌므로 가지가 다르면 목표 사이에서 팔이
  "뒤집히는" 재구성 동작이 생깁니다. 이 도구는 이것을 **필수 조건**으로 둡니다.
- **J1 이 ±360° 인 한 베이스 yaw 는 탐색 변수가 아닙니다.** 베이스를 자기 z 축으로 돌리는 것은 q1 을 바꾸는 것과
  같고 하우징이 원통이라, 도달·충돌(가능 여부)은 yaw 와 무관합니다(장착면 7종 × yaw 12개 실측). 점수는 J1 관절한계
  여유 항만 조금 바뀝니다. yaw 는 **케이블 출구 방향으로 현장에서 정하면** 됩니다. J1 을 좁히면 자동으로 yaw 도 탐색합니다.
- 실무 정답은 "점수 1등"이 아니라 **설치 오차(±2 cm/±3°)에도 같은 가지로 유지되고 MoveIt 계획이 통과하는 후보**입니다.
  예제 셀에서 점수 1등(받침대 15 cm)은 강건 점수 0.15 에 MoveIt 계획 실패(2회 모두), 추천(천장, 강건 0.33)은 통과였습니다
  (옛 XRDF 기준).

## 2. 문제 정의

| 항목 | 내용 |
|---|---|
| 입력 | 셀 좌표계의 TCP 목표 자세 N 개(+방문 순서), 셀 경계·장애물, 허용 장착면(바닥/받침대 높이/천장/벽), 툴(TCP) |
| 결정 변수 | 장착면 종류(이산) × 장착면 위 위치 (u, v) (× yaw: J1 이 제한된 경우만) |
| 필수 조건 | ① 베이스·받침대·어댑터 판이 셀 안, 장애물과 안 겹침 ② 모든 목표에 관절한계 내 IK 존재 ③ **하나의 IK 가지**로 전 목표 도달 ④ 모든 목표 자세에서 셀·자기 충돌 없음 |
| 점수(0~1) | 조작성 \|det J\| 최소값, UR 특이점 여유(\|sin q5\|, \|sin q3\|, 어깨 거리), 관절한계 여유, 속도한계 기반 사이클 타임, 팔 여유거리, 목표 간 직선 관절이동의 충돌 없음 비율 |
| 강건성 | 상위 후보를 설치 오차 범위로 흔들어 같은 가지 유지율·평균 점수 |

## 3. 방법·도구 검토 (2026-10-02 조사, 1차 출처 확인)

### 3.1 방법

| 방법 | 원리 | 협소 셀 적합성 |
|---|---|---|
| 역도달성 맵 (Vahrenkamp ICRA 2013; Reuleaux, Makhal & Goins 2017) | 도달성 맵을 뒤집어 목표마다 "베이스가 있을 수 있는 분포"를 만들고 교집합 | **환경 충돌 미고려**(Reuleaux 논문이 한계로 명시). 1차 필터 용 |
| 능력 맵 (Zacharias IROS 2007) | 복셀별 도달 가능한 방향 비율 | 작업공간 이해용. 배치 자체는 별도 탐색 필요 |
| 전수/그리드 탐색 + 해석적 IK (**이 도구**) | 후보마다 8개 IK 해를 모두 평가 | 차원 3~4 에서 전역해 보장(해상도 내), 충돌·가지 일관성 직접 반영 |
| 진화/군집 최적화 (GA·PSO·CMA-ES; Mitsi 2008, Doan & Lin 2017, Kapusta & Kemp) | 연속 공간에서 점수 최대화 | 해석 IK 가 있으면 그리드로 충분. 차원이 커질 때 유리 |
| 연속 최적화 B\* (Zhao 외 RA-L 2025) | 베이스와 목표별 관절을 함께 변수로, 부호거리 충돌 제약 | 정교하지만 상용 솔버(COPT) 의존 |
| 사이클타임 최적화 (Mayer & Althoff 2025; Wachter, Kugi & Hartl-Nesic IROS 2024) | 충돌 없는 경로계획 + 시간 파라미터화까지 비용에 포함 | 가장 현실적이나 후보당 수 초. 이 도구 결과 상위 몇 개에 후처리로 적용할 만함 |
| 설치 오차 강건성 (Gautier 외 2025; Kapusta & Kemp) | 가능 영역 안의 최대 내접원 / 몬테카를로 | 이 도구는 후보 주변 섭동으로 반영 |

### 3.2 도구 (ROS 2 Jazzy + UR16e 기준)

| 도구 | 하는 일 | 상태 |
|---|---|---|
| ROS-Industrial **REACH** + reach_ros2 | **주어진** 배치 하나를 점수화(조작성·충돌거리 등), 히트맵 | 베이스 탐색은 안 함(스크립트로 감싸야 함). Jazzy 는 소스 빌드만(apt 없음) |
| Reuleaux + base_placement_plugin | 역도달성 맵으로 베이스 후보 제시 | ROS 1, 2020 년 이후 커밋 없음, IKFast(기본 Motoman MH5), 환경 충돌 없음 |
| NVIDIA cuRobo v0.8 (Apache-2.0) | GPU 배치 IK + 충돌, 도달성 데모 | 베이스 배치 API·UR16e 설정 없음. 이 컨테이너에 없음 |
| Isaac Sim 6.1 번들 cuMotion (`CollisionFreeIkSolver`) | 충돌 고려 배치 IK | NVIDIA 독점 라이선스, Isaac python 안에서만 |
| Pinocchio / Coal | 운동학·야코비안·충돌 질의 | 빌딩블록. apt 후보 있음(미설치) |
| ur_analytic_ik (PyPI) | UR 계열 해석적 IK, UR16e 파라미터 동일 | Python 3.12 휠 없음(소스 빌드) → 이 도구는 numpy 로 직접 구현 |
| 상용 OLP | Siemens Process Simulate "Robot smart placement", ABB RobotStudio AutoReach(ABB 중심), FANUC ROBOGUIDE Auto place(FANUC 전용) | RoboDK 는 자동 배치 최적화 기능 없음(포럼 답변) |

결론: **Jazzy 에서 "TCP 목표 + 셀 벽 → 최적 베이스"를 바로 내주는 유지보수되는 오픈소스는 없습니다.**
그래서 해석 IK 기반 전수 탐색을 이 폴더에 직접 구현했습니다(의존성: numpy, scipy, PyYAML, xacro).

### 3.3 UR16e 에서 확인한 사실

- DH: d1 0.1807, a2 −0.4784, a3 −0.36, d4 0.17415, d5 0.11985, d6 0.11655 (ur_description 과 동일).
- 프레임: DH 기준 프레임 = ROS-I `base` = **`base_link` 를 z 축으로 180° 돌린 것**, DH 6번 프레임 = `tool0`.
  xacro 로 만든 URDF 와 FK 차이 3e-10 m, IK 왕복 오차 3e-10 m(`self_test.py`).
- 특이점(닫힌 식): det J = a2·a3·sin q3·sin q5·(a2 cos q2 + a3 cos(q2+q3) + d5 sin(q2+q3+q4))
  → 팔꿈치(q3 = 0), 손목(q5 = 0/±180°), 어깨(손목점이 J1 축에서 d4 = 0.174 m 원통 위).
- 관절 한계: **UR16e PolyScope 5 사용자 매뉴얼(SW 5.22) 기술사양(PDF 17쪽) "Joint ranges ±360° for all joints except
  Elbow ±160°"**. 이전 개정판 매뉴얼과 데이터시트는 ±360°, ur_description 은 elbow ±180° 로 문서 간 불일치가 있습니다.
  예제 YAML 은 매뉴얼의 160 을 씁니다 — 실제 로봇의 안전 설정을 확인하세요. 속도: base·shoulder 120°/s, 나머지 180°/s.
- 장착: 어느 방향이든 성능 동일. 단 PolyScope **Installation > General > Mounting** 에 장착 각도(바닥 0°, 벽 90°,
  천장 180°, 기울기)와 페이로드를 반드시 입력해야 함(안 하면 잦은 보호정지).
- 2F-85 손가락은 **tool x 방향으로 여닫힙니다**(패드가 ±x 를 봄, 손가락 폭은 tool y). 목표의 `xaxis` 가 곧 집는 방향입니다.

## 4. 이 도구의 파이프라인과 충돌 모델

```
cell.yaml ─▶ 장착면별 (u,v) 그리드 ─▶ [필수] 발판 → IK 8해 + 관절한계 → 단일 가지 → 목표 자세 충돌(셀+자기)
                                       └▶ [점수] 조작성·특이점·관절여유·사이클타임·여유거리·직선이동 충돌샘플
          ─▶ 상위 지점 주변 5배 촘촘한 국소 그리드 ─▶ 후보 3×K 개를 직선이동까지 검사해 재정렬 → 상위 K 개
          ─▶ 강건성(±dxy·±dyaw 평면 섭동 75개 + 6자유도 무작위 200개) ─▶ results.json / report.md / CSV / PNG
          ─▶ moveit_validate.py: MoveIt 메시 충돌·실제 계획기로 확정
```

- **충돌 모델은 구(sphere) 근사**입니다. 이 워크스페이스의 cuMotion XRDF(`ur_bringup/cumotion/ur16e_2f85.xrdf`) 구
  세트를 쓰고, 장애물은 축정렬 박스, 받침대는 원통입니다.
  - **2026-10-16 이후**: XRDF 의 **world 구**(메시를 덮는 195 개, 오버슈트 ≤ 20 mm, 버퍼 5 mm)를 셀과 자기충돌 **둘 다**에
    씁니다(`ur16e_kin.load_spheres(which=...)`, 쌍 9,036). XRDF 의 별도 self 구(453 개, 베이스 하우징 25 개를 빼면 428 개·SRDF 쌍
    43,492)는 MoveIt 대비 0 mm 에서 놓침 5·오경보 45 로 더 정확하지만, 최적화 전 코드에서 그리드 ~12 배·정밀화 ~30 배
    느려(당시 관찰, 중단) 쓰지 않습니다. 덮는 구는 이웃 링크끼리 겹쳐도 메시가 안 닿으므로 self_margin 은 음수가 맞습니다 —
    MoveIt 대비(무작위 1000 자세) −20 mm 놓침 2·오경보 62, **−25 mm 11·17**, −30 mm 24·9. 아래 항목들(무시 목록·10 mm·20 mm·
    95 %)은 **옛 XRDF 기준의 기록**입니다. **현재 기본값은 env 0 mm · self −25 mm**(2026-10-20 MoveIt 3회 확정, 맨 위 요약·
    `HISTORY.md` §54.2).
  - **자기충돌 쌍은 MoveIt SRDF 기준**(인접 링크만 제외)입니다. 옛 cuMotion XRDF 의 무시 목록은 forearm↔wrist_2,
    forearm↔wrist_3 까지 빼서, MoveIt 이 충돌로 보는 자세를 통과시켰습니다(`HISTORY.md` §50.1; 재생성 XRDF 에서 수정, §51).
  - 구가 메시보다 작은 만큼 **자기충돌 여유 10 mm**(`scoring.self_margin`)를 둡니다. 예제 셀 상위 10개를 MoveIt 으로
    판정해 정했습니다: 0 mm → 7/10 유효, 5 → 8, **10 → 9**, 15 → 9 이지만 MoveIt 상 유효한 받침대 15 cm 후보를 전부 잘라냄.
  - **환경 여유 20 mm**(`scoring.env_margin`, XRDF 환경 버퍼 5 mm 위에). UR16e 충돌 메시는 XRDF 구(+버퍼) 밖으로
    최대 shoulder 28 · upper arm 34 · forearm 39 mm 튀어나옵니다(메시 표면 표본 실측). 10 mm 에서는 예제 상위 10개 중 2개의
    목표 자세가 MoveIt 에서 선반·고정구와 접촉했고(메시 여유 0.2 mm, −9 mm), **20 mm 에서는 0개에 추천은 그대로**,
    30 mm 는 추천이 바뀌고 40 mm 는 해가 없어집니다. 메시 돌출보다 작으니 **보수적인 값이 아니라 예제에서 맞춘 절충값**입니다.
  - 무작위 자세 1000개의 **자기충돌만** 비교하면 MoveIt 메시 판정과 약 95 % 일치합니다(출하 설정 기준 구 모델이 놓친 것 16,
    과잉 거부 32). 셀(벽·박스)과의 충돌 일치율은 따로 재지 않았습니다. **그래서 최종 판정은 MoveIt 으로 합니다.**
  - 발판(베이스 하우징·벽/천장 어댑터 판·받침대)과 박스의 간섭은 원통-박스 해석식으로 판정합니다.
- **관절 별칭(±2π)**: ±360° 관절은 같은 자세에 두 값이 있습니다. 방문 순서를 따라 이동량이 최소가 되는 별칭을
  동적계획으로 고른 뒤(순환이면 마지막→처음까지), 이동량이 같은 2π 균일 이동 중 관절한계 여유가 가장 큰 쪽으로 맞춥니다.
- **직선 관절이동 검사**는 "계획기 없이 바로 가면 부딪히는가"일 뿐입니다(샘플 간격: 관절 변화량 합 2.3°). BLOCKED 는
  실패가 아니라 우회 경로가 필요하다는 뜻이고, 점수의 한 항(가중치 0.25)으로만 들어갑니다.
- 해가 하나도 없으면 **목표별 차단 원인**(발판이 맞는 베이스 중 그 목표가 도달 가능한 비율 / 충돌 없는 비율)과
  **장착면별 한 줄 진단**(깔때기의 어느 단계에서 전부 떨어졌는지, 예: "모든 목표가 어떤 가지로는 충돌이 없지만 한 가지로
  전부는 안 됨 — 최선 가지 5/6")을 출력합니다. 협소 셀에서는 "베이스를 어디 두나"보다 "어느 고정구·목표를 옮겨야 하나"가
  답인 경우가 많습니다.

## 5. 사용법

```bash
cd /isaac-sim/volume/ur_ws/src/base_placement
source /opt/ros/jazzy/setup.bash   # xacro + ur_description (+ apt robotiq_description: 세트 2/3)
                                   # ur_bringup 의 xacro 는 이 소스 트리에서 직접 읽음 — 1·2단계는 colcon 빌드 불필요

python3 self_test.py                                                         # 1) 회귀 점검: 30항목 PASS (약 2 s)
python3 place_base.py examples/confined_cell.yaml --out out/confined --plot  # 2) 탐색 (약 1 분)
python3 moveit_validate.py out/confined/results.json --pick all              # 3) MoveIt 으로 확정
```

3단계만 **빌드된 워크스페이스**가 필요합니다(`ur_bringup` 런치). 모든 터미널에서
`source /opt/ros/jazzy/setup.bash && source /isaac-sim/volume/ur_ws/install/setup.bash && export ROS_DOMAIN_ID=<id>`
(검증 스크립트와 move_group 이 같은 도메인이어야 함). 그리고 **같은 툴 세트**의 move_group 이 떠 있어야 합니다.
세트 2 는 Isaac 없이도 됩니다(2026-10-16 확인):
```bash
ros2 launch ur_bringup ur16e_2f85.launch.py use_sim_time:=false
ros2 launch ur_bringup ur16e_2f85_moveit.launch.py use_sim_time:=false launch_rviz:=false
```
`--pick all` 은 상위 후보를 차례로 검사해 통과한 후보를 모두 보여 주고, **점수가 가장 높은 통과 후보**와 (다르면)
**강건 점수가 가장 높은 통과 후보**의 `<origin>` 을 출력합니다. 종료 코드: 0 = 하나라도 통과, 1 = 검사한 후보 전부 실패,
2 = move_group 이 없거나 도중에 사라져 검사가 끊김(나머지는 "not checked"). 셀을 `base_link` 기준 충돌 객체로 넣으므로
URDF 를 옮기지 않아도 됩니다. 계획 요청은 이동마다 **최대 3번**(첫 요청 + 재시도 2번, `--retries`) 보냅니다(OMPL 은
무작위이고 협소 셀에서는 찾은 경로가 MoveIt 의 사후 검증에서 무효가 되는 일이 많습니다). 계획 궤적이 YAML 의 관절한계
덮어쓰기(예: elbow ±160°)를 넘으면 FAIL 로 셉니다. 자세가 무효인 목표로 드나드는 이동은 건너뜁니다.
**OMPL 이 무작위라 한 번의 실행은 표본 하나입니다** — 결정 전에는 `--pick all` 을 2~3번 돌려 매번 통과하는 후보를 고르세요.
YAML 을 `place_base.py` 이후에 고쳤으면 경고하고, 장착면이 없어졌거나 높이·종류가 바뀌었거나 장착 자세가 달라진 후보는
FAIL 로 표시합니다(다시 `place_base.py` 를 돌리라는 뜻).

CPU 사용(이 머신은 다른 컨테이너와 CPU 를 공유):
- 기본은 **단일 프로세스, BLAS 1 스레드**. `--workers N` 을 줄 때만 N 코어를 씁니다(그리드·정밀화·강건성 분산).
- 실측(옛 XRDF 기준; 예제 셀, 5 cm 그리드, 후보 3,877): 기본 53 s(그리드 4 · 정밀화 17 · 재정렬 2 · 강건성 31),
  `--no-path` 12 s, `--no-path --no-refine --no-robust` 4.2 s. 가장 무거운 단계는 강건성과 정밀화입니다.
- 가볍게: `--no-robust`, `--no-refine`, `--mounts floor,pedestal_30`(`--workers` 와 같이 써도 됨), `--grid 0.1`.
  우선순위는 `nice -n 19`.
- `--no-path` 는 **탐색용**입니다. 그리드·정밀화의 직선이동 샘플링을 생략하고 그 항을 만점으로 가정합니다. 최종 상위 3×K 개는
  직선이동을 검사해 다시 점수를 매기므로 **표시되는 점수와 목록 안의 순서는 정확**하지만, 어느 지점을 정밀화할지·
  군집의 대표를 어느 점으로 할지·어느 지점이 후보 목록에 드는지는 낙관 점수로 정해져 **best/top 이 기본 실행과 다를 수
  있습니다**(예제: 현재 설정에선 같지만, env 여유 10 mm 에서는 best 가 (0.44, 0.56) 대신 (0.45, 0.55) 였음). 강건성 섭동도
  직선이동을 건너뛰고, 그 대신 후보 자신의 실측 직선이동 비율로 강건 점수를 보정합니다(예제: recommended 는 기본 실행과
  같음). **결정은 `--no-path` 없이.**

YAML 스키마는 `examples/confined_cell.yaml` 의 주석을 보세요. 핵심만:
- `targets`: `xyz` + 자세(`approach`(툴 z)+`xaxis`(툴 x = 2F-85 여닫는 방향), 또는 `rpy_deg`, 또는 `quat_xyzw`). `order`, `cyclic`.
- `tool.set`: 1(맨 팔, tool0) / 2(2F-85) / 3(D405 브래킷 + 2F-85). `tool.tcp`: `tool0 | gripper_frame | pad | custom`.
  `gripper_frame` 은 이 워크스페이스에서 MoveIt·cuMotion 이 계획하는 프레임(tool0 + 11 mm, 세트3 + 18 mm),
  `pad` 는 그리퍼를 연 상태의 손가락 패드 중심(tool0 + 141 mm). 세트 1 에 `pad`/`gripper_frame` 을 주면 오류입니다.
  세트 3 의 D405 는 XRDF 에 없으니 `tool.extra_spheres` 로 넣습니다(예시는 YAML 주석).
- `cell.walls`: 열린 면(라이트 커튼 개구부 등)은 `false`(따옴표 없는 true/false 만). 단 **장착면 자신은 항상 막힌 면**으로
  봅니다(예: `zmax: false` 여도 천장 장착 후보에서는 천장이 벽). 장착면·장애물 **이름은 중복 불가**.
- `robot.joint_limits_deg`: 대칭 한계 덮어쓰기(예: `elbow_joint: 160`, 케이블 때문에 `shoulder_pan_joint: 180`).
- `mounts`: `kind` = `floor`(+`height` 받침대) / `ceiling` / `wall_xmin|xmax|ymin|ymax`, `u`/`v` 범위는 그 면의 축.
- `scoring`: `weights`(manip/ur/jlim/time/path/clear) 와 여유값.
- **모든 섹션에서 모르는 키는 오류**입니다(오타면 비슷한 키를 제안: `heigth` → `height`). 오타가 조용히 기본값으로
  바뀌지 않게 하기 위함입니다.

## 6. 출력과 적용

`report.md` 에 깔때기(단계별 남은 후보 수), 목표별 차단 원인, `recommended`·`best` 의 자세표가 있습니다.
`results.json` 에는 상위 후보 전부의 목표별 관절값, 사용한 URDF 의 해시, 단계 설정이 들어 있습니다.

예제 셀 실측(2026-10-16, 옛 XRDF 기준 — 새 XRDF 로는 재측정 전):
- 5 cm 그리드 후보 3,877 개 중 가능 15 개(받침대 15 cm 3·천장 3·ymin 벽 9). 2.5 cm 그리드(yaw 0)는 14,851 개 중 64 개
  (받침대 15 cm 8·천장 22·ymin 벽 34)입니다. 자기충돌·환경 여유를 바꾸기 전 코드로는 111 개였고, yaw 15° 24 개로 돌린
  356,424 개 중 2,664 개(= 111 × 24, yaw 중복)가 2026-10-02 프로토타입과 장착면별로 일치했습니다.
- **바닥 직접 설치가 0 인 이유는 천장 근처 목표가 아니라 `shelf_place`**(선반 위 위에서 내려 잡기, TCP z 0.79 m)입니다.
  손목점이 어깨 축 위 0.867 m 로 |a2|+|a3| = 0.838 m 를 넘어 바닥 어디서도 IK 가 없습니다(베이스를 3 cm 이상 올리면 생김).
  뒷벽(xmin)은 반대편의 `tray_pick`·`bench_regrasp` 에 거의 닿지 않습니다(도달 3 %).
- MoveIt 확정(`--pick all` 2회): 상위 10개 중 9개가 두 번 모두 통과, 추천 후보(천장 x 0.91, y 0.28) 통과, 점수 1등(받침대
  15 cm (0.45, 0.55))은 bench→shelf·shelf→tray 이동이 두 번 모두 요청 3번 실패. env 여유 10 mm 때는 상위 10개 중 2개가
  목표 자세부터 무효였고 통과 수가 7/10·5/10 으로 실행마다 달랐습니다.

결정한 베이스를 실제로 반영하는 순서(아직 이 워크스페이스에 자동화돼 있지 않음):
1. **MoveIt 확정**: `moveit_validate.py --pick all` (위).
2. **URDF**: 세트별 sim xacro 의 `base_joint` origin 이 `xyz="0 0 0"` 으로 고정돼 있습니다. 셀 좌표계를 `world` 로
   쓰려면 `report.md` 의 `<origin .../>` 를 넣을 xacro 인자를 추가해야 합니다(`ur_bringup` 변경 — 별도 작업).
   cuMotion 은 `base_link` 기준이라 셀 벽을 `StaticPlanningSceneServer` 의 장면 파일로 줘야 합니다.
3. **Isaac Sim**: `/UR16e` prim 위치·자세(`report.md` 의 position / orientation_wxyz). 스크립트에 아직 인자가 없고,
   articulation 고정 조인트가 Xform 이동을 따라가는지는 **미확인**입니다.
4. **실물**: PolyScope 장착 각도·페이로드 입력, M8 볼트 4개(8.8, 20 Nm), 재장착 정확도가 필요하면 Ø8 핀.

## 7. 한계

- 충돌은 구 근사 + 축정렬 박스 장애물. 회전된 장애물·메시는 아직 없음(박스 여러 개로 근사). 새 XRDF 의 world 구는 메시를
  덮어 환경 충돌은 보수적이지만, 덮는 구로 자기충돌을 보려면 음수 여유가 필요하고 그 값에서도 MoveIt 대비 놓침이 남습니다
  (−25 mm 에서 11/210). 그래서 `moveit_validate.py` 로 확정합니다(옛 XRDF 때는 구가 메시 안쪽이라 환경 충돌을 놓쳤고,
  env 10 mm 시절 예제 상위 10개 중 2개를 이것이 잡았음).
- 목표 자세는 정확히 그 자세(툴 축 회전 자유도 미지원). 파지처럼 축 회전이 자유로우면 목표를 회전시켜
  여러 개로 넣는 방식으로 우회.
- 동역학·가감속·페이로드 감소(도달거리별)는 미반영. 사이클 타임은 관절 속도한계만 쓴 하한 추정.
- 직선 관절이동 검사는 계획기가 아님. 최종 판단은 MoveIt/cuMotion 계획 결과로.
- 케이블·호스·안전 거리(사람 접근)는 모델에 없음 — 박스 장애물이나 `env_margin` 으로 반영.

## 8. 파일

| 파일 | 내용 |
|---|---|
| `ur16e_kin.py` | URDF FK, DH 해석적 IK(8해), 특이점 지표, TCP, cuMotion 구 세트, SRDF 기준 자기충돌 쌍, xacro 실행 |
| `cell.py` | 셀 YAML → 목표·장애물·장착면 자세(+ 입력 검증) |
| `evaluator.py` | 후보 일괄 평가(필수 조건·점수·직선이동·목표별 진단) |
| `place_base.py` | 탐색 진입점(그리드 → 국소 정밀화 → 직선이동 재정렬 → 강건성 → 출력) |
| `self_test.py` | 회귀 점검 30항목(운동학, 쿼터니언, 링크·TCP, 발판, 별칭, 이름, 자기충돌 쌍, YAML 키 오타, 장착면 막힘, 그리드 끝점, 중복 제거, xacro 임시 prefix, 장면 객체 id) |
| `moveit_validate.py` | 후보를 MoveIt 으로 확정(FK·상태 유효성·계획, 재시도, 관절한계 덮어쓰기 검사, YAML 변경 감지) |
| `examples/confined_cell.yaml` | 예제 협소 셀(스키마 주석 포함) |

## 9. 참고

- Vahrenkamp, Asfour, Dillmann, "Robot placement based on reachability inversion", ICRA 2013, doi:10.1109/ICRA.2013.6630839
- Makhal, Goins, "Reuleaux: Robot Base Placement by Reachability Analysis", arXiv:1710.01328
- Zacharias, Borst, Hirzinger, "Capturing robot workspace structure", IROS 2007, doi:10.1109/IROS.2007.4399105
- Zhao 외, "B\*" (RA-L 2025), arXiv:2504.12719, github.com/leiyaocui/B_STAR
- Mayer, Althoff, "Smart Placement, Faster Robots", Front. Manuf. Technol. 2025, arXiv:2504.19577
- Wachter, Kugi, Hartl-Nesic, "Time-Optimal TCP and Robot Base Placement for Pick-and-Place Tasks in Highly Constrained Environments", IROS 2024, doi:10.1109/IROS58592.2024.10801373
- Gautier 외, ASME IDETC 2025, arXiv:2511.15290 (설치 허용오차 영역)
- Kapusta, Kemp, arXiv:1804.07328 (CMA-ES, 오차 강건성)
- Hawkins, "Analytic Inverse Kinematics for the Universal Robots UR-5/UR-10 Arms", Georgia Tech 2013
- ROS-Industrial REACH: github.com/ros-industrial/reach, github.com/ros-industrial/reach_ros2
- UR16e PolyScope 5 User Manual (SW 5.22): 기술사양(관절 범위 17쪽), 장착, 페이로드
