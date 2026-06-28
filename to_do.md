# to_do.md — 통합 Pick & Place 파이프라인 (foundation-model perception, sim2real 최소화)

> 빠져 있던 **"작업(task) 레이어"**: perception → 물체 → grasp → pick → place(장애물 회피)를 기존 ROS2 스택
> (cuMotion + nvblox + 2F-85 + D405)에 얹는다. **목표: sim2real gap 최소화** — 그래서 GT 대신 **실물에서 쓸
> foundation model 들을 sim 카메라(RGB)에 그대로** 돌린다.
> 관련: 학습(IL/RL) 설계는 [`LEARNING.md`](LEARNING.md), 제어/실행 현황은 [`README.md`](README.md).
> (이 scripted pick&place 는 나중에 **IL 시연 자동생성 소스**로도 재사용 가능 — LEARNING.md §5.)

---

## 0. 핵심 설계 원칙 — "카메라는 RGB only, 나머지는 전부 foundation model"

센서 출력(특히 depth)은 sim(깨끗)↔real(노이즈·홀) 갭이 크다. → **카메라는 스테레오 RGB 만** 내보내고,
depth/mask/pose 는 **sim·real 동일한 foundation model** 로 만든다. 그러면 갭이 **RGB 도메인 하나**로 좁혀지고
(domain randomization 으로 대응), 그 아래 모든 추론이 sim에서 미리 측정·튜닝된다.

> **DR 적용 대상 주의**: domain randomization 은 **학습하는 컴포넌트**(IL/RL 정책 — LEARNING.md, 옵션인 SAM3 fine-tune)에만 강건화로 작용한다.
> **zero-shot 인식 3종(FoundationStereo/SAM3/FoundationPose)은 추가 학습 안 함** → DR 로 "강건화"되지 않고, 거대 데이터 **사전학습 강건성**에 의존한다.
> 이들에 대한 sim 변동(마모·조명 등) 렌더는 **강건화(학습)가 아니라 측정(eval)** — pose/마스크 오차를 재서 마진·폴백 임계값을 정하는 용도(D8b).

**파이프라인 확정(2026-06-29): FoundationStereo / SAM3 / FoundationPose.** 검출기(RT-DETR·Grounding DINO)와 SAM2는
**SAM3 하나가 검출+세그를 통합**하므로 전부 드롭. 타깃 지정은 **텍스트가 아니라 이미지 exemplar**(잡을 물체 crop 한 장)로 한다.

```
카메라(스테레오 RGB only)  ── sim: Isaac stereo  /  real: D405·D455 스테레오 ──┐
                                                                              ▼
  FoundationStereo (RGB L/R → metric depth)        [depth foundation model]
        │
        ├─→ robot_segmenter (로봇 기하 마스킹) ─→ nvblox depth
        │
  SAM3 (이미지 exemplar → 타깃 모든 인스턴스 마스크+ID)   [검출+세그 통합 foundation model]
        │   exemplar = 잡을 물체 crop 한 장(zero-shot, 텍스트·학습 불필요). 검출기 별도 노드 불필요.
        ├─→ nvblox (use_segmentation, human_with_static_tsdf): 타깃을 static ESDF 에서 제외
        └─→ FoundationPose (RGB + foundation-depth + mask + CAD mesh → 6D pose)  [pose foundation model]
                 │   sim=model-based(CAD 보유). CAD 없는 실물 물체는 model-free/Any6D 대안(D8).
         /target/pose ─→ cuRobo known object(pre-grasp 클리어런스) + grasp 계산
                 │
  nvblox static ESDF (환경=장애물만) ─→ cuMotion(read_esdf_world, 장애물만 회피) + add_ground_plane(테이블/바닥)
                 │
         pick&place 상태머신(신규): HOME→DETECT→PRE_GRASP→REFINE→GRASP→CLOSE→LIFT→TRANSFER(회피)→PLACE→OPEN→복귀
```

**두 단계 마스킹(=NVIDIA 정석)**: 로봇=`cumotion_robot_segmenter`(기하), 타깃=SAM3 이미지 마스크(`nvblox use_segmentation`).
남은 것 = 순수 환경 장애물 ESDF. 파지 시 타깃은 cuRobo attach, 들어올린 자리는 nvblox occupancy/TSDF decay 가 비움.

### 카메라 2대 — 역할 분담 (파이프라인 분리, 필수)
정적 오버헤드와 손목 eye-in-hand 는 **역할도 파이프라인도 다르다**(섞으면 안 됨).

| | **External 정적 오버헤드** | **Wrist eye-in-hand (D405)** |
|---|---|---|
| 역할 | ① 장애물 매핑(nvblox ESDF) + ② 타깃 **개략(coarse)** pose | ③ 타깃 **정밀(fine)** 6D pose (파지 직전) |
| 파이프라인 | FoundationStereo→nvblox, SAM3→FoundationPose(개략) | FoundationStereo→SAM3→FoundationPose(정밀+tracking) |
| nvblox 장애물 | **사용** | **절대 사용 안 함** (로봇 자신을 보고 팔과 함께 이동 → TSDF 오염, HISTORY §12) |

**핸드오프(닭-달걀 해소)**: HOME→**DETECT**(정적 cam 개략 위치)→**PRE_GRASP**(근접 이동)→**REFINE**(손목 cam 정밀 pose)→GRASP.
정적=전역(어디에/장애물), 손목=국소(정확히 어떻게 잡나). **v1 은 정적 1대로 장애물+타깃 모두** 처리 → 근접 정밀 부족 시 손목 추가(D4).

---

## 1. 컴포넌트 가용성 (실측, 2026-06-29 갱신)

| 역할 | 패키지 | 상태 |
|---|---|---|
| **depth foundation** | `isaac_ros_foundationstereo` (스테레오 RGB→metric depth) | apt 가용(release-4), **미설치**. Apache-2.0 |
| depth 경량 대안 | `isaac_ros_ess` (DNN stereo disparity) | apt 가용, 미설치 |
| **세그+검출 (확정)** | **SAM3** (이미지 exemplar/text → 전 인스턴스 마스크+ID, 검출+세그 통합) | ROS 노드 **자체 래핑 필요**. Meta SAM License(상업 OK·군사 금지) |
| 6D pose (확정) | `isaac_ros_foundationpose` (+`-models-install`) | apt 가용, 미설치. model-based(CAD)/model-free 양모드 |
| pose 대안 | SAM-6D / **Any6D**(CAD-free 단일 RGB-D) / FreeZeV2(BOP'24 최고정확도) | 정확도·CAD 유무에 따라 교체 후보(D8) |
| 로봇 마스킹 | `isaac_ros_cumotion_robot_segmenter` | ✅ 설치됨(현 nvblox 파이프라인에서 사용 중) |
| 장애물 맵 | `nvblox`(+segmentation/dynamics specialization) | ✅ 설치됨, `use_segmentation`/`human_with_static_tsdf` 지원 확인 |
| 플래너/실행 | cuMotion + `scaled_joint_trajectory_controller` | ✅ 설치·검증됨 |
| ~~SAM2 / Grounding DINO / RT-DETR~~ | ~~검출+세그 분리 경로~~ | **드롭** — SAM3 가 통합 |

설치 예정: `ros-jazzy-isaac-ros-foundationstereo(+models)`, `...-foundationpose(+models)`. release-4 레포는 이미 설정됨.
**SAM3 ROS 노드는 apt 에 없음 → 직접 래핑**(아래 §5 참조).

### SAM3 ROS 래핑 — Meta 원본 직접(상업 대비), ultralytics(AGPL) 회피
- **상업 사용 가능성 열어둠** → ultralytics(AGPL-3.0) 의존을 피하고 **Meta SAM3 원본**(`sam3.pt`, SAM License = 상업 OK)을 직접 래핑한다.
- 참고용으로 `TeamSOBITS/sam3_ros`(ROS2 jazzy-devel, BSD-3) 가 있으나 **반드시 따를 필요 없음** — 내부가 ultralytics `SAM3SemanticPredictor` 기반이고 **text prompt 전용**이라 우리 exemplar 방식·라이선스 방침과 안 맞음. 인터페이스 설계 힌트로만.
- 출력: 타깃 마스크 → (a) nvblox `mask/image`, (b) FoundationPose 마스크 입력. 표준 `sensor_msgs/Image`(단일채널)로 정규화하는 **어댑터** 자체 작성.

---

## 2. 결정 (✅=확정 2026-06-29 / ⏳=미정)

| # | 결정 | 결론 |
|---|---|---|
| **D1 depth model** | foundation depth 선택 | ✅ **FoundationStereo**(foundation·metric). ESS는 경량 대안. **스테레오 RGB 쌍 필요** |
| **D2 seg model** | 타깃 마스크 모델 | ✅ **SAM3**(검출+세그 통합). SAM2/Grounding DINO/RT-DETR 드롭. **Meta 원본 직접 래핑**(ultralytics AGPL 회피) |
| **D3 타깃 지정** | 무엇을 잡을지 | ✅ **SAM3 이미지 exemplar**(잡을 물체 crop 한 장, zero-shot·학습불필요). 텍스트 아님 |
| **D4 카메라** | 카메라 구성·역할 | ✅ **정적 오버헤드=장애물+coarse / 손목 D405=fine grasp**(§0 표). v1은 정적 1대로 시작 가능 |
| **D5 파지 유지** | grasp 시 물체 잡기 | ✅ **접촉 물리 시도→안되면 fixed-joint attach**(견고) |
| **D6 추론 rate** | 느린 perception vs 빠른 제어 | ✅ pose 저rate + ESDF 중rate + 제어 고rate **비동기 분리**. SAM3는 저rate(재획득), FoundationPose tracking이 중rate |
| **D7 컨테이너 토폴로지** | GXF segfault 회피 | ✅ FoundationStereo/SAM3/FoundationPose/nvblox 전부 **별도 컨테이너/프로세스** 분리(필수) |
| **D8 물체/장면·pose 모드** | 대상물·CAD 유무 | ✅ **기본 = CAD 보유 → FoundationPose model-based**. 장애물=낮고작게(z>0.1), place 고정. 향후 **CAD-free 확장**(model-free/**Any6D**)은 pose 노드 교체로(모듈러 유지) |
| **D8b CAD↔실물 불일치** | 마모·스크래치·변형 | ✅ **경미한 마모는 model-based 가 견딤**(depth/기하 앵커, refinement). **큰 형상결손·변형 시 model-free(실물 reference 이미지)로 폴백**. M4에서 마모 정도별 pose 오차 정량화 |
| **D9 sim depth 처리** | Isaac 센서 depth 사용 | ✅ **다운스트림 센서 depth 미사용**. Isaac은 스테레오 RGB만. (GT depth는 FoundationStereo 평가 비교용) |

> 모든 결정 확정(2026-06-29). **D8 = CAD 보유 기본**, CAD-free 는 향후 확장(pose 노드만 교체).

---

## 3. 빌드 마일스톤 (체크리스트)

### M0 — 장면/센서 준비 (Isaac)
- [ ] Isaac 에 **잡을 물체**(물리: rigid body+collider+mass+friction, **CAD 메시 보유**) 스폰 + place 타깃 영역
- [ ] 물체 **exemplar 이미지**(crop) 1장 확보(SAM3 입력) — sim 렌더 crop 또는 별도 캡처
- [ ] **장애물 낮고 작게** 재구성(z 0.1~0.38 정도, pick↔place 경로 사이) — `--obstacle-pose/-size` 조정 또는 기본값 변경
- [ ] **정적 오버헤드 스테레오 RGB 카메라** 추가, intrinsics/extrinsics 발행 (장애물+coarse, D4). 손목 D405는 fine용(v2)
- [ ] **센서 depth 다운스트림 차단** — RGB만 사용(D9). (현 `--with-static-cam` depth 경로는 비교/디버그용으로만)
- [ ] domain randomization 훅(조명/재질/텍스처) 자리 마련

### M1 — foundation perception 단독 검증 (sim 이미지에서 출력 확인)
- [ ] 설치: foundationstereo(+models), foundationpose(+models)
- [ ] ★ **SAM3 ROS 노드 래핑**(Meta 원본 직접, ultralytics 비의존) — 입력 RGB+**exemplar**, 출력 타깃 마스크 → `sensor_msgs/Image` 어댑터로 `/static_cam/mask/image` 발행
- [ ] **FoundationStereo**: 스테레오 RGB → metric depth 발행, GT depth 와 정량 비교(오차 맵)
- [ ] **FoundationPose**: RGB+foundation-depth+SAM3 mask+CAD mesh → `/target/pose`(6D) — GT pose 와 오차 비교
- [ ] 각 노드 **별도 컨테이너**로 분리(D7), TensorRT 엔진 빌드(최초 수 분) 확인, VRAM 모니터

### M2 — nvblox 마스킹 통합 (타깃 제외 / 장애물만)
- [ ] nvblox depth 입력을 **FoundationStereo depth** 로 교체
- [ ] nvblox `mask/image` ← SAM3 타깃 마스크 remap, `mapping_type: human_with_static_tsdf` + `use_segmentation: true`
- [ ] robot_segmenter(로봇) 유지 → 두 단계 마스킹
- [ ] **A/B 검증**: 마스크 on → 타깃이 ESDF에서 빠지고 cuMotion이 접근 가능 / off → 타깃이 장애물로 막힘
- [ ] 마스크 **dilation**(껍데기 voxel 방지), depth↔mask **프레임/해상도/시간 동기** 확인

### M3 — pick&place 상태머신 (통합, GT-free)
- [ ] 신규 `pick_place_demo.py`: `/target/pose` + MoveGroup(cuMotion) + gripper 액션
- [ ] 시퀀스: HOME→**DETECT**(정적 coarse)→PRE_GRASP(위)→**REFINE**(손목 fine, v2)→GRASP(하강)→CLOSE(파지, D5)→LIFT→TRANSFER(회피)→하강→OPEN→RETRACT→HOME
- [ ] cuRobo: 타깃을 known object(pre-grasp 클리어런스) → 파지 시 attach → place 후 detach
- [ ] **GUI 관찰**: 장애물 피해 pick&place 성공, decay 로 들린 자리 정리

### M4 — sim2real 하드닝
- [ ] RGB domain randomization 본격 적용, FoundationStereo depth 정확도/노이즈 sim↔real 분포 맞추기
- [ ] **CAD↔실물 불일치 특성화**(D8b, eval): sim 에 마모·스크래치·텍스처 perturbation **렌더 → FoundationPose pose 오차 측정**(학습 아님, zero-shot) → grasp 마진·model-free 폴백 임계값 결정. 강건성 레버는 메시 갱신/confidence 게이팅/멀티뷰/마진
- [ ] 카메라 intrinsics/extrinsics 실물 일치, 추론 rate/지연 실측
- [ ] perception 실패 모드(오검출/마스크 누락/pose 점프) 복구 로직

### M5 — 실물 (HW 준비 시)
- [ ] 동일 그래프로 real 카메라 스테레오 RGB 입력만 교체, hand-eye/extrinsic 반영 (HARDWARE.md 연계)

---

## 4. 함정 / 리스크 (정직)

- **GXF/NITROS segfault**: FoundationStereo·FoundationPose·nvblox 모두 GXF → 같은 `component_container_mt`에 올리면
  segfault 알려짐(Isaac for Manipulation 릴리스 노트). → **별도 컨테이너 분리**(D7).
- **foundation depth 정확도 트레이드오프**: FoundationStereo metric depth 는 근거리 mm급에선 실물 active-stereo D405 보다
  떨어질 수 있음. **갭 일관성↑ vs 절대정확도↓** — M1에서 GT 대비 정량 평가 후 채택 판단.
- **리소스(RTX 5090 32GB)**: Isaac + FoundationStereo + SAM3 + FoundationPose + nvblox + cuMotion 동시 =
  매우 무거움. 추론 rate 낮추거나 노드 스테이징 필요. VRAM 상시 모니터.
- **마스크 dilation / 프레임·시간 동기**: 안 맞으면 ESDF에 타깃 "껍데기" 잔존 또는 엉뚱한 영역 제외.
- **FoundationPose 전제**: model-based 는 **CAD 메시 필수**. CAD 없으면 model-free(reference 이미지)/Any6D 경로(D8).
- **CAD↔실물 불일치(마모·변형)**: model-based 는 depth/기하 매칭이 앵커라 **경미한 마모·텍스처 변화엔 둔감**(외형보다 형상 의존). 단 **큰 결손/변형 시 정확도 저하**.
  ★ **주의: FoundationPose 는 zero-shot — 물체별 학습 안 함 → "마모 DR 로 강건화"는 성립 안 됨**(갱신할 가중치 없음). 강건성 레버는 학습이 아니라:
  ① **메시 충실도**(마모 인스턴스 스캔해 메시 갱신, model-free 쪽 수렴), ② **pose confidence 게이팅+재관측**, ③ **tracking·멀티뷰 누적**,
  ④ cuRobo 클리어런스·grasp 마진, ⑤ 심하면 **실물 reference 이미지로 model-free 폴백**(CAD-free 확장과 동일 경로).
  sim 의 마모 렌더는 **강건화(학습)가 아니라 측정(eval)** — 마모 정도 vs pose 오차를 재서 마진·폴백 임계값을 정하는 용도.
- **TensorRT 엔진 빌드**: 모델별 최초 1회 수 분 + 입력 해상도 고정.
- **SAM3 래핑**: apt ROS 노드 없음 → 자체 작성. ★ **ultralytics(AGPL-3.0) 의존 금지**(상업 배포 시 copyleft) → **Meta SAM3 원본 직접**. SAM License = 상업 OK·군사/무기 금지.
- **exemplar 품질**: SAM3 exemplar(물체 crop)가 부실하면 오검출/누락. sim↔real exemplar 도메인 차이도 점검(domain randomization 으로 완화).
- **카메라 혼동 금지**: 손목 eye-in-hand 를 nvblox 장애물 입력에 절대 넣지 말 것(TSDF 오염, HISTORY §12). 장애물=정적 cam 전담.
- **커스텀 마스크 메시지**: 외부 SAM3 래퍼 참고 시 출력이 비표준일 수 있음 → 우리 다운스트림(nvblox/FoundationPose)에 맞춰 `sensor_msgs/Image` 어댑터 필요.

---

## 5. 다음 액션
**파이프라인·D1~D7·D9 확정(2026-06-29).** 남은 미정: D8(실물 CAD-free 처리).
진입점: **M1 = ① SAM3 ROS 래핑(Meta 원본, exemplar 입력) → ② FoundationStereo/FoundationPose 설치·검증 → sim 카메라(RGB)에 붙여 depth/mask/pose 출력 확인.**
(현재 GUI nvblox 스택은 떠 있음 — M2에서 마스킹 통합 시 재사용.)

---

## 6. 실시간 동적 장애물 회피 — 옵션1: 연속 재계획 (TODO, 다음에)

> 배경: 현재 cuMotion+nvblox 는 **"한 번 계획 → 궤적 재생(open-loop)"**. Plan 누른 순간에만 ESDF 를
> 1회 조회(`update_esdf_on_request: True`)하고, 실행 중엔 충돌 재검사 없음 → **실행 중 장애물을 옮겨도
> 회피 못 하고 통과**(2026-06-28 sim 에서 확인, `HISTORY.md` §13 연계 관찰). 이건 정상 동작이고,
> "움직이는 중 갑툭튀 장애물 회피"는 별도 계층이 필요.

### 설계 평가 결론 (옵션1 vs 옵션2)
- **궁극 = 하이브리드**: 전역 재계획(옵션1, 지역최소 탈출) + 지역 MPC(옵션2, 고rate 매끄러운 추종). cuRobo 의도 설계.
- **옵션2(반응형 MPC)**: **추후 검토** — 상세는 아래 **§7**.
- **옵션1 먼저 하는 이유**: 효과 ~80% / 비용 ~20%. **sim==real 의 MoveIt/JTC 아키텍처를 안 깨고** 기존
  cuMotion·nvblox·`allow_nonzero_velocity_at_trajectory_end`(이음새 완화) 재사용. pick&place 상태머신(M3)도
  어차피 이 재계획 루프 위에 얹힘. 옵션1은 옵션2의 전제이기도 함.

### 옵션1 구현 스케치 (다음 작업)
- **루프**: `현재 joint state → 목표`를 **fresh ESDF 에 대해 주기적으로 재계획**(목표 ~2–10Hz), 기존 실행을
  preempt 하고 새 궤적으로 교체. cuMotion 이 짧은 구간은 빠름(전체 plan ~1.9s 였지만 잔여구간은 더 짧음).
- **실행 교체 안정화**: JTC 가 종단 잔여속도로 goal 거부 안 하도록 이미 `allow_nonzero_velocity_at_trajectory_end:
  true`(§HISTORY 11 함정②). 이음새 blending / 너무 잦은 preempt 의 떨림 주의.
- **속도 vs MoveIt 오버헤드**: 높은 재계획 rate 가 필요하면 MoveIt action 왕복 대신 **cuRobo MotionGen Python
  직접 호출** 고려(플러그인 경로는 1회 plan 오버헤드 큼). 1차 데모는 cumotion action 저rate 로 시작해도 됨.
- **트리거 정책**: 매 주기 무조건 재계획 vs "경로가 ESDF 와 충돌할 때만" 재계획(현 궤적 valid 체크 후). 후자가 효율적.
- **최소 안전판**: 재계획이 늦으면 최소한 **실행 중 충돌 감지 시 정지(abort)** — 현 nvblox 런치엔 octomap updater
  없어 MoveIt 실행 모니터에 살아있는 월드가 없음(`No 3D sensor plugin(s)`) → 정지조차 안 됨. 정지 경로도 같이 설계.
- **검증(sim)**: 실행 중 Isaac 에서 `--obstacle` 박스를 경로 위로 이동 → UR 이 경로를 다시 짜 우회하면 PASS.
  (A/B 데모 `nvblox_obstacle_demo.py` 를 "실행 중 장애물 이동" 시나리오로 확장.)

### 상태
⏳ **대기 — 다음에 구현.** 현재는 "Plan 시점 회피"가 정상임만 확인. 사용자: 옵션1 진행 의사 확정(2026-06-28).

---

## 7. (추후 검토) 옵션2 — 반응형 MPC (cuRobo MpcSolver)

> 옵션1(전역 재계획, §6) 위에 얹는 **지역 고rate 반응 계층**. 지금은 도입 안 함 — **필요해질 때 검토.**

### 무엇 / 언제
- cuRobo **MpcSolver**: 짧은 horizon 을 **~50–100Hz 로 연속 최적화**, 살아있는 ESDF 를 보며 관절 명령을 스트림.
  → **매끄럽고 즉각적인 동적 회피**(옵션1 의 preempt 떨림/지연 해소).
- **도입 트리거**: ① 진짜 연속 동적 환경(사람 협업·컨베이어·계속 움직이는 물체), 또는 ② 매끄러운 고rate 반응이
  spec 일 때. 정적·준정적 테이블 pick&place 엔 불필요(옵션1 로 충분).
- **RL/insertion(contact-rich) 단계에서 같이 검토**가 자연스러움 (`LEARNING.md` 연계).

### 비용 / 리스크 (도입 시)
- **단독 불가**: MPC 는 지역 최적 → 큰/오목 장애물 뒤에서 **지역최소에 갇힘**. **전역 재계획(옵션1)이 waypoint
  가이드로 반드시 동반**. 그래서 순서가 "옵션1 먼저, 옵션2는 그 위".
- **ROS 통합 부재**: isaac_ros_cumotion 에 MPC ROS 노드 없음 → **cuRobo Python API 로 노드 신설** 필요.
- **제어 경로 신설**: MoveIt/JTC 밖에서 **고rate 관절 속도/위치 스트리밍**(예: forward/velocity controller,
  실물은 UR servoj/speedj) → **sim==real 무손실이던 아키텍처에 sim2real 표면이 새로 생김**. 튜닝·안전(속도/충돌
  한계, watchdog) 별도 설계.
- 리소스: MPC GPU 상시 점유 — Isaac+perception+nvblox+cuMotion 와 동시 VRAM/rate 예산 확인.

### 상태
🅿️ **보류(추후 검토).** 옵션1 완료 + 동적성 요구가 실제로 생기면 재평가.
