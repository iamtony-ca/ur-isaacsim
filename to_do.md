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

```
카메라(스테레오 RGB only)  ── sim: Isaac 2-cam  /  real: D405/D455 스테레오 ──┐
                                                                              ▼
  FoundationStereo (RGB L/R → metric depth)        [depth foundation model]
        │
        ├─→ robot_segmenter (로봇 기하 마스킹) ─→ nvblox depth
        │
  Grounding DINO (텍스트 프롬프트 "the cube" → bbox)                 [open-vocab 검출]
        └─→ SAM2/SAM3 (→ 타깃 마스크)            [세그멘테이션 foundation model]
                 ├─→ nvblox (use_segmentation, human_with_static_tsdf): 타깃을 static ESDF 에서 제외
                 └─→ FoundationPose (RGB + foundation-depth + mask + mesh → 6D pose)  [pose foundation model]
                          │
                  /target/pose ─→ cuRobo known object(pre-grasp 클리어런스) + grasp 계산
                          │
  nvblox static ESDF (환경=장애물만) ─→ cuMotion(read_esdf_world, 장애물만 회피)
                          │
                  pick&place 상태머신(신규): HOME→PRE_GRASP→GRASP→CLOSE→LIFT→TRANSFER(회피)→PLACE→OPEN→복귀
```

**두 단계 마스킹(=NVIDIA 정석)**: 로봇=`cumotion_robot_segmenter`(기하), 타깃=이미지 마스크(`nvblox use_segmentation`).
남은 것 = 순수 환경 장애물 ESDF. 파지 시 타깃은 cuRobo attach, 들어올린 자리는 nvblox occupancy/TSDF decay 가 비움.

---

## 1. 컴포넌트 가용성 (실측, 2026-06-23)

| 역할 | 패키지 | 상태 |
|---|---|---|
| **depth foundation** | `isaac_ros_foundationstereo` (스테레오 RGB→metric depth) | apt 가용(release-4), **미설치** |
| depth 경량 대안 | `isaac_ros_ess` (DNN stereo disparity) | apt 가용, 미설치 |
| **세그멘테이션** | `isaac_ros_segment_anything2`(SAM2) / `..._segment_anything`(SAM) | apt 가용, 미설치 |
| 세그(실물 모델) | **SAM3** | **ComfyUI 안에만**(`testcomfyui/.../sam3*`) — ROS 노드 없음 → 래핑 필요 |
| **검출/프롬프트** | `isaac_ros_grounding_dino`(open-vocab), `isaac_ros_rtdetr`, `isaac_ros_detectnet` | apt 가용, 미설치 |
| **6D pose** | `isaac_ros_foundationpose` (+`-models-install`) | apt 가용, 미설치 |
| 로봇 마스킹 | `isaac_ros_cumotion_robot_segmenter` | ✅ 설치됨(현 nvblox 파이프라인에서 사용 중) |
| 장애물 맵 | `nvblox`(+segmentation/dynamics specialization) | ✅ 설치됨, `use_segmentation`/`human_with_static_tsdf` 지원 확인 |
| 플래너/실행 | cuMotion + `scaled_joint_trajectory_controller` | ✅ 설치·검증됨 |

설치 예정: `ros-jazzy-isaac-ros-foundationstereo(+models)`, `...-segment-anything2(+models)`,
`...-foundationpose(+models)`, (옵션) `...-grounding-dino`. release-4 레포는 이미 설정됨.

---

## 2. 결정이 필요한 부분 (추천 표기)

| # | 결정 | 옵션 / 추천 |
|---|---|---|
| **D1 depth model** | foundation depth 선택 | **FoundationStereo**(foundation·metric, 무거움) ▶추천 / ESS(경량 DNN stereo) / (비추)센서 depth. **둘 다 스테레오 RGB 쌍 필요** |
| **D2 seg model** | 타깃 마스크 모델 | **SAM2**(ROS 즉시) ▶v1 / **SAM3**(실물과 동일 모델, ComfyUI→ROS 래핑 필요) — 실물이 SAM3 확정이면 래핑 |
| **D3 타깃 지정** | 무엇을 잡을지 알려주는 법 | **Grounding DINO 텍스트 프롬프트**("the cube") ▶추천(open-vocab) / RT-DETR·DetectNet(클래스 학습) / 첫프레임 클릭 |
| **D4 카메라** | 스테레오 RGB 소스 | v1 **정적 오버헤드 스테레오**(nvblox+검출 통합, 단순) ▶ / 이후 eye-in-hand D405 로 grasp pose 보강 |
| **D5 파지 유지** | grasp 시 물체 잡기 | **접촉 물리 시도→안되면 fixed-joint attach** ▶추천(견고) / 처음부터 kinematic attach |
| **D6 추론 rate** | 느린 foundation model vs 빠른 제어 | pose 저rate 갱신 + ESDF 중rate + 제어 고rate **비동기 분리** ▶ |
| **D7 컨테이너 토폴로지** | GXF segfault 회피 | FoundationStereo/FoundationPose/nvblox(전부 GXF/NITROS)를 **별도 컨테이너/프로세스** 분리 ▶필수 |
| **D8 물체/장면** | 대상물·place·장애물 | 대상물=**메시 보유**(FoundationPose 필수) cube/known part; 장애물=**낮고 작게**(z>0.1 유지, pick↔place 경로 사이); place 고정 위치 |
| **D9 sim depth 처리** | Isaac 센서 depth 사용 여부 | **다운스트림은 센서 depth 미사용**(원칙). Isaac은 스테레오 RGB만 발행. (GT depth 는 FoundationStereo 정확도 평가용 비교에만) |

---

## 3. 빌드 마일스톤 (체크리스트)

### M0 — 장면/센서 준비 (Isaac)
- [ ] Isaac 에 **잡을 물체**(물리: rigid body+collider+mass+friction, **메시 보유**) 스폰 + place 타깃 영역
- [ ] **장애물 낮고 작게** 재구성(z 0.1~0.38 정도, pick↔place 경로 사이) — `--obstacle-pose/-size` 조정 또는 기본값 변경
- [ ] **스테레오 RGB 카메라**(좌/우, 알려진 baseline) 추가, intrinsics/extrinsics 발행. (정적 오버헤드, D4)
- [ ] **센서 depth 다운스트림 차단** — RGB만 사용(D9). (현 `--with-static-cam` depth 경로는 비교/디버그용으로만)
- [ ] domain randomization 훅(조명/재질/텍스처) 자리 마련

### M1 — foundation perception 단독 검증 (sim 이미지에서 출력 확인)
- [ ] 설치: foundationstereo(+models), segment-anything2(+models), foundationpose(+models), (옵션)grounding-dino
- [ ] **FoundationStereo**: 스테레오 RGB → metric depth 발행, GT depth 와 정량 비교(오차 맵)
- [ ] **Grounding DINO → SAM2**: 텍스트 프롬프트 → 타깃 마스크 발행(`/static_cam/mask/image`)
- [ ] **FoundationPose**: RGB+foundation-depth+mask+mesh → `/target/pose`(6D) — GT pose 와 오차 비교
- [ ] 각 노드 **별도 컨테이너**로 분리(D7), TensorRT 엔진 빌드(최초 수 분) 확인, VRAM 모니터

### M2 — nvblox 마스킹 통합 (타깃 제외 / 장애물만)
- [ ] nvblox depth 입력을 **FoundationStereo depth** 로 교체
- [ ] nvblox `mask/image` ← SAM 타깃 마스크 remap, `mapping_type: human_with_static_tsdf` + `use_segmentation: true`
- [ ] robot_segmenter(로봇) 유지 → 두 단계 마스킹
- [ ] **A/B 검증**: 마스크 on → 타깃이 ESDF에서 빠지고 cuMotion이 접근 가능 / off → 타깃이 장애물로 막힘
- [ ] 마스크 **dilation**(껍데기 voxel 방지), depth↔mask **프레임/해상도/시간 동기** 확인

### M3 — pick&place 상태머신 (통합, GT-free)
- [ ] 신규 `pick_place_demo.py`: `/target/pose` + MoveGroup(cuMotion) + gripper 액션
- [ ] 시퀀스: HOME→PRE_GRASP(위)→GRASP(하강)→CLOSE(파지, D5)→LIFT→TRANSFER(장애물 회피)→하강→OPEN→RETRACT→HOME
- [ ] cuRobo: 타깃을 known object(pre-grasp 클리어런스) → 파지 시 attach → place 후 detach
- [ ] **GUI 관찰**: 장애물 피해 pick&place 성공, decay 로 들린 자리 정리

### M4 — sim2real 하드닝
- [ ] RGB domain randomization 본격 적용, FoundationStereo depth 정확도/노이즈 sim↔real 분포 맞추기
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
- **리소스(RTX 5090 32GB)**: Isaac + FoundationStereo + SAM2 + FoundationPose + (G-DINO) + nvblox + cuMotion 동시 =
  매우 무거움. 추론 rate 낮추거나 노드 스테이징 필요. VRAM 상시 모니터.
- **마스크 dilation / 프레임·시간 동기**: 안 맞으면 ESDF에 타깃 "껍데기" 잔존 또는 엉뚱한 영역 제외.
- **FoundationPose 전제**: 대상물 **메시(CAD/USD) 필수** + 마스크·depth 입력 의존. 메시 없는 물체엔 다른 경로 필요.
- **TensorRT 엔진 빌드**: 모델별 최초 1회 수 분 + 입력 해상도 고정.
- **SAM3 래핑**(D2=SAM3 시): ComfyUI 런타임 → ROS 노드 브리지 작업 필요.

---

## 5. 다음 액션 (대기)
사용자 결정 필요: **D1(FoundationStereo vs ESS), D2(SAM2 vs SAM3 래핑), D3(타깃 지정 방식)**.
결정 후 진입점: **M1 = foundation perception 설치 + sim 카메라(RGB)에 붙여 depth/mask/pose 출력 검증**.
(현재 GUI nvblox 스택은 떠 있음 — 유지/정리 여부도 결정.)

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
