❯ 해당 프로젝트에서 usd 와 omnigraph에 대해서 이해를 돕기위한 예시 기반으로 한 개념 설명을 해줬줬으면 좋겠어.

프로젝트의 실제 코드를 근거로 설명하기 위해 Isaac 스크립트를 먼저 확인하겠습니다.

  Searched for 1 pattern, read 2 files

프로젝트 코드를 근거로, USD와 OmniGraph를 이 워크스페이스의 실제 파일로 설명하겠습니다.

---
1. USD — "장면(scene)을 기술하는 파일 포맷·데이터 모델"

USD(Universal Scene Description, Pixar 발)는 Isaac Sim의 월드 그 자체입니다. 로봇, 카메라, 조명, 장애물 박스 — 뷰포트에 보이는 모든 것이 USD로 표현됩니다. 핵심은 이 프로젝트에서 실제로 쓰는 몇 가지 개념뿐입니다.

1-1. Prim = 장면 속 "노드 하나" (파일시스템의 파일/폴더 같은 것)

Stage(=열려있는 장면)는 경로로 주소가 매겨진 Prim들의 트리입니다. 우리 코드의 실제 경로들:

/UR16e                         ← 로봇 루트 (Xform)
/UR16e/root_joint              ← ArticulationRoot (관절체의 뿌리)  ★함정5
/UR16e/wrist_3_link            ← 손목 링크
/UR16e/wrist_3_link/d405_camera← eye-in-hand 카메라 (UsdGeom.Camera)
/World/static_cam              ← 워크스페이스 내려다보는 정적 카메라
/World/demo_obstacle           ← 빨간 장애물 박스
/background                    ← Simple_Room 환경

이 경로가 그냥 문자열이 아니라 API의 1급 주소라는 게 중요합니다. ur16e_isaac_ros2.py:198:

CAM_PRIM = f"{ROBOT_PRIM}/{args.camera_parent}/d405_camera"   # "/UR16e/wrist_3_link/d405_camera"

카메라를 wrist_3_link 아래에 Prim으로 만들었기 때문에 → 손목이 움직이면 카메라도 따라 움직입니다(eye-in-hand). 반면 /World/static_cam은 로봇 트리 밖(/World)에 두었기 때문에 팔과 무관하게 고정됩니다. 이게 CLAUDE.md의 nvblox 핵심(정적 카메라로 매핑)이 코드 상에서는 그냥 Prim을 어느 부모 밑에 두느냐로 구현된다는 뜻입니다.

1-2. Type과 Schema(API) = "이 Prim이 무엇이고 무슨 능력을 갖는가"

Prim은 타입을 갖고, 그 위에 **Schema(API)**를 덧발라 능력을 부여합니다. demo_obstacle을 만드는 실제 코드(:422):

obs = UsdGeom.Cube.Define(stage_obj, OBS_PRIM)   # 타입: Cube (기하)
obs.GetSizeAttr().Set(1.0)                        # 속성(attribute): 크기
oxf = UsdGeom.Xformable(obs.GetPrim())            # Xformable 스키마: 변환 가능
oxf.AddTranslateOp().Set(Gf.Vec3d(...))           # 위치
oxf.AddScaleOp().Set(Gf.Vec3f(0.12, 0.5, 0.1))    # 스케일
obs.GetDisplayColorAttr().Set(... 빨강 ...)        # 색

- Attribute(속성): size, translate, displayColor — Prim이 들고 있는 값들.
- Schema/API: UsdGeom.Xformable(움직일 수 있음), UsdPhysics.RigidBodyAPI(물리 강체), UsdPhysics.ArticulationRootAPI(관절체의 뿌리), PhysxMimicJointAPI(따라 움직이는 관절).

build_ur16e_2f85.py가 하는 일이 바로 Schema를 손보는 것입니다. 2F-85 그리퍼 USD는 자기 자신이 독립 관절체(자체 ArticulationRootAPI)라서 팔에 그냥 붙이면 깨집니다. 그래서(:110):

if "PhysicsArticulationRootAPI" in prim.GetAppliedSchemas():
    prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)   # 그리퍼의 "독립 관절체" 딱지를 뗌

→ 그리퍼의 뿌리 딱지를 떼서, 팔 하나 + 그리퍼가 **단일 관절체(articulation)**가 되게 만듭니다. sanity 리포트가 ArticulationRoot=1 (expect 1)을 확인하는 이유가 이것입니다(:229).

1-3. Reference / Composition = "USD를 레고처럼 조립"

USD의 킬러 기능. 다른 USD 파일을 **참조(reference)**로 끌어와 합칩니다. build_ur16e_2f85.py가 세트2 에셋을 만드는 방식(:138):

root = out.DefinePrim("/UR16e", "Xform")
root.GetReferences().AddReference(UR16E_URL)          # 순정 UR16e를 참조
gripper_prim.GetReferences().AddReference(EE_FIXED)   # 고친 2F-85를 wrist_3 밑에 참조
fj = UsdPhysics.FixedJoint.Define(out, ...)           # 손목↔그리퍼 고정관절 하나 저작

세 개의 물건(팔 USD, 그리퍼 USD, 고정관절)을 조립만 해서 ur16e_with_2f85.usd를 굽습니다. 세트3는 여기에 커플링/카메라 마운트 메시를 visual reference로 더 얹은 것(:188)이고요. CLAUDE.md 함정7 "USD를 재베이크하라"가 바로 이 조립 스크립트를 다시 돌리라는 뜻입니다.

▎ 여기서 나오는 Flatten()(:99, :211)은 반대 방향 — 참조로 조립된 여러 레이어를 하나로 눌러 굽는(bake) 것입니다. 런타임에 참조가 깨지지 않도록 최종 에셋을 단일 파일로 만듭니다.

1-4. 좌표/단위 — 왜 코드에 회전행렬이 잔뜩인가

Prim의 위치는 Xformable의 변환 op로 정합니다. 카메라는 특히 골치인데, 세 좌표 관례가 다르기 때문입니다:
- URDF/ROS 광학 프레임: z = 정면
- USD 카메라: -z가 정면

그래서 :224에서 _rpy(np.pi, 0, 0)(ROS optical → USD camera)로 뒤집습니다. 이 변환을 URDF의 <origin>과 똑같이 맞춰야 RViz와 Isaac의 카메라가 일치합니다(코드 주석이 "Change these numbers HERE and in realsense_d405_macro together"라고 경고하는 이유).

---
2. OmniGraph — "매 시뮬 스텝마다 실행되는 비주얼 데이터플로우 프로그램"

USD가 정적인 장면이라면, OmniGraph(OG)는 그 장면 위에서 매 프레임 흐르는 로직입니다. 노드를 선으로 연결한 데이터플로우 그래프이고, GUI에서 마우스로 그릴 수도 있지만 우리는 파이썬으로 저작합니다(og.Controller.edit).

이 프로젝트에서 OmniGraph의 유일한 임무: Isaac ↔ ros2_control 사이에서 관절을 주고받는 ROS2 브리지.

2-1. 노드 · 연결 · 실행순서 — 실제 그래프

ur16e_isaac_ros2.py:136의 /ActionGraph를 그림으로 풀면:

OnPlaybackTick ─(매 렌더 스텝마다 tick)─┬─▶ PublishJointState ──/isaac_joint_states──▶ (ROS: ros2_control이 읽음)
                                        ├─▶ SubscribeJointState ◀─/isaac_joint_commands── (ROS: MoveIt가 명령)
                                        │        └─(positionCommand)─▶ ArticulationController ─▶ 관절 구동
                                        └─▶ PublishClock ──────────────/clock──────────▶ (ROS: use_sim_time)
ReadSimTime ─(simulationTime)─▶ PublishJointState / PublishClock 의 timeStamp
Context(ROS2Context) ─(context)─▶ 모든 ROS 노드 (DDS 도메인 등)

세 종류의 연결선이 있다는 게 핵심입니다:

(a) 실행 흐름 (execution / execIn·tick) — "언제 실행할지". OnPlaybackTick이 매 스텝 발화하고, 그게 execIn으로 연결된 노드들을 깨웁니다(:149):
("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
이게 없으면 노드가 존재해도 아무 일도 안 일어납니다. CLAUDE.md 함정4의 "clock 없으면 컨트롤러가 timeout"이 여기서 나옵니다 — PublishClock이 tick으로 굴러야 /clock이 나갑니다.

(b) 데이터 흐름 (data) — "무슨 값을 넘길지"(:158):
("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
ROS에서 받은 목표 관절각을 실제 로봇 구동 노드로 넘기는 배선.

(c) 설정값 (SET_VALUES) — 노드의 파라미터(:163):
("ArticulationController.inputs:robotPath", ARTICULATION_ROOT),  # 어느 Prim을 구동?
("PublishJointState.inputs:topicName", "isaac_joint_states"),    # 어느 토픽으로?

2-2. OmniGraph가 USD를 어떻게 만지는가 — 둘의 접점

여기가 USD와 OG가 만나는 지점입니다. OG 노드는 결국 USD Prim을 읽고 씁니다:

("ArticulationController.inputs:robotPath", ARTICULATION_ROOT)          # = "/UR16e/root_joint"
("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(ARTICULATION_ROOT)])

- ArticulationController 노드는 매 스텝 /UR16e/root_joint(USD의 관절체 Prim)의 관절각을 씁니다.
- PublishJointState 노드는 같은 Prim에서 현재 관절각을 읽어 /isaac_joint_states로 냅니다.

즉 USD = 상태(관절이 지금 어디), OmniGraph = 그 상태를 매 스텝 읽고 쓰는 배관. usdrt.Sdf.Path는 성능용 런타임 USD(usdrt)로 Prim을 가리키는 것이고요.

이래서 함정5(articulation root = /UR16e/root_joint)가 치명적입니다 — 이 경로 하나가 틀리면 OG의 두 노드 모두 엉뚱한 Prim을 읽고 써서 로봇이 안 움직이거나 /joint_states가 안 나갑니다.

2-3. 여러 그래프 = 관심사 분리

한 스크립트가 독립 그래프 3개를 만듭니다. 각각 별도 graph_path:

┌─────────────────┬───────────────────┬──────────────────────────────────────────┐
│     그래프      │       언제        │                 하는 일                  │
├─────────────────┼───────────────────┼──────────────────────────────────────────┤
│ /ActionGraph    │ 항상              │ 관절 pub/sub + clock (제어의 심장)       │
├─────────────────┼───────────────────┼──────────────────────────────────────────┤
│ /CameraGraph    │ --with-camera     │ eye-in-hand D405 → RGB/depth/points 발행 │
├─────────────────┼───────────────────┼──────────────────────────────────────────┤
│ /StaticCamGraph │ --with-static-cam │ 정적 카메라 → depth 발행 (nvblox용)      │
└─────────────────┴───────────────────┴──────────────────────────────────────────┘

카메라 그래프의 구조(:268)도 똑같은 패턴입니다:
OnTick ─▶ RenderProduct(카메라 Prim을 렌더) ─▶ ROS2CameraHelper(RGB) ──/camera/color/image_raw─▶
                                            ├─▶ ROS2CameraHelper(Depth) ─/camera/depth/...─▶
                                            └─▶ ROS2CameraInfoHelper ────/camera/.../camera_info─▶
RenderProduct.inputs:cameraPrim이 아까 USD로 만든 /UR16e/wrist_3_link/d405_camera Prim을 가리킵니다 — 다시 USD와 OG의 접점. 그리고 토픽명을 실제 realsense2_camera 드라이버와 똑같이 맞췄기 때문에(/camera/color/image_raw 등), sim과 real이 인식 스택 입장에서 구별이 안 됩니다. 이게 이 프로젝트 sim2real 전략의 핵심이고, OmniGraph 노드의 topicName 설정 한 줄로 달성됩니다.

---
3. 한 장 요약 — 이 프로젝트의 정신 모델

┌─────────────────────────── USD (정적 장면 = "명사") ───────────────────────────┐
│  /UR16e/root_joint (관절체)   /World/static_cam (카메라)   /World/demo_obstacle │
│  · Prim 트리 + Type + Schema(API) + Attribute                                   │
│  · Reference로 조립(build_ur16e_2f85.py), Flatten으로 굽기                      │
└───────────────────────────────────┬────────────────────────────────────────────┘
                                     │ OG 노드가 Prim을 매 스텝 read/write
┌───────────────────────────────────┴────────────────────────────────────────────┐
│              OmniGraph (동적 로직 = "동사", 매 tick 실행)                        │
│  OnPlaybackTick ▶ PublishJointState / SubscribeJointState / ArticulationController│
│  · 실행선(execIn) + 데이터선 + 설정값(SET_VALUES)                                │
│  · ROS2 토픽 = sim↔real 공통 인터페이스                                          │
└──────────────────────────────────────────────────────────────────────────────────┘
                                     │ /isaac_joint_{states,commands}, /clock
                          ros2_control (topic_based HW) ↔ MoveIt

- USD를 바꾼다 = 로봇 구조/에셋을 바꾼다 → build_ur16e_2f85.py 재실행 → 새 .usd (함정7).
- OmniGraph를 바꾼다 = 무엇을 어느 토픽으로 주고받을지 바꾼다 → ur16e_isaac_ros2.py의 og.Controller.edit 수정.
- 둘의 접점은 항상 Prim 경로(ARTICULATION_ROOT, CAM_PRIM) — 여기가 틀리면 조용히 안 움직입니다.

---
더 깊게 보고 싶은 방향이 있으면 골라주세요: (a) og.Controller.edit의 CREATE_NODES/CONNECT/SET_VALUES 문법을 한 줄씩, (b) USD Composition(reference/sublayer/variant/Flatten)을 그리퍼 조립 예제로 더 깊게, (c) 카메라 3좌표 관례(URDF↔ROS optical↔USD) 변환을 수식으로.

✻ Worked for 1m 49s

※ recap: I explained USD and OmniGraph conceptually, grounded in this project's actual Isaac scripts. Next action is optional: I offered to go deeper on OmniGraph node syntax, USD composition, or the camera coordinate conventions if you pick one. (disable recaps in /config)

❯ Flatten 은 뭐야? 그리고 설명해준 내용들을 실제 gui 기반으로 설명해줄수도 있나? 너가 gui를 실행해주고, 실제로 확인하는 방법도 알려주면, 실행된 gui에서 내가 직접 확인하면 더 이해가 잘 될거 같아아.

좋은 방향이에요. 먼저 Flatten을 개념으로 답하고, 바로 GUI를 띄워서 직접 확인할 수 있게 하겠습니다. 우선 디스플레이 환경을 확인하고 시뮬을 실행하겠습니다.

  Ran 2 shell commands

DISPLAY=:0, RTX 5090 여유(1.3/32GB), 실행 중인 sim 없음 — GUI를 띄우겠습니다. 제가 설명한 Prim들이 Stage 트리에 다 보이도록 카메라·정적카메라·장애물 플래그를 켜서 실행합니다.

  Ran 1 shell command

시작하는 동안(Isaac은 부팅에 보통 1~2분, 셰이더 컴파일 포함) 로그를 지켜보겠습니다. 그사이 Flatten을 설명할게요.

  Ran 1 shell command

I'll wait for the startup banner to appear using a monitor loop.

  Ran 1 shell command

GUI가 떴습니다. ✅ 제가 설명했던 Prim들이 실제로 다 생성됐습니다 — /UR16e/root_joint, /UR16e/wrist_3_link/d405_camera, /World/static_cam, /World/demo_obstacle, 그리고 그래프 3개(/ActionGraph·/CameraGraph·/StaticCamGraph). 화면에는 팔이 home 자세(위로)로 서 있고, 옆에 빨간 장애물 박스, 파란 정적 카메라가 보일 겁니다.

OmniGraph가 실제로 돌고 있다는 걸 토픽으로 확인해 드리겠습니다.

  Ran 1 shell command

토픽이 전부 살아있고 /isaac_joint_states가 ~69 Hz로 발행되고 있습니다 — 이게 /ActionGraph의 OnPlaybackTick → PublishJointState 노드가 매 스텝 도는 증거입니다. 이제 화면에서 직접 확인할 준비가 됐습니다.

---
Part 1. Flatten 이 뭐야?

한 줄 정의

Flatten = 참조(reference)·레이어(layer)로 여러 조각으로 나뉜 USD를, 하나의 자기완결적(self-contained) 파일로 눌러 굽는 것.

왜 필요한가 — 요리 비유

USD는 레시피 참조로 조립됩니다:

ur16e_with_2f85.usd (레시피)
 ├─ "UR16e는 저 아마존 URL에서 가져와"     ← AddReference(UR16E_URL)
 ├─ "그리퍼는 robotiq_2f85_fixed.usda에서" ← AddReference(EE_FIXED)
 └─ "손목↔그리퍼 고정관절은 내가 직접 저작" ← FixedJoint.Define(...)

이 상태(composed, 조립됨)는 편하지만 문제가 있습니다:
- 외부 URL/파일이 사라지거나 바뀌면 깨집니다.
- 런타임에 매번 여러 파일을 열어 합성(composition)해야 합니다.

Flatten은 이 참조들을 전부 따라가 실제 지오메트리·관절·속성을 한 파일 안에 복사해 박아 넣습니다. 결과물은 URL이 죽어도 혼자 열립니다.

이 프로젝트의 실제 두 번의 Flatten

build_ur16e_2f85.py에서 정확히 두 번 씁니다:

① EE 고칠 때 (:99)
work_layer = src.Flatten()      # 원본 2F-85 USD의 모든 참조를 눌러서
work = Usd.Stage.Open(work_layer)  # 편집 가능한 단일 스테이지로
→ 눌러야 그 위에서 ArticulationRootAPI 제거·instanceable 해제 같은 수술을 안정적으로 할 수 있습니다(참조 너머의 Prim은 직접 수정 어려움).

② 최종 에셋 구울 때 (:210)
flat_layer = out.Flatten()      # 팔+그리퍼+고정관절 조립본을 하나로
flat.GetRootLayer().Export(str(COMPOSED))   # → ur16e_with_2f85.usd (12.7 MB)
→ 그래서 결과 파일이 12.7 MB입니다(ls에서 확인). 참조만이면 몇 KB겠지만, Flatten이 UR16e·그리퍼 메시를 전부 복사해 넣었기 때문에 큽니다. 런타임 ur16e_isaac_ros2.py는 이 단일 파일 하나만 참조하면 됩니다(함정6: 절대경로로).

Instanceable / resetXformStack 을 굳이 Flatten 후에 하는 이유

- Instanceable: USD가 "이 서브트리는 여러 곳에서 공유하는 읽기전용 인스턴스"라고 표시한 것. 공유본은 개별 수정이 안 됩니다. Flatten + SetInstanceable(False)로 풀어야 우리 것으로 고칠 수 있습니다(:104, :214).

▎ 정리: Composition(reference)은 조립·재사용에 좋고, Flatten은 배포·수정·안정성에 좋다. build 스크립트는 "참조로 조립 → Flatten으로 굽기"를 하고, 런타임은 구워진 결과만 쓴다.

---
Part 2. GUI에서 직접 확인하기

지금 화면(DISPLAY :0)에 Isaac Sim 창이 떠 있습니다. 아래를 순서대로 눌러보면 위에서 글로 설명한 게 전부 눈에 보입니다.

① USD Prim 트리 = Stage 패널 (오른쪽 위)

화면 오른쪽 위 Stage 탭. 여기가 §1에서 말한 Prim 트리입니다. 펼치면(▶):

World
├─ UR16e                     ← 로봇 루트 (Xform)
│  ├─ root_joint             ← ★ 여기가 articulation root
│  ├─ base_link ... wrist_3_link
│  │  └─ d405_camera         ← 손목에 매달린 카메라 (자식이라 팔 따라 움직임)
│  │     └─ body             ← 42x42x23mm 회색 박스 (visual)
├─ static_cam                ← 팔 밖에 있음 → 고정
│  └─ body                   (파란 박스)
├─ demo_obstacle             ← 빨간 장애물
├─ ActionGraph               ← OmniGraph 도 Prim 이다!
├─ CameraGraph
└─ StaticCamGraph

직접 확인 포인트:
- d405_camera가 wrist_3_link 아래, static_cam은 World 바로 아래(팔 밖) — §1-1에서 말한 "부모가 eye-in-hand냐 고정이냐를 결정"을 트리 위치로 눈으로 확인.
- 그래프도 Prim으로 트리에 있음 — OmniGraph가 USD 위에 얹힌다는 걸 보여줍니다.

② Prim의 Type·Schema·Attribute = Property 패널 (오른쪽 아래)

Stage에서 demo_obstacle을 클릭 → 오른쪽 아래 Property 패널을 보세요:
- 맨 위에 타입 Cube
- Transform 섹션의 Translate (0.5, 0.1, 0.6), Scale (0.12, 0.5, 0.1) ← 코드 :426의 값 그대로
- Display Color 빨강

→ §1-2에서 코드로 본 UsdGeom.Cube.Define + AddTranslateOp + displayColor가 패널의 필드로 1:1 대응합니다. 여기서 Translate 숫자를 바꾸면 박스가 즉시 움직입니다(직접 해보세요).

이번엔 root_joint을 클릭 → Property에서 Physics → Articulation Root 스키마가 붙어있는 걸 확인. §1-2의 "Schema = 능력"이 이겁니다. (함정5의 그 경로.)

③ OmniGraph = Action Graph 에디터 로 그림 보기

상단 메뉴 Window → Graph Editors → Action Graph 를 엽니다. 에디터 안에서 Edit Action Graph 드롭다운(또는 폴더 아이콘) → /ActionGraph 선택.

그러면 §2에서 제가 ASCII로 그렸던 그래프가 진짜 노드 박스+연결선으로 나옵니다:

[OnPlaybackTick] ──▶ [PublishJointState]   [SubscribeJointState] ──▶ [ArticulationController]
                 ──▶ [PublishClock]        [ReadSimTime]  [ROS2Context]

직접 확인 포인트:
- 노드 사이 연결선 색이 두 종류 — 실행선(execIn/tick)과 데이터선. §2-1의 (a)(b)를 눈으로 구분.
- PublishJointState 노드 클릭 → Property 패널에 topicName = isaac_joint_states, targetPrim = /UR16e/root_joint ← §2-2에서 말한 "OG 노드가 USD Prim을 가리킨다"의 그 필드. 여기 topicName을 바꾸면 발행 토픽이 바뀝니다.
- 드롭다운에서 /CameraGraph, /StaticCamGraph 로 바꿔가며 카메라 그래프도 확인.

④ Composition / Flatten = Layer 패널 로 보기

상단 Window → Layers (또는 오른쪽 위 Layer 탭). 여기가 참조/레이어 구조를 보여줍니다.
- 지금 이 씬은 런타임에 add_reference_to_stage로 배경을, create_prim(usd_path=...)로 UR16e를 참조해 올린 상태 → Layer 패널에서 root layer 아래에 참조된 payload/reference들이 계층으로 보입니다.
- demo_obstacle처럼 런타임에 직접 저작한 Prim은 root(session) layer에 노란색으로 표시 — "참조로 온 것 vs 내가 만든 것"이 색으로 구분됩니다.
- Flatten의 의미: 이 Layer 패널에 쌓인 여러 층을 한 장으로 병합하는 게 Flatten. GUI에서도 레이어 우클릭 → Flatten Layers / Merge Down 으로 직접 해볼 수 있고, File → Save Flattened As...로 §1의 build 스크립트가 코드로 한 것(Export)을 메뉴로 재현할 수 있습니다.

⑤ 카메라가 실제로 보는 것 = Viewport 카메라 전환

Viewport 왼쪽 위 카메라 아이콘(👁 Perspective 드롭다운) → Cameras → d405_camera 선택. 뷰가 손목 시점으로 바뀝니다(팔을 움직이면 따라감). static_cam으로 바꾸면 워크스페이스를 위에서 내려다보는 고정 시점 — CLAUDE.md의 "정적 카메라로 nvblox 매핑" 이유가 시점 하나로 이해됩니다.

---
정리 — 글 ↔ GUI 대응표

┌───────────────────────┬───────────────────────────────────┬─────────────────────────────────────────────────┐
│       개념(글)        │          GUI에서 보는 곳          │                    확인 액션                    │
├───────────────────────┼───────────────────────────────────┼─────────────────────────────────────────────────┤
│ Prim 트리             │ Stage 패널                        │ d405_camera가 wrist_3 밑, static_cam은 World 밑 │
├───────────────────────┼───────────────────────────────────┼─────────────────────────────────────────────────┤
│ Type·Schema·Attribute │ Property 패널                     │ demo_obstacle 클릭 → Cube/Transform/Color       │
├───────────────────────┼───────────────────────────────────┼─────────────────────────────────────────────────┤
│ ArticulationRoot      │ Property                          │ root_joint → Articulation Root 스키마           │
├───────────────────────┼───────────────────────────────────┼─────────────────────────────────────────────────┤
│ OmniGraph 노드·연결   │ Window→Graph Editors→Action Graph │ /ActionGraph 열기, 노드 클릭해 topicName        │
├───────────────────────┼───────────────────────────────────┼─────────────────────────────────────────────────┤
│ Composition / Flatten │ Window→Layers                     │ 참조 계층 확인, 우클릭 Flatten/Merge            │
├───────────────────────┼───────────────────────────────────┼─────────────────────────────────────────────────┤
│ eye-in-hand vs 정적   │ Viewport 카메라 드롭다운          │ d405_camera ↔ static_cam 시점 전환              │
└───────────────────────┴───────────────────────────────────┴─────────────────────────────────────────────────┘

시뮬은 계속 켜둘게요(PID 223133, 로그는 스크래치패드 isaac_gui.log). 직접 눌러보다가 특정 패널에서 막히거나 "이 노드는 왜 이렇게 연결됐냐" 같은 게 생기면 말해주세요. 다 보고 나서 끄고 싶으면 pkill -f ur16e_isaac_ros2.py 하시면 됩니다.

✻ Baked for 2m 28s

❯ Window → Graph Editors → Action Graph 이 경로가 잘 못 된거 같아. 5.1.0의 문서를 검토해서 경로를 다시 알려줄래?

확인해보겠습니다. 로컬 설치된 5.1.0의 메뉴 등록과 공식 문서를 함께 대조하겠습니다.

  Searched for 1 pattern

로컬 설치에 "Visual Scripting"(40회)과 "Graph Editors"(3회)가 둘 다 있네요. 실제 메뉴 등록 코드를 확인하겠습니다.

  Searched for 1 pattern

확인됐습니다. 제가 틀렸고, 당신 말이 맞습니다. 로컬 설치의 실제 등록 소스에서 확정된 경로입니다.

action_graph_extension.py:50 (설치본 omni.graph.window.action-1.50.2, Isaac Sim 5.1.0):
WINDOW_NAME = "Action Graph"
MENU_PATH   = "Window/Visual Scripting/Action Graph"

정정된 경로

Window → Visual Scripting → Action Graph ✅

(제가 앞 메시지에서 쓴 Window → Graph Editors → Action Graph는 틀렸습니다. "Graph Editors"는 이 설치본에 3번밖에 안 나오고, Action Graph 창을 여는 메뉴가 아닙니다. Action Graph 메뉴 등록은 위 소스대로 Visual Scripting 아래 하나뿐입니다.)

이후 절차는 동일

1. Window → Visual Scripting → Action Graph 로 에디터 창을 엽니다.
2. 창 상단의 그래프 선택 드롭다운(Edit Action Graph)에서 /ActionGraph 를 고릅니다.
3. 노드 박스(OnPlaybackTick, PublishJointState, SubscribeJointState, ArticulationController, PublishClock, ReadSimTime, ROS2Context)와 연결선이 나옵니다.
4. /CameraGraph, /StaticCamGraph 로 바꿔 카메라 그래프도 확인.

참고로, 같은 메뉴 안에서 새 그래프를 만들 때 쓰는 항목(Create → Visual Scripting → Action Graph)도 있는데, 우리는 스크립트가 이미 만든 그래프를 여는 것이므로 위의 Window → Visual Scripting → Action Graph 창을 쓰는 게 맞습니다.

다른 메뉴 경로도 의심되는 게 있으면 같은 방식으로 설치 소스에서 확인해 드릴게요.

✻ Worked for 30s

※ recap: Explaining USD and OmniGraph concepts using this project's actual code, then via the live Isaac Sim GUI (still running, PID 223133). Corrected the menu path: Action Graph opens at Window → Visual Scripting → Action Graph. Next, you inspect the panels in the GUI and ask about anything unclear. (disable recaps in /config)

❯  Composition / Flatten = Layer 패널 로 보기 부분에 대해 더 자세히 설명해줄래?

좋은 질문이에요. 이 부분은 USD에서 가장 헷갈리는 지점이라 정확히 짚겠습니다. 먼저 이 실행 중인 씬의 레이어 구조가 실제로 어떻게 생겼는지와 GUI 메뉴 경로를 설치본에서 확인해서, 또 틀린 안내를 하지 않도록 하겠습니다.

  Searched for 2 patterns, read 1 file, listed 1 directory

확정: Layer 패널은 Window → Layer 로 엽니다(설치본 window_menu.py:64, top-level 항목 "Layer"). Flatten 관련 실제 컨텍스트 메뉴 항목도 확인됐습니다 — Flatten All Layers, Flatten Sublayers, Merge Down.

여기서 매우 중요한 정정부터 하고 들어가야 합니다. 제가 앞서 "Layer 패널에서 참조 계층이 보인다"고 뭉뚱그린 건 부정확했습니다. 참조(reference)와 서브레이어(sublayer)는 다른 것이고, Layer 패널에는 원칙적으로 서브레이어만 보입니다. 이 씬은 대부분 참조로 만들어져서, Layer 패널만 봐서는 오히려 단순하게 보입니다. 이걸 정확히 풀어드릴게요.

먼저 개념을 100% 재현 가능한 형태로 보여드리기 위해, 디스크의 실제 에셋으로 composition→flatten을 직접 출력해보겠습니다.

  Listed 1 directory

/isaac-sim/python.sh 로 앱을 띄우지 않고 pxr(USD 라이브러리)만 써서, 참조·서브레이어·Flatten의 차이를 실제로 출력해보겠습니다(가볍고 GPU 안 씀).

  Searched for 1 pattern, listed 1 directory, ran 4 shell commands

거의 다 나왔는데 제 grep 필터가 대괄호 줄을 지웠네요. 깨끗하게 다시 출력하겠습니다.

  Ran 1 shell command

완벽하게 재현됐습니다. 이 실제 출력을 근거로 깊이 설명하겠습니다.

---
Composition / Flatten — Layer 패널 심화

0. 핵심 오해부터 정리: 서브레이어 ≠ 참조

제가 이전에 "Layer 패널에서 참조 계층이 보인다"고 한 건 틀렸습니다. USD 합성(composition)에는 여러 종류의 "arc"가 있고, 그중 Layer 패널에 나오는 건 서브레이어(sublayer) 하나뿐입니다. 참조(reference)는 완전히 다른 arc라서 Layer 패널에 안 나옵니다. 방금 실제 USD로 이 둘을 갈라서 출력한 게 위 결과입니다.

┌────────────┬─────────────────────────────────────────┬──────────────────────────────────────────────────────────────┐
│            │          Sublayer (서브레이어)          │                       Reference (참조)                       │
├────────────┼─────────────────────────────────────────┼──────────────────────────────────────────────────────────────┤
│ 비유       │ 포토샵 레이어 (같은 캔버스에 겹쳐 쌓기) │ 심볼릭 링크 / 인스턴스 (남의 트리를 끌어옴)                  │
├────────────┼─────────────────────────────────────────┼──────────────────────────────────────────────────────────────┤
│ 대상       │ 스테이지 전체를 같은 경로에 겹침        │ 특정 Prim 아래에 다른 파일을 꽂음                            │
├────────────┼─────────────────────────────────────────┼──────────────────────────────────────────────────────────────┤
│ Layer 패널 │ 보임 (Root Layer 밑에 들여쓰기)         │ 안 보임 (Prim에 배지로 표시)                                 │
├────────────┼─────────────────────────────────────────┼──────────────────────────────────────────────────────────────┤
│ 우리 코드  │ (이 씬엔 없음)                          │ AddReference(UR16E_URL), add_reference_to_stage(simple_room) │
├────────────┼─────────────────────────────────────────┼──────────────────────────────────────────────────────────────┤
│ 위 데모    │ §1의 sublayer.usd                       │ §2의 /Robot → asset.usd                                      │
└────────────┴─────────────────────────────────────────┴──────────────────────────────────────────────────────────────┘

1. 스테이지 = "opinion들을 합성한 결과"

USD의 밑바닥 모델: 각 **레이어(layer, =.usd 파일 또는 익명 메모리 레이어)**는 어떤 Prim에 대한 **의견(opinion / spec)**을 담습니다. 스테이지는 이 의견들을 규칙(강한 레이어가 약한 레이어를 덮음)에 따라 합성한 최종 결과입니다.

위 데모 §3이 이걸 정확히 보여줍니다 — 한 Prim /Robot이 세 레이어의 의견을 동시에 받습니다:
opinion from: root.usd        ← 내가 로컬로 쓴 것 (가장 강함)
opinion from: sublayer.usd    ← 서브레이어 (약함)
opinion from: asset.usd       ← 참조로 끌어온 것 (더 약함)
그 결과 §4처럼 /Robot의 자식으로 Wrist(참조에서 온 것)가 실제로 보입니다. 아무 레이어도 "Wrist"를 직접 안 썼는데 참조를 통해 합성돼 나타난 것 — 이게 composition입니다.

2. 그럼 지금 실행 중인 씬의 Layer 패널엔 뭐가 보이나

Window → Layer 를 열면(위 데모 §1과 같은 구조):
Root Layer            ← 익명 레이어. static_cam, demo_obstacle, /ActionGraph 등
                        스크립트가 런타임에 직접 쓴 의견이 전부 여기 담김 (= edit target)
  (서브레이어 없음)     ← 스크립트가 add sublayer 를 안 했으므로 비어 보임
Session Layer         ← 뷰포트 카메라·선택 등 세션 전용 (데모 §1의 [0] root-session)

중요: UR16e와 배경(simple_room)은 여기 안 보입니다. 둘은 서브레이어가 아니라 참조니까요. 그래서 Layer 패널만 보면 "어? UR16e는 어디?" 하고 허전한데, 정상입니다. 리치한 구조는 참조(Prim 쪽)에 있는 겁니다.

3. 그럼 참조는 GUI 어디서 보나

두 군데:
1. Stage 패널: /UR16e 옆에 작은 arc 배지 아이콘(화살표/체인 모양)이 붙어 있습니다 — "이 Prim은 참조로 채워졌다"는 표시.
2. Property 패널: /UR16e 선택 → 아래로 스크롤하면 References(또는 Payloads) 섹션에 ur16e.usd 파일 경로가 나옵니다. 이게 "UR16e는 저 USD에서 온다"를 눈으로 보는 곳입니다. demo_obstacle을 선택하면 이 섹션이 없습니다 — 얘는 참조가 아니라 Root Layer에 직접 쓴 로컬 Prim이니까(색도 다르게, 보통 "authored" 표시).

4. Layer 패널을 만질 때 알아야 할 것

- Edit target (편집 대상): 여러 레이어 중 지금 내가 쓰는 값이 어느 레이어에 기록되는가. 굵게/하이라이트된 레이어가 그것(데모 §1의 edit-target=True인 root.usd). 지금 씬에선 Root Layer가 edit target이라, GUI에서 obstacle을 움직이면 그 델타가 Root Layer에 쌓입니다.
- Delta 표시: 어떤 레이어가 특정 Prim에 의견을 갖고 있으면 그 레이어 행에 표시가 뜹니다.
- Mute / Lock: 레이어를 잠시 끄거나(뮤트) 잠글 수 있음 — 디버깅용.

5. Flatten — Layer 패널에서 실제로 하는 세 가지

컨텍스트 메뉴(레이어 우클릭)에 실제로 있는 항목(설치본에서 확인): Flatten Sublayers, Flatten All Layers, Merge Down.

- Flatten All Layers / Flatten Sublayers: 서브레이어 스택을 눌러 Root Layer 하나로 합침. → 지금 이 씬에선 거의 no-op (서브레이어가 없으니까). 좋은 학습 포인트입니다: "Flatten Layers"는 서브레이어를 접는 것이지 참조를 굽는 게 아님.
- 참조까지 굽는 진짜 Flatten = 데모 §5의 stage.Flatten(), 그리고 GUI에선 File → Save Flattened As.... 이게 build 스크립트가 코드로 한 것과 동일합니다:

flat_layer = out.Flatten()          # build_ur16e_2f85.py:211  ← 참조+서브레이어 전부 인라인
flat.GetRootLayer().Export(...)     # → ur16e_with_2f85.usd (12.7 MB)

데모 §5가 그 효과를 정확히 증명합니다:
flatten 후: subLayerPaths = []            ← 서브레이어 스택 사라짐
            /Robot references = 없음       ← 참조 arc 사라짐
            Wrist 가 로컬 spec 으로 인라인  ← 참조 너머 있던 게 복사돼 박힘
→ 참조로 "가리키던" 것이 실제 데이터로 복사되어 단일 레이어에 박힌다. 그래서 결과가 12.7 MB(§Part1의 그 크기)이고, URL이 죽어도 혼자 열립니다.

6. 지금 GUI에서 직접 해볼 실험 (순서대로)

1. Window → Layer 열기 → Root Layer + Session Layer만, 서브레이어 없음 확인. (§2)
2. Stage에서 UR16e 선택 → 옆 arc 배지 확인 → Property 패널의 References 섹션에서 ur16e.usd 경로 확인. "참조는 Prim에 있지 Layer 패널에 없다"를 체감. (§3)
3. demo_obstacle 선택 → Property에 References 섹션이 없음 확인 → 얘는 Root Layer 로컬 Prim.
4. Viewport에서 obstacle을 드래그(또는 Property의 Translate 수정) → Layer 패널의 Root Layer 행이 dirty(*) 로 바뀜 → 델타가 edit target(Root Layer)에 기록됐다는 증거. (§4)
5. (선택) File → Save Flattened As... 로 .usda로 저장 후 텍스트 에디터로 열기 → UR16e·그리퍼 지오메트리가 전부 인라인된 걸 눈으로 확인 → 이게 데모 §5 / build 스크립트 Export의 GUI판. (§5)

---
한 줄 요약

- 서브레이어 = 스테이지 전체를 겹쳐 쌓기 → Layer 패널에 보임.
- 참조 = 특정 Prim 아래 남의 USD를 꽂기 → Prim의 배지/Property에 보임, Layer 패널엔 안 보임.
- 이 씬의 리치함은 대부분 참조라서 Layer 패널은 단출하다(정상).
- Flatten = 이 모든 arc(참조+서브레이어)를 눌러 실제 데이터로 인라인한 단일 자기완결 레이어를 만드는 것. build 스크립트가 코드로, GUI가 Save Flattened As로 하는 그것.

방금 만든 재현 스크립트는 스크래치패드 usd_comp.py에 있으니 숫자 바꿔가며 직접 돌려봐도 됩니다(PYTHONPATH/LD_LIBRARY_PATH는 위 명령 그대로). 이 중 특정 단계에서 GUI 화면이 설명과 다르게 나오면 캡처해서 말해주세요 — 그 위젯을 설치 소스에서 다시 짚어드리겠습니다.

✻ Churned for 4m 17s

※ recap: Explaining USD and OmniGraph concepts using this project's live Isaac Sim GUI (PID 223133, still running). Just finished the deep dive on Composition/Flatten and the Layer panel. Next: you try the GUI experiments, or tell me what to explain further. (disable recaps in /config)

