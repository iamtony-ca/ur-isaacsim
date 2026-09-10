# GR00T N1.7 파이프라인 설계 (카메라 2대)

> 정본은 `plan_il_vla.md`. 이 문서는 그 §2.7(모델 순서)의 **3단계 = GR00T** 를 실행 가능한
> 설계로 내린 것이다. 작성 2026-09-10, 근거는 전부 **설치된 lerobot 0.6.1 소스와
> 내려받은 `nvidia/GR00T-N1.7-3B` 체크포인트 사이드카 실측**이며 추측은 §5 에 따로 모았다.

## 0. 설계 원칙 (이 문서가 지키는 것)

**lerobot / GR00T 오픈소스를 베이스로 쓰고, 없는 기능만 우리가 만든다.**
따라서 설계의 첫 작업은 "무엇을 만들까"가 아니라 **"upstream 이 어디까지 해 주는가"를 소스로
확정하는 것**이다. §1 이 그 경계표이고, 우리 구현 대상은 §1 의 마지막 칸 하나뿐이다.

ACT 단계에서 얻은 교훈을 그대로 적용한다 — **측정한 것과 측정하려던 것이 같은지 확인할 것**
(`HISTORY.md` §35.4). 그래서 §4 는 "무엇을 잰다"가 아니라 **"어떤 수가 나오면 통과인가"**로 썼다.

---

## 1. 재사용 경계표 — upstream vs 우리

| 층 | 무엇 | 출처 | 우리가 할 일 |
|---|---|---|---|
| 사전학습 가중치 | `nvidia/GR00T-N1.7-3B` (27파일, 6.9 GB, **gated 아님**) | HF | **없음** — 받기만 |
| 백본 | `nvidia/Cosmos-Reason2-2B` (Qwen3-VL 계열) | 체크포인트가 알아서 로드 | **없음** |
| 정책 구현 | `lerobot/policies/groot/` (`GrootConfig`·`modeling_groot`·`processor_groot`, 4,912 줄) | lerobot 0.6.1 | **없음** |
| 학습 루프 | `lerobot-train --policy.type=groot` | lerobot | **없음** — 인자만 |
| 데이터셋 | `outputs/lerobot_ds_240_v2` (LeRobotDataset v3.0) | 우리가 이미 만든 것 | **없음 — 스키마 그대로** |
| 수집·변환 | `il_recorder.py` → `raw_to_lerobot.py` | 우리 기존 코드 | **없음 — 수정 불필요** |
| 추론 서버/클라이언트 | `lerobot.async_inference.{policy_server,robot_client}` | lerobot | **없음** — `--policy_type=groot` 로 바꾸기만 |
| 로봇 어댑터 | `UR16eROS` (`--robot.type=ur16e_ros`) | 우리 플러그인 | **없음 — 정책 비의존** |
| **운영 스크립트** | HF 캐시 격리·학습·롤아웃 래퍼, 문서 | — | **여기만 우리가 만든다** |

**결론: 모델·데이터 경로에 새로 작성할 코드가 0 이다.** ACT → GR00T 전환은 전부 **설정 변경**이다.
이건 운이 아니라 §2.1(LeRobot 포맷)·§2.6(절대 관절 액션)을 **첫날에 못 박아 둔 대가**다.

### 1.1 ACT 와 실제로 달라지는 것

| 항목 | ACT | GR00T N1.7 | 비고 |
|---|---|---|---|
| `--policy.type` | `act` | `groot` | |
| 파라미터 | ~80M | 3B (대부분 **동결**) | §2.3 |
| 언어 | **못 읽음** | **읽음** (`language_key="task"`) | §2.4 — 데이터 전략이 바뀐다 |
| 데이터셋 | 태스크 **1종**만 | 3종 **합친 그대로** | `plan_il_vla.md` §2.8 표대로 |
| 액션 청크 | 50 | **40** (`chunk_size`/`n_action_steps`) | 롤아웃 `--actions_per_chunk=40` |
| 정규화 | lerobot Normalizer | **IDENTITY** — GR00T 가 내부에서 함 | §2.5 |
| 이미지 크기 | 데이터셋 해상도 그대로 | **256×256 로 강제 리사이즈** + 230² 크롭 | §2.2 |

---

## 2. 카메라 2대가 어떻게 연결되는가 (이번 작업의 본론)

### 2.0 ★ `base_model_path` 는 **로컬 스냅샷 디렉터리**여야 한다 (2026-09-10 실측)

`_load_n1_7_checkpoint_processor_assets()` 는 맨 앞에서 `is_raw_groot_n1_7_checkpoint(base_model_path)`
를 부르고, 그 함수는 **`Path(...).is_dir()`** 로 판정한다. 즉 `nvidia/GR00T-N1.7-3B` 같은 **repo id 를
주면 `None` 을 반환**한다 — 예외 없이, 경고 없이.

`None` 이면 아래가 **전부 체크포인트 값이 아니라 lerobot 기본값으로 조용히 바뀐다**:

| 설정 | 체크포인트 실제값 | repo id 를 줬을 때 |
|---|---|---|
| `use_albumentations` | **True** | False |
| `state_dropout_prob` | **0.2** | 0.0 |
| `use_percentiles` | **True** | False |
| `shortest_image_edge` / `crop_fraction` | **256 / 0.95** | None / None |
| `use_relative_action` | **True** | False |

가중치는 repo id 로도 정상 로드되므로 **학습은 그냥 돌아간다.** 그래서 더 위험하다 — 사전학습 때와
다른 전처리로 파인튜닝하고 있다는 걸 알 방법이 없다. 이건 `HISTORY.md` §35.4 가 적어 둔
"측정한 것 ≠ 측정하려던 것"의 정확히 같은 부류다.

**정본 방식** — repo id 로 받되 **경로를 해석해서** 넘긴다. 다른 PC 재현성도 이쪽이 낫다:

```bash
BASE=$(deps/.venv-ml/bin/python -c \
  "from huggingface_hub import snapshot_download; print(snapshot_download('nvidia/GR00T-N1.7-3B'))")
# ... --policy.base_model_path="$BASE"
```

`snapshot_download` 는 캐시가 있으면 받지 않고 경로만 돌려준다. `HF_HOME` 만 맞추면 어느 PC 에서든 같다.

### 2.1 결론부터 — **rename 없이 2대 다 들어간다** ✅ 실측 확인

우리 데이터셋의 시각 피처는 두 개다:

```
observation.images.exterior   video (240, 320, 3)
observation.images.wrist      video (240, 320, 3)
```

`processor_groot.py` 의 `_ordered_image_keys()`(L1553)는 `video_modality_keys` 가 비어 있으면
`sorted(available)` 로 **관측에 있는 모든 카메라를 알파벳 순서로** 넣는다. 우리 경우
`exterior` → `wrist` 순서. **드롭도 rename 도 없다.**

소스를 읽고 추론한 게 아니라 **upstream 함수를 직접 불러 확인했다**(`scratchpad/groot_probe.py`):

```
[V1] dataset-meta visual keys       -> ['exterior', 'wrist']
[V1] checkpoint video_modality_keys -> None          (None => 데이터셋 폴백)
[V1] _ordered_image_keys(obs)       -> ['observation.images.exterior', 'observation.images.wrist']
[V1] checkpoint has stats for 'new_embodiment' -> False
[V1] checkpoint use_relative_action -> True
[V1] image target/crop              -> [256, 256] / [230, 230]
```

**카메라 2대 문제는 여기서 닫힌다.** 남은 미검증은 카메라가 아니라 메모리·처리량·게이트다.

### 2.2 왜 비어 있는가 — `new_embodiment` 의 의미

체크포인트 `processor_config.json` 의 `modality_configs` 에 들어 있는 embodiment 는 8종이고
**`new_embodiment` 는 없다**:

```
real_g1_relative_eef_relative_joints        video: ['ego_view']
real_r1_pro_sharpa_relative_eef             video: ['ego_view...', 'left_wrist_view...', 'right_wrist_view...']
xdof_relative_eef_relative_joint            video: ['top_camera...', 'left_camera...', 'right_camera...']
oxe_droid_relative_eef_relative_joint       video: ['exterior_image_1_left', 'wrist_image_left']   ← 우리와 같은 배치
...
```

`embodiment_tag="new_embodiment"`(기본값)이면 `modality_configs.get("new_embodiment")` 가 없어
`video_modality_keys=None` → 위의 폴백이 발동한다. **즉 "새 로봇"으로 붙이는 정규 경로가 곧
"내 데이터셋의 카메라를 그대로 쓴다"는 뜻이다.**

`new_embodiment` 는 버려지는 태그가 아니라 `N1_7_EMBODIMENT_MAPPING` 에 **id 10** 으로 등록된
정식 슬롯이다(`processor_groot.py` L120). 그 슬롯의 projector 가 우리가 학습시킬 대상이다.

> **의미 있는 방증**: 사전학습 embodiment 중 `oxe_droid_relative_eef_relative_joint` 가
> **exterior + wrist 2뷰 단일팔**이다 — 3B 가 우리와 같은 카메라 배치를 이미 대량으로 봤다.
> 카메라 2대는 GR00T 에게 예외적 구성이 아니라 흔한 구성이다.

### 2.3 해상도 — 우리 320×240 수집이 여기서 맞아떨어진다

체크포인트 `processor_kwargs`: `image_target_size=[256,256]`, `image_crop_size=[230,230]`,
`shortest_image_edge=256`, `crop_fraction=0.95`, `use_albumentations=True`.

`make_groot_pre_post_processors` 는 체크포인트에 이 값이 있으면 그대로 쓴다(L1240 부근). 주석이
그 이유를 명시한다 — 폴백이 없으면 **풀해상도 프레임을 패치화해 VLM 토큰 수가 폭증**한다.
우리 데이터셋은 320×240 이라 이미 목표 크기 근처이고, 사전학습 embodiment 들도 대부분
`res320x240` 이다. **ACT 때 320×240 으로 낮춘 결정(`SETUP.md` §2-C)이 GR00T 에서도 그대로 이득**이다.

### 2.4 언어 — 여기서 3종 태스크가 처음으로 값을 한다

`GrootN17PackInputsStep(language_key="task", formalize_language=True)`. ACT 는 토크나이저가
아예 없어서 3종 혼합 데이터셋이 **모순된 지도학습**이 됐지만(`HISTORY.md` §30), GR00T 는 task
문자열을 조건으로 받는다. → **`lerobot_ds_240_v2` 를 필터 없이 그대로 쓴다.**
이게 `plan_il_vla.md` §2.8 이 "첫날부터 지킬 것"으로 적어 둔 항목의 회수 시점이다.

### 2.5 상태/액션 정규화 — 통계가 어디서 오는가

`statistics.json` 에도 `new_embodiment` 는 **없다**. 그래서 `checkpoint_has_stats=False` 이고,
`padded_stats = dataset_stats` — **우리 데이터셋에서 계산한 min/max 통계**로 정규화하고,
디코드도 같은 통계로 역변환한다(L1290 주석이 이 대칭성을 명시: 안 그러면 정규화된 [-1,1] 을
그대로 액션으로 뱉는다). 7차원 상태/액션은 132차원으로 zero-pad 된다.

---

## 3. 결정해야 하는 것 하나 — 절대 액션 vs 상대 액션

이번 설계에서 **유일하게 기본값을 그냥 따라가면 안 되는 지점**이다.

체크포인트 `processor_kwargs` 에 **`use_relative_action: True`** 가 박혀 있고, embodiment 이름이
전부 `..._relative_eef_relative_joint...` 다. 즉 **N1.7 은 상대 액션 청크로 사전학습됐다.**
그런데 `GrootConfig.use_relative_actions` 의 기본값은 **False** 다.

| | A. 절대 (기본값) | B. 상대 (`--policy.use_relative_actions=true`) |
|---|---|---|
| 액션 정의 | 절대 관절 목표 | 현재 상태 기준 델타 청크 |
| 사전학습 정합 | **불일치** — head 가 다른 파라미터화를 배움 | **일치** |
| 통계 | 데이터셋 min/max | 데이터셋에서 **horizon 보존 상대 통계**를 계산 (`_build_n1_7_relative_action_processor_assets`) |
| 우리 스키마 영향 | 없음 | 없음 — 변환은 processor 안에서, 데이터셋은 §2.6 절대 관절 그대로 |
| 블록 위치 랜덤화 | 시작 자세에 민감 | 델타라 상대적으로 강건 |
| 코드 경로 | 단순 | 분기 하나 더 (검증 필요) |

**설계 결정: B(상대)를 본선으로 하되, `relative_exclude_joints=["gripper"]` 를 함께 준다.**
근거는 소스 주석에 그대로 있다 — *"set e.g. ["gripper"] to keep the gripper absolute, matching the
Isaac-GR00T single-arm + absolute-gripper convention"*. 우리 액션 피처 이름 7번째가 `gripper`라
부분문자열 매칭이 바로 걸린다. 그리퍼를 델타로 두면 파지/해제 같은 **이산적 사건**이
누적 오차에 녹아버린다.

단, B 는 A 보다 코드 경로가 길다. **스모크에서 A/B 둘 다 빌드되는지 먼저 확인하고**, B 가
깨지면 A 로 간다(그때는 "사전학습 전이가 약해진 결과"라는 걸 알고 해석해야 한다).

> ⚠️ `use_relative_actions=True` + 체크포인트에 우리 embodiment 통계가 없음 → `dataset_meta`
> 가 **반드시** 넘어가야 한다(없으면 명시적 `ValueError`). `lerobot-train` 은 넘겨준다.
> 이건 §5 의 미검증 항목이 아니라, 실패 시 조용하지 않고 **크게 터지는** 항목이다.

---

## 4. 검증 계획 — 무엇이 나와야 통과인가

ACT 때 "돌아간다"와 "의도대로 된다"를 구분하지 않아 여러 번 되돌아갔다. 단계마다 합격 기준을 먼저 적는다.

| # | 단계 | 측정값 | 합격 기준 | 실패 시 |
|---|---|---|---|---|
| V1 | 스모크 학습 (batch 8, 200 step) | 빌드 성공 + 카메라 2개 인식 | 로그에 카메라 드롭/미스매치 **경고 없음**, loss 하강 | rename_map 검토 |
| V2 | VRAM | 피크 MiB | **< 30,000 / 32,607** | batch↓ → grad accum |
| V3 | 처리량 | step/s | 아래 §4.1 예산 계산에 투입 | — |
| V4 | `/dev/shm` | 사용량 | 64 MiB 안 | `num_workers=0` (ACT 와 같은 처방) |
| V5 | A/B (절대/상대) | 둘 다 빌드 | B 성공 → B 채택 | A 로 폴백 |
| V6 | 본학습 | 총 소요 | **6시간 이하** | **사용자에게 보고 후 데이터/스텝 조정** |
| V7 | 추론 지연 | 청크 1회 추론 시간 | **< 1.33 s** (=40 step @30 Hz) | `num_inference_timesteps` ↓ |
| V8 | 롤아웃 | 성공률 / 배치 오차 | ACT 기준선(9/10, 7.6 mm)과 **비교 가능한 수준** | 데이터 부족 판정 |

### 4.1 학습 예산 — 사용자 게이트 (6시간)

```
총 시간 = max_steps / (step/s)
```

`max_steps` 기본값 10,000. **V3 측정 후 이 식으로 계산해 6시간을 넘으면 학습을 시작하지 않고
먼저 보고한다**(사용자 지시). 조절 손잡이는 세 개, 우선순위 순:

1. `--steps` 축소 — 21 에피소드/30,658 프레임에 10,000 step 은 과할 수 있다
2. `--batch_size` 조정 — 크게 하면 step/s 는 줄지만 step 당 학습량이 는다
3. 데이터 개수 — 사용자가 열어 둔 옵션. **마지막 수단** (수집을 다시 해야 하므로)

### 4.2 V7 이 별도 항목인 이유

ACT(80M)는 추론이 사실상 공짜였다. GR00T 는 2뷰 × 256² VLM 인코딩 + flow-matching 디노이징
(기본 4 step)이다. lerobot 의 async inference(policy_server/robot_client)는 **청크를 미리 받아
실행하는 구조**라 어느 정도 지연을 흡수하지만, 청크 소진(1.33 s)보다 추론이 느리면
로봇이 멈칫한다. 넘치면 `num_inference_timesteps` 를 낮춘다 — **학습을 다시 하지 않아도 되는
추론 전용 손잡이**다(`configuration_groot.py` 주석).

---

## 5. 미검증 — 실측으로만 닫히는 항목

이 문서에서 **"소스에서 읽었다"가 아닌 것은 아래가 전부**다. 확정될 때마다 표를 갱신한다.

| 항목 | 현재 상태 | 닫는 방법 |
|---|---|---|
| 카메라 2대가 그대로 들어가는가 | **✅ 확정 (2026-09-10)** — §2.1 | 완료 |
| 상대 액션 경로가 우리 데이터로 빌드되는가 | **✅ 확정 (2026-09-10)** — §8 | 완료 |
| 32 GB 에서 3B 부분 파인튜닝이 도는가 | **⚠️ 기본 설정으로는 안 됨** — §8.2 | 설정 변경 후 V2 |
| step/s → 총 학습시간 | 미검증 — **HF 게이트에 막힘** | V3 (게이트 해제 후) |
| 추론이 실시간을 따라가는가 | 미검증 | V7 |
| 21 에피소드로 3B 가 학습되는가 | 미검증 — ACT 는 100 에피소드가 필요했다 | V8 |

> **21 에피소드는 ACT 기준으로도 적다.** ACT 는 50→100 으로 늘려서야 가장자리 실패가 없어졌다
> (`HISTORY.md` §37). 사전학습 전이 덕에 GR00T 가 더 적은 데이터로 될 것이라는 **기대는 있지만
> 근거는 없다.** V8 이 나쁘면 "GR00T 가 나쁘다"가 아니라 **데이터가 부족한 것**일 가능성을 먼저 본다.

---

## 6. 환경 격리 (공유 컨테이너 원칙)

이 머신은 다른 프로젝트와 공유한다. GR00T 는 6.9 GB 를 받으므로 **캐시가 새 나가면 안 된다.**

```bash
export HF_HOME=/isaac-sim/volume/ur_ws/deps/hf_cache   # 공유 ~/.cache/huggingface 를 건드리지 않음
export WANDB_DISABLED=true                              # report_to='wandb' 가 기본값
export TOKENIZERS_PARALLELISM=false
```

- `deps/hf_cache` 는 **워크스페이스 안**이라 다른 ws 에 영향 0. `.gitignore` 대상.

### 6.1 의존성 — `lerobot[groot]` extra (2026-09-10 실측)

lerobot 은 정책별 의존성을 **extra 로 분리**하고 런타임에 `require_package("transformers", extra="groot")`
로 막는다(`policies/groot/modeling_groot.py` L76 외 5곳). 그래서 GR00T 는 ACT 와 달리 추가 설치가 있다.

> ⚠️ **파일 상단 import 만 보고 "transformers 불필요"라고 판단했다가 스모크에서 즉시 틀렸다.**
> lerobot 의 정책 의존성은 import 문이 아니라 **`require_package` 가드**에 있다. 다음에도
> 같은 실수를 하지 않으려면 `grep require_package policies/<정책>/` 를 먼저 볼 것.

**정본 명령은 개별 패키지 나열이 아니라 upstream extra 다** (§0 원칙):

```bash
deps/.venv-ml/bin/pip install 'lerobot[groot]'
```

끌어오는 것: `transformers`, `peft`, `diffusers`, `timm`, `dm-tree`, `decord` + 전이 의존성.

**설치 전 반드시 dry-run 으로 기존 패키지 변경 여부를 확인한다** (공유 컨테이너 원칙, `SETUP.md` §0-A 와 동일):

```bash
deps/.venv-ml/bin/pip install --dry-run --report /tmp/plan.json 'lerobot[groot]'
```

2026-09-10 실측 결과 **19개 전부 신규, 기존 패키지 변경 0**. 특히 **torch 2.11.0+cu128 /
torchvision 0.26.0+cu128 이 그대로 유지**된다 — 여기가 유일한 진짜 위험이었다.
PyPI 기본 인덱스의 torch 로 갈아치워지면 sm_120 이 깨진다(`SETUP.md` §2-C 함정 ②, §14 nvblox 와 같은 부류).
설치 후 확인:

```bash
deps/.venv-ml/bin/python -c "import torch;print(torch.__version__, torch.cuda.get_device_capability(0))"
# 2.11.0+cu128 (12, 0)  이어야 한다
```

실측 버전: `transformers 5.5.4`, `diffusers 0.39.0`, `peft 0.20.0`, `timm 1.0.29`, `decord 0.6.0`, `dm-tree 0.1.10`.

- 이 설치는 **`.venv-ml` 안에서만** 일어난다 — 시스템/Isaac python 오염 없음(`check_env.sh` 가 검사).
- `--policy.push_to_hub=false`, `--wandb.enable=false` — 둘 다 기본값이 켜져 있어 명시 해제.
- 재현성: `base_model_path` 는 **절대 스냅샷 경로가 아니라 repo id `nvidia/GR00T-N1.7-3B`** 로 둔다.
  다른 PC 에서는 `HF_HOME` 만 맞추면 같은 명령이 그대로 돈다.

---

## 7. 실행 순서

```
S0  다운로드 (6.9 GB)                                   → deps/hf_cache
S1  V1/V2/V3/V4  스모크 200 step, batch 8               → step/s·VRAM·shm
S2  V5           절대/상대 A/B 빌드 확인
S3  §4.1 예산 계산  ── 6h 초과? ──▶ 사용자 보고 후 대기
                        │
S4  본학습 (택1 확정)   ▼ 6h 이하면 진행
S5  V7           추론 지연 측정
S6  V8           롤아웃 N회 (ACT 와 같은 판정기 judge_rollout.py 재사용)
S7  문서화       PIPELINE.md 에 GR00T 절 추가, HISTORY.md 기록
```

**S6 은 ACT 롤아웃 하네스를 그대로 쓴다** — `rollout_n.sh` 에서 바뀌는 건
`--policy_type=groot`, `--pretrained_name_or_path`, `--actions_per_chunk=40` 세 개뿐이다.
판정기(GT 기반 `judge_rollout.py`)가 같으므로 **ACT 9/10 · 7.6 mm 와 직접 비교 가능**하다.
이 비교가 `plan_il_vla.md` §2.8 이 설계한 "ACT vs VLA 동일조건 비교"의 실현이다.

---

## 8. 실측 결과 (2026-09-10)

### 8.1 ★ 막힌 것 — `nvidia/Cosmos-Reason2-2B` 는 **gated repo**

`GR00T-N1.7-3B` 자체는 gated 가 아니라 그냥 받아진다(6.5 GB, 27파일). 그런데 학습을 시작하면
**백본 토크나이저**를 받으러 가서 401 로 죽는다:

```
Cannot access gated repo for url https://huggingface.co/nvidia/Cosmos-Reason2-2B/resolve/main/config.json
Access to model nvidia/Cosmos-Reason2-2B is restricted.
```

**무엇이 게이트에 걸리고 무엇이 안 걸리는지 정확히 갈랐다:**

| 필요한 것 | 어디서 오는가 | 게이트 |
|---|---|---|
| 백본 **아키텍처 설정** | `groot_n1_7.py:_cosmos_reason2_qwen3_vl_config()` — **소스에 하드코딩** | ✅ 불필요 |
| 백본 **가중치** | `GR00T-N1.7-3B` safetensors 안에 `backbone.model.*` **494개 포함** | ✅ 불필요 |
| **토크나이저 + 이미지/비디오 프로세서** | `_build_n1_7_processor()` → `nvidia/Cosmos-Reason2-2B` | ❌ **게이트** |

즉 **2 GB 백본을 다시 받는 게 아니라 토크나이저 몇 MB 때문에 막힌다.** 그래도 우회는 없다 —
다른 Qwen3-VL 토크나이저로 대체하는 건 vocab 이 어긋나면 조용히 틀린 학습이 되므로 하지 않는다.

**사용자 조치 필요 (내가 대신 할 수 없음)**:
1. https://huggingface.co/nvidia/Cosmos-Reason2-2B 에서 라이선스 동의
2. https://huggingface.co/settings/tokens 에서 read 토큰 발급
3. `export HF_TOKEN=hf_...` (또는 `HF_HOME` 을 맞춘 뒤 `huggingface-cli login`)

### 8.2 ★ VRAM — **기본 설정은 32 GB 에 안 들어간다**

`GrootPolicy` 를 실제로 만들어 파라미터를 세어 보면(`scratchpad/groot_mem.py`):

```
module                          total   trainable
_groot_model.action_head        1621M      1621M      ← 전부 학습
_groot_model.backbone           1524M         0M      ← 전부 동결
TOTAL                           3144M      1621M  (51.5%)
```

동결은 의도대로 걸린다(`tune_llm/tune_visual=False` 가 실제로 먹는다). 문제는 **동결이 되는데도
학습 대상이 1.62 B** 이라는 것 — GR00T 의 flow-matching action head 가 백본보다 크다.
`plan_il_vla.md` §2.7 이 "projector + diffusion head 만 학습하니 가볍다"고 적은 건 **비율을 낙관한 것**이다.

`model_params_fp32=True`(기본, NVIDIA 정식 레시피) 기준 정적 소요:

```
params 11.7 + grads 6.0 + AdamW 12.1 = 29.8 GiB   /  카드 31.8 GiB   → 활성값 자리가 없다
```

**배치 크기와 무관하다** — 정적分만으로 이미 차서, `--batch_size=1` 이어도 안 된다.

`--policy.model_params_fp32=false` 로 바꾸면 action head 가 bf16 이 되고(백본은 fp32 유지):

```
params 8.7 + grads 3.0 + AdamW 6.0 ≈ 17.8 GiB      → 활성값 여유 14 GiB
```

**설계 결정: `--policy.model_params_fp32=false` 를 기본으로 한다.** upstream 이 제공하는 손잡이를
쓰는 것이지 우리가 만든 우회가 아니다. 대신 **fp32 마스터 가중치를 포기하는 것**이므로,
학습이 발산하면 제일 먼저 의심할 항목으로 기록해 둔다.
(fp32 를 지키려면 8-bit optimizer(bitsandbytes) 가 필요한데, `lerobot[groot]` extra 밖이라
의존성이 늘고 검증도 새로 해야 한다 — 필요해지면 그때.)

### 8.3 상대 액션 경로 — 빌드 확인

`use_relative_actions=true` + `relative_exclude_joints=["gripper"]` 로
`_build_n1_7_relative_action_processor_assets()` 가 우리 데이터셋 메타에서 **정상 빌드**된다.
§3 의 B안이 실현 가능하다는 뜻이고, 남은 건 학습 품질 비교뿐이다.

### 8.4 의존성 — §6.1 로 해소

`lerobot[groot]` 설치로 `transformers 5.5.4` 외 19개 신규, **기존 패키지 변경 0**,
`torch 2.11.0+cu128` / sm_120 유지 확인.
