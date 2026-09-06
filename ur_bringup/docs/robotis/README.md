# ROBOTIS OMY-L100 (leader) HW 원본 자료

teleop leader 후보인 **ROBOTIS AI OMY-L100** 의 제조사 제공 HW 자료.
검증 결과와 UR16e 대조는 → `../plan_il_vla.md` §3.5, `../../../HISTORY.md` §21.

| 파일 | 내용 | 확인 상태 |
|---|---|---|
| `omy_l100_kr_03_layout.png` | 제품 사양표 + 치수 도면 (**가장 정보량 많음**) | ✅ 판독 완료 |
| `OMY-L100.pdf` | 위 도면의 벡터 PDF | ⚠️ 벡터라 텍스트 추출 불가. 치수는 PNG 와 동일 |
| `OMY-L100.stp` | CREO 조립체 `PR44_P08_ASM` (2025-06-11), 단위 **mm** | ✅ 부품구조·단위·바운딩박스 검증 |
| `OMY-L100.dwg` | AutoCAD 원본 | ⬜ 미확인 (전용 뷰어 필요) |

온라인 보완 자료: <https://docs.robotis.com/docs/systems/omy/specifications/hardware>
— **링크 질량(`g`) + COG 기준 3×3 관성텐서(`g·mm²`)** 가 L100/F3M 양쪽 모두 제공된다.
URDF 로 옮길 때 질량 ×1e−3(kg), 관성 ×1e−9(kg·m²) 환산 필요.

## ★ 이 자료보다 URDF 를 먼저 볼 것

**L100 공식 URDF 가 있다** — [`ROBOTIS-GIT/open_manipulator`](https://github.com/ROBOTIS-GIT/open_manipulator)
(Apache-2.0) `open_manipulator_description/urdf/omy_l100/omy_l100.urdf` + STL + ros2_control +
`omy_l100_leader_ai` 리더 런치. **링크 치수가 여기 도면·STEP 과 정확히 일치**함을 확인했다(3중 교차검증).
따라서 기구학 작업은 URDF 로 하고, 이 폴더는 **도면 치수 대조 + 질량/관성 출처**로 쓴다.

## 읽을 때 반드시 알아야 할 것

1. **L100(리더) 과 F3M(팔로워) 를 섞지 말 것.** 관절 한계가 다르다 —
   L100 `J3 ±180°` vs F3M `J3 ±150°`. `robotis_mujoco_menagerie/robotis_omy/omy.xml`
   은 **F3M** 모델이므로 리더 사양의 근거로 쓰면 안 된다. (실제로 한 번 틀렸다 → HISTORY §21)
2. **DOF 표기 불일치는 오류가 아니다.** PNG `7` = 팔 6 + 그리퍼 1, 웹 문서 `6` = 팔만.
3. **DH 파라미터·좌표계 그림·관절 회전방향 규약은 어느 자료에도 없다.**
   → leader→UR16e 부호/오프셋은 **실물 측정으로만** 확정 가능.

## 주의: 저장소 용량

이 폴더는 바이너리 약 **8 MB**(`.stp` 7.2 MB, `.dwg` 640 KB)다. git 에 그대로 들어간다.
필요한 수치는 위 문서들에 모두 옮겨 적었으므로, 원본을 repo 밖에 두고 싶다면
`.gitignore` 에 `ur_bringup/docs/robotis/*.stp`, `*.dwg` 를 추가해도 재현에 지장 없다.
