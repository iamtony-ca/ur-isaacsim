# archive — IL/VLA 전환 이전 트랙의 설계문서

이 폴더의 문서는 **폐기가 아니라 보류**다. 워크스페이스가
**teleoperation → ACT 모방학습 → GR00T N1.7 VLA** 로 방향을 잡으면서, 그 이전 트랙의
설계문서를 루트에서 치워 현재 진행 중인 문서와 섞이지 않게 했다.

현재 유효한 문서는 루트에 있다 — `README.md`(현재 상태), `SETUP.md`(재현),
`HARDWARE.md`(실물 연결), `HISTORY.md`(이력), `qna.md`(개념),
그리고 IL/VLA 정본인 `ur_bringup/docs/plan_il_vla.md`.

| 문서 | 무엇인가 | 왜 보류인가 | 살아남은 부분은 어디로 |
|---|---|---|---|
| `to_do.md` | foundation-model perception + pick&place 작업레이어 | perception(FoundationStereo/SAM3/FoundationPose)과 동적 장애물 회피는 현재 IL/VLA 경로에 없다 | **M3 pick&place 상태머신은 실현됨** → `ur_bringup/scripts/pick_place_demo.py`, 검증 이력은 `HISTORY.md` §25·§26 |
| `LEARNING.md` | RL(insertion) 설계, Isaac Lab 기반 | 현재 목표에 RL 이 없다. `oht_bolting` 참조는 애초에 다른 워크스페이스 자산 | §2(컨트롤러 인터페이스 불일치) → `ur_bringup/docs/plan_il_vla.md` §2.3 으로 이관 완료 |

## 다시 꺼내 쓸 때 주의

두 문서 모두 **2026-09-06~07 시점**의 서술이다. 그 뒤 이 워크스페이스에서 확정된 사실
(Isaac 2F-85 에셋의 URDF 불일치 3건, `/dev/shm` 한계, lerobot 상류 파이프라인 채택 등)은
반영돼 있지 않다. 되살릴 때는 `HISTORY.md` §26 이후를 먼저 확인할 것.
