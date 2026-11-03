# 집기·옮기기 실험 기록 (단계별 시험)

같은 조건으로 여러 번 해서 **성공률과 오차 분포**를 얻기 위한 기록 방법이에요. 한 번 할 때마다 `ExperimentLog`가 설정 스냅샷(J1 영점, 보정 gain, 흡착 기준, 처짐 각, 카메라 보정 출처, git 커밋)을 같이 남겨요. 그래서 설정을 바꾼 뒤에 결과가 달라졌는지 비교할 수 있어요.

저장: `~/robot_ws/data/experiments/<실험이름>/trials.jsonl` (git 제외). CSV와 요약:

```bash
python3 -m soarm4_workcell.experiment summary ~/robot_ws/data/experiments/pickplace
python3 -m soarm4_workcell.experiment csv ~/robot_ws/data/experiments/pickplace
```

## 단계와 기록 항목

| 단계 | 시험 내용 | 기록 항목 (예) | 합격 기준 (처음 제안) |
|---|---|---|---|
| `S1_pose` | 빈 컵을 목표 위 5 mm에 세우고 가장자리까지 거리로 컵 중심 실측 | 목표, 관절각, 실측 좌우상하(cm), `actual_xy_m`, `err_mm` | 턴테이블·패드 모두 2 mm 이내 |
| `S2_recon` | 스캔 → Gemini 박스 → SAM2 → 점군 융합 → 흡착면 선택 | 세션 폴더, 마스크 점수, 윗면 높이·기울기·평탄도, 안에 들어가는 원 반지름, Gemini 선택, `pickup_xyz_m` | 윗면 평탄도 2 mm 이내, 원 반지름 ≥ 15.5 mm |
| `S3_grip` | 컵을 윗면에 닿게 하고 펌프 ON | 접촉 높이(엔코더), 첫 `diff`, 성공 여부, 재시도 횟수 | 첫 시도 성공, `diff` ≤ 기준 |
| `S4_carry` | 3 cm 들어 올려 목표 위로 이동 | 이동 중 `diff` 최악값·최강값, 압력 풀림 여부 | 최악값이 유지 기준(−300)보다 강함 |
| `S5_place` | 내려놓고 해제 | 목표 xy, 카메라로 잰 놓인 xy(`placed_xy_m`), `err_mm`, 블록 각도, 해제 후 `diff` | 목표 중심에서 5 mm 이내 |
| `S6_cycle` | 한 사이클 전체 | 성공 여부, 걸린 시간(초), 실패한 단계 | 10번 중 9번 이상 |

## 시험하는 순서 (단계별)

1. **S1**: 새 보정(영점, gain 등)을 바꾼 뒤마다 턴테이블·ID5·ID6에서 한 번씩. 안 맞으면 다음 단계로 가지 않아요.
2. **S2**: 같은 블록을 3번 스캔해서 윗면 중심이 몇 mm 흔들리는지(반복성).
3. **S3 + S4**: 한 자리에서 집었다 놓기를 5번 반복. 압력 풀림이 있는지.
4. **S5**: 위치 두 곳(턴테이블, 패드)에서 번갈아 놓고 카메라로 오차 분포 확인. 오늘 결과는 **ID5에서 +6 mm, 턴테이블에서 −12 mm**로 서로 달랐어요. 같은 방향으로 일정한지, 위치에 따라 다른지 먼저 가려야 해요.
5. **S6**: 위가 모두 통과한 뒤 같은 조건으로 10번.

## 코드로 기록하기

```python
from soarm4_workcell.experiment import ExperimentLog
log = ExperimentLog('pickplace')
t = log.new_trial(note='턴테이블 → ID5')
log.add(t, 'S3_grip', first_diff=-404, success=True, retries=0)
log.add(t, 'S5_place', target_xy_m=[1.251, 0.367], placed_xy_m=[1.2512, 0.3731], block_yaw_deg=37)   # err_mm 자동
```

## 주의
- **한 번의 결과로 결론을 내지 않아요.** 최소 3~5번은 모아서 평균과 흩어짐을 같이 봐요.
- 설정을 바꾸면(영점, gain, 처짐 각 등) 새 실험 이름으로 시작하는 게 비교하기 쉬워요.
- 실측값이 cm인지 mm인지 항목 이름에 적어 둬요 (`_cm`, `_mm`, `_m`).
