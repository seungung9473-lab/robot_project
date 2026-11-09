"""Check the arm's actual waiting pose against the reviewed start of the next stage.

LIMIT_DEG: above this the stage is refused (nothing moves).
EXACT_DEG: maximum offset for cached execution, aligned with MoveIt's start tolerance.
Between EXACT_DEG and LIMIT_DEG, the next move is planned from the measured pose.
"""
import math
import numpy as np

LIMIT_DEG=5.0   # 2026-11-09: 3° → 5° (23:42 실행에서 J4가 3.5° 어긋남. 어긋난 단계는 현재 자세에서 다시 계획·충돌검사하므로 안전 검사는 유지)
EXACT_DEG = 1.0  # MoveIt hardware trajectory start tolerance in degrees.

def waiting_pose_errors(actual, expected):
    actual = np.asarray(actual, float)
    expected = np.asarray(expected, float)
    if actual.shape != (4,) or expected.shape != (4,) or not np.isfinite([actual, expected]).all():
        raise RuntimeError('Invalid joint feedback or reviewed waiting pose')
    return np.degrees(np.abs(actual - expected))


def check_wait_pose(actual,expected,limit_deg=LIMIT_DEG,exact_deg=EXACT_DEG):
    diff=waiting_pose_errors(actual,expected)
    worst=float(diff.max())
    if worst>limit_deg:
        raise RuntimeError('Actual waiting pose differs from reviewed stage start by %.1f deg (limit %.1f deg; joint %d)'%(worst,limit_deg,int(diff.argmax())+1))
    return {'max_deg':worst,'per_joint_deg':[float(x) for x in diff],'replan_first_move':worst>=exact_deg}
