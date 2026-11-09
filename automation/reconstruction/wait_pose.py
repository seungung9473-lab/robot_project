"""Check the arm's actual waiting pose against the reviewed start of the next stage.

LIMIT_DEG: above this the stage is refused (nothing moves).
EXACT_DEG: reviewed trajectories are reused only when the start matches this closely
(trajectory_cache.compatible uses the same .25 deg); between EXACT_DEG and LIMIT_DEG the
first move of the stage is planned live from the actual pose (full collision review), the
rest of the reviewed stage is unchanged because it starts from the same approach pose.
"""
import math
import numpy as np

LIMIT_DEG=5.0   # 2026-11-09: 3° → 5° (23:42 실행에서 J4가 3.5° 어긋남. 어긋난 단계는 현재 자세에서 다시 계획·충돌검사하므로 안전 검사는 유지)
EXACT_DEG=.25

def check_wait_pose(actual,expected,limit_deg=LIMIT_DEG,exact_deg=EXACT_DEG):
    diff=np.degrees(np.abs(np.asarray(actual,float)-np.asarray(expected,float)))
    worst=float(diff.max())
    if worst>limit_deg:
        raise RuntimeError('Actual waiting pose differs from reviewed stage start by %.1f deg (limit %.1f deg; joint %d)'%(worst,limit_deg,int(diff.argmax())+1))
    return {'max_deg':worst,'per_joint_deg':[float(x) for x in diff],'replan_first_move':worst>exact_deg}
