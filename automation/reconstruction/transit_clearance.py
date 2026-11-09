"""Clearance for the empty-cup wait pose after a release.

The turntable top is z=59 mm and objects rise up to about z=99 mm, so a 30 mm wait above the pad (tip z=89 mm)
sits inside the objects' height and the next move/rotation can sweep them. Lift 100 mm (tip z about 159 mm) so the
cup and arm clear every object before the turntable rotates and before the next approach.
Planning (check_inclined_motion) and execution (verified_place_left) must use this same value.
"""
RETREAT_UP_M=0.10
# 다른 물체(턴테이블 위 이웃·이미 놓은 물체)의 충돌 형상을 키우는 여유. 기본 1 mm에서 4 mm로 올려 이동 중 이웃 물체와 더 떨어지게 한다.
# 패드 위 물체 사이 실제 간격(간격 2 mm + 발자국 여유 2.5 mm×2 = 7 mm)보다 작아야 배치 계획이 막히지 않는다. 집는 물체·든 물체에는 적용하지 않는다.
OBSTACLE_PADDING_M=0.004

# Prefer a 100 mm rise above the approach pose and try other heights as needed.
# Transit keeps the cup tip above the absolute table-frame floor before descent.
TRANSIT_UP_M=0.10
TRANSIT_MIN_TCP_Z_M=0.14

def transit_height_candidates(approach_z):
    """Prefer the original 100 mm rise, then try shorter rises above the transit floor."""
    minimum_rise = max(.01, TRANSIT_MIN_TCP_Z_M + .003 - approach_z)
    rises = (TRANSIT_UP_M, .08, .06, .04, .02, minimum_rise)
    return tuple(dict.fromkeys(rise for rise in rises if rise >= minimum_rise))


JOINT_FOLD_WEIGHTS=(1.,1.,1.5,1.5)

def _wrap(a):
    import math
    return (a+math.pi)%(2*math.pi)-math.pi

def travel_cost(start_q,target_q,weights=JOINT_FOLD_WEIGHTS):
    """Weighted joint travel (rad) from the stage start to a candidate approach pose; J3/J4 (folding) weigh more."""
    return float(sum(w*abs(_wrap(float(t)-float(s))) for w,s,t in zip(weights,start_q,target_q)))

def min_tcp_z(fk,positions):
    return min(float(fk(q)[2,3]) for q in positions)
