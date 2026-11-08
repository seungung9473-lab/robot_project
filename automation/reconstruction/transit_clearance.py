"""Clearance for the empty-cup wait pose after a release.

The turntable top is z=59 mm and objects rise up to about z=99 mm, so a 30 mm wait above the pad (tip z=89 mm)
sits inside the objects' height and the next move/rotation can sweep them. Lift 100 mm (tip z about 159 mm) so the
cup and arm clear every object before the turntable rotates and before the next approach.
Planning (check_inclined_motion) and execution (verified_place_left) must use this same value.
"""
RETREAT_UP_M=0.10

# Approach to the next object: first go to a pose TRANSIT_UP_M straight above the approach pose (same elbow/wrist
# configuration as the approach), keep the cup tip above TRANSIT_MIN_TCP_Z_M on the way (turntable top 59 mm + tallest object
# ~40 mm + margin), then descend in a straight vertical line. Planning and execution use the same reviewed transit pose.
TRANSIT_UP_M=0.10
TRANSIT_MIN_TCP_Z_M=0.14
JOINT_FOLD_WEIGHTS=(1.,1.,1.5,1.5)

def _wrap(a):
    import math
    return (a+math.pi)%(2*math.pi)-math.pi

def travel_cost(start_q,target_q,weights=JOINT_FOLD_WEIGHTS):
    """Weighted joint travel (rad) from the stage start to a candidate approach pose; J3/J4 (folding) weigh more."""
    return float(sum(w*abs(_wrap(float(t)-float(s))) for w,s,t in zip(weights,start_q,target_q)))

def min_tcp_z(fk,positions):
    return min(float(fk(q)[2,3]) for q in positions)
