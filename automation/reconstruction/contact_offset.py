"""Software compensation: grip every object slightly to the arm's RIGHT of the planned point.

2026-11-09: the cup lands slightly LEFT of the object centre for every object. The encoders say the arm reaches the planned
point to 0.2 mm, so the bias sits in the calibration chain (J1 zero / base yaw, camera -> table, TCP side offset), not in IK.
Until that is measured and fixed at its source, the contact point is shifted CONTACT_RIGHT_M to the right,
looking from the robot base along the arm towards the object (horizontal, then projected into the suction face).
The shifted point is only used when the cup still fits on the face (edge margin, coverage, flatness, both cameras).
"""
import numpy as np
CONTACT_RIGHT_M=0.005

def right_direction(point,j1):
    """Unit horizontal vector pointing to the right of the base -> point direction (seen from above, +X right, +Y away)."""
    radial=np.asarray(point,float)[:2]-np.asarray(j1,float)[:2];norm=np.linalg.norm(radial)
    if norm<1e-6:raise ValueError('Contact point is on the arm base axis')
    radial/=norm;return np.array([radial[1],-radial[0],0.])

def shifted_point(point,normal,j1,right_m=CONTACT_RIGHT_M):
    """Point moved right_m to the right, kept in the suction face plane (the normal component of the shift is removed)."""
    n=np.asarray(normal,float);n=n/np.linalg.norm(n)
    shift=right_direction(point,j1)*right_m;shift=shift-n*(shift@n)
    return np.asarray(point,float)+shift
