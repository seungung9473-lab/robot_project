"""Bounded retry offsets along the observed face normal, no actuator IO."""
import numpy as np

def retry_targets(first_tcp,outward_normal):
    origin=np.asarray(first_tcp,float);normal=np.asarray(outward_normal,float)
    if origin.shape!=(3,) or normal.shape!=(3,) or not np.isfinite(origin).all() or not np.isfinite(normal).all() or np.linalg.norm(normal)<1e-8:raise ValueError('Finite TCP and nonzero surface normal required')
    normal=normal/np.linalg.norm(normal)
    return [origin-normal*(i*.001) for i in range(1,4)]

FINAL_ADVANCE_M=.005
SUCTION_WAIT_S=5.0

def final_advance_target(first_tcp,outward_normal,distance_m=FINAL_ADVANCE_M):
    """Single bounded advance along the inward normal from the first actual contact TCP."""
    if not 0<distance_m<=.005:raise ValueError('Final advance must be within 0..5 mm')
    origin=np.asarray(first_tcp,float);normal=np.asarray(outward_normal,float)
    if origin.shape!=(3,) or normal.shape!=(3,) or not np.isfinite(origin).all() or not np.isfinite(normal).all() or np.linalg.norm(normal)<1e-8:raise ValueError('Finite TCP and nonzero surface normal required')
    return origin-normal/np.linalg.norm(normal)*distance_m
