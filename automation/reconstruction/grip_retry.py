"""Bounded retry offsets along the observed face normal, no actuator IO."""
import numpy as np

def retry_targets(first_tcp,outward_normal):
    origin=np.asarray(first_tcp,float);normal=np.asarray(outward_normal,float)
    if origin.shape!=(3,) or normal.shape!=(3,) or not np.isfinite(origin).all() or not np.isfinite(normal).all() or np.linalg.norm(normal)<1e-8:raise ValueError('Finite TCP and nonzero surface normal required')
    normal=normal/np.linalg.norm(normal)
    return [origin-normal*(i*.001) for i in range(1,4)]
