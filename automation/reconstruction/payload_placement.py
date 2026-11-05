"""Preserve pickup pitch while placing payload bottom; no hardware IO."""
import math,numpy as np
from scipy.optimize import least_squares

def centered_payload_origin(points, origin):
    """Choose an XY placement reference enclosing every measured point.

    This changes only the local frame, never the collider or its Z reference.
    """
    from scipy.spatial import ConvexHull
    from scipy.optimize import minimize
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all():raise ValueError('Finite Nx3 payload required')
    xy=p[ConvexHull(p[:,:2]).vertices,:2];base=xy.mean(0);local=(xy-base)*1000.
    # Solve in millimetres with distance constraints. Squared metre residuals
    # made SLSQP report spurious failures after rigid table rotations.
    def jac(x):
        delta=x[:2]-local;distance=np.linalg.norm(delta,axis=1)
        return np.column_stack([-delta/np.maximum(distance[:,None],1e-12),np.ones(len(local))])
    fit=minimize(lambda x:float(x[2]),[0.,0.,float(np.max(np.linalg.norm(local,axis=1)))+1e-6],jac=lambda x:np.array([0.,0.,1.]),method='SLSQP',constraints=[{'type':'ineq','fun':lambda x:x[2]-np.linalg.norm(local-x[:2],axis=1),'jac':jac}],bounds=[(None,None),(None,None),(0,None)],options={'ftol':1e-9,'maxiter':500})
    if not fit.success or np.max(np.linalg.norm(local-fit.x[:2],axis=1))>fit.x[2]+1e-4:raise RuntimeError('Payload centre fit failed')
    result=np.asarray(origin,float).copy();result[:2]=fit.x[:2]/1000.+base;return result

def signed_pitch(T,j1):
    radial=T[:2,3]-np.asarray(j1)[:2];radial/=np.linalg.norm(radial)
    return math.atan2(float(T[:2,0]@radial),float(-T[2,0]))

def place_payload(fk,lo,hi,target,seed,L,dims,j1,pitch):
    target=np.asarray(target,float);dims=np.asarray(dims,float)
    def residual(q):
        T=fk(q);obj=T@L;position=obj[:3,3].copy();position[2]-=np.sum(np.abs(obj[2,:3])*dims)/2
        return np.r_[(position-target)*10,(signed_pitch(T,j1)-pitch)*.3]
    solutions=[]
    for start in [np.asarray(seed)]+list(np.random.default_rng(8).uniform(lo,hi,(20,4))):
        fit=least_squares(residual,np.clip(start,lo,hi),bounds=(lo,hi),max_nfev=500)
        if np.linalg.norm(residual(fit.x)[:3])<.002 and abs(residual(fit.x)[3])<math.radians(.2)*.3:solutions.append(fit.x)
    if not solutions:return None
    return min(solutions,key=lambda q:np.linalg.norm(q-seed))
