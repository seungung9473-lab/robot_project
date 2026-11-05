"""Inclined planar suction geometry and 4-DOF orientation; no hardware IO."""
import numpy as np
from scipy.spatial import ConvexHull,cKDTree
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares,brentq,linprog

PLANE_INLIER_TOLERANCE_M = .0025
PATCH_RMS_LIMIT_MM = 2.5

def planes(points,max_planes=10,seed=9473):
    rng=np.random.default_rng(seed);remaining=np.asarray(points,float);out=[]
    for _ in range(max_planes):
        if len(remaining)<120:break
        best=np.zeros(len(remaining),bool)
        for _ in range(600):
            v=remaining[rng.choice(len(remaining),3,replace=False)];n=np.cross(v[1]-v[0],v[2]-v[0]);length=np.linalg.norm(n)
            if length<1e-8:continue
            n/=length;mask=np.abs((remaining-v[0])@n)<PLANE_INLIER_TOLERANCE_M
            if mask.sum()>best.sum():best=mask
        if best.sum()<100:break
        p=remaining[best];center=p.mean(0);_,_,V=np.linalg.svd(p-center,full_matrices=False);n=V[-1]
        if n[2]<0:n=-n
        mask=np.abs((remaining-center)@n)<PLANE_INLIER_TOLERANCE_M;p=remaining[mask];remaining=remaining[~mask]
        out.append((p,n,float(-center@n)))
    return out

def patch(points,n,d,radius=.0125,margin=.003,max_tilt=90.,center_point=None,prefer_high=False):
    n=np.asarray(n,float);n/=np.linalg.norm(n);p=np.asarray(points,float)
    tilt=float(np.degrees(np.arccos(np.clip(n[2],-1,1))))
    if tilt>max_tilt:return None
    inliers=p[np.abs(p@n+d)<PLANE_INLIER_TOLERANCE_M]
    if len(inliers)<100:return None
    # Choose a reference axis that cannot be parallel to the plane normal.
    # Vertical +/-Y faces made the previous fixed-Y cross product singular.
    reference=np.eye(3)[np.argmin(np.abs(n))]
    center=inliers.mean(0);u=np.cross(reference,n);u/=np.linalg.norm(u);v=np.cross(n,u)
    xy=np.column_stack([(inliers-center)@u,(inliers-center)@v])
    try:hull=ConvexHull(xy)
    except Exception:return None
    low,high=xy.min(0),xy.max(0);xx,yy=np.meshgrid(np.arange(low[0],high[0],.001),np.arange(low[1],high[1],.001));grid=np.column_stack([xx.ravel(),yy.ravel()])
    if center_point is not None:grid=np.array([[float((np.asarray(center_point)-center)@u),float((np.asarray(center_point)-center)@v)]])
    if not len(grid):return None
    eq=hull.equations
    if center_point is None:
        # A1mm grid can miss a narrow valid centre. Add the exact largest
        # inscribed-circle centre; hole/coverage/RMS checks still apply below.
        fit=linprog([0.,0.,-1.],A_ub=np.c_[eq[:,:2],np.ones(len(eq))],b_ub=-eq[:,2],bounds=[(None,None),(None,None),(0,None)],method='highs')
        if fit.success and np.isfinite(fit.x).all():grid=np.vstack([grid,fit.x[:2]])
    clear=-(grid@eq[:,:2].T+eq[:,2]).max(1)
    # Test several interior points, preserving hole/groove checks beyond convex hull.
    best=None
    indices=np.argsort(clear)[-30:][::-1]
    if prefer_high and center_point is None:
        # Keep an extra2mm edge buffer where available before favouring height.
        eligible=np.flatnonzero(clear>=radius+margin+.002)
        if not len(eligible):eligible=np.flatnonzero(clear>=radius+margin)
        if len(eligible):
            heights=(center+grid[:,0,None]*u+grid[:,1,None]*v)[:,2]
            indices=eligible[np.argsort(heights[eligible])[-64:][::-1]]
    for idx in indices:
        g=grid[idx];point=center+u*g[0]+v*g[1];aa=np.linspace(0,2*np.pi,64,endpoint=False);rr=np.linspace(0,radius,9);disk=np.array([g+r*np.array([np.cos(a),np.sin(a)]) for r in rr for a in aa]);dist=cKDTree(xy).query(disk)[0];coverage=float(np.mean(dist<.003));local=np.linalg.norm(np.column_stack([(p-center)@u,(p-center)@v])-g,axis=1)<radius;res=p[local]@n+d
        # Include near-surface points only, avoid far face on opposite side.
        res=res[np.abs(res)<.008]
        rms=float(np.sqrt(np.mean(res**2))*1000) if len(res) else 1e9
        record={'point_table_m':point.tolist(),'normal_table':n.tolist(),'plane_d':d,'tilt_deg':tilt,'boundary_clearance_mm':float(clear[idx]*1000),'coverage':coverage,'patch_rms_mm':rms,'inliers':len(inliers),'plane_inlier_tolerance_mm':PLANE_INLIER_TOLERANCE_M*1000,'patch_rms_limit_mm':PATCH_RMS_LIMIT_MM,'edge_margin_mm':margin*1000,'geometry_pass':bool(clear[idx]>=radius+margin and coverage>.9 and rms<PATCH_RMS_LIMIT_MM)}
        rank=lambda r:(r['geometry_pass'],r['point_table_m'][2],r['boundary_clearance_mm']) if prefer_high else (r['geometry_pass'],r['boundary_clearance_mm'])
        if best is None or rank(record)>rank(best):best=record
    return best

def rotate_surface(point,normal,axis,angle_deg):
    R=Rotation.from_euler('z',angle_deg,degrees=True).as_matrix();a=np.asarray(axis,float)
    return R@(np.asarray(point)-a)+a,R@np.asarray(normal)

def alignment_angles(point,normal,axis,j1):
    """Angles where inclined normal lies in the arm's radial pitch plane."""
    def cross(a):
        p,n=rotate_surface(point,normal,axis,a);r=p[:2]-np.asarray(j1)[:2]
        return n[0]*r[1]-n[1]*r[0]
    if np.linalg.norm(np.asarray(normal)[:2])<1e-6:return [0.]
    roots=[];xs=np.linspace(0,360,721)
    for a,b in zip(xs,xs[1:]):
        if abs(cross(a))<1e-9:roots.append(float(a))
        elif cross(a)*cross(b)<0:roots.append(float(brentq(cross,a,b)))
    return sorted(set(round(x%360,6) for x in roots))

def compensated_axis(point,normal,j1,sag_deg=3.):
    radial=np.asarray(point)[:2]-np.asarray(j1)[:2];radial/=np.linalg.norm(radial)
    tangent=np.cross([0.,0.,1.],np.r_[radial,0.])
    return Rotation.from_rotvec(-np.radians(sag_deg)*tangent).apply(-np.asarray(normal))

def solve_axis(fk,lo,hi,point,axis,seed=None,attempts=16):
    point=np.asarray(point,float);axis=np.asarray(axis,float);axis/=np.linalg.norm(axis)
    def error(q):
        T=fk(q);return np.r_[(T[:3,3]-point)*10,(T[:3,0]-axis)*.3]
    seeds=([np.clip(seed,lo,hi)] if seed is not None else [])+list(np.random.default_rng(4).uniform(lo,hi,(attempts,4)))
    solutions=[]
    for start in seeds:
        fit=least_squares(error,start,bounds=(lo,hi),max_nfev=500);T=fk(fit.x);position=float(np.linalg.norm(T[:3,3]-point));angle=float(np.degrees(np.arccos(np.clip(T[:3,0]@axis,-1,1))))
        if position<.0003 and angle<.5:solutions.append((fit.x,position,angle))
    return min(solutions,key=lambda x:np.linalg.norm(x[0]-(seed if seed is not None else np.array([0,np.pi,-np.pi,0])))) if solutions else None
