"""Pure coordinate and geometry operations; no ROS, serial or motor access."""
from pathlib import Path
import hashlib,math
import numpy as np
import yaml

def matrix(value):
    T=np.asarray(value,dtype=float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-8):raise ValueError('Invalid homogeneous transform')
    R=T[:3,:3]
    if not np.allclose(R.T@R,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(R),1.,atol=1e-6):raise ValueError('Rotation must be right-handed and orthonormal')
    return T

def quaternion(R):
    R=np.asarray(R,float);trace=np.trace(R)
    if trace>0:
        s=math.sqrt(trace+1)*2;q=np.array([(R[2,1]-R[1,2])/s,(R[0,2]-R[2,0])/s,(R[1,0]-R[0,1])/s,.25*s])
    else:
        i=int(np.argmax(np.diag(R)))
        if i==0:
            s=math.sqrt(1+R[0,0]-R[1,1]-R[2,2])*2;q=np.array([s*.25,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s,(R[2,1]-R[1,2])/s])
        elif i==1:
            s=math.sqrt(1+R[1,1]-R[0,0]-R[2,2])*2;q=np.array([(R[0,1]+R[1,0])/s,s*.25,(R[1,2]+R[2,1])/s,(R[0,2]-R[2,0])/s])
        else:
            s=math.sqrt(1+R[2,2]-R[0,0]-R[1,1])*2;q=np.array([(R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,s*.25,(R[1,0]-R[0,1])/s])
    return q/np.linalg.norm(q)

def rotation(q):
    x,y,z,w=np.asarray(q,float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def translated(p):
    T=np.eye(4);T[:3,3]=p;return T

def load(path):
    d=yaml.safe_load(Path(path).read_text());matrix(d['T_table_world'])
    if d['schema_version']!=1:raise ValueError('Unsupported schema')
    if not np.allclose(d['turntable']['axis_direction_table'],[0,0,1]):raise ValueError('Only confirmed vertical-axis envelope is supported')
    for c in d['cameras'].values():matrix(c['T_table_optical'])
    for body in [d['turntable'],*d['pads'].values()]:
        if body['diameter_m']<=0:raise ValueError('Invalid diameter')
    return d

def verify_model(d,description,moveit):
    expected=d['robot_registration'];urdf=Path(description)/'urdf/soarm4.urdf';model=Path(description)/'config/model.json'
    for path,key in [(urdf,'urdf_sha256'),(model,'model_sha256')]:
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected[key]:raise ValueError('Registration model changed: '+str(path))
    # Reuse the MoveIt package's own audit guard.
    import json
    manifest=json.loads((Path(moveit)/'config/model_manifest.json').read_text())
    if hashlib.sha256(urdf.read_bytes()).hexdigest()!=manifest['urdf_sha256']:raise ValueError('MoveIt URDF manifest mismatch')
    for name,digest in manifest['meshes'].items():
        if hashlib.sha256((Path(description)/'meshes'/name).read_bytes()).hexdigest()!=digest:raise ValueError('Audited mesh mismatch: '+name)

def collision_geometry(d):
    """All geometry returned in existing URDF 'world', never changing robot model."""
    T=np.linalg.inv(matrix(d['T_table_world']));depth=d['collision']['below_table_guard_depth_m'];gap=d['collision']['support_contact_clearance_m'];L,W=d['table']['size_xy_m']
    if not 0<=gap<.005 or depth<=0:raise ValueError('Invalid below-table guard')
    values=[('workcell_table_guard','box',[L,W,depth],[L/2,W/2,-depth/2-gap])]
    t=d['turntable'];p=t['axis_point_table_m'];h=t['top_height_m'];values.append(('workcell_turntable','cylinder',[h,t['diameter_m']/2],[p[0],p[1],h/2]))
    for name,pad in d['pads'].items():
        p=pad['center_table_m'];h=p[2]
        if h<=0:raise ValueError('Pad height must be positive')
        values.append(('workcell_'+name,'cylinder',[h,pad['diameter_m']/2],[p[0],p[1],h/2]))
    for name,o in (d.get('obstacles') or {}).items():
        lo=np.asarray(o['min_table_m'],float);hi=np.asarray(o['max_table_m'],float)
        if lo.shape!=(3,) or np.any(hi<=lo):raise ValueError('Invalid obstacle box: '+name)
        values.append(('workcell_'+name,'box',(hi-lo).tolist(),((lo+hi)/2).tolist()))
    return [{'id':name,'kind':kind,'dimensions':dims,'T_world_object':(T@translated(p)).tolist()} for name,kind,dims,p in values]
