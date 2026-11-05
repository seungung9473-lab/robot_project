"""Provisional pitch compensation at fixed TCP; no motor zero changes or I/O."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def compensated_axis(fk, q, config_path=None):
    path=Path(config_path) if config_path else Path(__file__).with_name('sag_compensation.json')
    cfg=json.loads(path.read_text())
    if not cfg['enabled']:
        return fk(q)[:3,0].copy(), {'enabled':False}
    q=np.asarray(q,dtype=float)
    up=-fk(q)[:3,0]
    probe=q.copy();probe[3]+=1e-5
    tangent=(-fk(probe)[:3,0]-up)/1e-5
    axis=np.cross(up,tangent)
    norm=np.linalg.norm(axis)
    if norm<1e-8:raise ValueError('Pitch compensation axis undefined')
    axis/=norm
    angle=float(cfg['pitch_offset_deg'])
    if not np.isfinite(angle) or abs(angle)>float(cfg['max_abs_pitch_deg']):
        raise ValueError('Pitch compensation exceeds configured bound')
    nominal=np.asarray(cfg['nominal_tool_axis_table'],dtype=float)
    if not np.isclose(np.linalg.norm(nominal),1):raise ValueError('Nominal axis must be unit length')
    result=Rotation.from_rotvec(axis*np.deg2rad(angle)).apply(nominal)
    return result,{'enabled':True,'pitch_offset_deg':angle,'axis_table':axis.tolist(),'target_tool_axis_table':result.tolist(),'physically_validated':False}
