"""Calibrated turntable support coordinates, independent of object position."""
import json
from pathlib import Path
import cv2,numpy as np,yaml
ROOT=Path(__file__).resolve().parent.parent

def load_context(session):
    path=Path(session)/'spatial_context.json'
    if path.exists():return json.loads(path.read_text())
    cfg=yaml.safe_load((Path.home()/'robot_ws/src/TUMST_ARM_workcell/config/workcell.yaml').read_text())['turntable']
    active=json.loads((ROOT/'camera_calibration/active.json').read_text())
    axis=np.asarray(cfg['axis_direction_table'],dtype=float)
    if not np.allclose(axis,[0,0,1],atol=1e-6):raise ValueError('Vertical-axis reconstruction required; recalibrate tilted axis')
    data={'axis_point_table_m':cfg['axis_point_table_m'],'axis_direction_table':axis.tolist(),'support_radius_m':float(cfg['diameter_m'])/2,'observation_radius_m':float(cfg['diameter_m'])/2+.03,'top_height_m':float(cfg['top_height_m']),'calibrations':{c:json.loads(Path(p).read_text()) for c,p in active['calibrations'].items()},'object_position_assumed_at_axis':False}
    path.write_text(json.dumps(data,indent=2)+'\n');return data

def depth_table(raw,scale,info,T):
    z=raw.astype(float)*scale;v,u=np.indices(z.shape)
    pixels=np.column_stack([u.ravel(),v.ravel()]).astype(float)
    rays=cv2.undistortPoints(pixels.reshape(-1,1,2),np.array(info['k']).reshape(3,3),np.array(info['d'])).reshape(-1,2)
    p=np.column_stack([rays*z.reshape(-1,1),z.ravel()]);T=np.asarray(T,dtype=float)
    return (p@T[:3,:3].T+T[:3,3]).reshape(*raw.shape,3),np.isfinite(z)&(z>.08)&(z<.9)

def object_region(points,context):
    center=np.asarray(context['axis_point_table_m'])
    return (np.linalg.norm(points[...,:2]-center[:2],axis=-1)<context['observation_radius_m'])&(points[...,2]>context['top_height_m']+.006)&(points[...,2]<context['top_height_m']+.25)

def contact_target(nominal,offset):
    result=np.array(nominal,dtype=float).copy();result[2]+=float(offset);return result
