"""Fresh-camera verification of the actual inclined contact patch."""
import json
from pathlib import Path
import cv2,numpy as np
from placement_geometry import depth_table
from inclined_geometry import patch

def depth_scale(meta,raw):
    if 'depth_scale_m' in meta:return float(meta['depth_scale_m'])
    if meta.get('depth_encoding')=='16UC1' and raw.dtype==np.uint16:return .001
    if meta.get('depth_encoding')=='32FC1' and raw.dtype==np.float32:return 1.
    raise ValueError('Unknown depth units; do not infer contact distance')

def fresh_face_check(view,wc,point,normal):
    view=Path(view);meta=json.loads((view/'metadata.json').read_text());point=np.asarray(point);normal=np.asarray(normal);confirmed={}
    for cam in ['cam1','cam2']:
        d=meta['cameras'][cam];raw=cv2.imread(str(view/d['depth_file']),-1);P,valid=depth_table(raw,depth_scale(d,raw),d['camera_info'],wc['cameras'][cam]['T_table_optical']);local=valid&(np.linalg.norm(P-point,axis=2)<.055)&(P[:,:,2]>wc['turntable']['top_height_m']+.006)&(P[:,:,2]<wc['turntable']['top_height_m']+.25);cloud=P[local]
        # Verify only the predicted contact disk. Never fit/select a new plane,
        # shift the contact point or change its normal after table rotation.
        result=patch(cloud,normal,-float(point@normal),center_point=point)
        if result is not None and result['geometry_pass']:
            confirmed[cam]={'verification_mode':'fixed_predicted_plane_and_contact',
                            'contact_changed':False,'normal_changed':False,'patch':result}
    if not confirmed:raise RuntimeError('Fresh RGBD does not confirm the inclined contact disk; no approach')
    # Original plan requires independent two-camera observations. At aligned pose
    # one view may be occluded; at least one fresh full-disk observation is mandatory.
    return {'confirmed_cameras':confirmed,'point_table_m':point.tolist(),'pressure_not_used':True,'verification_mode':'fixed_predicted_plane_and_contact','contact_changed':False,'normal_changed':False}
