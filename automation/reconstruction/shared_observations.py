"""Calibrate each stopped depth frame once, then apply independent object masks."""
import json
from pathlib import Path
import cv2,numpy as np
from scipy.spatial.transform import Rotation
from placement_geometry import depth_table

def collect_plane_observations(session,objects,workcell):
    session=Path(session);objects=list(objects)
    result={obj.name:{c:[] for c in ['cam1','cam2']} for obj in objects}
    if not objects:return result
    manifest=json.loads((session/'session.json').read_text());axis=np.asarray(workcell['turntable']['axis_point_table_m'])
    for view in manifest['views']:
        folder=session/view['path'];metadata=json.loads((folder/'metadata.json').read_text())
        rotation=Rotation.from_euler('z',-metadata['turntable']['angle_deg'],degrees=True).as_matrix()
        for camera in ['cam1','cam2']:
            info=metadata['cameras'][camera];raw=cv2.imread(str(folder/info['depth_file']),-1)
            if raw is None:raise ValueError('Missing stopped depth frame')
            points,valid=depth_table(raw,info['depth_scale_m'],info['camera_info'],workcell['cameras'][camera]['T_table_optical'])
            for obj in objects:
                mask=cv2.imread(str(obj/'sam2'/view['path']/(camera+'_mask.png')),0)
                if mask is None or mask.shape!=valid.shape:raise ValueError('Missing/mismatched object mask')
                selected=points[valid&(mask>127)];selected=(selected-axis)@rotation.T+axis
                if len(selected)>=120:result[obj.name][camera].append((view['path']+'/'+camera,selected))
    return result
