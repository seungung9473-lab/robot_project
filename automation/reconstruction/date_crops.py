"""Prepare unchanged per-object OCR evidence with one RGB read per view/camera."""
from pathlib import Path
import json
import cv2
import numpy as np

def collect_date_crops(session,manifest,objects):
    session=Path(session);objects=list(objects);candidates={o.name:[] for o in objects}
    for view in manifest['views']:
        folder=session/view['path'];metadata=json.loads((folder/'metadata.json').read_text())
        for camera in ['cam1','cam2']:
            image=cv2.imread(str(folder/metadata['cameras'][camera]['color_file']))
            if image is None:raise ValueError('Numeric evidence image missing')
            hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
            white=(hsv[:,:,1]<70)&(hsv[:,:,2]>155)
            for obj in objects:
                mask=cv2.imread(str(obj/'sam2'/view['path']/(camera+'_mask.png')),0)
                if mask is None or mask.shape!=image.shape[:2]:raise ValueError('Numeric evidence mask missing/mismatched')
                foreground=mask>127;y,x=np.nonzero(foreground)
                if len(x)<100:continue
                score=int(np.count_nonzero(white&foreground))
                x0,y0=max(0,int(x.min())-5),max(0,int(y.min())-5)
                x1,y1=min(image.shape[1],int(x.max())+6),min(image.shape[0],int(y.max())+6)
                crop=image[y0:y1,x0:x1].copy();crop[~foreground[y0:y1,x0:x1]]=128
                candidates[obj.name].append((score,camera,view['path']+'/'+camera,crop))
    return candidates
