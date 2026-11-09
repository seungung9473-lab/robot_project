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



def initial_date_views(ranked, angles):
    """Keep two bright label views and add a distant view if the top three cluster."""
    selected = ranked[:3]
    if len(selected) < 3:
        return selected

    def angle(candidate):
        return angles.get(candidate[2].rsplit('/', 1)[0])

    reference = angle(selected[0])
    if reference is None or any(angle(candidate) is None for candidate in selected):
        return selected

    def separation(candidate):
        return abs((angle(candidate) - reference + 180) % 360 - 180)

    if any(separation(candidate) > 65 for candidate in selected[1:]):
        return selected
    alternatives = [candidate for candidate in ranked[3:]
                    if angle(candidate) is not None and candidate[0] >= .35 * selected[0][0]
                    and separation(candidate) >= 90]
    if alternatives:
        selected[2] = max(alternatives, key=lambda candidate: (separation(candidate), candidate[0]))
    return selected



def additional_date_views(ranked, angles, observations):
    """Prefer angles near a partial reading. Consensus still uses its own thresholds."""
    def angle(evidence_id):
        return angles.get(evidence_id.rsplit('/', 1)[0])

    hints = [angle(row['evidence_id']) for row in observations
             if row.get('date_text') and row.get('confidence', 0) >= .8]
    hints = [value for value in hints if value is not None]
    if not hints:
        return initial_date_views(ranked, angles)

    def rank(candidate):
        value = angle(candidate[2])
        distance = min(abs((value - hint + 180) % 360 - 180) for hint in hints) if value is not None else float('inf')
        return distance, -candidate[0]

    return sorted(ranked, key=rank)[:3]
