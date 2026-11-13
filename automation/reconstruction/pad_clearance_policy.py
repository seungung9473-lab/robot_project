"""Decide whether the destination pad may be used, from the live pad observation.

- Anything standing on the pad that the cameras can see always stops the batch.
- The reserved slot must normally be seen (enough floor pixels). When the arm/turntable hide it,
  the start-of-batch free-slot check (which reserved this slot while the arm was at home and the view was clear)
  stands in; only our own placements enter a pad during a batch and those are tracked separately.
- Without that start-of-batch evidence an unseen pad still stops the batch.
"""
MIN_SURFACE_PIXELS=100
MAX_ELEVATED_PIXELS=40

class PadObstructed(RuntimeError):
    pass


def pad_observation(view, pad, workcell, polygon=None):
    import json
    import cv2
    import numpy as np
    from scipy.spatial import ConvexHull
    from placement_geometry import depth_table

    metadata = json.loads((view/'metadata.json').read_text())
    center = np.array(pad['center_table_m'])
    hull = ConvexHull(polygon) if polygon is not None else None
    checks = {}
    for cam in ('cam1', 'cam2'):
        c = metadata['cameras'][cam]
        raw = cv2.imread(str(view/c['depth_file']), -1)
        points, valid = depth_table(raw, .001 if raw.dtype == np.uint16 else 1.,
                                    c['camera_info'], workcell['cameras'][cam]['T_table_optical'])
        radius = np.linalg.norm(points[:, :, :2]-center[:2], axis=2)
        height = points[:, :, 2]-center[2]
        floor = radius < .04
        raised = valid & (radius < .048) & (height > .012) & (points[:, :, 2] < .25)
        if hull is not None:
            floor = np.all(points[:, :, :2] @ hull.equations[:, :2].T + hull.equations[:, 2] <= .002, axis=2)
            raised &= floor
        checks[cam] = {'surface_pixels': int((valid & floor & (abs(height) < .012)).sum()),
                       'elevated_pixels': int(raised.sum())}
    return checks


def judge_pad(checks,start_of_batch_check):
    seen=[f"{cam}: floor {c['surface_pixels']} px, raised {c['elevated_pixels']} px" for cam,c in checks.items()]
    if any(c['elevated_pixels']>=MAX_ELEVATED_PIXELS for c in checks.values()):
        raise PadObstructed('Left pad obstructed: object seen on pad ('+'; '.join(seen)+')')
    if max(c['surface_pixels'] for c in checks.values())>=MIN_SURFACE_PIXELS:return 'observed'
    if start_of_batch_check:return 'start_of_batch_check'
    raise RuntimeError('Pad not observed sufficiently and no start-of-batch free-slot check ('+'; '.join(seen)+')')
