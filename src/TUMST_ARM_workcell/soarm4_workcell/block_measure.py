"""빨간 블록의 윗면 중심을 두 카메라 깊이로 계산한다 (색 분할, Gemini·SAM2 없이 ~1초). 계산·촬영만, 모터·펌프 접근 없음.
같은 장면에서 Gemini+SAM2 결과와 비교해 맞는지 확인한 뒤 반복 시험의 위치 측정에 쓴다.

사용: python3 -m soarm4_workcell.block_measure            # 지금 카메라로 한 장 찍어 측정
      python3 -m soarm4_workcell.block_measure DIR       # 저장된 view_001 폴더(capture 결과)로 측정
"""
import json, sys
from pathlib import Path
import numpy as np
import cv2
import yaml
from scipy import ndimage

CONFIG = Path(__file__).resolve().parents[1] / 'config/workcell.yaml'


def red_mask(bgr, min_area=500):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV); h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    m = (((h < 8) | (h > 172)) & (s > 130) & (v > 60)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)); m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
    if n <= 1: return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return (lab == i) if stats[i, cv2.CC_STAT_AREA] >= min_area else None


def top_face(mask, depth_m, K, T_table_optical, table_z=0.062):
    m = ndimage.binary_erosion(mask, iterations=3); vs, us = np.nonzero(m & (depth_m > 0.2) & (depth_m < 1.5)); z = depth_m[vs, us]
    P = np.stack([(us - K[0, 2]) * z / K[0, 0], (vs - K[1, 2]) * z / K[1, 1], z], 1); P = (T_table_optical[:3, :3] @ P.T).T + T_table_optical[:3, 3]
    P = P[P[:, 2] > table_z]
    if len(P) < 200: return None
    zh = np.percentile(P[:, 2], 97); top = P[P[:, 2] > zh - 0.004]; r = cv2.minAreaRect(np.ascontiguousarray(top[:, :2].astype(np.float32)))
    return {'xy_m': top[:, :2].mean(0), 'z_m': float(np.median(top[:, 2])), 'yaw_deg': float(r[2] % 90), 'n': int(len(top))}


def measure_dir(view_dir, wc=None):
    """capture가 저장한 폴더 하나로 측정. 반환: {'xy_m','z_m','yaw_deg','cam1','cam2','cam_gap_mm'} 또는 None."""
    wc = wc or yaml.safe_load(CONFIG.read_text()); view_dir = Path(view_dir); meta = json.loads((view_dir / 'metadata.json').read_text()); out = {}
    for cam in ('cam1', 'cam2'):
        c = meta['cameras'][cam]; img = cv2.imread(str(view_dir / c['color_file'])); dep = cv2.imread(str(view_dir / c['depth_file']), cv2.IMREAD_UNCHANGED).astype(np.float32) * 1e-3
        K = np.array(c['camera_info']['k']).reshape(3, 3); m = red_mask(img)
        r = top_face(m, dep, K, np.array(wc['cameras'][cam]['T_table_optical'])) if m is not None else None
        if r: out[cam] = r
    if not out: return None
    xy = np.mean([v['xy_m'] for v in out.values()], 0); res = {'xy_m': [round(float(xy[0]), 5), round(float(xy[1]), 5)], 'z_m': round(float(np.mean([v['z_m'] for v in out.values()])), 5),
                                                             'yaw_deg': round(out.get('cam1', next(iter(out.values())))['yaw_deg'], 1), 'cams': sorted(out)}
    for cam, v in out.items(): res[cam] = [round(float(v['xy_m'][0]), 5), round(float(v['xy_m'][1]), 5)]
    if len(out) == 2: res['cam_gap_mm'] = round(float(np.linalg.norm(out['cam1']['xy_m'] - out['cam2']['xy_m']) * 1e3), 1)
    return res


def measure_now(tmp_dir):
    from .camera_calib import capture
    v = capture(tmp_dir, 1)[0]; return measure_dir(v)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    r = measure_dir(argv[0]) if argv else measure_now(str(Path.home() / 'robot_ws/data/block_measure_tmp'))
    print(json.dumps(r, ensure_ascii=False)); return 0 if r else 1


if __name__ == '__main__': raise SystemExit(main())
