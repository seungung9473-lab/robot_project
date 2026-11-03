"""[사용] ~/sam2_env/bin/python recon.py SESSION_DIR sam2.npz OUT_cloud.npz
SAM2 마스크 + 정렬 깊이 → 테이블 좌표 점군 → 12방향 융합 → 위를 향한 면(흡착 후보). 계산만."""
import os, sys, json, numpy as np, cv2, yaml
from scipy import ndimage
session, npz, out = sys.argv[1:4]
wc = yaml.safe_load(open(os.path.expanduser('~/robot_ws/src/soarm4_workcell/config/workcell.yaml')))
axis = np.array(wc['turntable']['axis_point_table_m'][:2]); top_z = wc['turntable']['top_height_m']
masks = np.load(npz); sess = json.load(open(f'{session}/session.json')); views = [v for v in sess['views']]
def cloud(view, cam):
    meta = json.load(open(f"{session}/{view['path']}/metadata.json"))['cameras'][cam]; K = np.array(meta['camera_info']['k']).reshape(3, 3)
    dep = cv2.imread(f"{session}/{view['path']}/{meta['depth_file']}", cv2.IMREAD_UNCHANGED).astype(np.float32) * 1e-3
    m = masks[f"{view['path']}_{cam}"]; m = ndimage.binary_erosion(m, iterations=3)
    vs, us = np.nonzero(m & (dep > 0.2) & (dep < 1.5)); z = dep[vs, us]
    P = np.stack([(us - K[0, 2]) * z / K[0, 0], (vs - K[1, 2]) * z / K[1, 1], z], 1)
    T = np.array(wc['cameras'][cam]['T_table_optical']); return (T[:3, :3] @ P.T).T + T[:3, 3]
def rot(P, deg):
    a = np.radians(deg); R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]); Q = P.copy(); Q[:, :2] = (P[:, :2] - axis) @ R.T + axis; return Q
raw = {(v['path'], c): cloud(v, c) for v in views for c in ('cam1', 'cam2')}; ang = {v['path']: v['angle_deg'] for v in views}
def fuse(sign, cams=('cam1', 'cam2'), ax_shift=(0, 0)):
    global axis
    old = axis; axis = old + np.array(ax_shift); P = np.vstack([rot(raw[(p, c)], -sign * ang[p]) for (p, c) in raw if c in cams]); axis = old; return P
def spread(P):
    P = P[(P[:, 2] > top_z + 0.003)]; v = P[:, :2][::5]; return np.sqrt(abs(np.linalg.det(np.cov(v.T)))) if len(v) > 10 else 1e9
def top_band(P, band=0.004):
    zh = np.percentile(P[:, 2], 97); return P[P[:, 2] > zh - band]
def yaw_of(P):  # 윗면 점들의 최소 외접 사각형 각도 (0~90 도로 접음)
    T = top_band(P); r = cv2.minAreaRect(np.ascontiguousarray(T[:, :2].astype(np.float32)[::2])); return r[2] % 90, r[1]
yaws = {}
for v in views:
    P = np.vstack([raw[(v['path'], c)] for c in ('cam1', 'cam2')]); P = P[P[:, 2] > top_z + 0.003]; yaws[v['path']] = yaw_of(P)[0]
y0 = yaws[views[0]['path']]
def wrap(a): return (a + 45) % 90 - 45
score = {}
for s in (1, -1):
    err = [abs(wrap((yaws[v['path']] - y0) - s * ang[v['path']])) for v in views[1:]]; score[s] = float(np.mean(err))
sign = min(score, key=score.get)
print('윗면 각도가 돌아간 방향 시험 (평균 어긋남 °, 작은 쪽이 맞음):', {k: round(v, 1) for k, v in score.items()}, '→ 부호', sign)
print('각 각도에서 본 윗면 각도 (°):', {v['path'][-3:]: round(yaws[v['path']], 1) for v in views})
def hull_area(P):
    T = np.ascontiguousarray(top_band(P[P[:, 2] > top_z + 0.003])[:, :2].astype(np.float32)[::3]); return cv2.contourArea(cv2.convexHull(T))
base = hull_area(fuse(sign)); best = (base, (0.0, 0.0))
for dx in np.arange(-0.012, 0.0121, 0.002):
    for dy in np.arange(-0.012, 0.0121, 0.002):
        a = hull_area(fuse(sign, ax_shift=(dx, dy)))
        if a < best[0]: best = (a, (dx, dy))
print('윗면 겹침 면적: 저장된 축 %.1f cm², 최적 이동 %s mm → %.1f cm²' % (base * 1e4, np.round(np.array(best[1]) * 1e3, 1), best[0] * 1e4))
P = fuse(sign, ax_shift=best[1]) if best[0] < base * 0.97 else fuse(sign)
P = P[(P[:, 2] > top_z + 0.003) & (P[:, 2] < 0.2)]
c = np.median(P[:, :2], 0); d = np.linalg.norm(P[:, :2] - c, axis=1); P = P[d < np.percentile(d, 99.5)]
np.savez_compressed(out, points=P, sign=sign, axis_shift=np.array(best[1]) if best[0] < base * 0.97 else np.zeros(2))
print('융합 점 수', len(P), ' x', P[:, 0].min().round(4), P[:, 0].max().round(4), ' y', P[:, 1].min().round(4), P[:, 1].max().round(4), ' z', P[:, 2].min().round(4), P[:, 2].max().round(4))
