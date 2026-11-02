"""Camera-to-table extrinsics from the fixed ArUco markers; no ROS, serial or motor access.

사용 (마커가 테이블 고정 자리에 붙어 있고 카메라에 보여야 함):
    ros2 run soarm_turntable turntable_scan capture --output /tmp/calib   # 몇 번 찍기 (카메라 2대 동시 저장)
    python3 -m soarm4_workcell.camera_calib /tmp/calib/*/view_000 [--json out.json]
저장된 workcell.yaml 값과 비교만 한다. workcell.yaml은 고치지 않는다.
"""
import argparse, json, math
from pathlib import Path
import numpy as np
import cv2
from .core import load, matrix

CONFIG = Path(__file__).resolve().parents[1] / 'config/workcell.yaml'
CAMERAS = ('cam1', 'cam2')


def marker_corners(center, size):
    """ArUco 모서리 순서 TL,TR,BR,BL. 마커 위쪽이 +Y(canonical_top_direction), 법선 +Z. 테이블 좌표(m)."""
    cx, cy, cz = (list(center) + [0.0])[:3]; h = size / 2.0
    return np.array([[cx - h, cy + h, cz], [cx + h, cy + h, cz], [cx + h, cy - h, cz], [cx - h, cy - h, cz]], float)


def table_markers(wc, use_pads=False):
    fm = wc['fixed_markers']; out = {int(k): marker_corners(v, fm['black_edge_m']) for k, v in fm['centers_table_m'].items()}
    if use_pads:  # 패드 마커는 높이가 달라 3D 점으로 들어간다 (z = 패드 윗면)
        for p in wc['pads'].values(): out[int(p['marker_id'])] = marker_corners(p['center_table_m'], p['black_edge_m'])
    return out


def detect(image_bgr, ids_wanted):
    d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50); p = cv2.aruco.DetectorParameters_create()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(image_bgr, d, parameters=p)
    if ids is None: return {}
    return {int(i): c.reshape(4, 2).astype(float) for i, c in zip(ids.ravel(), corners) if int(i) in ids_wanted}


def solve(frames, markers, K, D=None, init_T_table_optical=None):
    """frames: [{id: 4x2 픽셀}]. 모든 프레임·마커의 모서리를 한 번에 맞춰 T_table_optical(광학 → 테이블)을 구한다."""
    obj, img = [], []
    for f in frames:
        for i, c in f.items():
            if i in markers: obj.append(markers[i]); img.append(c)
    if len(obj) < 1: raise ValueError('사용할 마커가 없습니다')
    obj = np.vstack(obj).astype(np.float64); img = np.vstack(img).astype(np.float64)
    K = np.asarray(K, float); D = np.zeros(5) if D is None else np.asarray(D, float)
    rvec = tvec = None
    if init_T_table_optical is not None:  # 초기값으로만 쓴다 (결과는 픽셀로 다시 맞춤)
        Tc = np.linalg.inv(np.asarray(init_T_table_optical, float)); rvec = cv2.Rodrigues(Tc[:3, :3])[0]; tvec = Tc[:3, 3:4].copy()
    planar = np.ptp(obj[:, 2]) < 1e-6
    if rvec is None:
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_IPPE if planar and len(obj) >= 4 else cv2.SOLVEPNP_EPNP)
        if not ok: raise RuntimeError('PnP 실패')
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, rvec, tvec, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    proj = cv2.projectPoints(obj, rvec, tvec, K, D)[0].reshape(-1, 2); err = np.hypot(*(proj - img).T)
    Tc = np.eye(4); Tc[:3, :3] = cv2.Rodrigues(rvec)[0]; Tc[:3, 3] = tvec.ravel()
    return np.linalg.inv(Tc), {'corners': int(len(obj)), 'reproj_rms_px': float(np.sqrt(np.mean(err ** 2))), 'reproj_max_px': float(err.max())}


def gate(frame, markers, T_old, K, max_center_px=150.0, size_ratio=(0.6, 1.6)):
    """저장값으로 예상한 위치·크기와 너무 다른 검출은 가짜(모니터 화면 등)로 보고 버린다. (사용, 버림 이유) 반환."""
    Tc = np.linalg.inv(np.asarray(T_old, float)); K = np.asarray(K, float); ok, bad = {}, {}
    for i, c in frame.items():
        p = (Tc[:3, :3] @ markers[i].T + Tc[:3, 3:4]).T
        if np.any(p[:, 2] <= 0): bad[i] = '카메라 뒤쪽으로 예상'; continue
        uv = (K @ p.T).T; uv = uv[:, :2] / uv[:, 2:3]
        dc = float(np.linalg.norm(uv.mean(0) - c.mean(0))); ratio = float(np.mean([np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)]) / np.mean([np.linalg.norm(uv[k] - uv[(k + 1) % 4]) for k in range(4)]))
        if dc > max_center_px: bad[i] = f'예상 위치에서 {dc:.0f}px 떨어짐'
        elif not size_ratio[0] <= ratio <= size_ratio[1]: bad[i] = f'크기가 예상의 {ratio:.2f}배'
        else: ok[i] = c
    return ok, bad


def compare(T_new, T_old):
    """새 값과 저장 값의 차이: 카메라 위치(mm), 방향(deg)."""
    T_new, T_old = np.asarray(T_new, float), np.asarray(T_old, float)
    dR = T_old[:3, :3].T @ T_new[:3, :3]; ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2))))
    dt = (T_new[:3, 3] - T_old[:3, 3]) * 1e3
    return {'shift_mm': dt.round(1).tolist(), 'shift_norm_mm': round(float(np.linalg.norm(dt)), 1), 'rotation_deg': round(ang, 2)}


def table_depth_check(T, K, depth_m, z_max=0.03):
    """깊이로 본 테이블 면 높이(중앙값, mm). 카메라 위치가 맞으면 0에 가깝다. 턴테이블·로봇 쪽 점은 z 필터로 거른다."""
    h, w = depth_m.shape; v, u = np.mgrid[0:h:3, 0:w:3]; z = depth_m[v, u]; ok = (z > 0.1) & (z < 1.5)
    u, v, z = u[ok], v[ok], z[ok]; x = (u - K[0][2]) * z / K[0][0]; y = (v - K[1][2]) * z / K[1][1]
    P = (np.stack([x, y, z, np.ones_like(z)], 1) @ np.asarray(T).T)[:, :3]
    s = P[(P[:, 2] > -0.05) & (P[:, 2] < z_max)]
    return None if len(s) < 200 else round(float(np.median(s[:, 2])) * 1e3, 1)


def main(argv=None):
    ap = argparse.ArgumentParser(description='고정 ArUco 마커로 카메라 위치 보정 값 계산·비교 (workcell.yaml은 안 고침)')
    ap.add_argument('sessions', nargs='+', help='turntable_scan capture 가 만든 view_000 폴더들')
    ap.add_argument('--workcell', default=str(CONFIG)); ap.add_argument('--use-pads', action='store_true'); ap.add_argument('--json')
    ap.add_argument('--warn-mm', type=float, default=5.0, help='이 값 넘게 어긋나면 경고')
    a = ap.parse_args(argv); wc = load(Path(a.workcell)); markers = table_markers(wc, a.use_pads); result = {}
    for cam in CAMERAS:
        frames, K, depth = [], None, None
        for s in a.sessions:
            s = Path(s); meta = json.loads((s / 'metadata.json').read_text()); c = meta['cameras'][cam]; K = np.array(c['camera_info']['k']).reshape(3, 3)
            f = detect(cv2.imread(str(s / c['color_file'])), set(markers))
            if f: frames.append(f)
            if depth is None and c['depth_encoding'] == '16UC1': depth = cv2.imread(str(s / c['depth_file']), cv2.IMREAD_UNCHANGED).astype(np.float32) * 1e-3
        seen = sorted({i for f in frames for i in f}); print(f'[{cam}] 사진 {len(a.sessions)}장 중 마커가 보인 사진 {len(frames)}장, 보인 마커 ID {seen}')
        if not frames: print('  → 사용할 마커가 안 보입니다. 마커를 붙이고 카메라에 보이게 하세요.'); continue
        old = matrix(wc['cameras'][cam]['T_table_optical']); gated = []; rejected = {}
        for f in frames:
            okf, bad = gate(f, markers, old, K); rejected.update(bad)
            if okf: gated.append(okf)
        for i, why in sorted(rejected.items()): print(f'  버림: ID{i} ({why}) — 가짜 검출이거나 카메라가 크게 움직였을 수 있음')
        used = sorted({i for f in gated for i in f})
        if len(used) < 2: print(f'  → 쓸 수 있는 마커가 {len(used)}개뿐이라 계산하지 않습니다 (2개 이상 필요, 4개 권장).'); continue
        T, q = solve(gated, markers, K, init_T_table_optical=old); cmp_ = compare(T, old); print(f'  사용한 마커 ID {used}')
        old = matrix(wc['cameras'][cam]['T_table_optical']); T, q = solve(frames, markers, K, init_T_table_optical=old); cmp_ = compare(T, old)
        dz_old = table_depth_check(old, K, depth) if depth is not None else None; dz_new = table_depth_check(T, K, depth) if depth is not None else None
        print(f"  재투영 오차 rms {q['reproj_rms_px']:.2f}px (최대 {q['reproj_max_px']:.2f}px, 모서리 {q['corners']}개)")
        print(f"  저장값과 차이: 카메라 위치 {cmp_['shift_mm']} mm (크기 {cmp_['shift_norm_mm']} mm), 방향 {cmp_['rotation_deg']}°")
        if dz_old is not None: print(f'  깊이로 본 테이블 면 높이(0이어야 함): 저장값 {dz_old} mm, 새 값 {dz_new} mm')
        if q['reproj_rms_px'] > 3.0: print('  ⚠ 재투영 오차가 큼(>3px): 마커가 휘었거나 workcell.yaml의 마커 위치와 다릅니다. 이 값은 믿지 마세요.')
        elif cmp_['shift_norm_mm'] > a.warn_mm or cmp_['rotation_deg'] > 0.3: print(f'  ⚠ 어긋남 큼 (기준 {a.warn_mm} mm / 0.3°): 카메라가 움직였거나 마커 위치가 workcell.yaml과 다릅니다.')
        else: print('  ✓ 저장값과 일치')
        result[cam] = {'T_table_optical': T.tolist(), **q, **cmp_}
    if a.json: Path(a.json).write_text(json.dumps(result, indent=1, ensure_ascii=False)); print('저장:', a.json)
    return 0


if __name__ == '__main__': raise SystemExit(main())
