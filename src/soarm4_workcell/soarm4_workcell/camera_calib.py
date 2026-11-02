"""Camera-to-table extrinsics from the fixed ArUco markers; no ROS, serial or motor access.

사용 (마커가 테이블 고정 자리에 붙어 있고 카메라에 보여야 함):
    python3 -m soarm4_workcell.camera_calib --capture /tmp/calib            # 카메라 2대 노드에서 바로 몇 장 찍어 계산
    python3 -m soarm4_workcell.camera_calib /tmp/calib/view_*              # 이미 찍은 view 폴더로 계산
    --fresh : 카메라를 옮겨 저장값이 쓸모없을 때 (저장값으로 가짜 검출 거르기·초기값 안 씀)
    --write : 결과를 workcell.yaml 카메라 값에 반영 (기본은 비교만)
보정에 안 쓴 ID0(턴테이블)·패드 마커 중심을 새 값으로 다시 계산해 측정 좌표와 비교한다.
"""
import argparse, json, math
from pathlib import Path
import numpy as np
import cv2
from .core import load, matrix

CONFIG = Path(__file__).resolve().parents[1] / 'config/workcell.yaml'
CAMERAS = ('cam1', 'cam2')
CHECK_IDS = (0, 5, 6)


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


def marker_rms(T, frames, markers, K):
    Tc = np.linalg.inv(T); rvec = cv2.Rodrigues(Tc[:3, :3])[0]; per = {}
    for f in frames:
        for i, c in f.items():
            e = np.hypot(*(cv2.projectPoints(markers[i], rvec, Tc[:3, 3], np.asarray(K, float), np.zeros(5))[0].reshape(-1, 2) - c).T); per.setdefault(i, []).extend(e)
    return {i: float(np.sqrt(np.mean(np.square(e)))) for i, e in per.items()}


def solve_robust(frames, markers, K, init_T_table_optical=None, max_marker_px=4.0):
    """모든 마커로 풀고, 어느 마커가 크게 어긋나면 마커를 하나씩 빼 보며 나머지가 가장 잘 맞는 경우를 고른다
    (잘못 붙였거나 가짜인 마커 1개 제거). 3개 이상 남을 때만 뺀다. (T, 품질, 뺀 ID) 반환."""
    def run(fs):
        T, q = solve(fs, markers, K, init_T_table_optical=init_T_table_optical); per = marker_rms(T, fs, markers, K)
        q['per_marker_rms_px'] = {str(i): round(v, 2) for i, v in sorted(per.items())}; return T, q, per
    T, q, per = run(frames); ids = sorted(per)
    if max(per.values()) <= max_marker_px or len(ids) < 4: return T, q, []
    best = None
    for i in ids:
        fs = [{k: c for k, c in f.items() if k != i} for f in frames]; fs = [f for f in fs if f]
        r = run(fs)
        if best is None or r[1]['reproj_rms_px'] < best[1][1]['reproj_rms_px']: best = (i, r)
    i, (T2, q2, per2) = best
    return (T2, q2, [i]) if max(per2.values()) <= max_marker_px else (T, q, [])


def check_markers(wc):
    """보정에 안 쓰는 확인용 마커: id → (측정 중심 xyz, 한 변 길이)."""
    t = wc['turntable']; out = {int(t.get('marker_id', 0)): (np.asarray(t['axis_point_table_m'], float), float(t.get('marker_black_edge_m', 0.1)))}
    for p in wc['pads'].values(): out[int(p['marker_id'])] = (np.asarray(p['center_table_m'], float), float(p['black_edge_m']))
    return out


def marker_center(c):
    """사각형 마커의 진짜 중심 = 두 대각선의 교점 (비스듬히 보면 네 모서리 평균은 중심이 아님)."""
    c = np.asarray(c, float); d1 = c[2] - c[0]; d2 = c[3] - c[1]; A = np.array([d1, -d2]).T
    t = np.linalg.solve(A, c[1] - c[0])[0]; return c[0] + t * d1


def locate_on_plane(T, K, uv, z):
    """픽셀 uv를 지나는 광선이 테이블 높이 z 평면과 만나는 점 (테이블 좌표)."""
    K = np.asarray(K, float); d = T[:3, :3] @ np.array([(uv[0] - K[0, 2]) / K[0, 0], (uv[1] - K[1, 2]) / K[1, 1], 1.0]); o = T[:3, 3]
    return o + d * (z - o[2]) / d[2]


def validate(T, K, frames, wc):
    """확인용 마커마다 검출 중심(여러 장 평균)을 평면에 올려 측정 좌표와의 차이(mm)를 낸다."""
    out = {}
    for i, (ctr, _) in check_markers(wc).items():
        uvs = [marker_center(f[i]) for f in frames if i in f]
        if not uvs: continue
        p = locate_on_plane(T, K, np.mean(uvs, 0), ctr[2]); d = (p[:2] - ctr[:2]) * 1e3
        out[i] = {'measured_xy_m': ctr[:2].round(4).tolist(), 'seen_xy_m': p[:2].round(4).tolist(), 'dx_mm': round(float(d[0]), 1), 'dy_mm': round(float(d[1]), 1), 'err_mm': round(float(np.hypot(*d)), 1)}
    return out


def capture(out_dir, count=5, gap_s=0.3, timeout_s=15.0):
    """두 카메라의 컬러·정렬 깊이·내부행렬을 count장 저장 (읽기 전용, 모터 안 씀). 저장한 view 폴더 목록 반환."""
    import time, rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from rosidl_runtime_py import message_to_ordereddict
    rclpy.init(); n = rclpy.create_node('camera_calib_capture'); last = {}
    for c in CAMERAS:
        for kind, typ, topic in (('color', Image, 'color/image_raw'), ('depth', Image, 'aligned_depth_to_color/image_raw'), ('info', CameraInfo, 'color/camera_info')):
            n.create_subscription(typ, f'/camera/{c}/{topic}', lambda m, k=(c, kind): last.__setitem__(k, m), qos_profile_sensor_data)
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True); views = []
    try:
        for v in range(count):
            for k in list(last): del last[k]
            end = time.monotonic() + timeout_s
            while len(last) < 6:
                rclpy.spin_once(n, timeout_sec=0.05)
                if time.monotonic() > end: raise RuntimeError('카메라 영상이 안 들어옵니다 (없는 것: ' + str(sorted({(c, k) for c in CAMERAS for k in ('color', 'depth', 'info')} - set(last))) + ')')
            d = out / f'view_{v:03d}'; d.mkdir(exist_ok=True); meta = {'cameras': {}}
            for c in CAMERAS:
                m = last[(c, 'color')]; a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
                cv2.imwrite(str(d / f'{c}_color.png'), cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a)
                z = last[(c, 'depth')]; za = np.frombuffer(z.data, np.uint16).reshape(z.height, z.step // 2)[:, :z.width]; cv2.imwrite(str(d / f'{c}_depth_mm.png'), za)
                meta['cameras'][c] = {'color_file': f'{c}_color.png', 'depth_file': f'{c}_depth_mm.png', 'depth_encoding': z.encoding, 'camera_info': message_to_ordereddict(last[(c, 'info')])}
            (d / 'metadata.json').write_text(json.dumps(meta, indent=1)); views.append(d); time.sleep(gap_s)
    finally:
        n.destroy_node(); rclpy.try_shutdown()
    return views


def write_workcell(path, results, Ks, note):
    import yaml
    wc = yaml.safe_load(Path(path).read_text())
    for cam, r in results.items():
        c = wc['cameras'][cam]; c['T_table_optical'] = r['T_table_optical']; c['camera_info']['K'] = np.asarray(Ks[cam], float).tolist()
        c['source'] = note; c['independently_validated'] = bool(r.get('check_ok'))
        c['check_markers_mm'] = {str(i): v['err_mm'] for i, v in r.get('check', {}).items()}
    Path(path).write_text(yaml.safe_dump(wc, sort_keys=False))


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
    ap = argparse.ArgumentParser(description='고정 ArUco 마커로 카메라 위치 보정 값 계산·비교')
    ap.add_argument('sessions', nargs='*', help='view 폴더들 (metadata.json 있는 곳)')
    ap.add_argument('--capture', metavar='DIR', help='카메라 노드에서 바로 찍어 DIR에 저장하고 계산'); ap.add_argument('--frames', type=int, default=5)
    ap.add_argument('--workcell', default=str(CONFIG)); ap.add_argument('--use-pads', action='store_true'); ap.add_argument('--json')
    ap.add_argument('--fresh', action='store_true', help='카메라를 옮겨서 저장값을 거르기·초기값으로 쓰지 않음')
    ap.add_argument('--write', action='store_true', help='결과를 workcell.yaml 카메라 값에 반영')
    ap.add_argument('--warn-mm', type=float, default=5.0, help='이 값 넘게 어긋나면 경고')
    a = ap.parse_args(argv); wc = load(Path(a.workcell)); markers = table_markers(wc, a.use_pads); result = {}; Ks = {}
    sessions = [str(v) for v in capture(a.capture, a.frames)] if a.capture else a.sessions
    if not sessions: ap.error('view 폴더나 --capture DIR 가 필요합니다')
    wanted = set(markers) | set(check_markers(wc))
    for cam in CAMERAS:
        frames, checks, K, depth = [], [], None, None
        for s in sessions:
            s = Path(s); meta = json.loads((s / 'metadata.json').read_text()); c = meta['cameras'][cam]; K = np.array(c['camera_info']['k']).reshape(3, 3)
            f = detect(cv2.imread(str(s / c['color_file'])), wanted); checks.append({i: v for i, v in f.items() if i not in markers}); f = {i: v for i, v in f.items() if i in markers}
            if f: frames.append(f)
            if depth is None and c['depth_encoding'] == '16UC1': depth = cv2.imread(str(s / c['depth_file']), cv2.IMREAD_UNCHANGED).astype(np.float32) * 1e-3
        Ks[cam] = K; seen = sorted({i for f in frames for i in f}); print(f'[{cam}] 사진 {len(sessions)}장 중 마커가 보인 사진 {len(frames)}장, 보인 고정 마커 ID {seen}')
        if not frames: print('  → 사용할 마커가 안 보입니다. 마커를 붙이고 카메라에 보이게 하세요.'); continue
        old = matrix(wc['cameras'][cam]['T_table_optical'])
        if a.fresh: gated = frames
        else:
            gated = []; rejected = {}
            for f in frames:
                okf, bad = gate(f, markers, old, K); rejected.update(bad)
                if okf: gated.append(okf)
            for i, why in sorted(rejected.items()): print(f'  버림: ID{i} ({why}) — 가짜 검출이거나 카메라가 크게 움직였을 수 있음 (카메라를 옮겼으면 --fresh)')
        used = sorted({i for f in gated for i in f})
        if len(used) < 2: print(f'  → 쓸 수 있는 마커가 {len(used)}개뿐이라 계산하지 않습니다 (2개 이상 필요, 4개 권장).'); continue
        T, q, dropped = solve_robust(gated, markers, K, init_T_table_optical=None if a.fresh else old); cmp_ = compare(T, old)
        for i in dropped: print(f'  뺌: ID{i} — 다른 마커와 안 맞음 (붙인 자리가 workcell.yaml 좌표와 다를 수 있음)')
        print(f'  사용한 마커 ID {sorted(set(used) - set(dropped))}, 마커별 재투영 rms(px) {q["per_marker_rms_px"]}')
        dz_old = table_depth_check(old, K, depth) if depth is not None else None; dz_new = table_depth_check(T, K, depth) if depth is not None else None
        print(f"  재투영 오차 rms {q['reproj_rms_px']:.2f}px (최대 {q['reproj_max_px']:.2f}px, 모서리 {q['corners']}개)")
        print(f"  저장값과 차이: 카메라 위치 {cmp_['shift_mm']} mm (크기 {cmp_['shift_norm_mm']} mm), 방향 {cmp_['rotation_deg']}°")
        if dz_old is not None: print(f'  깊이로 본 테이블 면 높이(0이어야 함): 저장값 {dz_old} mm, 새 값 {dz_new} mm')
        chk = validate(T, K, checks, wc); bad_chk = [i for i, v in chk.items() if v['err_mm'] > a.warn_mm]
        for i, v in sorted(chk.items()): print(f"  확인 ID{i}: 측정 {v['measured_xy_m']} → 사진으로 본 위치 {v['seen_xy_m']}  차이 dx {v['dx_mm']} / dy {v['dy_mm']} mm (거리 {v['err_mm']} mm){'  ⚠' if v['err_mm'] > a.warn_mm else ''}")
        missing = [i for i in CHECK_IDS if i not in chk]
        if missing: print(f'  확인 마커 안 보임: {missing}')
        if q['reproj_rms_px'] > 3.0: print('  ⚠ 재투영 오차가 큼(>3px): 마커가 휘었거나 workcell.yaml의 마커 위치와 다릅니다. 이 값은 믿지 마세요.')
        elif not a.fresh and (cmp_['shift_norm_mm'] > a.warn_mm or cmp_['rotation_deg'] > 0.3): print(f'  ⚠ 어긋남 큼 (기준 {a.warn_mm} mm / 0.3°): 카메라가 움직였거나 마커 위치가 workcell.yaml과 다릅니다.')
        if bad_chk: print(f'  ⚠ 확인 마커 {bad_chk} 가 {a.warn_mm} mm 넘게 어긋남')
        result[cam] = {'T_table_optical': T.tolist(), **q, **cmp_, 'dz_new_mm': dz_new, 'check': chk, 'check_ok': bool(chk) and not bad_chk and q['reproj_rms_px'] <= 3.0, 'dropped': dropped}
    if a.json: Path(a.json).write_text(json.dumps(result, indent=1, ensure_ascii=False)); print('저장:', a.json)
    if a.write:
        good = {c: r for c, r in result.items() if r['reproj_rms_px'] <= 3.0}
        if good: write_workcell(a.workcell, good, Ks, f'camera_calib fixed markers {sorted(markers)}; views {sessions[0]} ..'); print('workcell.yaml 반영:', sorted(good))
        else: print('반영할 결과가 없습니다 (재투영 오차 큼 또는 마커 부족)')
    return 0


if __name__ == '__main__': raise SystemExit(main())
