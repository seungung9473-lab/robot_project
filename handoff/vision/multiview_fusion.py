#!/usr/bin/env python3
"""
multiview_fusion.py — 턴테이블 멀티뷰 (D415 x2) → 물체 점구름 → 석션 후보 점수
numpy + scipy 만 사용 (노트북·라파 둘 다 동작). open3d 있으면 ICP 미세정렬만 추가로 씀.

좌표계 3개
  cam   : 카메라 광학 좌표 (x 오른쪽, y 아래, z 앞)                    ← 뎁스에서 바로 나옴
  world : 턴테이블 좌표 (원점 = 회전축 ∩ 원판 윗면, z 위)              ← 캘리브레이션 T_world_cam
  obj   : 원판에 붙어서 같이 도는 좌표 (턴테이블 각도 0일 때 world 와 같음) ← 여러 각도를 합치는 기준

한 장(view) 처리 순서
  depth(uint16) × depth_scale → Z[m]
  (u,v,Z) → cam  :  X=(u-cx)Z/fx, Y=(v-cy)Z/fy        ← deproject
  cam → world   :  T_world_cam (4x4)                 ← 카메라마다 1번 캘리브레이션
  world → obj   :  Rz(-θ)                            ← θ = 그 순간 턴테이블 각도
  → 모든 view 를 obj 좌표에서 합침
"""
import numpy as np
from scipy.spatial import cKDTree

# ---------------------------------------------------------------- 기본 변환
def deproject(depth, K, depth_scale=0.001, zmin=0.15, zmax=1.2, stride=1, rgb=None):
    """뎁스 이미지 → cam 좌표 점 (N,3) [m]. depth 는 color 에 정렬된(aligned) 것이어야 rgb 와 픽셀이 맞음."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d = depth[::stride, ::stride].astype(np.float32) * depth_scale
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    m = (d > zmin) & (d < zmax)
    z = d[m]
    pts = np.stack([(u[m] - cx) * z / fx, (v[m] - cy) * z / fy, z], axis=1)
    cols = rgb[::stride, ::stride][m] if rgb is not None else None
    return pts, cols

def transform(T, pts):
    return pts @ T[:3, :3].T + T[:3, 3]

def Rz(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    T = np.eye(4); T[:2, :2] = [[c, -s], [s, c]]
    return T

# ---------------------------------------------------------------- 캘리브레이션 (ArUco 보드 → T_world_cam)
def calib_from_aruco(color_bgr, K, dist, board_pose_in_world=np.eye(4), marker_len=0.04, dict_id=None):
    """
    원판 중앙(회전축)에 ArUco 마커 1장을 평평하게 놓고 찍은 컬러 이미지로 T_world_cam 계산.
    board_pose_in_world: 마커 중심이 회전축에서 벗어나 있으면 그 위치 (기본 = 회전축 위, 원판 윗면).
    """
    import cv2
    aruco = cv2.aruco
    dic = aruco.getPredefinedDictionary(dict_id or aruco.DICT_4X4_50)
    det = aruco.ArucoDetector(dic, aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(color_bgr)
    if ids is None:
        raise RuntimeError("ArUco 마커 못 찾음")
    h = marker_len / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, corners[0].reshape(4, 2), K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    R, _ = cv2.Rodrigues(rvec)
    T_cam_board = np.eye(4); T_cam_board[:3, :3] = R; T_cam_board[:3, 3] = tvec.ravel()
    return board_pose_in_world @ np.linalg.inv(T_cam_board)   # T_world_cam

# ---------------------------------------------------------------- 합치기
def fuse_views(views, voxel=0.002, plate_r=0.075, z_floor=0.004, z_top=0.30):
    """
    views: [{'depth','K','depth_scale','T_world_cam','angle_deg', ('rgb')}, ...]
    → obj 좌표 점구름 (원판 위 물체만), 색
    """
    P, C = [], []
    for vw in views:
        p, c = deproject(vw['depth'], vw['K'], vw.get('depth_scale', 0.001), rgb=vw.get('rgb'))
        p = transform(vw['T_world_cam'], p)                 # cam → world
        p = transform(Rz(-vw['angle_deg']), p)              # world → obj (원판 회전 되돌리기)
        keep = (np.hypot(p[:, 0], p[:, 1]) < plate_r) & (p[:, 2] > z_floor) & (p[:, 2] < z_top)
        P.append(p[keep]); C.append(c[keep] if c is not None else None)
    P = np.concatenate(P)
    C = np.concatenate(C) if all(c is not None for c in C) else None
    P, C = voxel_down(P, C, voxel)
    P, C = remove_outliers(P, C)
    return P, C

def voxel_down(P, C, voxel):
    key = np.floor(P / voxel).astype(np.int64)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    Pd = np.zeros((cnt.size, 3)); np.add.at(Pd, inv, P); Pd /= cnt[:, None]
    if C is None:
        return Pd, None
    Cd = np.zeros((cnt.size, C.shape[1])); np.add.at(Cd, inv, C.astype(float)); Cd /= cnt[:, None]
    return Pd, Cd.astype(np.uint8)

def remove_outliers(P, C, k=12, std=2.0):
    d, _ = cKDTree(P).query(P, k + 1)
    md = d[:, 1:].mean(1)
    keep = md < md.mean() + std * md.std()
    return P[keep], (C[keep] if C is not None else None)

# ---------------------------------------------------------------- 석션 점수
def suction_candidates(P, cup_r=0.0125, top_n=5, w_flat=0.5, w_up=0.2, w_center=0.2, w_cover=0.1):
    """
    점마다 컵 반지름 안 이웃으로 평면 맞추기(PCA)
      flat  : 평평함 (가장 작은 고유값 비율이 작을수록 1)
      up    : 법선이 위/옆으로 팔이 접근 가능한 방향인지 (지금은 위쪽 선호)
      center: 무게중심(점구름 중심)에 가까울수록
      cover : 컵 면적을 점이 꽉 채우는지 (모서리·구멍 걸러냄)
    """
    tree = cKDTree(P)
    cen = P.mean(0)
    nb = tree.query_ball_point(P, cup_r)
    full = np.median([len(n) for n in nb])
    scores, normals = np.zeros(len(P)), np.zeros((len(P), 3))
    for i, idx in enumerate(nb):
        if len(idx) < 10:
            continue
        Q = P[idx] - P[idx].mean(0)
        w, V = np.linalg.eigh(Q.T @ Q / len(idx))
        n = V[:, 0] if V[2, 0] >= 0 else -V[:, 0]           # 법선 (바깥/위쪽)
        flat = 1 - min(1.0, w[0] / (w.sum() + 1e-12) * 30)
        up = max(0.0, n[2])
        center = np.exp(-np.linalg.norm(P[i, :2] - cen[:2]) / 0.03)
        cover = min(1.0, len(idx) / full)
        scores[i] = w_flat * flat + w_up * up + w_center * center + w_cover * cover
        normals[i] = n
    order = np.argsort(-scores)
    picks = []
    for i in order:                                         # 서로 1.5cm 이상 떨어진 후보만
        if all(np.linalg.norm(P[i] - P[j]) > 0.015 for j in picks):
            picks.append(i)
        if len(picks) == top_n:
            break
    return [{'p_obj': P[i], 'n_obj': normals[i], 'score': float(scores[i])} for i in picks]

def to_world(p_obj, angle_deg):
    """obj 좌표 점 → 턴테이블이 angle_deg 일 때 world 좌표"""
    return transform(Rz(angle_deg), p_obj[None])[0]

def best_turn_angle(p_obj, arm_dir_deg=180.0):
    """후보 점이 팔 쪽(world 에서 arm_dir_deg 방향)을 보도록 하는 턴테이블 각도"""
    a = np.degrees(np.arctan2(p_obj[1], p_obj[0]))
    return (arm_dir_deg - a) % 360

# ---------------------------------------------------------------- 자체 테스트 (가상 상자 + 가상 카메라 2대)
def _render(points, T_world_cam, K, shape=(480, 640)):
    """world 점 → 가상 뎁스 이미지 (z-buffer)"""
    pc = transform(np.linalg.inv(T_world_cam), points)
    pc = pc[pc[:, 2] > 0.05]
    u = np.round(K[0, 0] * pc[:, 0] / pc[:, 2] + K[0, 2]).astype(int)
    v = np.round(K[1, 1] * pc[:, 1] / pc[:, 2] + K[1, 2]).astype(int)
    ok = (u >= 0) & (u < shape[1]) & (v >= 0) & (v < shape[0])
    depth = np.full(shape, np.inf)
    np.minimum.at(depth, (v[ok], u[ok]), pc[ok, 2])
    depth[np.isinf(depth)] = 0
    return (depth * 1000).astype(np.uint16)

def _look_at(eye, target):
    z = target - eye; z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0]); x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else np.array([1.0, 0, 0])
    y = np.cross(z, x)
    T = np.eye(4); T[:3, :3] = np.stack([x, y, z], 1); T[:3, 3] = eye
    return T

if __name__ == "__main__":
    rng = np.random.default_rng(0)
    # 가상 물체: 60x40x30 mm 상자, 원판 중심에서 (10, 5) mm 떨어져 놓임
    sx, sy, sz, ox, oy = 0.06, 0.04, 0.03, 0.010, 0.005
    def box_surface(n=200000):
        f = rng.integers(0, 5, n); a, b = rng.random(n), rng.random(n)
        p = np.zeros((n, 3))
        p[f == 0] = np.c_[a[f == 0] * sx, b[f == 0] * sy, np.full((f == 0).sum(), sz)]
        p[f == 1] = np.c_[a[f == 1] * sx, np.zeros((f == 1).sum()), b[f == 1] * sz]
        p[f == 2] = np.c_[a[f == 2] * sx, np.full((f == 2).sum(), sy), b[f == 2] * sz]
        p[f == 3] = np.c_[np.zeros((f == 3).sum()), a[f == 3] * sy, b[f == 3] * sz]
        p[f == 4] = np.c_[np.full((f == 4).sum(), sx), a[f == 4] * sy, b[f == 4] * sz]
        return p + [ox - sx / 2, oy - sy / 2, 0]
    obj_pts = box_surface()
    K = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1.0]])
    cams = {'top': _look_at(np.array([0.0, -0.05, 0.45]), np.zeros(3)),
            'side': _look_at(np.array([0.40, 0.0, 0.12]), np.array([0, 0, 0.02]))}
    views = []
    for ang in range(0, 360, 30):
        world_pts = transform(Rz(ang), obj_pts)              # 원판이 ang 만큼 돈 상태
        for name, T in cams.items():
            d = _render(world_pts, T, K)
            noise = rng.normal(0, 1.0, d.shape) * (d > 0)    # 1 mm 노이즈
            views.append({'depth': np.clip(d + noise, 0, None).astype(np.uint16), 'K': K,
                          'T_world_cam': T, 'angle_deg': ang})
    P, _ = fuse_views(views)
    print(f"합친 점: {len(P)}  범위 x[{P[:,0].min()*1000:.1f},{P[:,0].max()*1000:.1f}] "
          f"y[{P[:,1].min()*1000:.1f},{P[:,1].max()*1000:.1f}] z[{P[:,2].min()*1000:.1f},{P[:,2].max()*1000:.1f}] mm")
    print(f"정답 x[{(ox-sx/2)*1000:.1f},{(ox+sx/2)*1000:.1f}] y[{(oy-sy/2)*1000:.1f},{(oy+sy/2)*1000:.1f}] z[0,{sz*1000:.1f}] mm")
    c = suction_candidates(P)
    for k, cand in enumerate(c):
        p = cand['p_obj'] * 1000
        print(f"후보{k+1}: ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm  법선 {np.round(cand['n_obj'],2)}  점수 {cand['score']:.2f}")
    ang = best_turn_angle(c[0]['p_obj'])
    print(f"1등 후보를 팔 쪽(180°)으로 보내려면 턴테이블 {ang:.1f}° → world {np.round(to_world(c[0]['p_obj'], ang)*1000,1)} mm")
