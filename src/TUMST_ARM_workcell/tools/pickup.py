"""[사용] GEMINI_API_KEY=... ~/sam2_env/bin/python pickup.py SESSION_DIR cloud.npz OUT_pickup.json gemini-robotics-er-2-preview
융합 점군 → 윗면 평면·평탄도 → Ø25 컵이 들어가는 후보 → Gemini가 사진으로 고름 → 3D로 최종 위치. 키는 환경변수 GEMINI_API_KEY."""
import sys, os, json, base64, numpy as np, cv2, yaml, urllib.request
from scipy import ndimage
session, cloud_npz, out_json, model = sys.argv[1:5]; key = os.environ['GEMINI_API_KEY']
wc = yaml.safe_load(open(os.path.expanduser('~/robot_ws/src/TUMST_ARM_workcell/config/workcell.yaml'))); CUP_R = 0.0125; MARGIN = 0.003
d = np.load(cloud_npz); P = d['points']; top_z = wc['turntable']['top_height_m']
zh = np.percentile(P[:, 2], 97); band = P[P[:, 2] > zh - 0.004]
c0 = band.mean(0); u, sv, vt = np.linalg.svd(band - c0, full_matrices=False); n = vt[2] if vt[2][2] > 0 else -vt[2]
rms = float(np.sqrt(np.mean(((band - c0) @ n) ** 2))); tilt = float(np.degrees(np.arccos(np.clip(n[2], -1, 1)))); z_top = float(np.median(band[:, 2]))
print(f'윗면: 높이(테이블 z) {z_top*1000:.1f} mm = 원판 위 {(z_top-top_z)*1000:.1f} mm, 기울기 {tilt:.2f}°, 평탄도 rms {rms*1000:.2f} mm, 점 {len(band)}')
res = 0.0005; xy = band[:, :2]; lo = xy.min(0) - 0.01; W = int(np.ceil((xy.max(0) - lo).max() / res)) + 40
g = np.zeros((W, W), bool); ij = ((xy - lo) / res).astype(int); g[ij[:, 1], ij[:, 0]] = True
g = ndimage.binary_closing(g, iterations=4); g = ndimage.binary_fill_holes(g); g = ndimage.binary_opening(g, iterations=3)
lab, nl = ndimage.label(g); sizes = ndimage.sum(g, lab, range(1, nl + 1)); g = lab == (1 + int(np.argmax(sizes)))
dt = ndimage.distance_transform_edt(g) * res; r_max = float(dt.max()); cy, cx = np.unravel_index(np.argmax(dt), dt.shape); center = lo + np.array([cx, cy]) * res
area = g.sum() * res * res; ok = r_max >= CUP_R + MARGIN
print(f'윗면 면적 {area*1e4:.1f} cm², 안에 들어가는 가장 큰 원 반지름 {r_max*1000:.1f} mm (컵 12.5 + 여유 3 → {"가능" if ok else "부족"}), 중심 ({center[0]:.4f}, {center[1]:.4f})')
def pt(c): return [float(c[0]), float(c[1]), z_top]
cands = {'A': center}
for lbl, off in zip('BCDE', [(1, 0), (-1, 0), (0, 1), (0, -1)]):
    c = center + np.array(off) * min(0.008, max(0.0, r_max - CUP_R - MARGIN)); cands[lbl] = c
def inside(c, margin=MARGIN):
    j = int(round((c[0] - lo[0]) / res)); i = int(round((c[1] - lo[1]) / res)); return bool(0 <= i < W and 0 <= j < W and dt[i, j] >= CUP_R + margin)
valid = {k: inside(v) for k, v in cands.items()}; print('후보 유효 (컵 전체가 윗면 안):', valid)
# 카메라 사진에 후보 표시 → Gemini
imgs = []
for cam in ('cam1', 'cam2'):
    meta = json.load(open(f'{session}/view_001/metadata.json'))['cameras'][cam]; K = np.array(meta['camera_info']['k']).reshape(3, 3)
    T = np.array(wc['cameras'][cam]['T_table_optical']); Ti = np.linalg.inv(T); im = cv2.imread(f'{session}/view_001/{cam}_color.png')
    def proj(p): q = Ti[:3, :3] @ np.array(p) + Ti[:3, 3]; uv = K @ q; return uv[:2] / uv[2]
    for lbl, c in cands.items():
        ring = np.array([proj([c[0] + CUP_R * np.cos(t), c[1] + CUP_R * np.sin(t), z_top]) for t in np.linspace(0, 2 * np.pi, 40)]).astype(np.int32)
        col = (0, 255, 255) if valid[lbl] else (0, 0, 255); cv2.polylines(im, [ring.reshape(-1, 1, 2)], True, col, 1)
        u0 = proj([c[0], c[1], z_top]); cv2.putText(im, lbl, (int(u0[0]) - 5, int(u0[1]) + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    cv2.imwrite(out_json.replace('.json', f'_{cam}.jpg'), im); imgs.append(im)
parts = []
for im in imgs: ok_, jpg = cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, 92]); parts.append({'inline_data': {'mime_type': 'image/jpeg', 'data': base64.b64encode(jpg).decode()}})
parts.append({'text': 'Two views (two cameras) of one block on a turntable. Circles A-E on the TOP face of the block are candidate positions for a 25 mm diameter vacuum suction cup (yellow = fits inside the top face, red = too close to an edge). '
              'Pick the ONE candidate that gives the most stable and secure suction: flat, well inside the face, near the center of mass. Answer JSON: {"best":"A-E","reason":"short Korean sentence"}'})
body = {'contents': [{'parts': parts}], 'generationConfig': {'temperature': 0, 'responseMimeType': 'application/json'}}
r = urllib.request.Request(f'https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent', data=json.dumps(body).encode(), headers={'x-goog-api-key': key, 'Content-Type': 'application/json'})
try:
    t = json.load(urllib.request.urlopen(r, timeout=120))['candidates'][0]['content']['parts'][0]['text']; ans = json.loads(t); ans = ans[0] if isinstance(ans, list) else ans
except Exception as e:
    ans = {'best': None, 'reason': 'Gemini 호출 실패: ' + str(e).replace(key, '***')[:120]}
print('Gemini 선택:', ans)
best = ans.get('best') if ans.get('best') in cands and valid.get(ans.get('best')) else 'A'
print('최종 후보:', best, '(Gemini가 고른 것이 유효하지 않으면 A로 대체)' if best != ans.get('best') else '(Gemini 선택 사용)')
result = {'pickup_table_m': pt(cands[best]), 'candidate': best, 'gemini': ans, 'plane_z_m': z_top, 'plane_tilt_deg': tilt, 'flatness_rms_m': rms, 'top_area_m2': area,
          'max_inscribed_radius_m': r_max, 'cup_fits': ok, 'candidates': {k: pt(v) for k, v in cands.items()}, 'valid': valid, 'axis_shift_m': d['axis_shift'].tolist(),
          'height_above_disc_m': z_top - top_z}
json.dump(result, open(out_json, 'w'), indent=1, ensure_ascii=False); print('흡착 위치 (테이블 m):', np.round(result['pickup_table_m'], 4))
