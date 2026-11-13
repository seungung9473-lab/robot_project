"""[사용] ~/sam2_env/bin/python sam2_masks.py SESSION_DIR boxes.json OUT.npz OVERLAY_DIR
Gemini 박스를 프롬프트로 SAM2 마스크를 만든다 (CPU). 결과: npz (view/cam → bool 마스크) + 확인용 겹침 사진."""
import sys, os, json, numpy as np, cv2, torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
session, boxes_json, out_npz, overlay_dir = sys.argv[1:5]
torch.set_num_threads(max(1, os.cpu_count() // 2))
model = build_sam2('configs/sam2.1/sam2.1_hiera_s.yaml', os.path.expanduser('~/sam2_env/checkpoints/sam2.1_hiera_small.pt'), device='cpu')
pred = SAM2ImagePredictor(model); boxes = json.load(open(boxes_json)); masks = {}; os.makedirs(overlay_dir, exist_ok=True); info = {}
for key, v in sorted(boxes.items()):
    view, cam = key.split('/'); img = cv2.cvtColor(cv2.imread(f'{session}/{view}/{cam}_color.png'), cv2.COLOR_BGR2RGB); H, W = img.shape[:2]
    y0, x0, y1, x1 = v['objects'][0]['box_2d']; box = np.array([x0 * W / 1000, y0 * H / 1000, x1 * W / 1000, y1 * H / 1000])
    pred.set_image(img); m, sc, _ = pred.predict(box=box, multimask_output=True); i = int(np.argmax(sc)); mk = m[i].astype(bool)
    masks[key.replace('/', '_')] = mk; info[key] = {'score': float(sc[i]), 'area_px': int(mk.sum()), 'box_area_px': int((box[2]-box[0])*(box[3]-box[1]))}
    ov = img.copy(); ov[mk] = (0.5 * ov[mk] + 0.5 * np.array([0, 255, 0])).astype(np.uint8); cv2.rectangle(ov, tuple(box[:2].astype(int)), tuple(box[2:].astype(int)), (255, 255, 0), 1)
    cv2.imwrite(f'{overlay_dir}/{view}_{cam}.jpg', cv2.cvtColor(ov, cv2.COLOR_RGB2BGR))
np.savez_compressed(out_npz, **masks); json.dump(info, open(out_npz.replace('.npz', '_info.json'), 'w'), indent=1)
a = [x['area_px'] / x['box_area_px'] for x in info.values()]; print('마스크', len(masks), '장; 박스 대비 면적 비율 최소/평균', round(min(a), 2), round(float(np.mean(a)), 2), '; 최저 점수', round(min(x['score'] for x in info.values()), 3))
