"""[사용] python3 gemini_detect.py SESSION_DIR gemini-robotics-er-2-preview OUT_boxes.json     # 환경변수 GEMINI_API_KEY 필요
세션의 모든 컬러 사진에서 Gemini로 물체 박스를 찾는다. 키는 환경변수 GEMINI_API_KEY로만 읽고 어디에도 쓰지 않는다."""
import os, sys, json, base64, concurrent.futures as cf, urllib.request, urllib.error
import cv2
session = sys.argv[1]; model = sys.argv[2]; out = sys.argv[3]; key = os.environ['GEMINI_API_KEY']
PROMPT = ('This photo shows a round white turntable disc with a black square marker, and one object placed on it. '
          'Find ONLY the object standing on the turntable disc. Ignore markers, stickers, pads, cables, the robot and the background. '
          'Return JSON: {"objects":[{"label":"...","box_2d":[ymin,xmin,ymax,xmax]}]} with coordinates normalized to 0-1000.')
def ask(path):
    img = cv2.imread(path); ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 92])
    body = {'contents': [{'parts': [{'inline_data': {'mime_type': 'image/jpeg', 'data': base64.b64encode(jpg).decode()}}, {'text': PROMPT}]}],
            'generationConfig': {'temperature': 0, 'responseMimeType': 'application/json'}}
    for attempt in range(3):
        r = urllib.request.Request(f'https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent', data=json.dumps(body).encode(),
                                   headers={'x-goog-api-key': key, 'Content-Type': 'application/json'})
        try:
            d = json.load(urllib.request.urlopen(r, timeout=90)); t = d['candidates'][0]['content']['parts'][0]['text']
            return json.loads(t)
        except urllib.error.HTTPError as e:
            msg = e.read().decode()[:200].replace(key, '***')
            if e.code in (429, 500, 503) and attempt < 2:
                import time; time.sleep(3 * (attempt + 1)); continue
            return {'error': e.code, 'msg': msg}
        except Exception as e:
            if attempt == 2: return {'error': str(e)[:150]}
jobs = {}
for v in sorted(os.listdir(session)):
    if not v.startswith('view_') or v == 'view_000': continue
    for cam in ('cam1', 'cam2'): jobs[f'{v}/{cam}'] = f'{session}/{v}/{cam}_color.png'
res = {}
with cf.ThreadPoolExecutor(6) as ex:
    futs = {ex.submit(ask, p): k for k, p in jobs.items()}
    for f in cf.as_completed(futs): res[futs[f]] = f.result()
res = {k: ({'objects': v} if isinstance(v, list) else v) for k, v in res.items()}
json.dump(dict(sorted(res.items())), open(out, 'w'), ensure_ascii=False, indent=1)
bad = [k for k, v in res.items() if 'error' in v or not v.get('objects')]
print('완료', len(res), '장, 실패/없음', len(bad), bad[:6])
