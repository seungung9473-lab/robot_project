"""Read the 할인/정규 stickers on the two pads and map each one to a known pad.

Gemini only says WHICH sticker text sits WHERE in the image (inference only, no robot tools).
The pad's position for the robot is never taken from that answer: the sticker pixel is turned into a
table coordinate with the measured depth and the calibrated camera, and then matched to the nearest
CALIBRATED pad centre of the workcell snapshot. Anything unclear stops the run (nothing is guessed).

Rule used by the sorter: nearer non-expired date -> 할인 (discount) pad, farther date -> 정규 (regular) pad.
"""
import base64,json,re,sys,time
from pathlib import Path
import cv2,numpy as np,yaml
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R.parent/'er2'))

PAD_KEYS={'pad2':5,'pad1':6}       # workcell snapshot key -> pad (marker) id
MAX_PAD_DISTANCE_M=.075            # pad radius 50 mm + sticker offset from the centre
DEPTH_WINDOW_PX=4                  # median over a 9x9 window around the sticker centre
MAX_AGE_S=1800

def normalize_label(text):
    """'할 인', '할인.' -> 'discount'; '정 규' -> 'regular'; anything else -> None."""
    t=re.sub(r'[\s\.\-_:·,]+','',str(text or ''))
    return {'할인':'discount','정규':'regular'}.get(t)

def build_payload(png_bytes):
    prompt=('The picture shows a robot workcell. Two round black pads each carry a small white paper sticker with Korean text. '
            'Find every sticker whose printed text is exactly "할인" or "정규" (letters may be spaced apart). '
            'Ignore date labels, QR codes and everything on the objects or on the large turntable. '
            'For each such sticker return its text and the point at the sticker centre as [y, x] normalised to 0-1000. '
            'Do not invent stickers that are not visible.')
    schema={'type':'OBJECT','properties':{'stickers':{'type':'ARRAY','maxItems':4,'items':{'type':'OBJECT','properties':{
        'text':{'type':'STRING'},'point_yx':{'type':'ARRAY','items':{'type':'INTEGER'},'minItems':2,'maxItems':2}},
        'required':['text','point_yx']}}},'required':['stickers']}
    return {'systemInstruction':{'parts':[{'text':'You read paper stickers only. There are no robot control tools available.'}]},
            'contents':[{'parts':[{'inlineData':{'mimeType':'image/png','data':base64.b64encode(png_bytes).decode()}},{'text':prompt}]}],
            'generationConfig':{'maxOutputTokens':1024,'responseMimeType':'application/json','responseSchema':schema}}

def parse_response(response,shape):
    """Validate the model answer; returns [(label, (u,v) pixel)] for stickers whose text is 할인/정규."""
    text=''.join(p.get('text','') for c in response.get('candidates',[]) for p in c.get('content',{}).get('parts',[]) if not p.get('thought',False))
    data=json.loads(text);rows=[];h,w=shape[:2]
    for item in data['stickers']:
        y,x=item['point_yx']
        if not(0<=y<=1000 and 0<=x<=1000):raise ValueError('Sticker point outside the image')
        label=normalize_label(item['text'])
        if label:rows.append((label,(min(w-1,int(round(x*w/1000))),min(h-1,int(round(y*h/1000))))))
    return rows

def table_xy(raw,scale,info,T,pixel,windows=(4,10,20,30)):
    """Table-frame XY of a pixel. Depth = median of the valid depths in the smallest window that has enough of them
    (white stickers and surfaces close to the camera often leave holes right at the pixel); the pad is flat, so that depth
    is assigned to the sticker pixel and its ray is followed to the table."""
    u,v=pixel;h,w=raw.shape[:2];z=None
    for win in windows:
        patch=raw[max(0,v-win):v+win+1,max(0,u-win):u+win+1].astype(float)*scale;ok=np.isfinite(patch)&(patch>.08)&(patch<.9)
        if ok.sum()>=10:z=float(np.median(patch[ok]));break
    if z is None:return None
    ray=cv2.undistortPoints(np.array([[[float(u),float(v)]]]),np.array(info['k']).reshape(3,3),np.array(info['d'])).reshape(2)
    p=np.array([ray[0]*z,ray[1]*z,z]);T=np.asarray(T,float);p=T[:3,:3]@p+T[:3,3]
    return [float(p[0]),float(p[1])]

def assign(detections,pads,max_distance=MAX_PAD_DISTANCE_M):
    """detections: [{'cam','label','xy'}]; pads: {pad_id: (x,y)}. Returns the sticker->pad mapping or raises."""
    votes={'discount':{},'regular':{}};evidence=[]
    for d in detections:
        if d['xy'] is None:evidence.append({**d,'pad_id':None,'note':'no valid depth at sticker'});continue
        best=min(pads,key=lambda i:np.hypot(d['xy'][0]-pads[i][0],d['xy'][1]-pads[i][1]))
        dist=float(np.hypot(d['xy'][0]-pads[best][0],d['xy'][1]-pads[best][1]))
        if dist>max_distance:evidence.append({**d,'pad_id':None,'distance_m':dist,'note':'too far from every calibrated pad'});continue
        votes[d['label']].setdefault(best,[]).append(d['cam']);evidence.append({**d,'pad_id':best,'distance_m':dist})
    result={}
    for label in ('discount','regular'):
        ids=votes[label]
        if not ids:raise RuntimeError('Pad sticker "%s" not found on any pad'%('할인' if label=='discount' else '정규'))
        if len(ids)>1:raise RuntimeError('Pad sticker "%s" seen on different pads %s'%('할인' if label=='discount' else '정규',sorted(ids)))
        result[label]=next(iter(ids))
    if result['discount']==result['regular']:raise RuntimeError('Both stickers matched the same pad')
    return {'discount_pad_id':result['discount'],'regular_pad_id':result['regular'],'evidence':evidence}

def read_pad_labels(session,view='view_001',transport=None):
    from check_connection import load_key
    from model_router import generate
    s=Path(session).resolve();manifest=json.loads((s/'session.json').read_text())
    if manifest['status']!='complete' or time.time()-(s/'session.json').stat().st_mtime>MAX_AGE_S:raise RuntimeError('Fresh completed scan required')
    wc=yaml.safe_load((s/'workcell_snapshot.yaml').read_text())
    pads={PAD_KEYS[k]:wc['pads'][k]['center_table_m'][:2] for k in PAD_KEYS}
    key,_=load_key()
    if not key:raise RuntimeError('Gemini key missing; register it privately in the terminal')
    folder=s/view;meta=json.loads((folder/'metadata.json').read_text());detections=[];routing={};cams_used=[]
    for cam in ('cam1','cam2'):
        info=meta['cameras'][cam];color=folder/info['color_file'];png=color.read_bytes();img=cv2.imread(str(color))
        raw=cv2.imread(str(folder/info['depth_file']),-1);scale=.001 if raw.dtype==np.uint16 else 1.
        response,rows,route=generate(key,build_payload(png),lambda r,img=img:parse_response(r,img.shape),transport=transport)
        routing[cam]=route;cams_used.append(cam)
        for label,pixel in rows:
            detections.append({'cam':cam,'label':label,'pixel':list(pixel),'xy':table_xy(raw,scale,info['camera_info'],wc['cameras'][cam]['T_table_optical'],pixel)})
        try:result=assign(detections,pads);break      # one camera is enough when both stickers are clear
        except RuntimeError:
            if cam=='cam2':raise
    result.update(source_scan=str(s),read_at=time.time(),view=view,cameras_used=cams_used,model_used=[r.get('model_used') for r in routing.values()],
                  routing=routing,pad_centres_table_m={str(k):v for k,v in pads.items()},robot_commands_sent=False,
                  note='sticker position used only to pick the calibrated pad; motion uses the calibrated pad coordinates')
    (s/'pad_labels.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print('패드 표지: 할인=패드 %d · 정규=패드 %d'%(result['discount_pad_id'],result['regular_pad_id']),flush=True)
    return result

def load_pad_labels(session):
    """Fresh mapping for this scan, or None when no recognition was run for it."""
    path=Path(session)/'pad_labels.json'
    if not path.exists():return None
    d=json.loads(path.read_text())
    if d.get('source_scan')!=str(Path(session).resolve()) or time.time()-d.get('read_at',0)>MAX_AGE_S:raise ValueError('Pad labels belong to another scan or are too old; read them again')
    return {'discount':int(d['discount_pad_id']),'regular':int(d['regular_pad_id'])}
