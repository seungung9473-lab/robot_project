"""저장된 스캔(깊이+마스크)에서 카메라별 '보이는 깊이' 윗면 크기를 재서 사진 위에 그린다. 로봇·카메라 없이 실행.
방법: 마스크 안(3 px 침식) 깊이 점 → 가장 큰 평면(RANSAC, 3 mm) → 평면 위 최소 외접 직사각형. 실제 크기 제약·구멍 메우기 없음.
사용: python3 size_review.py <스캔 폴더> <출력 폴더> [--view 1]"""
import argparse,json,cv2,numpy as np
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('scan',type=Path);ap.add_argument('out',type=Path);ap.add_argument('--view',type=int,default=1);ap.add_argument('--tag',default='')
a=ap.parse_args();S=a.scan;a.out.mkdir(parents=True,exist_ok=True)
names=[o['label'] for o in json.loads((S/'gemini_detection.json').read_text())['objects']]
V=S/f'view_{a.view:03d}';meta=json.loads((V/'metadata.json').read_text())
def pts(cam,k):
    m=cv2.imread(str(S/f'objects/object_{k}/sam2/view_{a.view:03d}/{cam}_mask.png'),0)
    if m is None:return None,None
    m=cv2.erode((m>127).astype(np.uint8),np.ones((3,3),np.uint8),iterations=3)>0
    d=cv2.imread(str(V/f'{cam}_depth_mm.png'),cv2.IMREAD_UNCHANGED).astype(np.float32)/1000
    K=np.array(meta['cameras'][cam]['camera_info']['k']).reshape(3,3);v,u=np.nonzero(m&(d>0));z=d[v,u]
    return np.column_stack([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z]),K
def plane(P,rng=np.random.default_rng(0)):
    best=None
    for _ in range(300):
        s=P[rng.choice(len(P),3,replace=False)];n=np.cross(s[1]-s[0],s[2]-s[0]);l=np.linalg.norm(n)
        if l<1e-9:continue
        n/=l;inl=np.abs((P-s[0])@n)<.003
        if best is None or inl.sum()>best[0].sum():best=(inl,n,s[0])
    return best
def rect(P):
    inl,n,o=plane(P);Q=P[inl];c=Q.mean(0);e1=np.linalg.svd(Q-c)[2][0];e1-=n*(e1@n);e1/=np.linalg.norm(e1);e2=np.cross(n,e1)
    q=np.column_stack([(Q-c)@e1,(Q-c)@e2]).astype(np.float32);r=cv2.minAreaRect(q);return r,c,e1,e2,len(Q)
rows=[];res={}
for k,nm in enumerate(names,1):
    tiles=[];res[nm]={}
    for cam in ('cam1','cam2'):
        P,K=pts(cam,k)
        im=cv2.imread(str(V/f'{cam}_color.png'))
        if P is None or len(P)<300:tiles.append(im);res[nm][cam]=None;continue
        r,c,e1,e2,n=rect(P);dims=np.sort(r[1])*1000
        box=cv2.boxPoints(r);xyz=c+box[:,:1]*e1+box[:,1:]*e2;pix=(K@xyz.T).T;pix=(pix[:,:2]/pix[:,2:]).round().astype(np.int32)
        cv2.polylines(im,[pix],True,(0,255,0) if cam=='cam1' else (0,165,255),2)
        cv2.putText(im,f'{cam}: visible depth {dims[0]:.1f} x {dims[1]:.1f} mm',(10,24),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),2)
        tiles.append(im);res[nm][cam]={'visible_depth_extent_mm':[round(float(x),1) for x in dims],'plane_points':int(n)}
    cv2.imwrite(str(a.out/f'4c_치수_object{k}.png'),np.hstack(tiles))
(a.out/'치수_비교.json').write_text(json.dumps({'scan':str(S),'view':a.view,'method':'3px-eroded SAM2 mask depth, largest RANSAC plane (3mm), min-area rectangle; no nominal size, no hole filling','tag':a.tag,'results':res},ensure_ascii=False,indent=2))
print(json.dumps(res,ensure_ascii=False,indent=1))
