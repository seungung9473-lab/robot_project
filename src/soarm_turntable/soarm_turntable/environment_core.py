"""Image-space ROI/background filtering only. No robot/turntable commands."""
import json,hashlib,math,os,uuid,datetime,warnings
from pathlib import Path
import cv2,numpy as np

def calibration_key(info):
    subset={k:info.get(k) for k in ('width','height','distortion_model','d','k','r','p')}
    return hashlib.sha256(json.dumps(subset,sort_keys=True).encode()).hexdigest()

def validate_polygon(points,width,height):
    if len(points)<3:raise ValueError('작업 영역 꼭짓점을 3개 이상 지정하세요')
    p=np.array(points,dtype=np.float64)
    if p.shape!=(len(points),2) or not np.isfinite(p).all():raise ValueError('잘못된 작업 영역')
    if (p[:,0]<0).any() or (p[:,0]>=width).any() or (p[:,1]<0).any() or (p[:,1]>=height).any():raise ValueError('영역이 영상 밖입니다')
    if abs(cv2.contourArea(p.astype(np.float32)))<100:raise ValueError('작업 영역이 너무 작습니다')
    # Reject crossing edges rather than silently filling an unintended polygon.
    def orient(a,b,c):return float((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))
    def intersects(a,b,c,d):
        return orient(a,b,c)*orient(a,b,d)<0 and orient(c,d,a)*orient(c,d,b)<0
    n=len(p)
    for i in range(n):
        for j in range(i+1,n):
            if j==i or j==(i+1)%n or i==(j+1)%n:continue
            if intersects(p[i],p[(i+1)%n],p[j],p[(j+1)%n]):raise ValueError('작업 영역 선이 서로 교차합니다')
    return p.astype(np.int32)

def background_median(depths):
    if len(depths)<5:raise ValueError('빈 배경 프레임이 부족합니다')
    stack=np.stack(depths).astype(np.float32);valid=np.isfinite(stack)&(stack>0)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore',RuntimeWarning);median=np.nanmedian(np.where(valid,stack,np.nan),axis=0)
    median[(valid.sum(axis=0)<math.ceil(len(depths)*.6))|~np.isfinite(median)]=0
    return median.astype(np.float32)

def foreground(depth_mm,background_mm,polygon,threshold_mm=10.,near_mm=150.,far_mm=2000.,min_area=100):
    if depth_mm.ndim!=2 or background_mm.shape!=depth_mm.shape:raise ValueError('배경과 현재 깊이 해상도가 다릅니다')
    if not (0<threshold_mm<=200 and 0<near_mm<far_mm and min_area>=1):raise ValueError('깊이 필터 범위 오류')
    h,w=depth_mm.shape;p=validate_polygon(polygon,w,h);roi=np.zeros((h,w),np.uint8);cv2.fillPoly(roi,[p],255)
    valid=np.isfinite(depth_mm)&np.isfinite(background_mm)&(depth_mm>=near_mm)&(depth_mm<=far_mm)&(background_mm>0)
    mask=(valid&(roi>0)&((background_mm-depth_mm)>=threshold_mm)).astype(np.uint8)
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    selected=np.zeros_like(mask)
    for i in range(1,count):
        if stats[i,cv2.CC_STAT_AREA]>=min_area:selected[labels==i]=255
    return selected,roi

def save_environment(root,cameras,filters,mode='live'):
    root=Path(root).expanduser();root.mkdir(parents=True,exist_ok=True);token=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6];tmp=root/('.pending-'+token);tmp.mkdir();config={'version':1,'source_mode':mode,'calibrated_3d':False,'filters':filters,'cameras':{}}
    for name,cam in cameras.items():
        info=cam['info'];validate_polygon(cam['polygon'],info['width'],info['height'])
        bg=cam['background'];assert bg.shape==(info['height'],info['width'])
        if not np.isfinite(bg).all() or (bg<0).any():raise ValueError('잘못된 배경 깊이')
        np.save(tmp/(name+'_background_mm.npy'),bg.astype(np.float32),allow_pickle=False)
        if not cv2.imwrite(str(tmp/(name+'_background_color.png')),cam['color']):raise IOError('배경 영상 저장 실패')
        config['cameras'][name]={'polygon':cam['polygon'],'background_file':name+'_background_mm.npy','color_file':name+'_background_color.png','camera_info':info,'calibration_key':calibration_key(info)}
    (tmp/'environment.json').write_text(json.dumps(config,ensure_ascii=False,indent=2,allow_nan=False));dst=root/token;os.replace(tmp,dst)
    active=root/'active.json';pending=root/('.active-'+uuid.uuid4().hex+'.json');pending.write_text(json.dumps({'environment_file':str(dst/'environment.json')},ensure_ascii=False));os.replace(pending,active)
    return dst/'environment.json'

def load_environment(path,allow_replay=False):
    path=Path(path).expanduser();data=json.loads(path.read_text())
    if 'environment_file' in data:path=Path(data['environment_file']);data=json.loads(path.read_text())
    if data.get('version')!=1 or set(data.get('cameras',{}))!={'cam1','cam2'}:raise ValueError('잘못된 환경 설정')
    if data.get('source_mode')!='live' and not allow_replay:raise ValueError('재생 화면에서 만든 환경은 실제 촬영에 적용할 수 없습니다')
    for cam in data['cameras'].values():
        file=(path.parent/cam['background_file']).resolve()
        if file.parent!=path.parent.resolve():raise ValueError('잘못된 배경 경로')
        cam['background']=np.load(file,allow_pickle=False)
        validate_polygon(cam['polygon'],cam['camera_info']['width'],cam['camera_info']['height'])
    return data,path
