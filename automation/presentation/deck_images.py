#!/usr/bin/env python3
"""발표자료·홈페이지용 이미지와 데이터 요약을 한 번에 만든다 (저장된 실행 기록만 읽음, 로봇·카메라·네트워크 사용 안 함).

사용:  python3 deck_images.py <run_dir> <out_dir> [--focus N] [--tag "11/9 23:51 실행"]
  run_dir : automation/test_data/runs/<trial_id>   (전체 시연 1회, scans/ 아래 스캔 1개)
  out_dir : 결과 폴더 (없으면 만듦)
  --focus : 4·6·7단계 사진의 대표 물체 번호 (기본: 이름에 원통·컵이 들어간 물체, 없으면 1)

만드는 것: 0_알고리즘_구조도(png/svg), 1a~8b 단계별 사진 + 2c 패드 스티커 인식, 카메라_원본/(24장), 데이터_요약.md, 단계시간.csv
"""
import argparse,csv,glob,json,shutil
from pathlib import Path
import cv2,numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import Circle,Polygon,FancyBboxPatch,FancyArrowPatch
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
fm.fontManager.addfont(FONT);plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
PALETTE=[(60,200,90),(255,60,60),(255,150,0),(80,140,255),(200,80,200)]

ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);ap.add_argument('out',type=Path);ap.add_argument('--focus',type=int);ap.add_argument('--tag',default=None);a=ap.parse_args()
RUN=a.run.resolve();OUT=a.out.resolve();OUT.mkdir(parents=True,exist_ok=True)
scans=sorted((RUN/'scans').glob('*/session.json'))
if len(scans)!=1:raise SystemExit('scans/ 아래 스캔이 정확히 1개여야 합니다')
S=scans[0].parent;demo=json.loads((RUN/'demo_result.json').read_text())
g=json.loads((S/'gemini_detection.json').read_text());NM={o['object_id']:o['label'] for o in g['objects']}
cp=json.loads((S/'completed_placements.json').read_text());rt=json.loads((S/'expiry_route_report.json').read_text());ex=json.loads((S/'expiry_observations.json').read_text())
pl=json.loads((S/'pad_labels.json').read_text()) if (S/'pad_labels.json').exists() else None
MODEL=g.get('model_used','Gemini');stamp=RUN.name[:15];tag=a.tag or ('%s/%s %s:%s 실행 (%s)'%(stamp[4:6].lstrip('0'),stamp[6:8].lstrip('0'),stamp[9:11],stamp[11:13],{'complete':'완주','stopped':'중단','partial':'일부'}.get(demo.get('status'),demo.get('status'))))
OBJS=sorted(int(k.split('_')[1]) for k in cp)
DEST={};PREP={}
for k,v in cp.items():
    o=int(k.split('_')[1]);PREP[o]=Path(v['prepared_session'])
    DEST[o]='폐기 구역' if v['destination']=='trash_drop' else '패드 %s'%v['pad_id']
COL={o:PALETTE[i%len(PALETTE)] for i,o in enumerate(sorted(NM))}
F=a.focus or next((o for o in OBJS if any(w in NM[o] for w in ('원통','컵'))),OBJS[0])
ang={i:round((i-1)*30) for i in range(1,13)}
def rd(v,cam,k='color'):return cv2.imread(str(S/f'view_{v:03d}/{cam}_{k}.png'),cv2.IMREAD_UNCHANGED)
def rgb(im):return cv2.cvtColor(im,cv2.COLOR_BGR2RGB)
def mask(o,v,cam):
    m=cv2.imread(str(S/f'objects/object_{o}/sam2/view_{v:03d}/{cam}_mask.png'),0);return None if m is None else m>127
def save(fig,name):
    fig.text(.995,.004,f'{tag} · {MODEL}',ha='right',va='bottom',fontsize=9,color='#777');fig.savefig(OUT/name,dpi=200,facecolor='white');plt.close(fig);print(name)
def tag_ax(ax,t):ax.set_title(t,fontsize=15,fontweight='bold',pad=8);ax.axis('off')
def table(ax,rows,cols,fs=13,sc=2.1,colw=None,cell=None):
    ax.axis('off');t=ax.table(cellText=rows,colLabels=cols,loc='center',cellLoc='center',cellColours=cell,colWidths=colw);t.auto_set_font_size(False);t.set_fontsize(fs);t.scale(1,sc);return t
# ---------- 카메라 원본
cam_dir=OUT/'카메라_원본';cam_dir.mkdir(exist_ok=True)
for i in range(1,13):
    for c in ('cam1','cam2'):shutil.copy2(S/f'view_{i:03d}/{c}_color.png',cam_dir/f'{c}_view{i:02d}_{ang[i]:03d}deg.png')
# ---------- 1 스캔
f,ax=plt.subplots(1,2,figsize=(12,4.6))
for x,c in zip(ax,('cam1','cam2')):x.imshow(rgb(rd(1,c)));tag_ax(x,f'{c} · 턴테이블 0°')
f.suptitle('① 정지 촬영: 카메라 2대 RGB-D 동시 촬영',fontsize=17,fontweight='bold');f.tight_layout();save(f,'1a_스캔_카메라2대.png')
f,ax=plt.subplots(3,4,figsize=(13,7.8))
for i,x in enumerate(ax.ravel(),1):x.imshow(rgb(rd(i,'cam1')));tag_ax(x,f'{ang[i]}°')
f.suptitle('① 턴테이블 30° × 12방향 (cam1 기준, cam2도 같은 12방향 → 총 24장)',fontsize=17,fontweight='bold');f.tight_layout();save(f,'1b_스캔_12방향.png')
# ---------- 2 Gemini
im=rgb(rd(1,'cam1')).copy()
for o in g['objects']:
    x0,y0,x1,y1=o['box_full_image_xyxy'];cv2.rectangle(im,(x0,y0),(x1,y1),COL[o['object_id']],2);cv2.circle(im,tuple(o['point_full_image_xy']),4,COL[o['object_id']],-1)
f,ax=plt.subplots(1,2,figsize=(13,4.8),gridspec_kw={'width_ratios':[1.25,1]});ax[0].imshow(im);tag_ax(ax[0],f'{MODEL} 출력: 박스 + 중심점')
table(ax[1],[[o['object_id'],o['label'],str(o['box_full_image_xyxy']),str(o['point_full_image_xy'])] for o in g['objects']],['ID','라벨','박스 (x0 y0 x1 y1)','중심점'],fs=11)
ax[1].set_title(f"{MODEL}\n응답 {g['api_seconds']:.1f}초 · 물체 {g['object_count']}개 · 로봇 명령 없음",fontsize=12)
f.suptitle('② 물체 지정: VLM은 "무엇이 어디쯤"만 알려 줌 (잡는 위치는 깊이·3D가 결정)',fontsize=16,fontweight='bold');f.tight_layout();save(f,'2a_Gemini_물체지정.png')
f,ax=plt.subplots(1,len(g['objects']),figsize=(4.4*len(g['objects']),4.3))
for x,o in zip(np.atleast_1d(ax),g['objects']):
    x0,y0,x1,y1=o['box_full_image_xyxy'];p=25;x.imshow(rgb(rd(1,'cam1'))[max(0,y0-p):y1+p,max(0,x0-p):x1+p]);tag_ax(x,f"object_{o['object_id']} · {o['label']}")
f.suptitle('② Gemini가 지정한 영역 (물체별 확대)',fontsize=17,fontweight='bold');f.tight_layout();save(f,'2b_Gemini_영역확대.png')
# ---------- 2c 패드 스티커 인식
if pl:
    cams=pl['cameras_used'];cam=cams[0];im=rgb(rd(1,cam)).copy()
    for e in pl['evidence']:
        if e['cam']!=cam or e.get('pad_id') is None:continue
        u,v=e['pixel'];name='할인' if e['label']=='discount' else '정규';cv2.circle(im,(u,v),9,(255,60,60),2);cv2.putText(im,'%s'%('discount' if e['label']=='discount' else 'regular'),(u+12,v-10),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,0),2)
    f,ax=plt.subplots(1,2,figsize=(13,4.9),gridspec_kw={'width_ratios':[1.2,1]});ax[0].imshow(im);tag_ax(ax[0],f'{cam}: Gemini가 찾은 스티커 위치')
    rows=[[('할인' if e['label']=='discount' else '정규'),'패드 %s'%e['pad_id'],'%.0f mm'%(e['distance_m']*1000)] for e in pl['evidence'] if e.get('pad_id') is not None]
    table(ax[1],rows,['스티커 글자','연결된 패드','보정된 패드 중심까지'],fs=13);ax[1].set_title('깊이+카메라 보정으로 테이블 좌표 → 가장 가까운 보정된 패드',fontsize=12)
    f.suptitle('② 패드 스티커(할인·정규)를 AI가 읽어 목적지 패드 결정 (로봇은 보정된 패드 좌표로 이동)',fontsize=15,fontweight='bold');f.tight_layout();save(f,'2c_패드스티커_인식.png')
# ---------- 3 SAM2
im=rgb(rd(1,'cam1')).astype(np.float32)
for o in NM:
    m=mask(o,1,'cam1')
    if m is not None:
        im[m]=im[m]*.45+np.array(COL[o],np.float32)*.55;cnt,_=cv2.findContours(m.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(im,cnt,-1,COL[o],2)
f,ax=plt.subplots(1,2,figsize=(13,4.8));ax[0].imshow(rgb(rd(1,'cam1')));tag_ax(ax[0],'입력: cam1 0°');ax[1].imshow(im.astype(np.uint8));tag_ax(ax[1],f'SAM2 물체 마스크 ({len(NM)}개)')
f.suptitle('③ SAM2: 깊이를 물체별로 나누는 보조 단계 (Gemini 박스·점을 프롬프트로 윤곽 분할)',fontsize=15,fontweight='bold');f.tight_layout();save(f,'3a_SAM2_마스크.png')
f,ax=plt.subplots(2,4,figsize=(13,6.4))
for x,v in zip(ax.ravel(),(1,3,5,7,9,11,12,2)):
    im2=rgb(rd(v,'cam1')).astype(np.float32);m=mask(F,v,'cam1')
    if m is not None:im2[m]=im2[m]*.4+np.array(COL[F],np.float32)*.6
    x.imshow(im2.astype(np.uint8));tag_ax(x,f'{ang[v]}°')
f.suptitle(f'③ 방향이 바뀌어도 같은 물체({NM[F]})의 마스크를 유지 (object_{F})',fontsize=16,fontweight='bold');f.tight_layout();save(f,'3b_SAM2_방향별추적.png')
# ---------- 4 깊이·3D
f,ax=plt.subplots(1,3,figsize=(14,4.4));d=rd(1,'cam1','depth_mm').astype(np.float32);dm=np.ma.masked_where(d==0,d)
ax[0].imshow(rgb(rd(1,'cam1')));tag_ax(ax[0],'컬러');im_=ax[1].imshow(dm,cmap='turbo',vmin=250,vmax=900);tag_ax(ax[1],'깊이 (mm) — 3D 복원의 중심')
m=mask(F,1,'cam1');ys,xs=np.nonzero(m);ob=np.where(m&(d>0),d,np.nan);ax[2].imshow(ob,cmap='turbo',vmin=250,vmax=900);ax[2].set_xlim(xs.min()-30,xs.max()+30);ax[2].set_ylim(ys.max()+30,ys.min()-30);tag_ax(ax[2],f'object_{F} 마스크 안의 깊이만')
f.colorbar(im_,ax=ax[1],fraction=.046);f.suptitle('④ 깊이 영상: 마스크로 나눈 물체의 깊이를 3D 점으로 복원',fontsize=17,fontweight='bold');f.tight_layout();save(f,'4a_깊이영상.png')
c=np.load(S/f'objects/object_{F}/analysis/cloud.npz');pts=c['points'];col=c['colors_rgb']/(255. if c['colors_rgb'].max()>1 else 1.)
f=plt.figure(figsize=(13,5.6));x1=f.add_subplot(1,3,1,projection='3d');x1.scatter(pts[:,0],pts[:,1],pts[:,2],c=col,s=1.2);x1.view_init(25,-60);x1.set_box_aspect(np.ptp(pts,axis=0));x1.set_title('비스듬히',fontsize=14);x1.set_xlabel('X(m)');x1.set_ylabel('Y(m)');x1.set_zlabel('Z(m)')
x2=f.add_subplot(1,3,2);x2.scatter(pts[:,0],pts[:,1],c=col,s=1.2);x2.set_aspect('equal');x2.set_title('위에서 (X-Y)',fontsize=14);x2.set_xlabel('X(m)');x2.set_ylabel('Y(m)')
x3=f.add_subplot(1,3,3);x3.scatter(pts[:,0],pts[:,2],c=col,s=1.2);x3.set_aspect('equal');x3.set_title('옆에서 (X-Z)',fontsize=14);x3.set_xlabel('X(m)');x3.set_ylabel('Z(m)')
f.suptitle(f'④ 12방향 24장 → 테이블 좌표 점군 ({len(pts):,}점, object_{F}·{NM[F]})',fontsize=16,fontweight='bold',y=.99);f.tight_layout(rect=[0,.07,1,.95]);save(f,'4b_3D점군.png')
CLOUD={o:len(np.load(S/f'objects/object_{o}/analysis/cloud.npz')['points']) for o in NM}
# ---------- 5 날짜 판독·분류
ev={o['object']:[x for x in o['observations'] if x.get('date_text')] for o in ex['objects']}
f,ax=plt.subplots(1,len(NM),figsize=(4.4*len(NM),4.4))
for x,o in zip(np.atleast_1d(ax),sorted(NM)):
    x_=ev[f'object_{o}'][0];eid=x_['evidence_id'];v=int(eid[5:8]);cam=eid.split('/')[1];m=mask(o,v,cam);ys,xs=np.nonzero(m);p=20
    x.imshow(rgb(rd(v,cam))[max(0,ys.min()-p):ys.max()+p,max(0,xs.min()-p):xs.max()+p]);tag_ax(x,f"object_{o} · {NM[o]}\n인식: {x_['date_text']} (신뢰도 {x_['confidence']})")
f.suptitle('⑤ 인쇄된 날짜 판독 (Gemini): 여러 방향에서 같은 값이 나와야 채택',fontsize=16,fontweight='bold');f.tight_layout();save(f,'5a_날짜판독.png')
f,ax=plt.subplots(figsize=(13,4.6));rows=[];cc=[]
for o in rt['objects']:
    n=len(ev[o['object']]);st='만료' if o['status']=='expired' else '정상';num=int(o['object'][-1]);dest=DEST[num]+((' ('+o['pad_label']+')') if o.get('pad_label') else '')
    rows.append([NM[num],o['expiry_date'],f"{o['remaining_days']}일",f'{n}회 일치',st,dest]);cc.append(['#fff']*4+['#ffd6d6' if st=='만료' else '#d9f2dd','#fff3c4'])
table(ax,rows,['물체','유통기한','남은 기간','판독 일치','상태','목적지'],fs=14,sc=3,cell=cc)
f.suptitle(f"⑤ 날짜 규칙 적용: 오늘 {rt['today']} 기준\n만료 → 폐기 · 가까운 기한 → 할인 패드 · 먼 기한 → 정규 패드",fontsize=16,fontweight='bold');f.tight_layout();save(f,'5b_분류결과.png')
# ---------- 6 흡착면
cd=json.loads((PREP[F]/'candidate.json').read_text());cr=cd['cross_camera'];fa=cr['face'];fb=cr['other_camera_face'];pt=np.array(fa['point_table_m']);n=np.array(fa['normal_table']);n/=np.linalg.norm(n)
f=plt.figure(figsize=(13,5.2));x1=f.add_subplot(1,2,1,projection='3d');x1.scatter(pts[:,0],pts[:,1],pts[:,2],c=col,s=1.0,alpha=.55)
u=np.cross(n,[1,0,0]);u/=np.linalg.norm(u);v_=np.cross(n,u);th=np.linspace(0,2*np.pi,80);ring=pt[None]+.0125*np.cos(th)[:,None]*u+.0125*np.sin(th)[:,None]*v_
x1.plot(ring[:,0],ring[:,1],ring[:,2],c='deepskyblue',lw=3);x1.quiver(*pt,*(n*.03),color='lime',lw=3);x1.view_init(32,-60);x1.set_box_aspect(np.ptp(pts,axis=0));x1.set_title('점군 위의 흡착컵 Ø25 위치(파랑)와 법선(초록)',fontsize=13);x1.set_position([-.04,0,.58,.80])
a2=f.add_subplot(1,2,2)
rows=[['평면 inlier',f"{fa['inliers']}점 / {fb['inliers']}점"],['평면 RMS (한도 2.5)',f"{fa['patch_rms_mm']:.2f} / {fb['patch_rms_mm']:.2f} mm"],['기울기',f"{fa['tilt_deg']:.1f}° / {fb['tilt_deg']:.1f}°"],['가장자리 여유 (한도 3)',f"{fa['boundary_clearance_mm']:.1f} / {fb['boundary_clearance_mm']:.1f} mm"],['컵 면 덮임',f"{fa['coverage']*100:.0f}% / {fb['coverage']*100:.0f}%"],['두 카메라 법선 차',f"{cr['cross_camera_normal_deg']:.1f}°"],['두 카메라 평면 간격',f"{cr['cross_camera_plane_gap_mm']:.2f} mm"],['판정','통과' if fa['geometry_pass'] and fb['geometry_pass'] else '실패']]
table(a2,rows,['검사 항목','cam1 / cam2']);f.suptitle(f'⑥ 흡착면 선택: 평탄도·가장자리 여유·두 카메라 일치를 모두 통과한 면만 사용 (object_{F})',fontsize=15,fontweight='bold');save(f,'6a_흡착면_검사.png')
f,ax=plt.subplots(1,2,figsize=(12,5.2));top=pts[pts[:,2]>pts[:,2].max()-.004]
ax[0].scatter(pts[:,0],pts[:,1],c=col,s=1.2,alpha=.35);ax[0].scatter(top[:,0],top[:,1],c='orange',s=3,label='윗면 평면 (가장 높은 4 mm)')
ax[0].add_patch(Circle((pt[0],pt[1]),.0125,fill=False,ec='deepskyblue',lw=2.5,label='흡착컵 Ø25'));ax[0].add_patch(Circle((pt[0],pt[1]),.0155,fill=False,ec='gray',ls='--',lw=1.5,label='가장자리 여유 3 mm'))
ax[0].set_aspect('equal');ax[0].legend(loc='upper right',fontsize=10);ax[0].set_xlabel('X(m)');ax[0].set_ylabel('Y(m)');ax[0].set_title('위에서 본 흡착 위치',fontsize=14);ax[1].axis('off')
steps=[('① 위를 향한 평면 후보','기울기 한도 90° 이내'),('② Ø25 컵이 완전히 들어가는 곳','가장자리 여유 3 mm'),('③ 평탄도 검사','평면 RMS ≤ 2.5 mm'),('④ 두 카메라 일치','법선 차·평면 간격 확인'),('⑤ 최종 후보: 면 중심에 가까운 점','가장 높은 곳 1.5 mm 안에서 중심 우선, 가능하면 오른쪽 보정')]
for i,(h,s_) in enumerate(steps):
    y=.9-i*.19;ax[1].add_patch(FancyBboxPatch((.02,y-.07),.96,.14,boxstyle='round,pad=0.01',fc='#eef4ff',ec='#4a7bd0',transform=ax[1].transAxes));ax[1].text(.05,y,h,transform=ax[1].transAxes,fontsize=14,fontweight='bold',va='center');ax[1].text(.05,y-.045,s_,transform=ax[1].transAxes,fontsize=11,va='center',color='#444')
f.suptitle('⑥ 흡착면 선택 절차',fontsize=17,fontweight='bold');f.tight_layout();save(f,'6b_흡착면_선택절차.png')
# ---------- 7 경로·충돌
rtj=json.loads((PREP[F]/'reviewed_trajectories.json').read_text())
KN={'transit_above_target':'목표 위 경유 자세로 이동','approach_3cm':'접근 (흡착 위치 3 cm 위)'}
keys=[k for k in rtj][:3]
f,ax=plt.subplots(1,len(keys),figsize=(4.7*len(keys),4.4),sharey=True)
for x,k in zip(np.atleast_1d(ax),keys):
    jt=rtj[k]['trajectory']['joint_trajectory'];T=np.array([p['time_from_start']['sec']+p['time_from_start']['nanosec']*1e-9 for p in jt['points']]);Q=np.degrees(np.array([p['positions'] for p in jt['points']]))
    for j in range(4):x.plot(T,Q[:,j],lw=2.2,label=f'J{j+1}')
    nm=KN.get(k) or ('패드 %s로 이송'%k.split('ID')[-1] if k.startswith('transfer_to_pad') else '폐기 구역으로 이송' if 'trash' in k else '빈 컵 후퇴' if k.startswith('empty_retreat') else k)
    x.set_title(f"{nm}\n{T[-1]:.1f}초 · 속도 {rtj[k]['scaling']['velocity']*100:.0f}%",fontsize=13);x.set_xlabel('시간 (s)');x.grid(alpha=.3)
np.atleast_1d(ax)[0].set_ylabel('관절 각도 (°)');np.atleast_1d(ax)[0].legend(ncol=4,fontsize=10)
f.suptitle(f'⑦ MoveIt 계획: 충돌 검사를 통과한 관절 궤적 (object_{F}, 실행 전에 검증)',fontsize=15,fontweight='bold');f.tight_layout();save(f,'7a_경로_관절궤적.png')
f,ax=plt.subplots(figsize=(13,4.8));rows=[];cc=[]
for o in OBJS:
    c_=json.loads((PREP[o]/'collision_plan_check.json').read_text());ok=lambda b:'통과' if b else '실패'
    rows.append([f'{NM[o]} → {DEST[o]}',f"{c_['contact_samples']}개",f"{c_['attached_lift_samples']}개",ok(c_['lifted_payload_state_valid']),f"{len(c_['lifted_payload_contacts'])}건",f"{c_['placement_samples']}개" if 'placement_samples' in c_ else '해당 없음'])
    cc.append(['white']*3+['#d9f2dd' if c_['lifted_payload_state_valid'] else '#ffd6d6','#d9f2dd' if not c_['lifted_payload_contacts'] else '#ffd6d6','white'])
table(ax,rows,['물체 → 목적지','접근·접촉 검사','들어올림 검사','든 상태 유효','든 물체 충돌','배치 검사'],fs=13,sc=2.8,colw=[.30,.13,.13,.14,.14,.16],cell=cc)
f.suptitle('⑦ 충돌 검사 결과 (로봇 명령 없이 계획 단계에서 확인)',fontsize=16,fontweight='bold');f.tight_layout();save(f,'7b_충돌검사_결과.png')
# ---------- 8 실행·배치
PN={'approach_3cm':'접근','continuous_contact_approach':'접촉','grip_resting':'흡착','lift_clearance_40mm':'들어올림','holding_before_transfer':'유지','release_on_pad_ID6':'해제','release_on_pad_ID5':'해제','release_at_trash_drop':'해제','complete':'완료','grip_retry_resting_8mm':'재접근 흡착'}
KEEP=set(PN)-{'holding_before_transfer'}
EXE={o:json.loads(glob.glob(str(PREP[o]/'placement_left_*/execution.json'))[0] and Path(glob.glob(str(PREP[o]/'placement_left_*/execution.json'))[0]).read_text()) for o in OBJS}
f,ax=plt.subplots(len(OBJS),1,figsize=(13,2.9*len(OBJS)+.6))
for x,o in zip(np.atleast_1d(ax),OBJS):
    e=EXE[o];ph=e['phases'];t0=ph[0]['time'];T=[p['time']-t0 for p in ph];D=[p['pressure_diff'] for p in ph]
    x.plot(T,D,'-',lw=1,c='#bbb');x.scatter(T,D,s=60,c='#c0392b',zorder=3);x.axhline(-150,ls='--',c='gray',lw=1);x.axhline(-50,ls=':',c='gray',lw=1)
    for t_,d_,p_ in zip(T,D,ph):
        if p_['phase'] in KEEP:x.annotate(PN[p_['phase']],(t_,d_),textcoords='offset points',xytext=(0,10 if d_>-300 else 9),ha='center',fontsize=9)
    extra=''
    if e.get('retries'):
        adv=[r.get('advanced_mm') for r in e['retries'] if r.get('advanced_mm') is not None]
        if adv:extra=f' · 재접근 {adv[0]:.1f} mm'
    if e.get('seal_check_first'):extra+=f" · 1초 유지 확인 최악 {e['seal_check_first']['worst']}"
    x.set_title(f"{NM[o]} → {DEST[o]}   (소요 {T[-1]:.0f}초{extra})",fontsize=13,loc='left');x.set_ylim(-950,250);x.grid(alpha=.3);x.set_ylabel('압력 diff')
np.atleast_1d(ax)[-1].set_xlabel('경과 시간 (s) · 점 = 각 단계 시작 시점의 압력값, 점선 = 유지 −150 / 해제 확인 −50')
f.suptitle('⑧ 실행: 압력 센서로 흡착·운반 유지·해제를 단계마다 확인 (실측)',fontsize=16,fontweight='bold');f.tight_layout();save(f,'8a_실행_압력추이.png')
pads=[(int(cp[f'object_{o}']['pad_id']),o) for o in OBJS if cp[f'object_{o}']['destination']!='trash_drop'];trash=[o for o in OBJS if cp[f'object_{o}']['destination']=='trash_drop']
f,ax=plt.subplots(1,len(pads)+1,figsize=(4.6*(len(pads)+1),5))
for x,(pid,o) in zip(ax[:len(pads)],pads):
    plan=json.loads((S/f'pad_{pid}_placement_plan.json').read_text());cen=np.array(plan['pad_center_table_m']);poly=np.array(plan['placements'][f'object_{o}']['reserved_polygon_table_m'])
    x.add_patch(Circle(cen[:2],plan['radius_m'],fc='#eee',ec='k',lw=2));x.add_patch(Circle(cen[:2],plan['radius_m']-plan['edge_margin_m'],fill=False,ec='gray',ls='--'));x.add_patch(Polygon(poly[:,:2],closed=True,fc=(1,.2,.2,.45),ec='red',lw=2));x.plot(*cen[:2],'k+',ms=12)
    x.set_aspect('equal');x.set_xlim(cen[0]-.06,cen[0]+.06);x.set_ylim(cen[1]-.06,cen[1]+.06);x.set_title(f"패드 {pid} (Ø100) · {NM[o]}\n여유 {plan['edge_margin_m']*1000:.0f} mm · 간격 {plan['object_gap_m']*1000:.0f} mm",fontsize=12);x.set_xlabel('X(m)')
ax[-1].axis('off')
if trash:
    exd=[x_ for x_ in rt['objects'] if x_['object']==f'object_{trash[0]}'][0]
    ax[-1].text(.5,.6,'폐기 구역',ha='center',fontsize=22,fontweight='bold',transform=ax[-1].transAxes);ax[-1].text(.5,.4,f"{NM[trash[0]]}\n(유통기한 {exd['expiry_date']} 만료)\n패드 대신 지정 위치에서 해제",ha='center',fontsize=13,transform=ax[-1].transAxes)
f.suptitle('⑧ 목적지별 배치: 패드는 물체 발자국이 들어가는 빈자리 탐색, 만료 물체는 폐기',fontsize=15,fontweight='bold');f.tight_layout(rect=[0,0,1,.92]);save(f,'8b_목적지_배치.png')
# ---------- 구조도
T={s['stage']:s['seconds'] for s in demo['stages']};total=demo['total_seconds']
def tk(prefix):return sum(v for k,v in T.items() if k.startswith(prefix))
fig=plt.figure(figsize=(16,8.8));ax=fig.add_axes([0,0,1,1]);ax.set_xlim(0,160);ax.set_ylim(0,88);ax.axis('off')
ax.text(80,84,'TUMST_ARM 알고리즘 구조: 처음 보는 물체를 스캔·인식해 유통기한으로 분류',ha='center',fontsize=20,fontweight='bold')
ax.text(80,80.2,f'깊이(점군)가 위치·크기·흡착면 판단의 중심 · SAM2는 깊이를 물체별로 나누는 보조 단계 · {tag} 기준 총 {total:.1f}초',ha='center',fontsize=12.5,color='#44546a')
CLR={'in':'#e8eefc','vis':'#dff3e4','plan':'#fff1cc','act':'#fbe0dc','line':'#44546a'}
def box(x,y,w,h,title,lines,color,t=None):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.25,rounding_size=1.2',fc=color,ec=CLR['line'],lw=1.4));ax.text(x+w/2,y+h-2.3,title,ha='center',va='top',fontsize=11.8,fontweight='bold')
    for i,l in enumerate(lines):ax.text(x+w/2,y+h-6.2-i*3.1,l,ha='center',va='top',fontsize=9.2,color='#222')
    if t is not None:ax.text(x+w-1.0,y+1.0,f'{t:.1f}초',ha='right',va='bottom',fontsize=10.5,fontweight='bold',color='#a23b2c')
def arrow(x1,y1,x2,y2):ax.add_patch(FancyArrowPatch((x1,y1),(x2,y2),arrowstyle='-|>',mutation_scale=16,lw=1.8,color=CLR['line']))
W=23.5;G=2.6;X=lambda i:3+i*(W+G)
r1=[('① 명령 이해·사전 확인',['"테이블 정리해" → 정리 작업','Gemini 3.8 Flash','빈 컵·장치·펌프 확인'],CLR['in'],tk('장치')),
 ('② 초기화·HOME·스캔',['턴테이블 30° × 12방향','D415 2대 RGB-D 24장','이전 스캔 초기화 + HOME'],CLR['act'],tk('턴테이블·패드 이전')+tk('HOME')+tk('듀얼')),
 ('③ 물체 식별 + 3D 복원',['Gemini: 이름·위치(박스·점)','SAM2: 물체별 픽셀 마스크','깊이→점군 (마스크 3px 침식)'],CLR['vis'],tk('물체 식별')),
 ('④ 날짜 판독',['Gemini: 인쇄 날짜 판독','여러 방향에서 일치해야 채택','불확실하면 QR 보조'],CLR['vis'],tk('인쇄 날짜')),
 ('⑤ 패드 스티커 인식',['"할인"·"정규" 스티커를 Gemini가 읽음','깊이+보정으로 보정된 패드에 연결','못 읽으면 움직이지 않고 중단'],CLR['vis'],tk('패드 스티커')),
 ('⑥ 날짜 분류 (로컬 규칙)',['만료 → 폐기 구역','가까운 기한 → 할인 패드','먼 기한 → 정규 패드'],CLR['plan'],tk('로컬 날짜'))]
for i,(t_,l_,c_,s_) in enumerate(r1):box(X(i),56,W,20,t_,l_,c_,s_)
for i in range(5):arrow(X(i)+W+.4,66,X(i+1)-.4,66)
r2=[('⑦ 흡착면 선택',['위를 향한 평면 · Ø25 컵 + 여유 3 mm','평탄도 RMS ≤ 2.5 mm · 두 카메라 일치','면 중심 우선 + 오른쪽 보정'],CLR['plan'],None),
 ('⑧ 패드 빈자리 선확인',['패드 5·6 점유 확인','물체 발자국이 들어갈 위치 예약','간격 2 mm · 발자국 여유 2.5 mm'],CLR['plan'],tk('패드 5')+tk('패드 6')),
 ('⑨ 전체 경로 검토',['MoveIt/OMPL + 정밀 충돌 검사','이웃 물체 여유 4 mm','모든 물체 경로를 먼저 계산·검증'],CLR['plan'],None),
 ('⑩ 집기·이송·놓기',['서보 STS 4축 + 진공 컵 (ESP32)','흡착 ≤ −150, 1초 유지 확인','실패 시 8 mm 재접근(압력 닿으면 정지)'],CLR['act'],tk('전체 경로')),
 ('⑪ 실행 중 확인',['단계마다 팔 자세 ≤ 5° (어긋나면 재계획)','놓은 물체 위치 확인','패드 빈 상태 확인(못 보면 시작 확인 사용)'],CLR['act'],None)]
for i,(t_,l_,c_,s_) in enumerate(r2):box(X(i),27,W,20,t_,l_,c_,s_)
for i in range(4):arrow(X(i)+W+.4,37,X(i+1)-.4,37)
ax.plot([X(5)+W/2,X(5)+W/2,X(0)+W/2],[55.8,51.5,51.5],color=CLR['line'],lw=1.8);arrow(X(0)+W/2,51.5,X(0)+W/2,47.6);ax.text(80,52.6,'분류 결과(목적지)가 물체별 흡착·경로 계획으로 전달',fontsize=9.5,color=CLR['line'],ha='center',va='center')
items=[('명령',tk('장치'),'#9db4ec'),('스캔·HOME',tk('턴테이블·패드 이전')+tk('HOME')+tk('듀얼'),'#e69a8d'),('식별·3D',tk('물체 식별'),'#7fc79a'),('날짜 판독',tk('인쇄 날짜'),'#5aa879'),('패드 스티커',tk('패드 스티커'),'#8ed1c6'),('분류·패드',tk('로컬 날짜')+tk('패드 5')+tk('패드 6'),'#f0d27a'),('경로 검토·집기·분류',tk('전체 경로'),'#d96f5f')]
other=total-sum(v for _,v,_ in items);items.append(('단계 사이',other,'#c8c8c8'))
ax.text(6,21.5,f'총 {total:.1f}초 (사전 확인 ~ 마지막 HOME 복귀)',fontsize=13,fontweight='bold');x=6;WB=148;small=0
for name,v,c in items:
    w=WB*v/total;ax.add_patch(plt.Rectangle((x,10.5),w,6.2,fc=c,ec='white',lw=1.5))
    if w>9:ax.text(x+w/2,13.6,f'{name}\n{v:.1f}초',ha='center',va='center',fontsize=9.2,fontweight='bold')
    else:
        ax.text(x+w/2,9.2-3.4*(small%2),f'{name} {v:.1f}초',ha='center',va='top',fontsize=8,rotation=0);ax.plot([x+w/2,x+w/2],[10.5,9.4-3.4*(small%2)],color='#888',lw=.6);small+=1
    x+=w
ax.text(80,2.4,f'값은 {tag}의 단계 기록 · 실행 완료는 소프트웨어 기록이며 사람이 성공으로 확정한 값은 아님',ha='center',fontsize=9.5,color='#666')
fig.savefig(OUT/'0_알고리즘_구조도.png',dpi=190,facecolor='white');fig.savefig(OUT/'0_알고리즘_구조도.svg',facecolor='white');plt.close(fig);print('0_알고리즘_구조도')
# ---------- 데이터 요약
with (OUT/'단계시간.csv').open('w',newline='',encoding='utf-8-sig') as fh:
    w=csv.writer(fh);w.writerow(['단계','초']);[w.writerow([k,round(v,1)]) for k,v in T.items()];w.writerow(['단계 사이(기록 밖)',round(other,1)]);w.writerow(['합계',round(total,1)])
L=[f'# 데이터 요약 · {tag}','',f'- 실행 ID `{RUN.name}` · 결과 {demo["status"]} · 총 {total:.1f}초 · 모델 {MODEL}','- 완료는 소프트웨어 기록이며 사람이 성공으로 확정한 값은 아님','','## 단계 시간 (초)','| 단계 | 초 |','|---|---|']
L+=[f'| {k} | {v:.1f} |' for k,v in T.items()]+[f'| 단계 사이(기록 밖) | {other:.1f} |',f'| **합계** | **{total:.1f}** |','','## 물체·분류','| 물체 | 유통기한 | 남은 기간 | 판독 일치 | 목적지 | 점군 수 |','|---|---|---|---|---|---|']
for o in rt['objects']:
    num=int(o['object'][-1]);L.append(f"| {NM[num]} | {o['expiry_date']} | {o['remaining_days']}일 | {len(ev[o['object']])}회 | {DEST[num]}{(' ('+o['pad_label']+')') if o.get('pad_label') else ''} | {CLOUD[num]:,} |")
if pl:L+=['',f"## 패드 스티커 인식: 할인=패드 {pl['discount_pad_id']} · 정규=패드 {pl['regular_pad_id']} (사용 카메라 {', '.join(pl['cameras_used'])})"]
L+=['','## 집기 압력 (원시 diff)','| 물체 | 흡착 직후 | 들어올림 | 해제 | 1초 유지 확인 최악 | 재접근 |','|---|---|---|---|---|---|']
for o in OBJS:
    e=EXE[o];ph={p['phase']:p.get('pressure_diff') for p in e['phases']};rel=next((v for k,v in ph.items() if k.startswith('release')),None)
    adv=[r.get('advanced_mm') for r in (e.get('retries') or []) if r.get('advanced_mm') is not None]
    L.append(f"| {NM[o]} | {ph.get('grip_resting')} | {ph.get('lift_clearance_40mm')} | {rel} | {(e.get('seal_check_first') or {}).get('worst','-')} | {('%.1f mm'%adv[0]) if adv else '없음'} |")
(OUT/'데이터_요약.md').write_text('\n'.join(L)+'\n');print('데이터_요약.md')
