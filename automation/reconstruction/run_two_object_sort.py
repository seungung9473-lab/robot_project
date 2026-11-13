#!/usr/bin/env python3
"""Fresh two-object scan -> ER2/SAM2 geometry -> collision-checked sorting."""
import argparse,datetime,fcntl,json,os,subprocess,sys
from pathlib import Path
import cv2,numpy as np
from placement_geometry import load_context,depth_table,object_region
R=Path(__file__).resolve().parent;ROOT=R.parent
p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');args=p.parse_args()
if not args.execute:
 print('No movement. --execute runs home, 12-view scan, reconstruction, big->left ID5, small->right ID6, final home.');sys.exit(0)
if os.environ.get('ROS_DOMAIN_ID')!='73':raise RuntimeError('Actual hardware requires ROS_DOMAIN_ID=73')
lock=(R/'scan_pick_cycle.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
runs=R/'pipeline_runs';runs.mkdir(exist_ok=True);log=runs/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_two_object_sort.json');status={'success':False}
def stage(name):
 status['stage']=name;log.write_text(json.dumps(status,ensure_ascii=False,indent=2)+'\n');print('\n'+name,flush=True)
def run(*command):subprocess.run([str(x) for x in command],check=True,cwd=ROOT)
try:
 stage('로봇 홈 이동');run(sys.executable,Path.home()/'robot_ws/src/TUMST_ARM_moveit_config/scripts/go_home.py')
 stage('30도 간격 12방향 촬영 후 0도 복귀');base=R/'multiview';before=set(base.iterdir());run('ros2','run','soarm_turntable','turntable_scan','scan','--angles',*[str(-30*i) for i in range(12)],'--output',base)
 added=[x for x in set(base.iterdir())-before if (x/'session.json').exists()]
 if len(added)!=1:raise RuntimeError('New scan session could not be identified')
 session=added[0];status['session']=str(session);manifest=json.loads((session/'session.json').read_text());view=session/manifest['views'][0]['path'];md=json.loads((view/'metadata.json').read_text())['cameras']['cam2'];ctx=load_context(session);depth=cv2.imread(str(view/md['depth_file']),-1);points,valid=depth_table(depth,md['depth_scale_m'],md['camera_info'],ctx['calibrations']['cam2']['T_table_cam2_color_optical']);region=valid&object_region(points,ctx);region&=np.linalg.norm(points[:,:,:2]-np.array(ctx['axis_point_table_m'])[:2],axis=2)<ctx['support_radius_m'];yy,xx=np.where(region)
 if len(xx)<150:raise RuntimeError('No sufficient object depth on turntable; no robot pick')
 image=view/md['color_file'];height,width=depth.shape;crop=[max(0,int(xx.min())-30),max(0,int(yy.min())-30),min(width,int(xx.max())+31),min(height,int(yy.max())+31)]
 stage('ER2 두 물체 인식');before=set((ROOT/'er2/runs').iterdir());run(sys.executable,ROOT/'er2/select_object.py','prepare','--image',image,'--crop',*crop,'--instruction','턴테이블 위 두 물체를 모두 구분해줘','--all');new=set((ROOT/'er2/runs').iterdir())-before
 if len(new)!=1:raise RuntimeError('New recognition run ambiguous')
 recognition=new.pop();status['recognition']=str(recognition);run(sys.executable,ROOT/'er2/select_object.py','select',recognition)
 stage('물체별 SAM2 형상 복원 및 흡착면 검증');run(sys.executable,R/'process_two_objects.py',session,'--detection',recognition/'selection_result.json')
 stage('크기별 놓기 및 홈 복귀');run(sys.executable,R/'execute_two_object_sort.py',session,'--execute')
 status['success']=True;stage('완료: 큰 물체 왼쪽 / 작은 물체 오른쪽 / 홈 복귀')
except BaseException as exc:
 status['error']=str(exc);stage('중단: 실제 상태와 로그를 확인하세요. 흡착 유지 중이면 펌프를 자동 해제하지 않습니다.');raise
finally:fcntl.flock(lock,fcntl.LOCK_UN);lock.close()
