"""Same-face local depth refinement from two stopped cam1 observations; no motors."""
import sys,json,cv2,numpy as np,yaml
from pathlib import Path
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from placement_geometry import depth_table
from inclined_runtime import depth_scale
from inclined_geometry import planes,patch,rotate_surface,alignment_angles,compensated_axis,solve_axis
from motion.kinematics import fk,m,W
s=Path(sys.argv[1]).resolve();obs=s/'local_contact_observations';wc=yaml.safe_load((s/'workcell_snapshot.yaml').read_text());report=json.loads((s/'inclined_pick_plan.json').read_text());axis=wc['turntable']['axis_point_table_m'];state=json.loads((obs/'turntable_state.json').read_text());angle=state['angle_deg'];obj=next(o for o in report['objects'] if o['object']=='object_2');shape=np.load(Path(obj['session'])/'analysis/cloud.npz')['points'];R=Rotation.from_euler('z',angle,degrees=True).as_matrix();shape=(shape-np.array(axis))@R.T+axis;center=np.median(shape,0);tree=cKDTree(shape);clouds=[]
for view in ['view_000','view_001']:
 folder=obs/view;md=json.loads((folder/'metadata.json').read_text());d=md['cameras']['cam1'];raw=cv2.imread(str(folder/d['depth_file']),-1);im=cv2.imread(str(folder/d['color_file']));P,valid=depth_table(raw,depth_scale(d,raw),d['camera_info'],wc['cameras']['cam1']['T_table_optical']);hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);region=valid&(np.linalg.norm(P[:,:,:2]-center[:2],axis=2)<.045)&(P[:,:,2]>wc['turntable']['top_height_m']+.008)&(hsv[:,:,1]>55)&(hsv[:,:,0]>30)&(hsv[:,:,0]<95);clouds.append(P[region])
second_planes=planes(clouds[1],max_planes=6);confirmed=[]
for pts,n,d in planes(clouds[0],max_planes=6):
 r=patch(clouds[0],n,d,prefer_high=True)
 if not r or not r['geometry_pass'] or not 25<r['tilt_deg']<=90 or r['point_table_m'][2]<.088:continue
 p=np.array(r['point_table_m'])
 if tree.query(p)[0]>.008:continue
 for _,n2,d2 in second_planes:
  a=float(np.degrees(np.arccos(np.clip(n@n2,-1,1))));gap=abs(float(p@n2+d2));r2=patch(clouds[1],n2,d2,center_point=p)
  if a<2 and gap<.001 and r2 and r2['geometry_pass']:confirmed.append((r,r2,a,gap))
if not confirmed:raise RuntimeError('No full cup patch confirmed in both fresh stopped cam1 observations')
first,second,temporal_angle,gap=max(confirmed,key=lambda x:(x[0]['point_table_m'][2],min(x[0]['boundary_clearance_mm'],x[1]['boundary_clearance_mm'])));normal=np.array(first['normal_table'])+second['normal_table'];normal/=np.linalg.norm(normal);p0,n0=rotate_surface(first['point_table_m'],normal,axis,-angle);j1=(W@np.r_[[-.0177,.0177,.1598],1])[:3];lo=np.array([j['limit']['lower'] for j in m['joints']],float);hi=np.array([j['limit']['upper'] for j in m['joints']],float);options=[]
for turn in alignment_angles(p0,n0,axis,j1):
 p,n=rotate_surface(p0,n0,axis,turn);desired=compensated_axis(p,n,j1);contact=solve_axis(fk,lo,hi,p,desired)
 if contact is None:continue
 above=solve_axis(fk,lo,hi,p+n*.02,desired,seed=contact[0])
 if above is None:continue
 options.append({'turntable_angle_deg':turn,'point_table_m':p.tolist(),'normal_table':n.tolist(),'commanded_tool_axis_table':desired.tolist(),'contact_q_rad':contact[0].tolist(),'approach_q_rad':above[0].tolist(),'approach_clearance_m':.02,'lift_clearance_m':.03,'collision_checked':False,'local_pose_method':'two temporally independent stopped cam1 depth frames, not cross-camera same-point consensus'})
if not options:raise RuntimeError('Local patch has no fixed-axis contact/approach IK')
verification={'camera':'cam1','observations':[str(obs/'view_000'),str(obs/'view_001')],'normal_difference_deg':temporal_angle,'plane_gap_mm':gap*1000,'cup_coverage_each':[first['coverage'],second['coverage']],'edge_clearance_mm_each':[first['boundary_clearance_mm'],second['boundary_clearance_mm']],'patch_rms_mm_each':[first['patch_rms_mm'],second['patch_rms_mm']],'cross_camera_same_point_verified':False,'global_geometry_from':'two-camera12-angle reconstruction','current_turntable_state':state}
backup=s/'inclined_pick_plan_before_local_depth.json'
if not backup.exists():backup.write_text(json.dumps(report,indent=2))
new={'face':first,'other_camera_face':second,'cross_camera_normal_deg':obj['matched_faces'][0]['cross_camera_normal_deg'],'cross_camera_plane_gap_mm':obj['matched_faces'][0]['cross_camera_plane_gap_mm'],'alignment_options':options,'local_contact_verification':verification}
first.update(point_table_m=p0.tolist(),normal_table=n0.tolist());obj['matched_faces']=[new];report['status']='local contact refined with repeated cam1 depth; global reconstruction unchanged; collision planning pending';(s/'inclined_pick_plan.json').write_text(json.dumps(report,indent=2));(s/'local_contact_verification.json').write_text(json.dumps(verification,indent=2));print(json.dumps({'verification':verification,'options':options},indent=2))
