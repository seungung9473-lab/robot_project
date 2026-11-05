"""Offline common-disk refinement. No actuator commands."""
import sys,json,yaml,cv2,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from inclined_geometry import planes,patch,rotate_surface,alignment_angles,compensated_axis,solve_axis
from inclined_runtime import depth_scale
from placement_geometry import depth_table
from motion.kinematics import fk,m,W
import argparse
parser=argparse.ArgumentParser(description='Offline refinement from an aligned cam1 capture and independent scan observations');parser.add_argument('session',type=Path);parser.add_argument('prepared',type=Path);parser.add_argument('observation',type=Path);parser.add_argument('--min-contact-z',type=float,default=.088);args=parser.parse_args();s=args.session.resolve();prep=args.prepared.resolve();wc=yaml.safe_load((s/'workcell_snapshot.yaml').read_text());axis=wc['turntable']['axis_point_table_m'];o=json.loads((prep/'inclined_context.json').read_text())['option'];point=np.array(o['point_table_m']);v=args.observation.resolve();md=json.loads((v/'metadata.json').read_text());actual=json.loads((prep/'view_000/metadata.json').read_text())['turntable']['angle_deg'];d=md['cameras']['cam1'];raw=cv2.imread(str(v/d['depth_file']),-1);P,valid=depth_table(raw,depth_scale(d,raw),d['camera_info'],wc['cameras']['cam1']['T_table_optical']);cloud=P[valid&(np.linalg.norm(P-point,axis=2)<.055)];faces=[r for pp,n,pd in planes(cloud,max_planes=8) if (r:=patch(cloud,n,pd,prefer_high=True)) is not None and r['geometry_pass'] and r['tilt_deg']>25];face=max(faces,key=lambda x:x['boundary_clearance_mm']);p0,n0=rotate_surface(face['point_table_m'],face['normal_table'],axis,-actual);print('new reference',p0,n0,flush=True);matches=[]
manifest=json.loads((s/'session.json').read_text())
for view in manifest['views']:
 folder=s/view['path'];meta=json.loads((folder/'metadata.json').read_text());c=meta['cameras']['cam2'];raw=cv2.imread(str(folder/c['depth_file']),-1);P,valid=depth_table(raw,c['depth_scale_m'],c['camera_info'],wc['cameras']['cam2']['T_table_optical']);mask=cv2.imread(str(s/'objects/object_2/sam2'/view['path']/'cam2_mask.png'),0)>127;pts=P[valid&mask]
 from scipy.spatial.transform import Rotation
 R=Rotation.from_euler('z',-meta['turntable']['angle_deg'],degrees=True).as_matrix();pts=(pts-np.asarray(axis))@R.T+axis
 for pp,n,pd in planes(pts,max_planes=5):
  angle=float(np.degrees(np.arccos(np.clip(n@n0,-1,1))))
  if angle>5:continue
  old_p0,_=rotate_surface(o['point_table_m'],o['normal_table'],axis,-actual)
  for fraction in np.linspace(0,1,21):
   trial=old_p0*(1-fraction)+p0*fraction
   # Project compromise onto the freshly observed cam1 plane.
   fresh_point,fresh_n=rotate_surface(trial,n0,axis,actual);fresh_point-=fresh_n*(fresh_point@fresh_n+face['plane_d'])
   first_patch=patch(cloud,fresh_n,face['plane_d'],center_point=fresh_point)
   ref_point,_=rotate_surface(fresh_point,fresh_n,axis,-actual)
   gap=abs(float(ref_point@n+pd));r=patch(pts,n,pd,center_point=ref_point)
   if gap<.003 and r and r['geometry_pass'] and first_patch and first_patch['geometry_pass'] and ref_point[2]>=args.min_contact_z:
    matches.append({'view':view['path'],'angle':angle,'gap_mm':gap*1000,'patch':r,'first_patch':first_patch,'refined_point':ref_point.tolist()})
print('MATCHES',json.dumps(matches),flush=True)
if not matches:raise RuntimeError('No independent cam2 support for new point')
match=max(matches,key=lambda x:(x['refined_point'][2],min(x['patch']['boundary_clearance_mm'],x['first_patch']['boundary_clearance_mm'])));p0=np.array(match['refined_point']);face=match['first_patch'];normal=(n0+np.array(match['patch']['normal_table']));normal/=np.linalg.norm(normal);j1=(W@np.r_[[-.0177,.0177,.1598],1])[:3];lo=np.array([j['limit']['lower'] for j in m['joints']],float);hi=np.array([j['limit']['upper'] for j in m['joints']],float);options=[]
for turn in alignment_angles(p0,normal,axis,j1):
 p,n=rotate_surface(p0,normal,axis,turn);desired=compensated_axis(p,n,j1);contact=solve_axis(fk,lo,hi,p,desired)
 if contact is None:continue
 approach=solve_axis(fk,lo,hi,p+n*.02,desired,seed=contact[0])
 if approach is None:continue
 options.append({'turntable_angle_deg':turn,'point_table_m':p.tolist(),'normal_table':n.tolist(),'commanded_tool_axis_table':desired.tolist(),'contact_q_rad':contact[0].tolist(),'approach_q_rad':approach[0].tolist(),'contact_position_error_mm':contact[1]*1000,'contact_axis_error_deg':contact[2],'approach_clearance_m':.02,'lift_clearance_m':.03,'collision_checked':False})
print('OPTIONS',json.dumps(options),flush=True)
report=json.loads((s/'inclined_pick_plan.json').read_text());backup=s/'inclined_pick_plan_before_refinement.json';
if not backup.exists():backup.write_text(json.dumps(report,indent=2))
face.update(point_table_m=p0.tolist(),normal_table=n0.tolist(),observation='fresh aligned cam1');new={'face':face,'other_camera_face':match['patch'],'cross_camera_normal_deg':match['angle'],'cross_camera_plane_gap_mm':match['gap_mm'],'alignment_options':options,'refined_after_alignment':True}
for obj in report['objects']:
 obj['matched_faces']=[new] if obj['object']=='object_2' else []
(s/'inclined_pick_plan.json').write_text(json.dumps(report,indent=2))
