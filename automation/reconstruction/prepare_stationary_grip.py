"""Prepare current-pose pick with static table; no actuator messages."""
import sys,json,yaml,cv2,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from inclined_geometry import solve_axis,compensated_axis,patch,rotate_surface
from motion.kinematics import fk,m,W
from placement_geometry import depth_table
from inclined_runtime import depth_scale,fresh_face_check
from reconstruct import voxel
relative_turn=float(sys.argv[2]) if len(sys.argv)>2 else 0.
target_object=sys.argv[3] if len(sys.argv)>3 else None
s=Path(sys.argv[1]);wc=yaml.safe_load((s/'workcell_snapshot.yaml').read_text());measurement=json.loads((s/'flat_face_report.json').read_text());identities=json.loads((s/'stationary_reference/color_sort_plan.json').read_text())['objects'];lo=np.array([j['limit']['lower'] for j in m['joints']],float);hi=np.array([j['limit']['upper'] for j in m['joints']],float);j1=(W@np.r_[[-.0177,.0177,.1598],1])[:3];options=[];review=[]
for obj in measurement['objects']:
 if target_object and obj['object']!=target_object:continue
 for face in obj['faces']:
  r=face['patch']
  if not r['geometry_pass']:continue
  base=np.array(r['point_table_m']);normal=np.array(r['normal_table']);original_base=base.copy();original_normal=normal.copy();folder=s/face['view'];md=json.loads((folder/'metadata.json').read_text())['cameras'][face['camera']];raw=cv2.imread(str(folder/md['depth_file']),-1);P,valid=depth_table(raw,depth_scale(md,raw),md['camera_info'],wc['cameras'][face['camera']]['T_table_optical']);mask=(cv2.imread(str(s/'objects'/obj['object']/'sam2'/face['view']/(face['camera']+'_mask.png')),0)>127)&valid;cloud=P[mask]
  if relative_turn:
   base,normal=rotate_surface(base,normal,wc['turntable']['axis_point_table_m'],relative_turn);rot=Rotation.from_euler('z',relative_turn,degrees=True).as_matrix();pivot=np.array(wc['turntable']['axis_point_table_m']);cloud=(cloud-pivot)@rot.T+pivot
  plane_d=float(-base@normal);u=np.cross(np.eye(3)[np.argmin(abs(normal))],normal);u/=np.linalg.norm(u);v=np.cross(normal,u)
  candidates=[base+u*x+v*y for x in np.arange(-.008,.0081,.002) for y in np.arange(-.008,.0081,.002)];candidates.sort(key=lambda p:np.linalg.norm(p-base))
  for point in candidates:
   local=patch(cloud,normal,plane_d,center_point=point)
   if not local or not local['geometry_pass']:continue
   axis=compensated_axis(point,normal,j1);contact=solve_axis(fk,lo,hi,point,axis,attempts=4)
   info={'object':obj['object'],'point_table_m':point.tolist(),'tilt_deg':r['tilt_deg'],'contact_ik':contact is not None};review.append(info)
   if contact is None:continue
   approach=solve_axis(fk,lo,hi,point+normal*.02,axis,seed=contact[0],attempts=4);info['approach_ik']=approach is not None
   if approach is None:continue
   try:
    check_point,check_normal=rotate_surface(point,normal,wc['turntable']['axis_point_table_m'],-relative_turn) if relative_turn else (point,normal)
    checks=[fresh_face_check(s/view,wc,check_point,check_normal) for view in ['view_000','view_001']]
   except RuntimeError:continue
   chosen=dict(face);chosen['patch']=local;options.append((obj,chosen,axis,contact,approach,checks));break
(s/'stationary_ik_review.json').write_text(json.dumps(review,indent=2))
if not options:raise RuntimeError('No freshly supported face with current-table contact/approach IK')
obj,face,axis,contact,approach,checks=max(options,key=lambda row:(row[1]['patch']['point_table_m'][2],row[1]['patch']['boundary_clearance_mm']));r=face['patch'];point=r['point_table_m'];normal=r['normal_table'];prep=s/(sys.argv[4] if len(sys.argv)>4 else 'stationary_prepared');prep.mkdir();(prep/'analysis').mkdir();(prep/'reconstruction_sam2').mkdir();(prep/'obstacles').mkdir();(prep/'workcell_snapshot.yaml').write_text(yaml.safe_dump(wc));t=wc['turntable'];ctx={'axis_point_table_m':t['axis_point_table_m'],'axis_direction_table':t['axis_direction_table'],'support_radius_m':t['diameter_m']/2,'observation_radius_m':t['diameter_m']/2+.03,'top_height_m':t['top_height_m'],'calibrations':{c:{f'T_table_{c}_color_optical':wc['cameras'][c]['T_table_optical']} for c in ['cam1','cam2']}};(prep/'spatial_context.json').write_text(json.dumps(ctx,indent=2));other=[]
for identity in identities:
 clouds=[];colors=[]
 for view in ['view_000','view_001']:
  folder=s/view;md=json.loads((folder/'metadata.json').read_text())
  for cam in ['cam1','cam2']:
   d=md['cameras'][cam];raw=cv2.imread(str(folder/d['depth_file']),-1);P,valid=depth_table(raw,depth_scale(d,raw),d['camera_info'],wc['cameras'][cam]['T_table_optical']);mask=(cv2.imread(str(s/'objects'/identity['object']/'sam2'/view/(cam+'_mask.png')),0)>127)&valid&(P[:,:,2]>t['top_height_m']+.006)&(np.linalg.norm(P[:,:,:2]-np.array(t['axis_point_table_m'])[:2],axis=2)<.105);clouds.append(P[mask]);colors.append(cv2.imread(str(folder/d['color_file']))[mask][:,::-1])
 P=np.vstack(clouds);C=np.vstack(colors);P,C=voxel(P,C)
 if relative_turn:
  rot=Rotation.from_euler('z',relative_turn,degrees=True).as_matrix();pivot=np.array(wc['turntable']['axis_point_table_m']);P=(P-pivot)@rot.T+pivot
 if identity['object']==obj['object']:np.savez_compressed(prep/'analysis/cloud.npz',points=P,colors_rgb=C)
 else:
  path=prep/'obstacles'/(identity['object']+'.npz');np.savez_compressed(path,points=P);low=P.min(0);high=P.max(0);other.append({'id':'inclined_obstacle_'+identity['object'],'mesh_cloud_file':str(path),'padding_m':.001,'center_table_m':((low+high)/2).tolist(),'dims':(high-low).tolist(),'rgb_255':np.median(C,0).tolist()})
state=json.loads((s/'table_state.json').read_text());identity=next(i for i in identities if i['object']==obj['object']);option={'point_table_m':point,'normal_table':normal,'commanded_tool_axis_table':axis.tolist(),'contact_q_rad':contact[0].tolist(),'approach_q_rad':approach[0].tolist(),'approach_clearance_m':.02,'lift_clearance_m':.01}
candidate={'color':identity['color'],'camera':face['camera'],'position_table_m':point,'nominal_contact_position_table_m':point,'normal_table':normal,'commanded_tool_axis_table':axis.tolist(),'approach_clearance_m':.02,'lift_clearance_m':.01,'fresh_checks':checks};(prep/'candidate.json').write_text(json.dumps({'selected_candidate':candidate,'results':{face['camera']:{'geometry_pass':True,'normal_table':normal}}},indent=2));(prep/'reconstruction_sam2/processing_status.json').write_text(json.dumps({'candidate_file':str(prep/'candidate.json')}));(prep/'inclined_context.json').write_text(json.dumps({'selected_object':obj['object'],'selected_color':identity['color'],'destination_pad_id':identity['destination_pad_id'],'other_objects':other,'option':option,'source_scan':str(s),'simulated_aligned_scene':bool(relative_turn),'table_rotation_performed':False,'relative_turn_deg':relative_turn},indent=2));(prep/'session.json').write_text(json.dumps({'views':[{'path':'view_000'}],'aligned_turntable_angle_deg':state['angle_deg']+relative_turn,'return_to_zero':{'status':'not_performed_stationary_trial','state':state},'scope':'fresh stationary geometry; no encoder-zero assumption'},indent=2));print(json.dumps({'prepared':str(prep),'object':identity['label'],'destination_pad':identity['destination_pad_id'],'option':option},ensure_ascii=False,indent=2))
