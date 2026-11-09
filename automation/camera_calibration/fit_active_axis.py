from pathlib import Path
import itertools, json, numpy as np, cv2
r=Path(__file__).resolve().parent;sp=Path(json.loads((r/'turntable_marker/active_rescan.json').read_text())['series_file']);s=json.loads(sp.read_text());out={'series_file':str(sp),'calibrations':s['calibrations'],'reported_angles_deg':[o['reported_angle_deg'] for o in s['observations']],'angle_source':s['angle_source'],'axis_point_convention':'point on axis at mean observed marker height; not motor origin','independently_validated':False,'validated_for_robot_execution':False,'cameras':{}}
def mean_rotation(rotations):
 U,_,V=np.linalg.svd(np.mean(rotations,axis=0));return U@np.diag([1,1,np.linalg.det(U@V)])@V
for c in ['cam1','cam2']:
 T=np.array(json.loads(Path(s['calibrations'][c]).read_text())[f'T_table_{c}_color_optical']);poses=[]
 for obs in s['observations']:
  d=json.loads(Path(obs['cameras'][c]['source']).read_text());R=T[:3,:3]@mean_rotation([cv2.Rodrigues(np.array(a['rvec']))[0] for a in d['samples']]);p=T[:3,:3]@np.median([a['translation_m'] for a in d['samples']],axis=0)+T[:3,3];poses.append((R,p))
 pairs=[];axes=[]
 for i,j in itertools.combinations(range(len(poses)),2):
  Ri,pi=poses[i];Rj,pj=poses[j];Q=Rj@Ri.T;rv=cv2.Rodrigues(Q)[0].flatten();ang=np.linalg.norm(rv)
  if ang<np.radians(20):continue
  a=rv/ang;a=a if a[2]>=0 else -a;axes.append(a);pairs.append((Q,pj-Q@pi))
 n=np.mean(axes,axis=0);n/=np.linalg.norm(n);height=float(np.mean([n@p for R,p in poses]));A=np.vstack([np.eye(3)-Q for Q,b in pairs]+[n.reshape(1,3)]);b=np.concatenate([b for Q,b in pairs]+[[height]]);center=np.linalg.lstsq(A,b,rcond=None)[0];intercept=center-n*(center[2]/n[2]);res=np.array([np.linalg.norm((np.eye(3)-Q)@center-b)*1000 for Q,b in pairs]);axisdev=[float(np.degrees(np.arccos(np.clip(n@a,-1,1)))) for a in axes]
 # Transform each observed marker origin back into the first orientation.
 R0,p0=poses[0];dewarped=np.array([center+(R0@R.T)@(p-center) for R,p in poses]);spread=np.linalg.norm(dewarped-np.mean(dewarped,axis=0),axis=1)*1000
 out['cameras'][c]={'axis_direction_table_positive_z':n.tolist(),'axis_point_table_m':center.tolist(),'axis_intersection_table_z0_m':intercept.tolist(),'axis_tilt_from_vertical_deg':float(np.degrees(np.arccos(n[2]))),'pairwise_fit_rms_mm':float(np.sqrt(np.mean(res**2))),'pairwise_fit_max_mm':float(max(res)),'derotated_marker_center_max_deviation_mm':float(max(spread)),'relative_axis_direction_max_deviation_deg':max(axisdev),'observed_marker_height_mean_m':float(np.mean([p[2] for R,p in poses]))}
 print(c,json.dumps(out['cameras'][c]))
a=out['cameras']['cam1'];b=out['cameras']['cam2'];out['cross_camera_axis_intersection_difference_mm']=float(np.linalg.norm(np.array(a['axis_intersection_table_z0_m'])-b['axis_intersection_table_z0_m'])*1000);out['cross_camera_axis_direction_difference_deg']=float(np.degrees(np.arccos(np.clip(np.dot(a['axis_direction_table_positive_z'],b['axis_direction_table_positive_z']),-1,1))))
f=sp.parent/'axis_fit_preliminary.json';f.write_text(json.dumps(out,ensure_ascii=False,indent=2));print('cross camera mm',out['cross_camera_axis_intersection_difference_mm'],'saved',f)
