"""Refine suction location in the actual0deg view; retain full multi-view shape."""
import argparse,json,subprocess,sys,shutil
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
r=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('session',type=Path);a=p.parse_args();session=a.session.resolve();manifest=json.loads((session/'session.json').read_text());first=manifest['views'][0]
if abs(first['angle_deg'])>1:raise RuntimeError('Reference view must be observed at0deg')
z=session/'reference_view';z.mkdir(exist_ok=True);subset=dict(manifest);subset['views']=[first];(z/'session.json').write_text(json.dumps(subset,indent=2));shutil.copy(session/'spatial_context.json',z/'spatial_context.json');link=z/first['path']
if not link.exists():link.symlink_to(session/first['path'],target_is_directory=True)
out=session/'reconstruction_reference'
subprocess.run([sys.executable,str(r/'reconstruct.py'),str(z),'--direction','1','--reference-angle','0','--mask-root',str(session/'sam2'),'--output',str(out)],check=True)
subprocess.run([sys.executable,str(r/'find_suction_surface.py'),'--folder',str(out)],check=True,stdout=subprocess.DEVNULL)
d=json.loads((out/'suction_candidates.json').read_text());valid=[]
for camera,metrics in d['results'].items():
 if not metrics.get('geometry_pass'):continue
 other='cam2' if camera=='cam1' else 'cam1';op=out/(other+'_top_surface.npz');om=d['results'].get(other,{})
 if not op.exists() or 'normal_table' not in om:continue
 point=np.array(metrics['candidate_point_reference_table_m']);near=float(cKDTree(np.load(op)['points']).query(point)[0]);plane=abs(float(point@om['normal_table']+om['plane_d']));angle=float(np.degrees(np.arccos(np.clip(np.dot(metrics['normal_table'],om['normal_table']),-1,1))))
 if near<=.003 and plane<=.003 and angle<=5:valid.append((metrics['boundary_clearance_mm'],camera,{'same_point_other_camera_distance_mm':near*1000,'other_plane_distance_mm':plane*1000,'normal_difference_deg':angle}))
if not valid:raise RuntimeError('No reference candidate confirmed on both cameras')
_,camera,verification=max(valid);metrics=d['results'][camera];point=metrics['candidate_point_reference_table_m'];offset=float(json.loads((r/'contact_settings.json').read_text())['contact_z_offset_m']);target=point[:];target[2]+=offset
candidate={'camera':camera,'position_table_m':point,'nominal_contact_position_table_m':point,'contact_target_table_m':target,'contact_z_offset_m':offset,'approach_position_table_m':(np.array(point)+np.array([0,0,.03])).tolist(),'approved_for_automatic_execution':False}
d.update(selected_camera=camera,selected_candidate=candidate,reference='observed0deg view; current-coordinate suction location',same_point_cross_camera_verification=verification,full_shape_folder=str(session/'reconstruction_sam2'));(out/'suction_candidates.json').write_text(json.dumps(d,indent=2)+'\n');status={'session':str(session),'candidate_file':str(out/'suction_candidates.json'),'full_shape_folder':str(session/'reconstruction_sam2'),'geometry_pass':True,'same_point_cross_camera_verification':verification,'robot_execution_performed':False};(session/'reconstruction_sam2/processing_status.json').write_text(json.dumps(status,indent=2)+'\n');print(json.dumps(status,indent=2))
