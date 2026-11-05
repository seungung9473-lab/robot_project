"""Offline reconstruction with current workcell calibration and positive 30-degree scans."""
import argparse,json,subprocess,sys
from pathlib import Path
import cv2,numpy as np,yaml
r=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('session',type=Path);a=p.parse_args();s=a.session.resolve()
m=json.loads((s/'session.json').read_text())
assert m['status']=='complete' and len(m['views'])==12
assert m['return_to_zero']['status']=='complete'
assert abs(m['return_to_zero']['state']['angle_deg'])<1
angles=[v['angle_deg'] for v in m['views']]
assert abs(angles[0])<1 and all(abs(b-a-30)<1 for a,b in zip(angles,angles[1:]))
cfg=Path.home()/'robot_ws/src/soarm4_workcell/config/workcell.yaml';wc=yaml.safe_load(cfg.read_text())
(s/'workcell_snapshot.yaml').write_text(cfg.read_text())
t=wc['turntable'];ctx={'axis_point_table_m':t['axis_point_table_m'],'axis_direction_table':t['axis_direction_table'],'support_radius_m':t['diameter_m']/2,'observation_radius_m':t['diameter_m']/2+.03,'top_height_m':t['top_height_m'],'calibrations':{c:{f'T_table_{c}_color_optical':wc['cameras'][c]['T_table_optical'],'source':str(cfg)} for c in ('cam1','cam2')},'object_position_assumed_at_axis':False,'source':'current workcell.yaml frozen for this scan'}
assert np.allclose(ctx['axis_direction_table'],[0,0,1])
(s/'spatial_context.json').write_text(json.dumps(ctx,indent=2))
def run(*cmd):subprocess.run(list(map(str,cmd)),check=True)
run(sys.executable,r/'prepare_sam2_prompts.py','--session',s)
run(r/'.sam2_cpu/bin/python',r/'apply_sam2.py','--session',s)
an=s/'analysis';an.mkdir(exist_ok=True);masks={}
for v in m['views']:
 for c in ('cam1','cam2'):
  masks[v['path']+'_'+c]=cv2.imread(str(s/'sam2'/v['path']/(c+'_mask.png')),0)>127
np.savez_compressed(an/'sam2.npz',**masks)
run(sys.executable,Path.home()/'robot_ws/src/soarm4_workcell/tools/recon.py',s,an/'sam2.npz',an/'cloud.npz')
out=s/'reconstruction_sam2'
run(sys.executable,r/'reconstruct.py',s,'--direction','1','--reference-angle','0','--mask-root',s/'sam2','--output',out)
run(sys.executable,r/'find_suction_surface.py','--folder',out)
run(sys.executable,r/'refine_reference_view.py',s)
print('PROCESSING COMPLETE',s,flush=True)
