"""Offline candidate rotations for measured stationary faces. No actuator access."""
import json,sys,numpy as np,yaml
from pathlib import Path
from inclined_geometry import alignment_angles,rotate_surface,compensated_axis,solve_axis
from motion.kinematics import fk,m,W
s=Path(sys.argv[1]);color=sys.argv[2];report=json.loads((s/'flat_face_report.json').read_text());identities=json.loads((s/'stationary_reference/color_sort_plan.json').read_text())['objects'];wanted={r['object'] for r in identities if r['color']==color};wc=yaml.safe_load((s/'workcell_snapshot.yaml').read_text());state=json.loads((s/'table_state.json').read_text());j1=(W@np.r_[[-.0177,.0177,.1598],1])[:3];lo=np.array([j['limit']['lower'] for j in m['joints']],float);hi=np.array([j['limit']['upper'] for j in m['joints']],float);options=[]
for obj in report['objects']:
 if obj['object'] not in wanted:continue
 for face in obj['faces']:
  r=face['patch']
  if not r['geometry_pass']:continue
  for angle in alignment_angles(r['point_table_m'],r['normal_table'],wc['turntable']['axis_point_table_m'],j1):
   delta=(angle+180)%360-180;target=state['angle_deg']+delta;count=round(state['zero_count']+target*4096/360)
   if not 0<=count<=4095:continue
   p,n=rotate_surface(r['point_table_m'],r['normal_table'],wc['turntable']['axis_point_table_m'],delta);tool=compensated_axis(p,n,j1);q=solve_axis(fk,lo,hi,p,tool)
   if q is None:continue
   qa=solve_axis(fk,lo,hi,p+n*.02,tool,seed=q[0]);lift=solve_axis(fk,lo,hi,p+[0,0,.01],tool,seed=q[0])
   if qa and lift:options.append({'relative_turn_deg':delta,'target_angle_deg':target,'object':obj['object'],'label':obj['label'],'source':face['view']+'/'+face['camera'],'point':p.tolist(),'tilt_deg':r['tilt_deg']})
options.sort(key=lambda r:abs(r['relative_turn_deg']));(s/(color+'_alignment_options.json')).write_text(json.dumps(options,ensure_ascii=False,indent=2));print(json.dumps(options,ensure_ascii=False,indent=2))
