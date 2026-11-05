"""Reserve same-pad positions using actual 4-DOF placement IK, before first pick."""
import json,sys
from pathlib import Path
import numpy as np,yaml
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull
from motion.kinematics import fk,m,W
from payload_placement import place_payload,signed_pitch
from pad_packing import footprint,separated
s=Path(sys.argv[1]);report=json.loads((s/'inclined_pick_plan.json').read_text());wc=yaml.safe_load((s/'workcell_snapshot.yaml').read_text());routing=json.loads((s/'color_sort_plan.json').read_text())['objects'];pad_id=int(sys.argv[2]);pad=wc['pads']['pad2' if pad_id==5 else 'pad1'];center=np.array(pad['center_table_m']);correction=np.array(wc['place_correction_m']['pad_ID'+str(pad_id)]);lo=np.array([j['limit']['lower'] for j in m['joints']],float);hi=np.array([j['limit']['upper'] for j in m['joints']],float);j1=(W@np.r_[[-.0177,.0177,.1598],1])[:3];options={}
for identity in routing:
 if identity['destination_pad_id']!=pad_id:continue
 key=identity['object'];entry=next(o for o in report['objects'] if o['object']==key);choices=[(f,o) for f in entry['matched_faces'] for o in f['alignment_options']]
 if not choices:raise RuntimeError('No pickup candidate for '+key)
 candidates=[]
 for choice_index,(face,choice) in enumerate(choices):
  from scipy.spatial.transform import Rotation
  P=np.load(s/'objects'/key/'analysis/cloud.npz')['points'];axis=np.array(wc['turntable']['axis_point_table_m']);P=(P-axis)@Rotation.from_euler('z',choice['turntable_angle_deg'],degrees=True).as_matrix().T+axis
  low=np.percentile(P,.5,0);high=np.percentile(P,99.5,0);low[2]=wc['turntable']['top_height_m'];low[:2]-=.002;high+=.002;dims=high-low;O=np.eye(4);O[:3,3]=(low+high)/2;qc=np.array(choice['contact_q_rad']);L=np.linalg.inv(fk(qc))@O;pitch=signed_pitch(fk(qc),j1);target=center.copy();target[:2]+=correction;q0=place_payload(fk,lo,hi,target,qc,L,dims,j1,pitch)
  if q0 is None:continue
  # Use hull vertices only: rigid transforms preserve the footprint envelope.
  local=(P-O[:3,3])[ConvexHull(P).vertices]
  for x in np.arange(-.04,.0401,.002):
   for y in np.arange(-.04,.0401,.002):
    dest=target+[x,y,0]
    def residual(q):
     T=fk(q);obj=T@L;bottom=obj[:3,3].copy();bottom[2]-=np.sum(abs(obj[2,:3])*dims)/2
     return np.r_[(bottom-dest)*10,(signed_pitch(T,j1)-pitch)*.3]
    fit=least_squares(residual,q0,bounds=(lo,hi),max_nfev=80)
    if np.linalg.norm(residual(fit.x)[:3])>.002 or abs(residual(fit.x)[3])>.001:continue
    placed=fk(fit.x)@L
    if np.degrees(np.arccos(np.clip(placed[2,2],-1,1)))>1:continue
    poly=footprint(local@placed[:3,:3].T+placed[:3,3])
    if np.max(np.linalg.norm(poly-center[:2],axis=1))>pad['diameter_m']/2-.002:continue
    candidates.append({'offset_xy_m':[float(x),float(y)],'reserved_polygon_table_m':poly.tolist(),'placement_q_rad':fit.x.tolist(),'pickup_option_index':choice_index})
 candidates.sort(key=lambda a:np.linalg.norm(a['offset_xy_m']));options[key]=candidates;print(key,'feasible locations',len(candidates),flush=True)
chosen={};order=sorted(options,key=lambda k:len(options[k]))
def search(i):
 if i==len(order):return True
 k=order[i]
 for item in options[k]:
  if all(separated(np.array(item['reserved_polygon_table_m']),np.array(v['reserved_polygon_table_m']),.002) for v in chosen.values()):
   chosen[k]=item
   if search(i+1):return True
   del chosen[k]
 return False
if not search(0):raise RuntimeError('No jointly feasible placement found at 2mm grid; no first placement committed')
plan={'version':1,'pad_id':pad_id,'stacking':False,'pad_center_table_m':center.tolist(),'radius_m':pad['diameter_m']/2,'edge_margin_m':.002,'object_gap_m':.002,'placements':chosen,'completed':{},'source_scan':str(s),'scope':'actual placement IK footprints; collision path still required'};p=s/('pad_'+str(pad_id)+'_placement_plan.json');p.write_text(json.dumps(plan,indent=2));print(p)
