from pathlib import Path
import json, time, math, itertools, collections
import numpy as np
import fcl
from scipy.stats import qmc
from check_collisions import Auditor
OUT=Path(__file__).resolve().parent

# Review masks for the four mating servo/arm interfaces only, not whole adjacent links.
# Distances are relative to the hinge axis in meters. These are conditional proxy-contact masks,
# not manufacturer-certified collision allowances and not written to SRDF/active URDF.
ZONES=[
 {'joint':'joint1','servo':'base_link_servo','part':'link1_print','radius':.013,'axial':[-.0366,.0002]},
 {'joint':'joint2','servo':'link1_servo','part':'link2_print','radius':.014,'axial':[-.0184,.0320]},
 {'joint':'joint3','servo':'link2_servo','part':'link3_print','radius':.012,'axial':[-.0184,.0185]},
 {'joint':'joint4','servo':'link3_servo','part':'tool_link_print','radius':.021,'axial':[.01821,.01834],'shaft_radius':.014,'shaft_axial':[.0155,.0211]},
]

class ReviewAuditor(Auditor):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs);super().set_pose([0.]*4);self.zone_by_pair={};self.filtered_objects={};self.zone_geometry={}
  for zone in ZONES:
   z=dict(zone);i=next(i for i,m in enumerate(self.meshes) if m['name']==z['servo']);j=next(i for i,m in enumerate(self.meshes) if m['name']==z['part']);joint=next(jj for jj in self.joints if jj['name']==z['joint']);F=self.last_frames[self.meshes[i]['link']];J=self.last_frames[joint['child']];point=F[:3,:3].T@(J[:3,3]-F[:3,3]);axis=F[:3,:3].T@(J[:3,:3]@joint['axis']);z.update(i=i,j=j,point_local=point,axis_local=axis)
   m=self.meshes[i]['mesh'];verts=np.asarray(m.vertices);v=verts-point;s=v@axis;rad=np.linalg.norm(v-s[:,None]*axis,axis=1);inside=(rad<=z['radius'])&(s>=z['axial'][0])&(s<=z['axial'][1]);faces=np.asarray(m.faces,dtype=np.int32);keep=~np.all(inside[faces],axis=1);faces=faces[keep];geom=fcl.BVHModel();geom.beginModel(len(verts),len(faces));geom.addSubModel(np.asarray(verts,dtype=np.float64),faces);geom.endModel();self.zone_geometry[i]=geom;self.filtered_objects[i]=fcl.CollisionObject(geom);self.zone_by_pair[(min(i,j),max(i,j))]=z
 def set_pose(self,q):
  super().set_pose(q)
  for i,obj in self.filtered_objects.items():
   F=self.last_frames[self.meshes[i]['link']];obj.setTransform(fcl.Transform(np.ascontiguousarray(F[:3,:3]),np.ascontiguousarray(F[:3,3])))
 def outside_zone(self,zone,point):
  F=self.last_frames[self.meshes[zone['i']]['link']];v=F[:3,:3].T@(np.asarray(point)-F[:3,3])-zone['point_local'];s=v@zone['axis_local'];r=np.linalg.norm(v-s*zone['axis_local']);inside=r<=zone['radius']+1e-7 and zone['axial'][0]-1e-7<=s<=zone['axial'][1]+1e-7;shaft=('shaft_radius' in zone and r<=zone['shaft_radius']+1e-7 and zone['shaft_axial'][0]-1e-7<=s<=zone['shaft_axial'][1]+1e-7);return not (inside or shaft)
 def reviewed_contacts(self,pairs,save_details=False):
  result=[];coupling=[];unknown=[]
  for i,j in pairs:
   zone=self.zone_by_pair.get((i,j))
   if zone is None:
    result.append((i,j));continue
   a=self.filtered_objects[zone['i']];b=self.objects[zone['j']];r=fcl.CollisionResult();fcl.collide(a,b,self.boolean_request,r)
   if not r.is_collision:coupling.append((i,j));continue
   r=fcl.CollisionResult();req=fcl.CollisionRequest(num_max_contacts=16000,enable_contact=True);fcl.collide(a,b,req,r)
   outside=[c for c in r.contacts if self.outside_zone(zone,c.pos)]
   if outside:result.append((i,j))
   elif len(r.contacts)>=16000:unknown.append((i,j))
   else:coupling.append((i,j))
  return result,coupling,unknown
 def audit(self,q):
  self.set_pose(q);raw=self.collision_pairs();body,coupling,unknown=self.reviewed_contacts(raw);floor=[]
  for i,m in enumerate(self.meshes):
   if m['rigid_group']=='fixed_base':continue
   if self.last_bounds[i,0,2]>0:continue
   F=self.last_frames[m['link']];v=np.asarray(m['mesh'].vertices)@F[:3,:3].T+F[:3,3];z=float(v[:,2].min())
   if z< -1e-7:floor.append((i,z))
  return {'raw':raw,'outside_coupling_zones':body,'coupling_zone_only':coupling,'truncated_unknown':unknown,'below_table_plane':floor}
 def describe(self,q,tag):
  result=self.audit(q);self.set_pose(q);items=[]
  for i,j in result['raw']:
   d=self.details(i,j,max_contacts=16000);d['classification']='outside_joint_coupling_review_zone' if (i,j) in result['outside_coupling_zones'] else ('indeterminate_contact_cap' if (i,j) in result['truncated_unknown'] else 'joint_coupling_zone_only')
   zone=self.zone_by_pair.get((i,j))
   if zone is not None:d['outside_zone_contact_points_m']=[c for c in d.get('contact_points_m',[]) if self.outside_zone(zone,c)]
   items.append(d)
  return {'tag':tag,'angles_rad':np.asarray(q).tolist(),'angles_deg':np.degrees(q).tolist(),'within_canonical_limits':dict(zip(self.names,map(bool,self.inside_limits(q)))),'contacts':items,'table_plane_penetration_m':{self.meshes[i]['name']:-z for i,z in result['below_table_plane']}}

def angles_grid(lo,hi,step):
 return np.r_[np.arange(lo,hi,step),hi]

if __name__=='__main__':
 a=ReviewAuditor();start=time.monotonic();home=np.array(a.model['poses']['home']);reference=np.clip(home,a.limits[:,0],a.limits[:,1]);print('Loaded meshes. Canonical-clamped home degrees',np.degrees(reference),flush=True)
 np.savez(OUT/'scan_geometry_metadata.npz',limits=a.limits,reference=reference,home=home)
 (OUT/'joint_coupling_review_zones.json').write_text(json.dumps({'status':'CONDITIONAL REVIEW MASKS, NOT APPROVED HARDWARE CLEARANCES','zones':ZONES},indent=2))
 poses={name:a.describe(q,name) for name,q in [('home',home),('canonical_clamped_home',reference),('zero',a.model['poses']['zero']),('rest',a.model['poses']['rest'])]};(OUT/'reference_poses_reviewed.json').write_text(json.dumps(poses,indent=2));print('Reference pose review written',flush=True)
 records=[];stats=collections.defaultdict(lambda:collections.Counter());counts=collections.Counter();examples={};seen={};sequence=[]
 def sample(q,kind,label=None):
  q=np.asarray(q,dtype=float);assert a.inside_limits(q).all(),(q,a.limits)
  r=a.audit(q);bad=r['outside_coupling_zones'];floor=r['below_table_plane'];unknown=r['truncated_unknown'];status=int(bool(bad))+2*int(bool(floor))+4*int(bool(unknown));counts[kind+'_samples']+=1
  if bad:counts[kind+'_body_contact_samples']+=1
  if floor:counts[kind+'_below_table_samples']+=1
  if unknown:counts[kind+'_unknown_samples']+=1
  if not (bad or floor or unknown):counts[kind+'_no_outside_zone_contact_samples']+=1
  names=[]
  for i,j in bad:
   key=selfkey=a.meshes[i]['name']+' | '+a.meshes[j]['name'];stats[key][kind]+=1;names.append(key)
   if key not in examples:examples[key]={'angles_rad':q.tolist(),'source':kind,'label':label}
  for i,z in floor:
   key=a.meshes[i]['name']+' | table_plane';stats[key][kind]+=1;names.append(key)
   if key not in examples:examples[key]={'angles_rad':q.tolist(),'source':kind,'label':label}
  records.append({'kind':kind,'angles_rad':q.tolist(),'outside_zone_pairs':names,'conditional_clear':not(bad or floor or unknown),'unknown':bool(unknown)})
  return status
 # Single-joint sweeps at the canonical-clamped home: 1deg spacing, including exact endpoints.
 for j in range(4):
  sequence=[]
  for qj in angles_grid(a.limits[j,0],a.limits[j,1],math.pi/180):
   q=reference.copy();q[j]=qj;sample(q,'single_'+a.names[j]);sequence.append(records[-1])
  (OUT/('sweep_'+a.names[j]+'.json')).write_text(json.dumps(sequence,indent=2));print('Sweep',a.names[j],'done in',round(time.monotonic()-start,1),'s',flush=True)
 # Independent J2/J3 grid at J1=0,J4=0; this is a slice, not a full 4D proof.
 j2=angles_grid(a.limits[1,0],a.limits[1,1],2*math.pi/180);j3=angles_grid(a.limits[2,0],a.limits[2,1],2*math.pi/180);grid=np.zeros((len(j3),len(j2)),dtype=np.uint8)
 for row,q3 in enumerate(j3):
  for col,q2 in enumerate(j2):grid[row,col]=sample([0.,q2,q3,0.],'J2_J3_grid')
  if row%20==0:print('J2/J3 grid',row+1,'/',len(j3),'elapsed',round(time.monotonic()-start,1),'s',flush=True)
 np.savez(OUT/'J2_J3_grid.npz',q2=j2,q3=j3,status=grid)
 # Low-discrepancy 4D sweep and all16 limit corners.
 for n,u in enumerate(qmc.Sobol(d=4,scramble=True,seed=9473).random_base2(m=12)):
  sample(a.limits[:,0]+u*(a.limits[:,1]-a.limits[:,0]),'sobol_4d')
  if n%1024==0:print('Sobol',n,'/4096 elapsed',round(time.monotonic()-start,1),'s',flush=True)
 for bits in itertools.product([0,1],repeat=4):sample(np.array([a.limits[i,b] for i,b in enumerate(bits)]),'limit_corners')
 # One-dimensional home->clamped-home path explicitly starts outside canonical J3 bounds.
 route=[]
 for u in np.linspace(0,1,12):
  q=home*(1-u)+reference*u;r=a.audit(q);route.append({'angles_rad':q.tolist(),'within_limits':dict(zip(a.names,map(bool,a.inside_limits(q)))),'outside_zone_pairs':[[a.meshes[i]['name'],a.meshes[j]['name']] for i,j in r['outside_coupling_zones']]})
 (OUT/'home_to_limit_interior_path.json').write_text(json.dumps(route,indent=2))
 (OUT/'scan_records.json').write_text(json.dumps(records,indent=2));(OUT/'collision_pair_counts.json').write_text(json.dumps(stats,indent=2));(OUT/'scan_counts.json').write_text(json.dumps(counts,indent=2));(OUT/'first_collision_examples.json').write_text(json.dumps(examples,indent=2))
 # Keep detailed contact data for10 distinct mesh pairs and some conditional-clear examples.
 selected={}
 for n,(key,e) in enumerate(examples.items()):
  if n>=12:break
  selected[key]=a.describe(e['angles_rad'],key)
 clear=[r for r in records if r['conditional_clear']]
 if clear:selected['sample_without_outside_zone_contacts']=a.describe(clear[0]['angles_rad'],'sample_without_outside_zone_contacts')
 (OUT/'selected_pose_contacts.json').write_text(json.dumps(selected,indent=2))
 print('DONE',round(time.monotonic()-start,2),'seconds',json.dumps(counts),flush=True);print('Pairs:',json.dumps(stats),flush=True)
