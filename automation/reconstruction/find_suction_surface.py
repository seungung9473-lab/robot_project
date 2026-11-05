"""Offline planar suction candidates from observed clouds; never sends robot commands."""
from pathlib import Path
import json,numpy as np
import argparse
from scipy.spatial import ConvexHull,cKDTree
r=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--folder',type=Path,default=r/'multiview/20261101_194509_024461/reconstruction_sam2');args=parser.parse_args();folder=args.folder.resolve();rng=np.random.default_rng(9473);results={};allplanes={}
for camera in ['cam2','cam1']:
 data=np.load(folder/(camera+'_object.npz'));p=data['points'];remaining=np.arange(len(p));planes=[]
 for step in range(6):
  q=p[remaining]
  if len(q)<150:break
  best=None
  for trial in range(600):
   v=q[rng.choice(len(q),3,replace=False)];normal=np.cross(v[1]-v[0],v[2]-v[0]);length=np.linalg.norm(normal)
   if length<1e-8:continue
   normal/=length;d=-normal@v[0];mask=np.abs(q@normal+d)<.0015
   if best is None or mask.sum()>best.sum():best=mask
  if best is None or best.sum()<120:break
  pts=q[best];center=pts.mean(axis=0);_,_,V=np.linalg.svd(pts-center,full_matrices=False);normal=V[-1]
  # The above-support top is the actionable first candidate. Side planes retained for review.
  if normal[2]<0:normal=-normal
  d=-normal@center;mask=np.abs(q@normal+d)<.0015;pts=q[mask];res=pts@normal+d;idx=remaining[mask];remaining=remaining[~mask]
  item={'normal_table':normal.tolist(),'plane_d':float(d),'rms_mm':float(np.sqrt(np.mean(res**2))*1000),'p95_abs_mm':float(np.percentile(np.abs(res),95)*1000),'inliers':len(pts),'tilt_from_vertical_deg':float(np.degrees(np.arccos(np.clip(normal[2],-1,1)))),'center_table_m':pts.mean(axis=0).tolist()};planes.append((item,pts))
 allplanes[camera]=[a for a,b in planes]
 tops=[(item,pts) for item,pts in planes if item['tilt_from_vertical_deg']<15 and item['center_table_m'][2]>.09]
 if not tops:results[camera]={'status':'no top plane'};continue
 item,pts=max(tops,key=lambda x:x[0]['inliers']);n=np.array(item['normal_table']);center=pts.mean(axis=0);u=np.cross([0,1,0],n);u/=np.linalg.norm(u);v=np.cross(n,u);xy=np.column_stack([(pts-center)@u,(pts-center)@v]);hull=ConvexHull(xy);eq=hull.equations;lo=xy.min(axis=0);hi=xy.max(axis=0);xx,yy=np.meshgrid(np.arange(lo[0],hi[0],.001),np.arange(lo[1],hi[1],.001));grid=np.column_stack([xx.ravel(),yy.ravel()]);clear=-(grid@eq[:,:2].T+eq[:,2]).max(axis=1);bestidx=np.argmax(clear);g=grid[bestidx];point=center+u*g[0]+v*g[1];cup_radius=.0125;edge_margin=.003
 angles=np.linspace(0,2*np.pi,48,endpoint=False);radial=np.linspace(0,cup_radius,8);disk=np.array([g+rr*np.array([np.cos(a),np.sin(a)]) for rr in radial for a in angles]);dist,_=cKDTree(xy).query(disk);coverage=float(np.mean(dist<.003));raw_xy=np.column_stack([(p-center)@u,(p-center)@v]);raw_res=p@n+item['plane_d'];patch=p[(np.linalg.norm(raw_xy-g,axis=1)<=cup_radius)&(np.abs(raw_res)<.008)];patcherr=patch@n+item['plane_d'];metrics={**item,'candidate_point_reference_table_m':point.tolist(),'outward_normal_table':n.tolist(),'cup_approach_axis_table':(-n).tolist(),'boundary_clearance_mm':float(clear[bestidx]*1000),'required_radius_plus_margin_mm':(cup_radius+edge_margin)*1000,'disk_depth_coverage_within_3mm':coverage,'patch_points':len(patch),'patch_rms_mm':float(np.sqrt(np.mean(patcherr**2))*1000),'patch_p95_abs_mm':float(np.percentile(np.abs(patcherr),95)*1000),'patch_max_abs_mm':float(np.max(np.abs(patcherr))*1000),'observed_footprint_extent_mm':((hi-lo)*1000).tolist(),'geometry_pass':bool(clear[bestidx]>=cup_radius+edge_margin and coverage>.9 and np.sqrt(np.mean(patcherr**2))*1000<1.5),'caveat':'Convex hull boundary and sampled depth coverage are proxies; printing grooves, porosity, compliance and registration uncertainty not certified.'};results[camera]=metrics;np.savez_compressed(folder/(camera+'_top_surface.npz'),points=pts,boundary=pts[hull.vertices],candidate=point,normal=n)
 allplanes[camera]=[a for a,b in planes]
settings=json.loads((r/'contact_settings.json').read_text()) if (r/'contact_settings.json').exists() else {'contact_z_offset_m':0.0}
eligible=[c for c in ['cam2','cam1'] if results.get(c,{}).get('geometry_pass',False)]
selected_camera=eligible[0] if eligible else 'cam2'
selected=results.get(selected_camera,{})
if 'candidate_point_reference_table_m' in selected:
 nominal=selected['candidate_point_reference_table_m'][:];contact=nominal[:];contact[2]+=float(settings['contact_z_offset_m'])
 selected.update(nominal_contact_position_table_m=nominal,contact_target_table_m=contact,contact_z_offset_m=float(settings['contact_z_offset_m']))
object_centers={c:((np.percentile(np.load(folder/(c+'_object.npz'))['points'],99,axis=0)+np.percentile(np.load(folder/(c+'_object.npz'))['points'],1,axis=0))/2).tolist() for c in ['cam1','cam2']}
report={'object_centers_table_m':object_centers,'object_at_rotation_axis_assumed':False,'suction_point_method':'maximum boundary clearance on observed object top plane; not turntable axis','status':'geometry candidate only; no actuation or plan execution','cup_diameter_mm':25,'reference':'first completed view of supplied reconstruction; check report.reference_angle_deg before execution','source':str(folder),'selected_camera':selected_camera,'contact_settings':settings,'results':results,'planes':allplanes,'selected_candidate':({'camera':selected_camera,'position_table_m':selected['candidate_point_reference_table_m'],'nominal_contact_position_table_m':selected['nominal_contact_position_table_m'],'contact_target_table_m':selected['contact_target_table_m'],'contact_z_offset_m':selected['contact_z_offset_m'],'approach_position_table_m':(np.array(selected['candidate_point_reference_table_m'])+np.array(selected['normal_table'])*.03).tolist(),'approved_for_automatic_execution':False} if 'candidate_point_reference_table_m' in selected else {})};(folder/'suction_candidates.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(results,indent=2))
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
fig=plt.figure(figsize=(11,5))
for i,c in enumerate(['cam1','cam2']):
 ax=fig.add_subplot(1,2,i+1,projection='3d');data=np.load(folder/(c+'_object.npz'));p=data['points'];ax.scatter(p[:,0]*100,p[:,1]*100,p[:,2]*100,c='silver',s=.5,alpha=.25);f=folder/(c+'_top_surface.npz')
 if f.exists():
  d=np.load(f);p=d['points'];q=d['candidate'];n=d['normal'];ax.scatter(p[:,0]*100,p[:,1]*100,p[:,2]*100,c='royalblue',s=2);ax.scatter(* (q*100),c='red',s=55);u=np.cross([0,1,0],n);u/=np.linalg.norm(u);v=np.cross(n,u);t=np.linspace(0,2*np.pi,100);ring=q+.0125*(np.cos(t)[:,None]*u+np.sin(t)[:,None]*v);ax.plot(ring[:,0]*100,ring[:,1]*100,ring[:,2]*100,c='red')
 ax.set_title(c+' top plane and 25 mm cup');ax.set_xlabel('table X (cm)');ax.set_ylabel('table Y (cm)');ax.set_zlabel('table Z (cm)');ax.set_box_aspect((1,1,1))
fig.tight_layout();fig.savefig(folder/'suction_candidates.png',dpi=160)
