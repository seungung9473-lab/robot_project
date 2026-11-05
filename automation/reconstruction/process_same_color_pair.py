"""Depth/encoder associated pair segmentation, no actuator commands."""
import json,sys,subprocess,shutil
from pathlib import Path
import cv2,numpy as np,yaml
from scipy.cluster.vq import kmeans2
from placement_geometry import depth_table,object_region
R=Path(__file__).resolve().parent;s=Path(sys.argv[1]).resolve();wc=yaml.safe_load((Path.home()/'robot_ws/src/soarm4_workcell/config/workcell.yaml').read_text());m=json.loads((s/'session.json').read_text());assert m['status']=='complete' and len(m['views'])==12
(s/'workcell_snapshot.yaml').write_text(yaml.safe_dump(wc))
t=wc['turntable'];ctx={'axis_point_table_m':t['axis_point_table_m'],'axis_direction_table':t['axis_direction_table'],'support_radius_m':t['diameter_m']/2,'observation_radius_m':t['diameter_m']/2+.03,'top_height_m':t['top_height_m'],'calibrations':{c:{f'T_table_{c}_color_optical':wc['cameras'][c]['T_table_optical']} for c in ('cam1','cam2')},'object_position_assumed_at_axis':False}
(s/'spatial_context.json').write_text(json.dumps(ctx,indent=2));axis=np.array(t['axis_point_table_m'])[:2]
def observe(v,c):
 folder=s/v['path'];md=json.loads((folder/'metadata.json').read_text());d=md['cameras'][c];raw=cv2.imread(str(folder/d['depth_file']),-1);im=cv2.imread(str(folder/d['color_file']));P,valid=depth_table(raw,d['depth_scale_m'],d['camera_info'],wc['cameras'][c]['T_table_optical']);hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);fg=valid&object_region(P,ctx)&(hsv[:,:,1]>55)&(hsv[:,:,0]>30)&(hsv[:,:,0]<95)
 a=np.deg2rad(-md['turntable']['angle_deg']);rot=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]);ref=(P[:,:,:2]-axis)@rot.T+axis
 return folder,d,P,fg,ref
initial=[]
for c in ['cam1','cam2']:
 _,_,P,fg,ref=observe(m['views'][0],c);initial.append(ref[fg])
X=np.vstack(initial);centers,_=kmeans2(X[::3],2,minit='++',seed=0);centers=centers[np.argsort(centers[:,0])];assert np.linalg.norm(centers[0]-centers[1])>.035,'Objects too close for stable spatial association';print('Reference centres',centers.tolist(),flush=True)
outs=[];prompts=[{},{}]
for i in range(2):
 o=s/'objects'/f'object_{i+1}';o.mkdir(parents=True,exist_ok=True);outs.append(o);(o/'sam2').mkdir(exist_ok=True)
 for filename in ['session.json','spatial_context.json','workcell_snapshot.yaml']:shutil.copy(s/filename,o/filename)
 for v in m['views']:
  link=o/v['path']
  if not link.exists():link.symlink_to(s/v['path'],target_is_directory=True)
for v in m['views']:
 for c in ['cam1','cam2']:
  folder,d,P,fg,ref=observe(v,c);dist=np.linalg.norm(ref[:,:,None,:]-centers,axis=3);labels=dist.argmin(axis=2);entries=[];seeds=[]
  for i in range(2):
   mask=(fg&(labels==i)&(dist[:,:,i]<.045)).astype('uint8');n,cc,stats,_=cv2.connectedComponentsWithStats(mask);ids=[j for j in range(1,n) if stats[j,4]>80];entry={'image':str(folder/d['color_file']),'prompt_source':'green depth foreground + encoder-unrotated spatial identity'}
   if not ids:entry['occluded']=True;seeds.append(None)
   else:
    j=max(ids,key=lambda j:stats[j,4]);mask=(cc==j).astype('uint8');dt=cv2.distanceTransform(mask,cv2.DIST_L2,3);y,x=np.unravel_index(dt.argmax(),dt.shape);xx,yy,w,h,_=stats[j];entry.update(positive_xy=[int(x),int(y)],box_xyxy=[max(0,int(xx)-4),max(0,int(yy)-4),min(639,int(xx+w)+4),min(479,int(yy+h)+4)]);seeds.append([int(x),int(y)])
   entries.append(entry)
  for i,entry in enumerate(entries):
   if not entry.get('occluded'):entry['negative_xy']=[seed for j,seed in enumerate(seeds) if j!=i and seed is not None]
   prompts[i][v['path']+'/'+c]=entry
for i,o in enumerate(outs):
 (o/'sam2/prompts.json').write_text(json.dumps(prompts[i],indent=2))
 if '--reuse-masks' not in sys.argv:subprocess.run([str(R/'.sam2_cpu/bin/python'),str(R/'apply_sam2.py'),'--session',str(o)],check=True)
excluded=[]
for v in m['views']:
 for c in ['cam1','cam2']:
  paths=[o/'sam2'/v['path']/(c+'_mask.png') for o in outs];ms=[cv2.imread(str(p),0)>127 for p in paths];overlap=ms[0]&ms[1]
  if overlap.sum()>max(30,min(x.sum() for x in ms)*.08):
   excluded.append({'view':v['path'],'camera':c,'overlap_pixels':int(overlap.sum()),'reason':'ambiguous identity: both object observations excluded'})
   for path in paths:
    backup=path.with_name(path.stem+'_ambiguous_original.png')
    if not backup.exists():shutil.copy2(path,backup)
    cv2.imwrite(str(path),np.zeros(ms[0].shape,'uint8'))
   continue
  for p,mask in zip(paths,ms):cv2.imwrite(str(p),((mask&~overlap)*255).astype('uint8'))
(s/'mask_validation.json').write_text(json.dumps({'excluded_observations':excluded,'overlap_limit_fraction':.08},indent=2))
for o in outs:
 for c in ['cam1','cam2']:
  visible=sum(np.count_nonzero(cv2.imread(str(o/'sam2'/v['path']/(c+'_mask.png')),0))>100 for v in m['views'])
  if visible<6:raise RuntimeError('Too few unambiguous observations for '+str(o)+' '+c)
results=[]
for i,o in enumerate(outs):
 an=o/'analysis';an.mkdir(exist_ok=True);masks={v['path']+'_'+c:cv2.imread(str(o/'sam2'/v['path']/(c+'_mask.png')),0)>127 for v in m['views'] for c in ['cam1','cam2']};np.savez_compressed(an/'sam2.npz',**masks)
 # Geometry per camera retains actual measured angles and colours.
 out=o/'reconstruction_sam2';subprocess.run([sys.executable,str(R/'reconstruct.py'),str(o),'--direction','1','--reference-angle','0','--mask-root',str(o/'sam2'),'--output',str(out)],check=True)
 # Use a direct merged coloured cloud here; fitting a per-object rotation axis would be invalid.
 parts=[np.load(out/(c+'_object.npz')) for c in ['cam1','cam2']];np.savez_compressed(an/'cloud.npz',points=np.vstack([p['points'] for p in parts]),colors_rgb=np.vstack([p['colors'] for p in parts]))
 subprocess.run([sys.executable,str(R/'find_suction_surface.py'),'--folder',str(out)],check=True)
 refined=subprocess.run([sys.executable,str(R/'refine_reference_view.py'),str(o)])
 result={'object':i+1,'session':str(o),'reference_center_xy':centers[i].tolist(),'cross_camera_candidate_pass':refined.returncode==0}
 if result['cross_camera_candidate_pass']:
  d=json.loads((o/'reconstruction_reference/suction_candidates.json').read_text());result['candidate']=d['selected_candidate'];result['metrics']=d['results'][d['selected_camera']]
 results.append(result)
(s/'pair_analysis.json').write_text(json.dumps(results,indent=2));print('PAIR COMPLETE',json.dumps(results,indent=2),flush=True)
