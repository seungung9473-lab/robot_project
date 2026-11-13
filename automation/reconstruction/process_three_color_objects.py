"""Gemini identities + measured RGB-D/encoder association for multiple objects. No actuation."""
import json,sys,subprocess,shutil,os
from pathlib import Path
import cv2,numpy as np,yaml
from scipy.cluster.vq import kmeans2
from placement_geometry import depth_table,object_region
R=Path(__file__).resolve().parent;s=Path(sys.argv[1]).resolve();wc=yaml.safe_load((Path.home()/'robot_ws/src/TUMST_ARM_workcell/config/workcell.yaml').read_text());m=json.loads((s/'session.json').read_text());det=json.loads(Path(sys.argv[2]).read_text());policy_path=Path(sys.argv[sys.argv.index('--policy')+1]) if '--policy' in sys.argv else R/'color_sort_policy.json';policy=json.loads(policy_path.read_text());N=len(det['objects']);assert 1<=N<=8,'Expected1..8 separate detected objects';assert m['status']=='complete' and len(m['views'])==12
(s/'workcell_snapshot.yaml').write_text(yaml.safe_dump(wc))
t=wc['turntable'];ctx={'axis_point_table_m':t['axis_point_table_m'],'axis_direction_table':t['axis_direction_table'],'support_radius_m':t['diameter_m']/2,'observation_radius_m':t['diameter_m']/2+.03,'top_height_m':t['top_height_m'],'calibrations':{c:{f'T_table_{c}_color_optical':wc['cameras'][c]['T_table_optical']} for c in ('cam1','cam2')},'object_position_assumed_at_axis':False}
(s/'gemini_detection.json').write_text(json.dumps(det,ensure_ascii=False,indent=2));(s/'spatial_context.json').write_text(json.dumps(ctx,indent=2));axis=np.array(t['axis_point_table_m'])[:2]
def observe(v,c):
 folder=s/v['path'];md=json.loads((folder/'metadata.json').read_text());d=md['cameras'][c];raw=cv2.imread(str(folder/d['depth_file']),-1);im=cv2.imread(str(folder/d['color_file']));P,valid=depth_table(raw,d['depth_scale_m'],d['camera_info'],wc['cameras'][c]['T_table_optical']);hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);fg=valid&object_region(P,ctx)&(np.linalg.norm(P[:,:,:2]-axis,axis=2)<ctx['support_radius_m'])&((hsv[:,:,1]>55)|((hsv[:,:,1]<45)&(hsv[:,:,2]>70)))
 a=np.deg2rad(-md['turntable']['angle_deg']);rot=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]);ref=(P[:,:,:2]-axis)@rot.T+axis
 return folder,d,P,fg,ref
initial=[];initial_observations={}
for c in ['cam1','cam2']:
 initial_observations[c]=observe(m['views'][0],c);_,_,P,fg,ref=initial_observations[c];initial.append(ref[fg])
# Resolve Gemini pixel identities to measured 3D seeds in the same reference frame.
meta=json.loads(Path(sys.argv[2]).with_name('input_metadata.json').read_text());source=Path(meta['source_image_local_only']).parent;md=json.loads((source/'metadata.json').read_text());cam='cam1';dd=md['cameras'][cam];raw=cv2.imread(str(source/dd['depth_file']),-1)
from inclined_runtime import depth_scale
P,valid=depth_table(raw,depth_scale(dd,raw),dd['camera_info'],wc['cameras'][cam]['T_table_optical']);seeds=[]
for obj in det['objects']:
 x,y=obj['point_full_image_xy'];window=P[max(0,y-2):y+3,max(0,x-2):x+3];ok=valid[max(0,y-2):y+3,max(0,x-2):x+3];points=window[ok]
 if len(points)<5:raise RuntimeError('No measured depth at Gemini identity seed')
 seeds.append(np.median(points[:,:2],0))
X=np.vstack(initial);sample=X[::3];centers,cluster_labels=kmeans2(sample,np.array(seeds),minit='matrix',iter=30)
print('Measured seeds and clustered centres',np.array(seeds).tolist(),centers.tolist(),flush=True)
for i in range(N):
 # A visible face point is not the 3D object's centre (especially wedges).
 # Require measured membership in its cluster instead of a 25mm centre match.
 assigned=sample[cluster_labels==i]
 if not len(assigned) or np.min(np.linalg.norm(assigned-np.array(seeds[i]),axis=1))>.006 or np.linalg.norm(centers[i]-seeds[i])>.045:raise RuntimeError('Gemini depth seed is not supported by its spatial object cluster')
 for j in range(i):
  if np.linalg.norm(centers[i]-centers[j])<.035:raise RuntimeError('Objects too close for reliable spatial association')
print('Gemini associated centres',centers.tolist(),flush=True)
outs=[];prompts=[{} for _ in range(N)]
for i in range(N):
 o=s/'objects'/f'object_{i+1}';o.mkdir(parents=True,exist_ok=True);outs.append(o);(o/'sam2').mkdir(exist_ok=True)
 for filename in ['session.json','spatial_context.json','workcell_snapshot.yaml']:shutil.copy(s/filename,o/filename)
 for v in m['views']:
  link=o/v['path']
  if not link.exists():link.symlink_to(s/v['path'],target_is_directory=True)
for v in m['views']:
 for c in ['cam1','cam2']:
  folder,d,P,fg,ref=initial_observations[c] if v['path']==m['views'][0]['path'] else observe(v,c);dist=np.linalg.norm(ref[:,:,None,:]-centers,axis=3);labels=dist.argmin(axis=2);entries=[];seeds=[]
  for i in range(N):
   mask=(fg&(labels==i)&(dist[:,:,i]<.045)).astype('uint8');n,cc,stats,_=cv2.connectedComponentsWithStats(mask);ids=[j for j in range(1,n) if stats[j,4]>80];entry={'image':str(folder/d['color_file']),'prompt_source':'Gemini identity + measured depth foreground + encoder-unrotated spatial identity'}
   if not ids:entry['occluded']=True;seeds.append(None)
   else:
    j=max(ids,key=lambda j:stats[j,4]);mask=(cc==j).astype('uint8');dt=cv2.distanceTransform(mask,cv2.DIST_L2,3);y,x=np.unravel_index(dt.argmax(),dt.shape);xx,yy,w,h,_=stats[j];entry.update(positive_xy=[int(x),int(y)],box_xyxy=[max(0,int(xx)-4),max(0,int(yy)-4),min(P.shape[1]-1,int(xx+w)+4),min(P.shape[0]-1,int(yy+h)+4)]);seeds.append([int(x),int(y)])
   entries.append(entry)
  for i,entry in enumerate(entries):
   if not entry.get('occluded'):entry['negative_xy']=[seed for j,seed in enumerate(seeds) if j!=i and seed is not None]
   prompts[i][v['path']+'/'+c]=entry
initial_observations.clear()
for i,o in enumerate(outs):
 (o/'sam2/prompts.json').write_text(json.dumps(prompts[i],indent=2))
if '--reuse-masks' not in sys.argv:
 from desktop_runtime import gpu_python
 subprocess.run([gpu_python(),str(R/'apply_sam2.py'),'--device','cuda','--sessions',*map(str,outs)],check=True)
excluded=[]
for v in m['views']:
 for c in ['cam1','cam2']:
  paths=[o/'sam2'/v['path']/(c+'_mask.png') for o in outs];ms=[cv2.imread(str(p),0)>127 for p in paths];ambiguous=set();overlap=np.zeros(ms[0].shape,bool)
  for i in range(N):
   for j in range(i):
    common=ms[i]&ms[j];overlap|=common
    if common.sum()>max(30,min(ms[i].sum(),ms[j].sum())*.08):ambiguous.update([i,j])
  for i,(path,mask) in enumerate(zip(paths,ms)):
   if i in ambiguous:
    excluded.append({'view':v['path'],'camera':c,'object':i+1,'reason':'ambiguous overlap observation excluded'});shutil.copy2(path,path.with_name(path.stem+'_ambiguous_original.png'));mask[:]=False
   cv2.imwrite(str(path),((mask&~overlap)*255).astype('uint8'))
(s/'mask_validation.json').write_text(json.dumps({'excluded_observations':excluded,'overlap_limit_fraction':.08},indent=2))
for o in outs:
 for c in ['cam1','cam2']:
  visible=sum(np.count_nonzero(cv2.imread(str(o/'sam2'/v['path']/(c+'_mask.png')),0))>100 for v in m['views'])
  if visible<6:raise RuntimeError('Too few unambiguous observations for '+str(o)+' '+c)
results=[]
for i,o in enumerate(outs):
 an=o/'analysis';an.mkdir(exist_ok=True);masks={v['path']+'_'+c:cv2.imread(str(o/'sam2'/v['path']/(c+'_mask.png')),0)>127 for v in m['views'] for c in ['cam1','cam2']};np.savez_compressed(an/'sam2.npz',**masks)
 # Geometry per camera retains actual measured angles and colours.
 out=o/'reconstruction_sam2';subprocess.run([sys.executable,str(R/'reconstruct.py'),str(o),'--direction','1','--reference-angle','0','--mask-root',str(o/'sam2'),'--output',str(out),*(['--no-preview'] if '--identity-only' in sys.argv else [])],check=True)
 # Use a direct merged coloured cloud here; fitting a per-object rotation axis would be invalid.
 parts=[np.load(out/(c+'_object.npz')) for c in ['cam1','cam2']];np.savez_compressed(an/'cloud.npz',points=np.vstack([p['points'] for p in parts]),colors_rgb=np.vstack([p['colors'] for p in parts]))
 from identity_routing import identity_routing
 metadata=identity_routing(det['objects'][i],policy,identity_only='--identity-only' in sys.argv,color_parts=(p['colors'] for p in parts))
 results.append({'object':f'object_{i+1}','session':str(o),'reference_center_xy':centers[i].tolist(),**metadata})
(s/'color_sort_plan.json').write_text(json.dumps({'objects':results,'model_used':det['model_used'],'same_pad_policy':policy['occupied_destination'],'robot_execution_performed':False},ensure_ascii=False,indent=2));print('MULTI OBJECTS COMPLETE',json.dumps(results,ensure_ascii=False),flush=True)
