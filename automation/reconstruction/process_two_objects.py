"""ER2-seeded, appearance-associated per-object SAM2 + measured-angle geometry."""
import argparse,json,subprocess,sys,shutil
from pathlib import Path
import cv2,numpy as np
from placement_geometry import load_context,depth_table,object_region
R=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('--detection',type=Path,required=True);p.add_argument('--reuse-masks',action='store_true');args=p.parse_args();session=args.session.resolve();det=json.loads(args.detection.read_text());meta=json.loads(args.detection.with_name('input_metadata.json').read_text());objects=det['objects']
if len(objects)!=2:raise RuntimeError('This sort requires two verified object detections')
ctx=load_context(session);manifest=json.loads((session/'session.json').read_text());src=cv2.imread(meta['source_image_local_only']);lab=cv2.cvtColor(src,cv2.COLOR_BGR2LAB);prototypes=[]
for obj in objects:
 x,y=obj['point_full_image_xy'];patch=lab[max(0,y-3):y+4,max(0,x-3):x+4,1:];prototypes.append(np.median(patch.reshape(-1,2),axis=0))
prototypes=np.array(prototypes)
if np.linalg.norm(prototypes[0]-prototypes[1])<20:raise RuntimeError('Appearance association ambiguous; cannot separate these two objects reliably by this method')
clones=[];prompts=[{},{}]
for obj in objects:
 out=session/'objects'/('object_'+str(obj['object_id']));out.mkdir(parents=True,exist_ok=True);(out/'session.json').write_text(json.dumps(manifest,indent=2));shutil.copy(session/'spatial_context.json',out/'spatial_context.json');(out/'sam2').mkdir(exist_ok=True)
 for view in manifest['views']:
  link=out/view['path']
  if not link.exists():link.symlink_to(session/view['path'],target_is_directory=True)
 clones.append(out)
for view in manifest['views']:
 folder=session/view['path'];m=json.loads((folder/'metadata.json').read_text())
 for camera in ['cam1','cam2']:
  d=m['cameras'][camera];image=cv2.imread(str(folder/d['color_file']));raw=cv2.imread(str(folder/d['depth_file']),-1);pts,valid=depth_table(raw,d['depth_scale_m'],d['camera_info'],ctx['calibrations'][camera][f'T_table_{camera}_color_optical']);foreground=valid&object_region(pts,ctx);ab=cv2.cvtColor(image,cv2.COLOR_BGR2LAB)[:,:,1:].astype(float);dist=np.linalg.norm(ab[:,:,None,:]-prototypes,axis=3);assignment=np.argmin(dist,axis=2);seeds=[]
  for index in range(2):
   seed=(foreground&(assignment==index)&(dist[:,:,index]<35)).astype('uint8');con=cv2.morphologyEx(seed,cv2.MORPH_CLOSE,np.ones((3,3),'uint8'));n,labels,stats,_=cv2.connectedComponentsWithStats(con);possible=[]
   for i in range(1,n):
    pix=(labels==i)&(seed>0)
    if stats[i,4]<70 or not pix.any():continue
    center=np.median(pts[pix],axis=0)
    if np.linalg.norm(center[:2]-np.array(ctx['axis_point_table_m'])[:2])<ctx['support_radius_m']:possible.append(i)
   entry={'image':str((folder/d['color_file']).resolve()),'prompt_source':'ER2 identity with measured LAB appearance association and depth support; SAM2 generates final mask'}
   if not possible:entry.update(occluded=True);seeds.append(None)
   else:
    i=max(possible,key=lambda j:stats[j,4]);pix=(labels==i)&(seed>0);distance=cv2.distanceTransform(pix.astype('uint8'),cv2.DIST_L2,3);y,x=np.unravel_index(np.argmax(distance),distance.shape);sx,sy,w,h,_=stats[i];entry.update(positive_xy=[int(x),int(y)],box_xyxy=[max(0,int(sx)-6),max(0,int(sy)-6),min(image.shape[1]-1,int(sx+w)+6),min(image.shape[0]-1,int(sy+h)+6)],seed_pixels=int(pix.sum()));seeds.append([int(x),int(y)])
   prompts[index][view['path']+'/'+camera]=entry
  for index in range(2):
   entry=prompts[index][view['path']+'/'+camera]
   if not entry.get('occluded'):entry['negative_xy']=[s for j,s in enumerate(seeds) if j!=index and s is not None]
for index,out in enumerate(clones):
 (out/'sam2/prompts.json').write_text(json.dumps(prompts[index],indent=2));
 if not args.reuse_masks:subprocess.run([str(R/'.sam2_cpu/bin/python'),str(R/'apply_sam2.py'),'--session',str(out)],check=True)
# Never let one visible pixel belong to both reconstructed objects.
removed=0
for view in manifest['views']:
 for camera in ['cam1','cam2']:
  paths=[out/'sam2'/view['path']/(camera+'_mask.png') for out in clones];masks=[cv2.imread(str(p),0)>127 for p in paths];overlap=masks[0]&masks[1];n=int(overlap.sum())
  if n>max(30,.08*min(int(m.sum()) for m in masks)):raise RuntimeError('SAM2 object masks overlap too much; identities not reliable')
  if n:
   removed+=n
   for path,mask in zip(paths,masks):shutil.copy(path,path.with_name(camera+'_mask_before_overlap.png'));cv2.imwrite(str(path),((mask&~overlap)*255).astype('uint8'))
records=[]
for obj,out in zip(objects,clones):
 subprocess.run([sys.executable,str(R/'process_scan.py'),str(out),'--reuse-masks'],check=True)
 processing=json.loads((out/'reconstruction_sam2/processing_status.json').read_text());candidate=json.loads(Path(processing['candidate_file']).read_text());camera=candidate['selected_camera'];points=np.load(out/'reconstruction_sam2'/(camera+'_object.npz'))['points'];lo=np.percentile(points,.5,axis=0);hi=np.percentile(points,99.5,axis=0);lo[2]=ctx['top_height_m'];extent=hi-lo
 records.append({'object_id':obj['object_id'],'label':obj['label'],'session':str(out),'candidate_file':processing['candidate_file'],'extent_m':extent.tolist(),'volume_proxy_m3':float(np.prod(extent)),'center_table_m':((hi+lo)/2).tolist()})
ranked=sorted(records,key=lambda x:x['volume_proxy_m3'],reverse=True)
if ranked[0]['volume_proxy_m3']/ranked[1]['volume_proxy_m3']<1.25:raise RuntimeError('Size ordering is uncertain')
ranked[0].update(size_rank='large',destination='left',pad_marker_id=5);ranked[1].update(size_rank='small',destination='right',pad_marker_id=6);report={'objects':ranked,'removed_overlap_pixels':removed,'size_measure':'observed bounding-volume proxy; not exact physical volume','association':'ER2 identity + distinct measured appearance; not general identity tracking','robot_execution_performed':False};(session/'sort_plan.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False,indent=2))
