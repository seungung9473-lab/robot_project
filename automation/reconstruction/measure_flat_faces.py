"""Fresh stopped RGB-D/SAM2 planar face measurements. Never actuates hardware."""
import sys,json,subprocess,importlib.util
from pathlib import Path
import cv2,numpy as np,yaml
from scipy.spatial import ConvexHull
from placement_geometry import depth_table
from inclined_runtime import depth_scale
from inclined_geometry import planes,patch,PLANE_INLIER_TOLERANCE_M,PATCH_RMS_LIMIT_MM
R=Path(__file__).resolve().parent;out=Path(sys.argv[1]).resolve();reference=Path(sys.argv[2]).resolve();wc=yaml.safe_load((reference/'workcell_snapshot.yaml').read_text());identities=json.loads((reference/'color_sort_plan.json').read_text())['objects'];views=[{'path':p.name} for p in sorted(out.glob('view_*')) if (p/'metadata.json').exists()];assert len(views)>=2
centers=np.array([o['reference_center_xy'] for o in identities]);axis=np.array(wc['turntable']['axis_point_table_m']);objects=[]
(out/'session.json').write_text(json.dumps({'views':views,'scope':'stationary planar-area measurement only; no rotation'},indent=2));(out/'workcell_snapshot.yaml').write_text(yaml.safe_dump(wc));spec=importlib.util.spec_from_file_location('old_flat_geometry',R/'backups/inclined_geometry_before_planarity_2mm_20261104.py');old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
for i,identity in enumerate(identities):
 obj=out/'objects'/identity['object'];(obj/'sam2').mkdir(parents=True,exist_ok=True);prompts={}
 for view in views:
  folder=out/view['path'];md=json.loads((folder/'metadata.json').read_text())
  for cam in ['cam1','cam2']:
   d=md['cameras'][cam];raw=cv2.imread(str(folder/d['depth_file']),-1);im=cv2.imread(str(folder/d['color_file']));P,valid=depth_table(raw,depth_scale(d,raw),d['camera_info'],wc['cameras'][cam]['T_table_optical']);hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);dist=np.linalg.norm(P[:,:,None,:2]-centers,axis=3);labels=dist.argmin(2);fg=valid&(np.linalg.norm(P[:,:,:2]-axis[:2],axis=2)<.105)&(P[:,:,2]>axis[2]+.006)&(P[:,:,2]<axis[2]+.25)
   color=((hsv[:,:,1]>55)&(hsv[:,:,0]>30)&(hsv[:,:,0]<95)) if identity['color']=='green' else ((hsv[:,:,1]<45)&(hsv[:,:,2]>70))
   mask=(fg&color&(labels==i)&(dist[:,:,i]<.045)).astype('uint8');count,cc,stats,_=cv2.connectedComponentsWithStats(mask);ids=[j for j in range(1,count) if stats[j,4]>80];entry={'image':str(folder/d['color_file']),'prompt_source':'fresh measured RGBD identity seed; final mask from SAM2'}
   if not ids:entry['occluded']=True
   else:
    j=max(ids,key=lambda j:stats[j,4]);mask=(cc==j).astype('uint8');distance=cv2.distanceTransform(mask,cv2.DIST_L2,3);y,x=np.unravel_index(distance.argmax(),distance.shape);xx,yy,w,h,_=stats[j];negative=[]
    for k in range(len(identities)):
     if k==i:continue
     candidates=np.argwhere(fg&(labels==k)&(dist[:,:,k]<.035))
     if len(candidates):ny,nx=candidates[len(candidates)//2];negative.append([int(nx),int(ny)])
    entry.update(positive_xy=[int(x),int(y)],box_xyxy=[max(0,int(xx)-5),max(0,int(yy)-5),min(im.shape[1]-1,int(xx+w)+5),min(im.shape[0]-1,int(yy+h)+5)],negative_xy=negative)
   prompts[view['path']+'/'+cam]=entry
 (obj/'sam2/prompts.json').write_text(json.dumps(prompts,indent=2));
 if '--reuse-masks' not in sys.argv:subprocess.run([R/'.sam2_cpu/bin/python',R/'apply_sam2.py','--session',obj],check=True)
 objects.append(obj)
# Reject ambiguity instead of merging neighboring objects into a larger plane.
for view in views:
 for cam in ['cam1','cam2']:
  masks=[cv2.imread(str(obj/'sam2'/view['path']/(cam+'_mask.png')),0)>127 for obj in objects]
  for i in range(len(masks)):
   for j in range(i):
    if (masks[i]&masks[j]).sum()>max(30,.08*min(masks[i].sum(),masks[j].sum())):raise RuntimeError('Ambiguous fresh object masks; no area measurement')
results=[];overlay=cv2.imread(str(out/'view_000/cam1_color.png'))
for identity,obj in zip(identities,objects):
 records=[]
 for view in views:
  folder=out/view['path'];md=json.loads((folder/'metadata.json').read_text())
  for cam in ['cam1','cam2']:
   d=md['cameras'][cam];raw=cv2.imread(str(folder/d['depth_file']),-1);P,valid=depth_table(raw,depth_scale(d,raw),d['camera_info'],wc['cameras'][cam]['T_table_optical']);mask=cv2.imread(str(obj/'sam2'/view['path']/(cam+'_mask.png')),0)>127;cloud=P[mask&valid&(P[:,:,2]>axis[2]+.006)&(np.linalg.norm(P[:,:,:2]-axis[:2],axis=2)<.105)]
   for _,normal,dd in planes(cloud,max_planes=8):
    r=patch(cloud,normal,dd)
    if not r:continue
    pts=cloud[np.abs(cloud@normal+dd)<PLANE_INLIER_TOLERANCE_M];ref=np.eye(3)[np.argmin(abs(normal))];u=np.cross(ref,normal);u/=np.linalg.norm(u);v=np.cross(normal,u);origin=pts.mean(0);xy=np.c_[(pts-origin)@u,(pts-origin)@v];hull=ConvexHull(xy);rect=cv2.minAreaRect(xy.astype('float32'));dims=sorted([float(v)*1000 for v in rect[1]],reverse=True);older=old.patch(cloud,normal,dd,center_point=r['point_table_m']);record={'view':view['path'],'camera':cam,'observed_planar_hull_area_mm2':float(hull.volume*1e6),'occupied_2mm_cells_area_mm2':len(np.unique(np.floor(xy/.002).astype(int),axis=0))*4,'enclosing_rectangle_mm':dims,'center_to_edge_mm':r['boundary_clearance_mm'],'remaining_margin_outside_25mm_cup_mm':r['boundary_clearance_mm']-12.5,'patch':r,'old_1_5mm_pass_at_same_center':bool(older and older['geometry_pass']),'hull_area_is_not_guaranteed_unbroken_suction_area':True};records.append(record)
    if view['path']=='view_000' and cam=='cam1':
     corners=origin+xy[hull.vertices,0,None]*u+xy[hull.vertices,1,None]*v;T=np.array(wc['cameras'][cam]['T_table_optical']);optical=(corners-T[:3,3])@T[:3,:3];pixels,_=cv2.projectPoints(optical,np.zeros(3),np.zeros(3),np.array(d['camera_info']['k']).reshape(3,3),np.array(d['camera_info']['d']));cv2.polylines(overlay,[np.round(pixels).astype('int32')],True,(0,200,255),1)
 summary={'object':identity['object'],'label':identity['label'],'faces':records};results.append(summary)
report={'robot_commands_sent':False,'fresh_frames_per_camera':len(views),'plane_inlier_tolerance_mm':PLANE_INLIER_TOLERANCE_M*1000,'patch_rms_limit_mm':PATCH_RMS_LIMIT_MM,'cup_diameter_mm':25,'cup_edge_margin_mm':3,'required_center_to_edge_mm':15.5,'area_note':'visible planar-envelope area in each observation; occlusion and holes mean this is not the guaranteed sealable area','objects':results};(out/'flat_face_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));cv2.imwrite(str(out/'flat_faces_overlay.png'),overlay)
for result in results:
 faces=result['faces'];passing=[f for f in faces if f['patch']['geometry_pass']];best=max(passing or faces,key=lambda f:(f['patch']['geometry_pass'],f['observed_planar_hull_area_mm2'])) if faces else None
 print(result['label'],'faces',len(faces),'passing',len(passing),'best',json.dumps(best,ensure_ascii=False),flush=True)
print(out/'flat_face_report.json')
