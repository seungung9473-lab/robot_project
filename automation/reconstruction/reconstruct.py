"""Offline measured-angle RGB-D point-cloud reconstruction. No actuator interfaces."""
import argparse,json,math
from pathlib import Path
import cv2,numpy as np
from placement_geometry import load_context,object_region
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

def unrotate(points,angle,center,direction):
 R=Rotation.from_euler('z',-direction*math.radians(angle)).as_matrix()
 return (points-center)@R.T+center

def voxel(points,colors,size=.002):
 key=np.floor(points/size).astype(np.int64);_,inv=np.unique(key,axis=0,return_inverse=True);n=inv.max()+1;count=np.bincount(inv)
 p=np.column_stack([np.bincount(inv,weights=points[:,i],minlength=n)/count for i in range(3)]);c=np.column_stack([np.bincount(inv,weights=colors[:,i],minlength=n)/count for i in range(3)])
 if len(p)>12:
  dist,_=cKDTree(p).query(p,k=7);keep=dist[:,1:].mean(axis=1)<.010;p=p[keep];c=c[keep]
 if len(p)>12:
  pairs=cKDTree(p).query_pairs(.008,output_type='ndarray')
  if len(pairs):
   graph=coo_matrix((np.ones(len(pairs)),(pairs[:,0],pairs[:,1])),shape=(len(p),len(p)))
   _,labels=connected_components(graph,directed=False);keep=labels==np.argmax(np.bincount(labels));p=p[keep];c=c[keep]
 return p,c.astype(np.uint8)

def ply(path,p,c):
 with path.open('w') as f:
  f.write(f'ply\nformat ascii 1.0\nelement vertex {len(p)}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
  for x,rgb in zip(p,c):f.write(' '.join(map(str,[*x,*rgb]))+'\n')

def main():
 a=argparse.ArgumentParser();a.add_argument('session',type=Path);a.add_argument('--reference-angle',type=float,default=None);a.add_argument('--direction',type=int,choices=[-1,1],required=True);a.add_argument('--output',type=Path);a.add_argument('--mask-root',type=Path);a.add_argument('--no-preview',action='store_true',help='Skip optional merged preview and plot; retain measured camera clouds');args=a.parse_args();root=Path(__file__).resolve().parent.parent;out=args.output or args.session/'reconstruction';out.mkdir(parents=True,exist_ok=True)
 context=load_context(args.session);cal=context['calibrations'];center=np.array(context['axis_point_table_m']);manifest=json.loads((args.session/'session.json').read_text());views=manifest['views'];cloud={c:[] for c in ['cam1','cam2']};rows=[]
 if not views:raise RuntimeError('No completed views')
 reference=float(views[0]['angle_deg']) if args.reference_angle is None else float(args.reference_angle)
 for view in views:
  folder=args.session/view['path'];m=json.loads((folder/'metadata.json').read_text());angle=float(m['turntable']['angle_deg'])-reference
  for c in cloud:
   info=m['cameras'][c];raw=cv2.imread(str(folder/info['depth_file']),cv2.IMREAD_UNCHANGED) if info['depth_file'].endswith('.png') else np.load(folder/info['depth_file']);rgb=cv2.cvtColor(cv2.imread(str(folder/info['color_file'])),cv2.COLOR_BGR2RGB);ci=info['camera_info'];K=np.array(ci['k']).reshape(3,3);D=np.array(ci['d']);h,w=raw.shape;v,u=np.mgrid[0:h,0:w];z=raw*info['depth_scale_m'];ok=(z>.08)&(z<.9)
   if args.mask_root:
    mask_path=args.mask_root/view['path']/(c+'_mask.png');mask=cv2.imread(str(mask_path),cv2.IMREAD_GRAYSCALE)
    if mask is None or mask.shape!=raw.shape:raise RuntimeError('Missing or mismatched SAM2 mask '+str(mask_path))
    ok &= mask>127
   if not ok.any():
    rows.append({'view':view['path'],'camera':c,'points':0,'measured_angle_deg':m['turntable']['angle_deg'],'reason':'target not visible'});continue
   pixels=np.column_stack([u[ok],v[ok]]).astype(np.float64);rays=cv2.undistortPoints(pixels.reshape(-1,1,2),K,D).reshape(-1,2);p=np.column_stack([rays*z[ok,None],z[ok]]);T=np.array(cal[c][f'T_table_{c}_color_optical']);p=p@T[:3,:3].T+T[:3,3];color=rgb[ok]
   # Conservative cylinder above the known support: observed surfaces only.
   mask=object_region(p,context);before_crop=len(p);p=p[mask];color=color[mask];p=unrotate(p,angle,center,args.direction)
   rows.append({'view':view['path'],'camera':c,'points':len(p),'segmented_depth_points':before_crop,'removed_by_workspace_crop':int(before_crop-len(p)),'measured_angle_deg':m['turntable']['angle_deg']})
   if len(p):cloud[c].append((p,color))
 report={'scope':'observed surface point clouds; not watertight mesh or validated suction geometry','segmentation_masks':str(args.mask_root) if args.mask_root else None,'axis_source':'retained physical reference; estimated axis fit not applied','axis_point_table_m':center.tolist(),'axis_direction_table':[0,0,1],'object_centered_on_axis':False,'observation_radius_m':context['observation_radius_m'],'encoder_angle_sign_to_table_z':args.direction,'reference_angle_deg':reference,'views':rows,'calibration_snapshot':cal,'clouds':{}}
 combined=[]
 for c,parts in cloud.items():
  if not parts:continue
  p,color=voxel(np.vstack([x[0] for x in parts]),np.vstack([x[1] for x in parts]));ply(out/(c+'_object.ply'),p,color);np.savez_compressed(out/(c+'_object.npz'),points=p,colors=color);combined.append((p,color));report['clouds'][c]={'points':len(p),'object_center_table_m':((np.percentile(p,99,axis=0)+np.percentile(p,1,axis=0))/2).tolist(),'robust_extent_mm':((np.percentile(p,99,axis=0)-np.percentile(p,1,axis=0))*1000).tolist()}
 if not combined:raise RuntimeError('No object depth in ROI')
 if not args.no_preview:
  p,col=voxel(np.vstack([x[0] for x in combined]),np.vstack([x[1] for x in combined]));ply(out/'combined_preview.ply',p,col)
  import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
  fig=plt.figure(figsize=(10,5))
  for i,c in enumerate(['cam1','cam2']):
   ax=fig.add_subplot(1,2,i+1,projection='3d');f=out/(c+'_object.npz')
   if f.exists():
    d=np.load(f);p=d['points'];stride=max(1,len(p)//10000);p=p[::stride]-center;ax.scatter(p[:,0]*1000,p[:,1]*1000,p[:,2]*1000,c=d['colors'][::stride]/255,s=1);ax.set_box_aspect((1,1,1))
   ax.set_title(c+' observed surface');ax.set_xlabel('X (mm)');ax.set_ylabel('Y (mm)');ax.set_zlabel('Z above disc (mm)')
  fig.tight_layout();fig.savefig(out/'preview.png',dpi=160);plt.close(fig)
 report['preview_generated']=not args.no_preview
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report['clouds']));print(out)
if __name__=='__main__':main()
