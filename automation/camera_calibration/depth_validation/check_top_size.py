from pathlib import Path
import json,cv2,numpy as np
r=Path(__file__).resolve().parent
for folder in ['20261101_145243_cam1','20261101_145225_cam2']:
 p=r/folder;m=json.loads((p/'metadata.json').read_text());a=json.loads((p/'analysis.json').read_text());im=cv2.imread(str(p/'color.png'));frames=np.load(p/'depth_frames_m.npz')['depth'];valid=np.isfinite(frames)&(frames>.15)&(frames<1.5)
 import warnings
 with warnings.catch_warnings():
  warnings.simplefilter('ignore');z=np.nanmedian(np.where(valid,frames,np.nan),axis=0)
 H,W=z.shape;yy,xx=np.indices((H,W));uv=np.column_stack([xx.ravel(),yy.ravel()]).astype(float);K=np.array(m['K']).reshape(3,3);D=np.array(m['D']);rays=cv2.undistortPoints(uv.reshape(-1,1,2),K,D).reshape(H,W,2);P=np.dstack([rays[:,:,0]*z,rays[:,:,1]*z,z]);n=np.array(a['table_plane_camera'][:3]);d=a['table_plane_camera'][3];height=P@n+d
 x0,y0,x1,y1=a['target_bbox_xyxy'];roi=(xx>=x0)&(xx<=x1)&(yy>=y0)&(yy<=y1);hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);color=((hsv[:,:,0]<28)|(hsv[:,:,0]>170))&(hsv[:,:,1]>95)&(hsv[:,:,2]>45);ok=(valid.mean(0)>=.7)&np.isfinite(z)
 top=roi&color&ok&(np.abs(height-a['measured_height_mm']/1000)<.0025);points=P[top];origin=points.mean(0);_,_,V=np.linalg.svd(points-origin,full_matrices=False);nt=V[-1];nt=nt if nt@n>0 else -nt;dt=-nt@origin
 # Use a plane fit to actual top points, independently of tabletop normal.
 dist=np.abs(P@nt+dt);top2=roi&color&ok&(dist<.0015)&(height>.025);points2=P[top2];origin2=np.median(points2,axis=0);e1=V[0];e2=np.cross(nt,e1);q=np.column_stack([(points2-origin2)@e1,(points2-origin2)@e2]).astype(np.float32)
 print(folder,'initial top points',len(points),'plane residual mm',np.sqrt(np.mean((points@nt+dt)**2))*1000,'topnormal delta',np.degrees(np.arccos(np.clip(nt@n,-1,1))),'count',len(points2))
 for percentile in [0,1,3]:
  if percentile:
   # Remove only spatially extreme tails, do not impose nominal box size.
   lo=np.percentile(q,percentile,axis=0);hi=np.percentile(q,100-percentile,axis=0);v=q[np.all((q>=lo)&(q<=hi),axis=1)]
  else:v=q
  rect=cv2.minAreaRect(v);print('trim',percentile,'size mm',np.array(sorted(rect[1]))*1000)
 cv2.imwrite(str(p/'top_points_mask.png'),top2.astype(np.uint8)*255)
 np.savez(p/'top_size_points.npz',points=points2,origin=origin2,e1=e1,e2=e2,normal=nt,d=dt)
