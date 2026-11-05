from pathlib import Path
import sys,json,warnings
import cv2,numpy as np
p=Path(sys.argv[1]);meta=json.loads((p/'metadata.json').read_text());frames=np.load(p/'depth_frames_m.npz')['depth'];im=cv2.imread(str(p/'color.png'));K=np.array(meta['K']).reshape(3,3);D=np.array(meta['D']);H,W=frames.shape[1:];rng=np.random.default_rng(10)
valid=np.isfinite(frames)&(frames>.15)&(frames<1.5);a=np.where(valid,frames,np.nan)
with warnings.catch_warnings():
 warnings.simplefilter('ignore');depth=np.nanmedian(a,axis=0)
count=valid.sum(0);ok=np.isfinite(depth)&(count>=len(frames)*.7);hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);orange=((hsv[:,:,0]<28)|(hsv[:,:,0]>170))&(hsv[:,:,1]>95)&(hsv[:,:,2]>45)
# Restrict target search to the right-hand tabletop, excluding people and robot.
orange[:int(H*.2)]=False;orange[:,:int(W*.6)]=False;orange[int(H*.85):]=False
num,labels,stats,_=cv2.connectedComponentsWithStats(orange.astype(np.uint8));idx=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA]));mask=labels==idx;assert mask.sum()>150,'Target not found';ys,xs=np.where(mask);x0,x1=int(xs.min()),int(xs.max());y0,y1=int(ys.min()),int(ys.max())
yy,xx=np.indices((H,W));uv=np.column_stack([xx.ravel(),yy.ravel()]).astype(float);rays=cv2.undistortPoints(uv.reshape(-1,1,2),K,D).reshape(H,W,2);cloud=np.dstack([rays[:,:,0]*depth,rays[:,:,1]*depth,depth]);roi=(xx>=max(0,x0-65))&(xx<=min(W-1,x1+70))&(yy>=min(H-1,y1+8))&(yy<=min(H-1,y1+105));tablemask=roi&ok&~mask
pts=cloud[tablemask];assert len(pts)>200
if len(pts)>3000:pts=pts[rng.choice(len(pts),3000,replace=False)]
best=None
for _ in range(350):
 q=pts[rng.choice(len(pts),3,replace=False)];n=np.cross(q[1]-q[0],q[2]-q[0]);v=np.linalg.norm(n)
 if v<1e-9:continue
 n/=v;d=-n@q[0];inside=np.abs(pts@n+d)<.003
 if best is None or inside.sum()>best[0]:best=(int(inside.sum()),inside)
P=pts[best[1]];origin=P.mean(0);_,_,V=np.linalg.svd(P-origin,full_matrices=False);n=V[-1];n=n if n@origin<0 else -n;d=-n@origin
heights=np.einsum('ijk,k->ij',cloud,n)+d;inner=cv2.erode(mask.astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool);target=inner&ok;hs=heights[target];assert len(hs)>30
# Find the dense upper surface; no expected-height fit or depth hole filling.
hist,edges=np.histogram(hs,bins=np.arange(.005,.151,.002));dense=np.where(hist>=max(8,hist.max()*.15))[0];assert len(dense)
peak=dense[-1];topheight=float(np.median(hs[(hs>=edges[max(0,peak-2)])&(hs<=edges[min(len(edges)-1,peak+2)])]));top=target&(np.abs(heights-topheight)<.003)
# Refit parallel-plane separation from points near the visible top.
height=float(np.median(heights[top]));toppts=cloud[top];e1=V[0];e2=np.cross(n,e1);xy=np.column_stack([toppts@e1,toppts@e2]).astype(np.float32);rect=cv2.minAreaRect(xy);dims=sorted(rect[1])
frame_heights=[]
for f,v in zip(frames,valid):
 region=top&v
 if region.sum()<20:continue
 z=f[region];xyz=np.column_stack([rays[:,:,0][region]*z,rays[:,:,1][region]*z,z]);frame_heights.append(float(np.median(xyz@n+d)))
result={'camera':meta['camera'],'samples':meta['samples'],'target_bbox_xyxy':[x0,y0,x1,y1],'object_valid_depth_fraction':float(ok[mask].mean()),'table_plane_inliers':len(P),'table_plane_rms_mm':float(np.sqrt(np.mean((P@n+d)**2))*1000),'measured_height_mm':height*1000,'expected_height_mm':60.,'height_error_mm':height*1000-60.,'height_repeat_std_mm':float(np.std(frame_heights)*1000),'visible_top_extent_mm':[float(v*1000) for v in dims],'extent_note':'Visible valid depth extent only, not a verified full box dimension','top_surface_pixels':int(top.sum()),'table_plane_camera':[float(v) for v in n]+[float(d)]}
(p/'analysis.json').write_text(json.dumps(result,indent=2));review=im.copy();review[top]=(0,255,0);review[tablemask]=cv2.addWeighted(review[tablemask],.7,np.full_like(review[tablemask],(255,100,0)),.3,0);cv2.rectangle(review,(x0,y0),(x1,y1),(0,255,255),2);cv2.imwrite(str(p/'analysis_review.png'),review);print(json.dumps(result,indent=2))
