"""Offline cuboid silhouette pose fit; diagnostic estimate, no robot commands."""
from pathlib import Path
import json,numpy as np,cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
r=Path(__file__).resolve().parent;calroot=r.parent/'camera_calibration';active=json.loads((calroot/'active.json').read_text());session=r/'multiview/20261101_194509_024461';lift=r/'suction_retry/offset1mm_trial_20261101_203438';meta=json.loads((session/'view_001/metadata.json').read_text());cameras={}
for c in ['cam1','cam2']:
 T=np.array(json.loads(Path(active['calibrations'][c]).read_text())[f'T_table_{c}_color_optical']);ci=meta['cameras'][c]['camera_info'];cameras[c]={'C':np.linalg.inv(T),'K':np.array(ci['k']).reshape(3,3),'D':np.array(ci['d'])}
# User supplied box:50x50x60mm. Origin at box center.
verts=np.array([[x,y,z] for x in [-.025,.025] for y in [-.025,.025] for z in [-.030,.030]])
def observed(mask):
 contours=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[0];contour=max(contours,key=cv2.contourArea);hull=cv2.convexHull(contour).reshape(-1,2).astype(float);return hull

def sampled_edges(hull,n=12):return np.vstack([a+(b-a)*t for a,b in zip(hull,np.roll(hull,-1,axis=0)) for t in np.linspace(0,1,n,endpoint=False)])
def distance_segments(p,hull):
 a=hull;b=np.roll(hull,-1,axis=0);v=b-a;t=np.clip(np.sum((p[:,None]-a)*v,axis=2)/np.maximum(np.sum(v*v,axis=1),1e-9),0,1);d=np.linalg.norm(p[:,None]-(a+t[:,:,None]*v),axis=2);return d.min(axis=1)
def project(params,c):
 center=np.array([1.095,.235,.089])+params[:3];R=Rotation.from_euler('xyz',params[3:]).as_matrix();p=verts@R.T+center;cam=cameras[c];C=cam['C'];q=p@C[:3,:3].T+C[:3,3];image=cv2.projectPoints(q,np.zeros(3),np.zeros(3),cam['K'],cam['D'])[0].reshape(-1,2);idx=cv2.convexHull(image.astype(np.float32),returnPoints=False).ravel();return image[idx]
def fit(hulls,upright=False,initial=None):
 obs={c:sampled_edges(h) for c,h in hulls.items()}
 def unpack(x):return np.r_[x[:3],0.,0.,x[3]] if upright else x
 def error(x):
  params=unpack(x);err=[]
  for c in hulls:
   pred=project(params,c);weight=1. if c=='cam1' else 1.
   # Fixed-size samples keep residual dimensions stable even if hull topology changes.
   for points,hull in [(obs[c],pred)]:err.extend(distance_segments(points,hull)*weight)
  return np.array(err)
 bounds=([-.04,-.04,-.02,-np.pi], [.04,.04,.05,np.pi]) if upright else ([-.04,-.04,-.02,-.35,-.35,-np.pi],[.04,.04,.05,.35,.35,np.pi])
 starts=[]
 for yaw in [-.25,0,.25]:
  starts.append([0,0,.008,yaw] if upright else [0,0,.008,0,0,yaw])
 if initial is not None:starts=[initial]
 sols=[least_squares(error,np.array(x,dtype=float),bounds=bounds,diff_step=1e-4,max_nfev=300,loss='soft_l1',f_scale=1.) for x in starts];sol=min(sols,key=lambda x:np.mean(error(x.x)**2));param=unpack(sol.x);normal=Rotation.from_euler('xyz',param[3:]).apply([0,0,1]);return {'parameters':param.tolist(),'box_center_table_m':(np.array([1.095,.235,.089])+param[:3]).tolist(),'normal_table':normal.tolist(),'tilt_from_vertical_deg':float(np.degrees(np.arccos(np.clip(normal[2],-1,1)))),'contour_residual_rms_px':float(np.sqrt(np.mean(error(sol.x)**2))),'optimality':float(sol.optimality),'raw_solution':sol.x.tolist()}
result={'method':'two-camera 50x50x60mm cuboid convex-silhouette fitting; not independent metrology','limitations':['manual dimensions assumed exact','camera extrinsic and mask contour error remain','no fresh depth during lift; static saved RGB only','partial occlusion and box shape imperfections affect estimates'],'sets':{}}
for label in ['resting','lifted']:
 hulls={}
 for c in cameras:
  path=session/'sam2/view_001'/(c+'_mask.png') if label=='resting' else lift/('lifted_'+c+'_mask.png');mask=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE);hulls[c]=observed(mask>127)
 upright=fit(hulls,True);free=fit(hulls,False);result['sets'][label]={'upright_fit':upright,'free_fit':free};print(label,'upright rms',upright['contour_residual_rms_px'],'free rms',free['contour_residual_rms_px'],'tilt',free['tilt_from_vertical_deg'],flush=True)
 for c in cameras:
  path=session/'view_001'/(c+'_color.png') if label=='resting' else lift/('lifted_'+c+'.jpg');im=cv2.imread(str(path));cv2.polylines(im,[hulls[c].astype(np.int32)],True,(0,255,0),1);cv2.polylines(im,[project(upright['parameters'],c).astype(np.int32)],True,(0,255,255),1);cv2.polylines(im,[project(free['parameters'],c).astype(np.int32)],True,(255,0,255),1);cv2.putText(im,'green:mask yellow:upright magenta:free',(8,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1);cv2.imwrite(str(lift/(label+'_'+c+'_pose_review.jpg')),im)
 # Contour perturbations measure fit sensitivity only, not full confidence intervals.
 rng=np.random.default_rng(9473);tilts=[]
 for _ in range(12):
  perturbed={c:h+rng.normal(0,.7,h.shape) for c,h in hulls.items()};z=fit(perturbed,False,free['raw_solution']);tilts.append(z['tilt_from_vertical_deg'])
 result['sets'][label]['pixel_perturbation_tilt_range_deg']=[min(tilts),max(tilts)]
(lift/'horizontal_pick_validation.json').write_text(json.dumps(result,indent=2)+'\n');print('Saved',lift/'horizontal_pick_validation.json')
