"""Passive ROS image observer; never sends camera or motor commands."""
import sys,time,json,datetime,os
from pathlib import Path
import cv2,numpy as np,rclpy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
root=Path(__file__).resolve().parent;layout=json.loads((root/'marker_layout.json').read_text());c=sys.argv[1];assert c in ['cam1','cam2']
ids_wanted=[0];edge=0.1;h=edge/2;offset=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]])
obj=offset.astype(float)
rclpy.init();n=rclpy.create_node(c+'_fixed_markers_observer');bridge=CvBridge();data={};samples=[];seen=set();last=None;image=None;annot=None;quality_rejected=0
subscriptions=[n.create_subscription(Image,f'/camera/{c}/color/image_raw',lambda m:data.update(image=(time.monotonic(),m)),qos_profile_sensor_data),n.create_subscription(CameraInfo,f'/camera/{c}/color/camera_info',lambda m:data.update(info=(time.monotonic(),m)),qos_profile_sensor_data)]
dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50);params=cv2.aruco.DetectorParameters_create();params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
end=time.monotonic()+20
try:
 while len(samples)<45 and time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.025)
  if not all(k in data for k in ['image','info']):continue
  received,m=data['image'];it,info=data['info'];stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  if stamp==last or time.monotonic()-received>.5 or time.monotonic()-it>1:continue
  last=stamp
  if (info.width,info.height)!=(m.width,m.height):continue
  if info.distortion_model not in ['plumb_bob','rational_polynomial']:raise RuntimeError('Unsupported distortion model '+info.distortion_model)
  image=bridge.imgmsg_to_cv2(m,'bgr8');annot=image.copy();corners,ids,_=cv2.aruco.detectMarkers(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),dictionary,parameters=params)
  if ids is None:continue
  cv2.aruco.drawDetectedMarkers(annot,corners,ids);seen.update(map(int,ids.flatten()))
  if any(list(ids.flatten()).count(i)!=1 for i in ids_wanted):continue
  lookup={int(i):k for k,i in enumerate(ids.flatten())};pts=np.vstack([corners[lookup[i]].reshape(4,2) for i in ids_wanted]).astype(float);K=np.array(info.k).reshape(3,3);D=np.array(info.d)
  if min(K[0,0],K[1,1])<=0:continue
  ret=cv2.solvePnPGeneric(obj,pts,K,D,flags=cv2.SOLVEPNP_IPPE);candidates=[]
  for rv,tv in zip(ret[1],ret[2]):
   rv,tv=cv2.solvePnPRefineLM(obj,pts,K,D,rv,tv);R=cv2.Rodrigues(rv)[0];pos=-R.T@tv.flatten()
   if pos[2]<=0 or np.any((obj@R.T+tv.flatten())[:,2]<=0):continue
   e=np.linalg.norm(cv2.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2)-pts,axis=1);candidates.append((float(np.sqrt(np.mean(e**2))),rv,tv,e))
  if not candidates:continue
  err,rv,tv,e=min(candidates,key=lambda a:a[0])
  if err>3:quality_rejected+=1;continue
  R=cv2.Rodrigues(rv)[0];T=np.eye(4);T[:3,:3]=R.T;T[:3,3]=-R.T@tv.flatten();cv2.drawFrameAxes(annot,K,D,rv,tv,.1)
  samples.append({'stamp_ns':stamp,'T_marker_camera_optical':T.tolist(),'reprojection_rms_px':err,'per_marker_rms_px':dict(zip(map(str,ids_wanted),np.sqrt(np.mean(e.reshape(1,4)**2,axis=1)).tolist())),'corners_px':pts.tolist(),'K':K.tolist(),'D':D.tolist(),'frame_id':m.header.frame_id,'width':m.width,'height':m.height,'distortion_model':info.distortion_model})
 out=root/'measurements'/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+c+'_turntable');out.mkdir(parents=True)
 result={'camera':c,'ids_used':ids_wanted,'detected_ids':sorted(seen),'layout_file':str(root/'marker_layout.json'),'marker_layout_snapshot':layout['turntable_marker'],'samples':samples,'rejected_high_reprojection_frames':quality_rejected,'independently_validated':False,'validated_for_robot_execution':False}
 if samples:
  Ts=np.array([s['T_marker_camera_optical'] for s in samples]);U,_,V=np.linalg.svd(np.mean(Ts[:,:3,:3],axis=0));T=np.eye(4);T[:3,:3]=U@np.diag([1,1,np.linalg.det(U@V)])@V;T[:3,3]=np.median(Ts[:,:3,3],axis=0);result.update(T_marker_camera_optical=T.tolist(),median_reprojection_rms_px=float(np.median([s['reprojection_rms_px'] for s in samples])),position_sample_max_deviation_mm=float(max(np.linalg.norm(Ts[:,:3,3]-T[:3,3],axis=1)*1000)))
 (out/'measurement.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
 if annot is not None:cv2.imwrite(str(out/'review.png'),annot)
 print('OUTPUT',out,flush=True);print('samples',len(samples),'seen',sorted(seen),'quality_rejected',quality_rejected,flush=True)
 if samples:print('camera position in marker cm',T[:3,3]*100,'RMS px',result['median_reprojection_rms_px'],'spread mm',result['position_sample_max_deviation_mm'],flush=True)
finally:n.destroy_node();rclpy.try_shutdown()
