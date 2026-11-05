from pathlib import Path
import json,time,sys,datetime,collections
import rclpy,cv2,numpy as np
from sensor_msgs.msg import Image,CameraInfo
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
c=sys.argv[1];rclpy.init();n=rclpy.create_node(c+'_depth_validation_capture');b=CvBridge();buf={k:collections.deque(maxlen=30) for k in ['color','depth','info']};samples=[];last=None;subs=[]
for k,suffix,typ in [('color','color/image_raw',Image),('depth','aligned_depth_to_color/image_raw',Image),('info','color/camera_info',CameraInfo)]:
 subs.append(n.create_subscription(typ,f'/camera/{c}/'+suffix,lambda m,key=k:buf[key].append((time.monotonic(),m)),qos_profile_sensor_data))
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
end=time.monotonic()+20
try:
 while len(samples)<30 and time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.05)
  if not all(buf.values()):continue
  t,d=buf['depth'][-1]
  if stamp(d)==last or time.monotonic()-t>.5:continue
  last=stamp(d);_,color=min(buf['color'],key=lambda x:abs(stamp(x[1])-stamp(d)));_,info=min(buf['info'],key=lambda x:abs(stamp(x[1])-stamp(d)))
  if max(abs(stamp(color)-stamp(d)),abs(stamp(info)-stamp(d)))>100e6:continue
  assert (color.width,color.height)==(d.width,d.height)==(info.width,info.height)
  assert d.encoding in ['16UC1','32FC1']
  raw=b.imgmsg_to_cv2(d,'passthrough').copy();depth=raw.astype(float)*(.001 if d.encoding=='16UC1' else 1.)
  samples.append((b.imgmsg_to_cv2(color,'bgr8').copy(),depth,stamp(d)))
 if len(samples)<15:raise RuntimeError('Not enough fresh aligned depth frames: '+str(len(samples)))
 p=Path(__file__).parent/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+c);p.mkdir();arr=np.array([s[1] for s in samples]);np.savez_compressed(p/'depth_frames_m.npz',depth=arr,stamps_ns=np.array([s[2] for s in samples]));cv2.imwrite(str(p/'color.png'),samples[-1][0]);meta={'camera':c,'samples':len(samples),'K':list(info.k),'D':list(info.d),'width':info.width,'height':info.height,'distortion_model':info.distortion_model,'depth_encoding':d.encoding,'expected_box_m':[.05,.05,.06],'frame_id':color.header.frame_id};(p/'metadata.json').write_text(json.dumps(meta,indent=2));print('SAVED',p,flush=True)
finally:n.destroy_node();rclpy.try_shutdown()
