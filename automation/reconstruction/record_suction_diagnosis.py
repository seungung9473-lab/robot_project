"""Passive synchronized RGB-D/encoder/pressure logger. No command publishers."""
import json,time,sys
from pathlib import Path
import rclpy,cv2
from sensor_msgs.msg import Image,CameraInfo,JointState
from std_msgs.msg import Int32
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('suction_cause_diagnostic_recorder');bridge=CvBridge();data={};last={};saved={};subs=[];t0=time.monotonic();log=(out/'frames.jsonl').open('w');q=None;diff=None
for c in ['cam1','cam2']:
 for kind,suffix,typ in [('color','color/image_raw',Image),('depth','aligned_depth_to_color/image_raw',Image),('info','color/camera_info',CameraInfo)]:subs.append(n.create_subscription(typ,f'/camera/{c}/{suffix}',lambda m,k=c+'_'+kind:data.update({k:(time.monotonic(),m)}),qos_profile_sensor_data))
def joint(m):
 global q
 if all('joint'+str(i) in m.name for i in range(1,5)):q={'t':time.monotonic(),'position':[m.position[m.name.index('joint'+str(i))] for i in range(1,5)]}
def vacuum(m):
 global diff
 diff={'t':time.monotonic(),'value':m.data}
subs.append(n.create_subscription(JointState,'/joint_states',joint,qos_profile_sensor_data));subs.append(n.create_subscription(Int32,'/vacuum/diff',vacuum,20))
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
ready=False
try:
 while time.monotonic()-t0<900 and not (out/'STOP').exists():
  rclpy.spin_once(n,timeout_sec=.01)
  phase=(out/'phase.txt').read_text().strip() if (out/'phase.txt').exists() else 'initial'
  for c in ['cam1','cam2']:
   if not all(c+'_'+k in data for k in ['color','depth','info']):continue
   now=time.monotonic();ct,color=data[c+'_color'];dt,depth=data[c+'_depth'];_,info=data[c+'_info']
   if now-ct>.4 or now-dt>.4 or abs(stamp(color)-stamp(depth))>100_000_000 or last.get(c)==stamp(depth) or now-saved.get(c,0)<.18:continue
   last[c]=stamp(depth);saved[c]=now;folder=out/phase;folder.mkdir(parents=True,exist_ok=True);key=c+'_'+str(stamp(depth));cv2.imwrite(str(folder/(key+'.jpg')),bridge.imgmsg_to_cv2(color,'bgr8'));cv2.imwrite(str(folder/(key+'_depth.png')),bridge.imgmsg_to_cv2(depth,'passthrough'));item={'camera':c,'phase':phase,'elapsed_s':now-t0,'color_stamp_ns':stamp(color),'depth_stamp_ns':stamp(depth),'image':str(folder/(key+'.jpg')),'depth':str(folder/(key+'_depth.png')),'joint':q,'diff':diff};log.write(json.dumps(item)+'\n');log.flush();p=out/(c+'_camera_info.json')
   if not p.exists():p.write_text(json.dumps({'K':list(info.k),'D':list(info.d),'width':info.width,'height':info.height,'distortion_model':info.distortion_model,'frame_id':info.header.frame_id},indent=2)+'\n')
  if not ready and len(saved)==2:ready=True;(out/'READY').write_text('RGB-D recording');print('READY',out,flush=True)
finally:log.close();n.destroy_node();rclpy.shutdown();print('Recorder stopped',flush=True)
