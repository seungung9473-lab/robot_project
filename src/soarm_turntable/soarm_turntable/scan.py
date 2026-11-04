"""Stationary RGB-D capture; optional explicit turntable stop-and-shoot scanning."""
import argparse,collections,datetime,json,math,os,shutil,tempfile,time,uuid
from pathlib import Path
import cv2,numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from rosidl_runtime_py.convert import message_to_ordereddict
from .scan_core import preflight_angles,stable_states,stability_problem,select_bundle,StationaryCaptureGate
from .environment_core import load_environment,foreground,calibration_key

def write_json(path,value):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False));os.replace(tmp,path)

class Scanner(Node):
    def __init__(self,environment=None,settle_seconds=0.5):
        if settle_seconds!=0.5:raise ValueError('정지 확인 후 추가 대기는 0.5초로 고정됩니다')
        self.settle_seconds=StationaryCaptureGate(0.5).seconds
        self.environment,self.environment_path=load_environment(environment) if environment else (None,None)
        super().__init__('soarm_turntable_scan');self.buffers={};self.states=collections.deque(maxlen=25);self.arm=None;self.arm_reference=None;self.reference_zero=None;self.reference_stop=None;self.goal_handle=None;self.bridge=CvBridge()
        for camera in ('cam1','cam2'):
            for kind,suffix,typ in [('color','color/image_raw',Image),('depth','aligned_depth_to_color/image_raw',Image),('info','color/camera_info',CameraInfo)]:
                key=camera+'_'+kind;self.buffers[key]=collections.deque(maxlen=12)
                self.create_subscription(typ,'/camera/'+camera+'/'+suffix,lambda m,k=key:self.buffers[k].append((time.monotonic(),m)),qos_profile_sensor_data)
        self.create_subscription(String,'/turntable/state',self.state_cb,30)
        self.create_subscription(JointState,'/joint_states',self.arm_cb,qos_profile_sensor_data)
        self.action=ActionClient(self,FollowJointTrajectory,'/turntable/follow_joint_trajectory');self.stop_client=self.create_client(Trigger,'/turntable/stop')
    def state_cb(self,m):
        try:self.states.append((time.monotonic(),json.loads(m.data)))
        except (ValueError,TypeError):pass
    def arm_cb(self,m):
        values=dict(zip(m.name,m.position))
        if all('joint'+str(i) in values for i in range(1,5)):self.arm=(time.monotonic(),[values['joint'+str(i)] for i in range(1,5)])
    def state(self):
        if not self.states or time.monotonic()-self.states[-1][0]>.35:raise RuntimeError('턴테이블 상태 수신 없음/지연')
        s=self.states[-1][1]
        if s.get('hardware_error',1):raise RuntimeError('턴테이블 하드웨어 오류')
        if self.reference_zero is not None and s.get('zero_count')!=self.reference_zero:raise RuntimeError('스캔 중 영점 변경 감지')
        if self.reference_stop is not None and s.get('stop_generation')!=self.reference_stop:raise RuntimeError('외부 STOP 감지 · 스캔 중단')
        return s
    def guard_arm(self):
        if self.arm_reference is None:return
        if self.arm is None or time.monotonic()-self.arm[0]>1:raise RuntimeError('스캔 중 로봇팔 피드백 유실')
        if any(not math.isfinite(q) or abs(q-r)>math.radians(.5) for q,r in zip(self.arm[1],self.arm_reference)):raise RuntimeError('스캔 중 로봇팔 움직임 감지 · 턴테이블 중단')
    def spin(self,seconds=.02):rclpy.spin_once(self,timeout_sec=seconds);self.guard_arm()
    def await_future(self,f,timeout):
        end=time.monotonic()+timeout
        while not f.done():
            self.spin()
            if time.monotonic()>end:raise RuntimeError('턴테이블 응답 시간 초과')
        return f.result()
    def stop(self):
        if self.goal_handle is not None:self.goal_handle.cancel_goal_async()
        if self.stop_client.wait_for_service(timeout_sec=.5):
            f=self.stop_client.call_async(Trigger.Request())
            # Still spin cleanup even when the arm guard is what failed.
            end=time.monotonic()+2
            while not f.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)
            if not f.done() or not f.result().success:self.get_logger().error('정지 명령 확인 실패 · 실제 정지 상태 확인 필요')
    def move(self,angle,timeout):
        self.state();self.guard_arm()
        if not self.action.wait_for_server(timeout_sec=2):raise RuntimeError('턴테이블 액션 없음')
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=['turntable_joint'];pt=JointTrajectoryPoint(positions=[float(math.radians(angle))]);pt.time_from_start.sec=int(timeout);goal.trajectory.points=[pt]
        pending=self.action.send_goal_async(goal)
        # Do not orphan a goal accepted after a request timeout.
        abandoned=[False]
        pending.add_done_callback(lambda f:f.result().cancel_goal_async() if abandoned[0] and f.result() and f.result().accepted else None)
        try:self.goal_handle=self.await_future(pending,5)
        except BaseException:
            abandoned[0]=True
            if pending.done() and pending.result() and pending.result().accepted:self.goal_handle=pending.result()
            raise
        if not self.goal_handle.accepted:self.goal_handle=None;raise RuntimeError('회전 요청 거부: 읽기 전용/영점/중복 실행을 확인하세요')
        result=self.await_future(self.goal_handle.get_result_async(),timeout+3)
        self.goal_handle=None
        if result.status!=4 or result.result.error_code!=0:raise RuntimeError(result.result.error_string or '턴테이블 이동 실패')
    def wait_stable(self,timeout=8):
        end=time.monotonic()+timeout
        while True:
            self.spin();self.state()
            if stable_states(list(self.states),time.monotonic()):return
            if time.monotonic()>end:raise RuntimeError('턴테이블 정지 안정화 시간 초과')
    def capture(self,session,index,target=None):
        self.wait_stable();gate=StationaryCaptureGate(self.settle_seconds)
        end=time.monotonic()+max(10.,self.settle_seconds+8.);after=None;received_after=None;position_at_barrier=None
        while True:
            self.spin();state=self.state();now=time.monotonic()
            problem=stability_problem(list(self.states),now)
            ready=gate.update(problem,now)
            if not ready:
                # A status gap invalidates the exposure barrier as well as the dwell.
                after=None;received_after=None;position_at_barrier=None
            elif after is None:
                # Discard frames captured/received during rotation or settling.
                for buffer in self.buffers.values():buffer.clear()
                after=self.get_clock().now().nanoseconds;received_after=now
                position_at_barrier=state['position_count']
            else:
                if abs(state['position_count']-position_at_barrier)>2:
                    raise RuntimeError('정지 확인 후 촬영 중 위치가 2 count 넘게 바뀌었습니다')
                bundle=select_bundle(self.buffers,after,now,received_after=received_after)
                if bundle and self.states[-1][0]>=max(t for t,_ in bundle.values()):break
            if now>end:raise RuntimeError('정지 후 새 RGB/깊이 영상 동기화 시간 초과')
        tmp=session/('.pending-'+uuid.uuid4().hex);tmp.mkdir();meta={'index':index,'requested_angle_deg':target,'turntable':state,'camera_to_robot_transform':None,'rotation_axis_calibration':None,'cameras':{},'capture_timing':{'mode':'stop-settle-flush-fresh-capture','extra_settle_seconds':self.settle_seconds,'continuous_stationary_since_monotonic':gate.since,'frame_not_before_ros_ns':after,'frame_received_not_before_monotonic':received_after,'state_received_monotonic':self.states[-1][0],'state_position_count_at_barrier':position_at_barrier,'state_position_count_at_capture':state['position_count']}}
        stamps=[]
        for camera in ('cam1','cam2'):
            color=bundle[camera+'_color'][1];depth=bundle[camera+'_depth'][1];info=bundle[camera+'_info'][1]
            if (color.width,color.height)!=(depth.width,depth.height) or (info.width,info.height)!=(color.width,color.height):raise RuntimeError('컬러/정렬깊이/내부행렬 해상도 불일치')
            if info.k[0]<=0 or info.k[4]<=0:raise RuntimeError('카메라 내부행렬 없음')
            rgb=self.bridge.imgmsg_to_cv2(color,desired_encoding='bgr8');raw=self.bridge.imgmsg_to_cv2(depth,desired_encoding='passthrough')
            if depth.encoding=='16UC1':depth_file=camera+'_depth_mm.png';valid=raw>0;scale=.001
            elif depth.encoding=='32FC1':depth_file=camera+'_depth_m.npy';valid=np.isfinite(raw)&(raw>0);scale=1.
            else:raise RuntimeError('지원하지 않는 깊이 인코딩 '+depth.encoding)
            if not valid.any():raise RuntimeError(camera+' 깊이 유효 픽셀 없음')
            if not cv2.imwrite(str(tmp/(camera+'_color.png')),rgb):raise RuntimeError('컬러 저장 실패')
            if depth_file.endswith('.png'):
                if not cv2.imwrite(str(tmp/depth_file),raw):raise RuntimeError('깊이 저장 실패')
            else:np.save(tmp/depth_file,raw,allow_pickle=False)
            camera_stamps={kind:msg.header.stamp.sec*10**9+msg.header.stamp.nanosec for kind,msg in [('color',color),('depth',depth),('camera_info',info)]};stamps.extend(camera_stamps.values())
            meta['cameras'][camera]={'color_file':camera+'_color.png','depth_file':depth_file,'depth_encoding':depth.encoding,'depth_scale_m':scale,'valid_depth_fraction':float(valid.mean()),'stamps_ns':camera_stamps,'color_frame':color.header.frame_id,'depth_frame':depth.header.frame_id,'camera_info':message_to_ordereddict(info)}
            if self.environment:
                env=self.environment['cameras'][camera];current_info=message_to_ordereddict(info)
                if calibration_key(current_info)!=env['calibration_key']:raise RuntimeError(camera+' 환경 저장 당시와 카메라 설정이 다릅니다. 배경을 다시 수집하세요')
                mask,roi=foreground(raw.astype(np.float32)*(scale*1000),env['background'],env['polygon'],**self.environment['filters'])
                if not cv2.imwrite(str(tmp/(camera+'_object_mask.png')),mask):raise RuntimeError('후보 마스크 저장 실패')
                meta['cameras'][camera]['object_mask_file']=camera+'_object_mask.png';meta['cameras'][camera]['object_pixels']=int((mask>0).sum())
        if self.environment:meta['environment']={'file':str(self.environment_path),'filters':self.environment['filters'],'note':'2D foreground candidates only; not a suction or collision decision'}
        meta['max_timestamp_skew_ms']=(max(stamps)-min(stamps))/1e6
        write_json(tmp/'metadata.json',meta);dst=session/f'view_{index:03d}';os.replace(tmp,dst);return dst,meta

def main():
    p=argparse.ArgumentParser(description='두 D415와 실제 턴테이블 각도 저장');p.add_argument('command',choices=['capture','scan']);p.add_argument('--environment',help='환경 설정 environment.json 또는 active.json');p.add_argument('--angles',nargs='+',type=float);p.add_argument('--output',default=str(Path.home()/'robot_ws/data/turntable_scans'));p.add_argument('--timeout',type=int,default=30);p.add_argument('--settle-seconds',type=float,default=0.5,choices=[0.5],help='정지 확인 후 추가 대기 0.5초 고정');a=p.parse_args()
    if a.command=='scan' and (not a.angles or not all(math.isfinite(x) for x in a.angles)):p.error('scan에는 --angles 0 30 60 처럼 각도를 지정하세요')
    if not 1<=a.timeout<=120:p.error('timeout은 1~120초')
    root=Path(a.output).expanduser();root.mkdir(parents=True,exist_ok=True)
    if shutil.disk_usage(root).free<100*1024**2:raise RuntimeError('저장 공간이 부족합니다')
    session=root/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);session.mkdir();manifest={'version':1,'status':'initializing','command':a.command,'angles':a.angles,'views':[],'calibrated':False,'scope':'raw RGB-D and measured angle only; no reconstruction or suction planning'};write_json(session/'session.json',manifest)
    rclpy.init();n=Scanner(environment=a.environment,settle_seconds=a.settle_seconds);moving=False
    try:
        deadline=time.monotonic()+10
        while not n.states:
            n.spin()
            if time.monotonic()>deadline:raise RuntimeError('읽기 전용 turntable 브리지를 먼저 실행하세요')
        if n.count_publishers('/turntable/state')!=1:raise RuntimeError('턴테이블 상태 발행자가 없거나 중복입니다')
        # Validate camera readiness before sending any motor goal.
        deadline=time.monotonic()+10
        while not all(n.buffers.values()):
            n.spin()
            if time.monotonic()>deadline:raise RuntimeError('두 카메라 토픽을 먼저 실행하세요')
        angles=a.angles if a.command=='scan' else [None]
        if a.command=='scan':
            preflight_angles(n.state(),list(angles)+[0.0])
            deadline=time.monotonic()+5
            while (n.arm is None or time.monotonic()-n.arm[0]>1) and time.monotonic()<deadline:n.spin()
            if n.arm is None or time.monotonic()-n.arm[0]>1:raise RuntimeError('회전 스캔에는 로봇팔 정지 감시용 /joint_states가 필요합니다')
            n.arm_reference=list(n.arm[1]);n.reference_zero=n.state()['zero_count'];n.reference_stop=n.state()['stop_generation'];n.guard_arm()
        manifest['status']='recording';write_json(session/'session.json',manifest)
        if a.command=='scan':
            baseline,_=n.capture(session,0);manifest['baseline']=baseline.name;write_json(session/'session.json',manifest)
        for i,angle in enumerate(angles,start=1 if a.command=='scan' else 0):
            if angle is not None:moving=True;n.move(angle,a.timeout)
            path,meta=n.capture(session,i,angle);manifest['views'].append({'path':path.name,'angle_deg':meta['turntable']['angle_deg'],'position_count':meta['turntable']['position_count']});write_json(session/'session.json',manifest);print('저장 완료:',path,flush=True)
        if a.command=='scan':
            manifest['status']='returning_to_zero';write_json(session/'session.json',manifest)
            moving=True;n.move(0.0,a.timeout)
            manifest['return_to_zero']={'requested_angle_deg':0.0,'state':n.state(),'status':'complete'}
            print('촬영 완료 · 턴테이블 0도 복귀 완료',flush=True)
        manifest['status']='complete';write_json(session/'session.json',manifest);print('세션:',session,flush=True)
    except BaseException as e:
        if moving:n.stop()
        manifest['status']='interrupted' if isinstance(e,KeyboardInterrupt) else 'failed';manifest['error']=str(e);write_json(session/'session.json',manifest);raise
    finally:n.destroy_node();rclpy.try_shutdown()
