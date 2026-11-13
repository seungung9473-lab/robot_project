import json,math,threading,time,os,tempfile
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String,Header
from std_srvs.srv import Trigger,SetBool
from .controller import Controller,SDKBus

class TurntableBridge(Node):
    def __init__(self,bus=None):
        super().__init__('xl330_turntable')
        p=self.declare_parameter
        port=p('port','/dev/turntable_bus').value;baud=p('baud',57600).value;sid=p('motor_id',1).value
        self.controller=Controller(bus or SDKBus(port,baud,sid),p('read_only',True).value,p('profile_velocity',50).value,p('profile_acceleration',10).value)
        self.zero_file=Path(p('zero_file',str(Path.home()/'.ros/soarm4/turntable_zero.json')).value).expanduser();self.zero=None
        self.calibration={'port':port,'id':sid,'model':self.controller.info['model'],'drive_mode':self.controller.info['drive_mode']}
        if self.zero_file.exists():
            data=json.loads(self.zero_file.read_text())
            if data.get('calibration')==self.calibration and type(data.get('zero_count')) is int:self.zero=data['zero_count']
        self.active=threading.Lock();self.stop_event=threading.Event();self.stop_generation=0;self.pub=self.create_publisher(String,'/turntable/state',10);self.joints=self.create_publisher(JointState,'/turntable/joint_states',10);cb=ReentrantCallbackGroup()
        self.create_timer(.1,self.poll,callback_group=cb)
        self.create_service(Trigger,'/turntable/read',self.read,callback_group=cb)
        self.create_service(Trigger,'/turntable/set_zero',self.set_zero,callback_group=cb)
        self.create_service(SetBool,'/turntable/torque',self.torque,callback_group=cb)
        self.create_service(Trigger,'/turntable/stop',self.stop,callback_group=cb)
        self.action=ActionServer(self,FollowJointTrajectory,'/turntable/follow_joint_trajectory',execute_callback=self.execute,goal_callback=self.goal,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=cb)
        self.get_logger().info('턴테이블 연결: '+json.dumps(self.controller.info)+' · read_only='+str(self.controller.read_only)+' · 기동 시 쓰기 없음')
    def state(self):
        s=self.controller.snapshot();s.update(self.controller.info);s['zero_valid']=self.zero is not None;s['zero_count']=self.zero;s['stamp_ros_ns']=self.get_clock().now().nanoseconds;s['angle_deg']=None if self.zero is None else (s['position_count']-self.zero)*360/4096;s['read_only']=self.controller.read_only;s['stop_generation']=self.stop_generation
        return s
    def poll(self):
        try:
            s=self.state();self.pub.publish(String(data=json.dumps(s)))
            if self.zero is not None:self.joints.publish(JointState(header=Header(stamp=self.get_clock().now().to_msg()),name=['turntable_joint'],position=[math.radians(s['angle_deg'])],velocity=[s['velocity_raw']*.229*2*math.pi/60]))
        except Exception as e:self.get_logger().error(str(e),throttle_duration_sec=3.)
    def read(self,req,res):
        try:res.message=json.dumps(self.state());res.success=True
        except Exception as e:res.success=False;res.message=str(e)
        return res
    def set_zero(self,req,res):
        if not self.active.acquire(False):res.success=False;res.message='회전 처리 중에는 영점을 바꿀 수 없습니다';return res
        try:
            a=self.controller.snapshot();time.sleep(.35);b=self.controller.snapshot()
            if a['moving'] or b['moving'] or abs(a['position_count']-b['position_count'])>2:raise RuntimeError('정지 후 영점을 저장하세요')
            self.zero_file.parent.mkdir(parents=True,exist_ok=True);data={'zero_count':b['position_count'],'calibration':self.calibration};fd,tmp=tempfile.mkstemp(dir=self.zero_file.parent,prefix='.zero-')
            try:
                with os.fdopen(fd,'w') as f:json.dump(data,f);f.flush();os.fsync(f.fileno())
                os.replace(tmp,self.zero_file)
            finally:
                if os.path.exists(tmp):os.unlink(tmp)
            self.zero=b['position_count'];res.success=True;res.message='현재 위치를 소프트웨어 0도로 저장 · 모터 레지스터 변경 없음'
        except Exception as e:res.success=False;res.message=str(e)
        finally:self.active.release()
        return res
    def torque(self,req,res):
        if not self.active.acquire(False):res.success=False;res.message='회전 처리 중에는 토크를 바꿀 수 없습니다';return res
        try:self.controller.torque(req.data);res.success=True;res.message='토크 '+('ON' if req.data else 'OFF')
        except Exception as e:res.success=False;res.message=str(e)
        finally:self.active.release()
        return res
    def stop(self,req,res):
        self.stop_generation+=1
        self.stop_event.set()
        try:self.controller.stop();res.success=True;res.message='현재 위치 유지 명령 전송 · 토크 유지'
        except Exception as e:res.success=False;res.message='정지 명령 실패: '+str(e)+' · 통신 단절 시 물리 정지를 보장할 수 없습니다'
        return res
    def goal(self,req):
        if self.controller.read_only or self.zero is None or req.trajectory.joint_names!=['turntable_joint'] or len(req.trajectory.points)!=1:return GoalResponse.REJECT
        q=req.trajectory.points[0]
        if len(q.positions)!=1 or not math.isfinite(q.positions[0]) or not 0<q.time_from_start.sec+q.time_from_start.nanosec/1e9<=120:return GoalResponse.REJECT
        if not self.active.acquire(False):return GoalResponse.REJECT
        self.stop_event.clear()
        return GoalResponse.ACCEPT
    def execute(self,gh):
        result=FollowJointTrajectory.Result();started=False
        try:
            pt=gh.request.trajectory.points[0]
            if gh.is_cancel_requested:gh.canceled();return result
            if self.stop_event.is_set():gh.abort();result.error_code=-4;result.error_string='회전 전 정지 요청';return result
            target=self.controller.goto(math.degrees(pt.positions[0]),self.zero);started=True;deadline=time.monotonic()+pt.time_from_start.sec+pt.time_from_start.nanosec/1e9;stable_since=None
            while True:
                if self.stop_event.is_set() or gh.is_cancel_requested:
                    self.controller.stop();result.error_code=-4;result.error_string='정지/취소 요청'
                    if gh.is_cancel_requested:gh.canceled()
                    else:gh.abort()
                    return result
                s=self.controller.snapshot()
                if s['hardware_error'] or not s['torque']:raise RuntimeError('모터 오류 또는 토크 OFF')
                feedback=FollowJointTrajectory.Feedback();feedback.joint_names=['turntable_joint'];feedback.actual.positions=[(s['position_count']-self.zero)*2*math.pi/4096];feedback.desired.positions=[pt.positions[0]];gh.publish_feedback(feedback)
                stationary=abs(s['position_count']-target)<=6 and not s['moving'] and abs(s['velocity_raw'])<=1
                stable_since=(stable_since or time.monotonic()) if stationary else None
                if stable_since is not None and time.monotonic()-stable_since>=.5:gh.succeed();result.error_code=0;result.error_string='목표 도착/0.5초 정지 확인';return result
                if time.monotonic()>deadline:raise RuntimeError('도착/정지 확인 시간 초과')
                time.sleep(.05)
        except Exception as e:
            if started:
                try:self.controller.stop()
                except Exception as hold:self.get_logger().error('오류 후 위치 유지 실패: '+str(hold))
            result.error_code=-4;result.error_string=str(e);gh.abort();return result
        finally:self.active.release()
    def destroy_node(self):
        self.controller.bus.close();super().destroy_node()

def main():
    rclpy.init();node=TurntableBridge();ex=MultiThreadedExecutor(num_threads=4);ex.add_node(node)
    try:ex.spin()
    except KeyboardInterrupt:pass
    finally:
        # No torque/position write on shutdown. Stop explicitly before terminating.
        ex.shutdown(timeout_sec=3);node.destroy_node();rclpy.try_shutdown()
