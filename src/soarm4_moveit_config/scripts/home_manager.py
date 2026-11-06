#!/usr/bin/env python3
"""Human-triggered home planning/execution. No movement on startup."""
import json,math,threading,time
import yaml
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import RobotState,Constraints,JointConstraint,MoveItErrorCodes
from moveit_msgs.action import ExecuteTrajectory

class HomeManager(Node):
    def __init__(self):
        super().__init__('soarm4_home_manager');self.enabled=self.declare_parameter('allow_execution',False).value
        p=Path(get_package_share_directory('soarm4_moveit_config'));model=json.loads((p/'audit_model/config/model.json').read_text())
        bridge_params=yaml.safe_load((Path(get_package_share_directory('soarm_bridge'))/'config/soarm.yaml').read_text())['servo_bridge']['ros__parameters']
        self.feedback_timeout=float(self.declare_parameter('feedback_timeout',bridge_params.get('feedback_timeout',0.5)).value)
        if not math.isfinite(self.feedback_timeout) or self.feedback_timeout<=0:raise ValueError('Invalid feedback_timeout')
        self.get_logger().info(f'엔코더 피드백 허용 시간: {self.feedback_timeout:.3f}초')
        defaults=yaml.safe_load((p/'config/joint_limits.yaml').read_text())
        self.velocity_scaling=float(self.declare_parameter('home_velocity_scaling',defaults.get('default_velocity_scaling_factor',0.1)).value)
        self.acceleration_scaling=float(self.declare_parameter('home_acceleration_scaling',defaults.get('default_acceleration_scaling_factor',0.1)).value)
        if not (0.<self.velocity_scaling<=1. and 0.<self.acceleration_scaling<=1.):raise ValueError('HOME scaling must be in (0, 1]')
        self.home_velocity_scaling=self.velocity_scaling;self.home_acceleration_scaling=self.acceleration_scaling
        self.names=[j['name'] for j in model['joints']];self.home=model['poses']['home'];self.limits=[(float(j['limit']['lower']),float(j['limit']['upper'])) for j in model['joints']]
        self.latest=None;self.stamp=0.;self.lock=threading.Lock();self.stop=threading.Event();self.active_goal=None
        cb=ReentrantCallbackGroup();self.create_subscription(JointState,'/joint_states',self.receive,10,callback_group=cb)
        self.plan=self.create_client(GetMotionPlan,'/plan_kinematic_path',callback_group=cb)
        self.torque=self.create_client(SetBool,'/servo/torque',callback_group=cb)
        self.toggle=self.create_client(Trigger,'/servo/toggle_torque',callback_group=cb)
        self.rviz_scaling=self.create_client(Trigger,'/soarm4/rviz_scaling',callback_group=cb)
        self.driver_stop=self.create_client(Trigger,'/servo/stop',callback_group=cb)
        self.execute=ActionClient(self,ExecuteTrajectory,'/execute_trajectory',callback_group=cb)
        self.status_pub=self.create_publisher(String,'/soarm4/home_status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger,'/soarm4/go_home',self.go_home,callback_group=cb)
        self.create_service(Trigger,'/soarm4/stop_home',self.stop_home,callback_group=cb)
        self.create_service(Trigger,'/soarm4/toggle_torque',self.release_torque,callback_group=cb)
        self.status('홈 대기 · URDF 관절 리미트 적용' if self.enabled else '미리보기 모드 · 실제 홈 이동 비활성')
    def status(self,text):self.status_pub.publish(String(data=text));self.get_logger().info(text)
    def receive(self,msg):
        values=dict(zip(msg.name,msg.position))
        if all(n in values for n in self.names):self.latest=[values[n] for n in self.names];self.stamp=time.monotonic()
    def fresh(self):
        if self.latest is None:raise RuntimeError('실제 엔코더 데이터가 없습니다')
        age=time.monotonic()-self.stamp
        if age>self.feedback_timeout:raise RuntimeError(f'엔코더 피드백 지연 {age:.3f}초 (허용 {self.feedback_timeout:.3f}초)')
        return list(self.latest)
    def wait(self,future,seconds):
        done=threading.Event();future.add_done_callback(lambda f:done.set())
        if not done.wait(seconds):raise RuntimeError('서비스/액션 응답 시간 초과')
        result=future.result()
        if result is None:raise RuntimeError('응답 없음')
        return result
    def call(self,client,req,seconds=5):
        if not client.wait_for_service(timeout_sec=2):raise RuntimeError('서비스 연결 없음: '+client.srv_name)
        return self.wait(client.call_async(req),seconds)
    def stop_home(self,req,res):
        self.stop.set()
        if self.active_goal:self.active_goal.cancel_goal_async()
        if self.driver_stop.service_is_ready():self.driver_stop.call_async(Trigger.Request())
        res.success=True;res.message='정지 요청 · 토크는 유지';self.status(res.message);return res
    def release_torque(self,req,res):
        if not self.enabled:
            res.success=False;res.message='미리보기 모드에서는 토크를 변경하지 않습니다';return res
        if not self.lock.acquire(blocking=False):
            res.success=False;res.message='홈 처리 중입니다. STOP 후 정지가 완료되면 다시 해제하세요';return res
        try:
            result=self.call(self.toggle,Trigger.Request())
            res.success=result.success
            res.message=result.message
            self.status(res.message)
        except Exception as exc:
            res.success=False;res.message=str(exc)
        finally:self.lock.release()
        return res
    def read_rviz_scaling(self):
        response=self.call(self.rviz_scaling,Trigger.Request(),3.)
        if not response.success:raise RuntimeError(response.message)
        data=json.loads(response.message);velocity=data.get('velocity');acceleration=data.get('acceleration')
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not 0<v<=1 for v in (velocity,acceleration)):
            raise RuntimeError('RViz 속도/가속도 비율은 0 초과 1 이하로 설정하세요')
        return float(velocity),float(acceleration)

    def go_home(self,req,res):
        if not self.enabled:res.success=False;res.message='allow_execution:=true 모드에서만 홈 이동 가능합니다';return res
        if not self.lock.acquire(blocking=False):res.success=False;res.message='이미 홈 이동을 처리 중입니다';return res
        self.stop.clear()
        try:
            self.home_velocity_scaling,self.home_acceleration_scaling=self.read_rviz_scaling()
            q=self.fresh()
            for name,value,target,(lo,hi) in zip(self.names,q,self.home,self.limits):
                if not math.isfinite(value) or not lo-1e-9<=value<=hi+1e-9:raise RuntimeError(f'{name} 현재 각도 {math.degrees(value):.2f}°가 계획 리미트 밖입니다')
            # Hold the measured position before planning, so the arm cannot sag while torque is off.
            self.status('현재 자세 유지 중…');held=self.call(self.torque,SetBool.Request(data=True))
            if not held.success:raise RuntimeError(held.message)
            if self.stop.wait(.1):raise RuntimeError('홈 이동이 취소됐습니다')
            q=self.fresh()
            if max(abs(a-b) for a,b in zip(q,self.home))<1e-6:
                res.success=True;res.message='이미 홈입니다 · 현재 위치 유지';self.status(res.message);return res
            self.status('계산중: 홈 복귀 Pilz PTP · 충돌/관절 점검 포함')
            reqplan=GetMotionPlan.Request();r=reqplan.motion_plan_request;r.group_name='arm';r.pipeline_id='ompl';r.planner_id='RRTConnectkConfigDefault';r.num_planning_attempts=1;r.allowed_planning_time=8.;r.max_velocity_scaling_factor=float(self.home_velocity_scaling);r.max_acceleration_scaling_factor=float(self.home_acceleration_scaling)
            r.start_state=RobotState();r.start_state.joint_state=JointState(name=self.names,position=q)
            goal=Constraints()
            for name,value in zip(self.names,self.home):goal.joint_constraints.append(JointConstraint(joint_name=name,position=value,tolerance_above=1e-6,tolerance_below=1e-6,weight=1.))
            r.goal_constraints=[goal]
            plan=None
            for pipeline,planner in [('pilz_industrial_motion_planner','PTP'),('ompl','RRTConnectkConfigDefault')]:
                if self.stop.is_set():raise RuntimeError('홈 이동이 취소됐습니다')
                r.pipeline_id=pipeline;r.planner_id=planner
                plan=self.call(self.plan,reqplan,30).motion_plan_response
                if plan.error_code.val==MoveItErrorCodes.SUCCESS:
                    self.status('안전점검 통과: 홈 경로 '+planner);break
                if planner=='PTP':
                    self.status('안전점검 실패: Pilz PTP ('+str(plan.error_code.val)+') · 재탐색합니다: RRT-Connect')

            if self.stop.is_set():raise RuntimeError('홈 이동이 취소됐습니다')
            if plan.error_code.val!=MoveItErrorCodes.SUCCESS:raise RuntimeError('홈 경로 계획 실패: '+str(plan.error_code.val))
            jt=plan.trajectory.joint_trajectory
            if not jt.points or set(jt.joint_names)!=set(self.names):raise RuntimeError('유효한 홈 경로가 없습니다')
            actual=self.fresh();index=[jt.joint_names.index(n) for n in self.names]
            if any(abs(actual[i]-jt.points[0].positions[index[i]])>8*2*math.pi/4096 for i in range(4)):raise RuntimeError('계획 중 시작 자세가 바뀌었습니다 · 다시 요청하세요')
            for pt in jt.points:
                if any(not math.isfinite(pt.positions[index[i]]) or not self.limits[i][0]-1e-9<=pt.positions[index[i]]<=self.limits[i][1]+1e-9 for i in range(4)):raise RuntimeError('계획 경로가 URDF 관절 리미트를 넘었습니다')
            if self.stop.is_set():raise RuntimeError('홈 이동이 취소됐습니다')
            if not self.execute.wait_for_server(timeout_sec=2):raise RuntimeError('실제 실행 액션이 없습니다')
            self.status('홈 이동 중…');goalexec=ExecuteTrajectory.Goal();goalexec.trajectory=plan.trajectory;goalexec.controller_names=['arm_controller']
            handle=self.wait(self.execute.send_goal_async(goalexec),5);self.active_goal=handle
            if not handle.accepted:raise RuntimeError('제어기가 홈 실행을 거부했습니다')
            if self.stop.is_set():handle.cancel_goal_async()
            duration=jt.points[-1].time_from_start.sec+jt.points[-1].time_from_start.nanosec*1e-9
            result=self.wait(handle.get_result_async(),max(30.,duration+15.))
            if result.result.error_code.val!=MoveItErrorCodes.SUCCESS:raise RuntimeError('홈 이동 중단: '+str(result.result.error_code.val))
            actual=self.fresh()
            if any(abs(a-b)>5*2*math.pi/4096 for a,b in zip(actual,self.home)):raise RuntimeError('종료됐지만 홈 도착 오차가 큽니다')
            res.success=True;res.message='홈 도착 · 오프셋 보정 J2 180° / J4 0°, 토크 유지';self.status(res.message)
        except Exception as exc:
            if self.active_goal:
                self.active_goal.cancel_goal_async()
                if self.driver_stop.service_is_ready():self.driver_stop.call_async(Trigger.Request())
            res.success=False;res.message=str(exc);self.status('홈 중단: '+res.message)
        finally:self.active_goal=None;self.lock.release()
        return res

def main():
    rclpy.init();n=HomeManager();ex=MultiThreadedExecutor(num_threads=4);ex.add_node(n)
    try:ex.spin()
    except KeyboardInterrupt:pass
    finally:
        n.stop.set()
        if n.active_goal:n.active_goal.cancel_goal_async()
        ex.shutdown(timeout_sec=3.);n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
