#!/usr/bin/env python3
"""Teaching and single-cycle execution. No automatic motion on startup."""
import hashlib,json,math,time,collections,threading
from pathlib import Path
import rclpy,yaml
from std_msgs.msg import Int32,Bool,String
from std_srvs.srv import Trigger,SetBool
from moveit_msgs.msg import RobotState,Constraints,JointConstraint,MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan,GetPlanningScene
from rosidl_runtime_py.convert import message_to_ordereddict
from plan_cache import scene_digest,digest,start_matches
from moveit_msgs.action import ExecuteTrajectory
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from home_manager import HomeManager
from pick_place_core import RecipeStore,SLOTS,LABELS,run_sequence

class PickPlaceManager(HomeManager):
    def __init__(self):
        self.history=collections.deque(maxlen=40)
        super().__init__()
        p=Path(get_package_share_directory('soarm4_moveit_config'));d=Path(get_package_share_directory('soarm4_description'));b=Path(get_package_share_directory('soarm_bridge'))
        params=yaml.safe_load((b/'config/soarm.yaml').read_text())['servo_bridge']['ros__parameters']
        self.encoder={k:params[k] for k in ['ids','centers','signs','min_counts','max_counts','margin_lo','margin_hi']}
        # Respect both URDF angles and the stricter integer hardware count bounds.
        safe=[]
        for i,(lo,hi) in enumerate(self.limits):
            angles=sorted(params['signs'][i]*(c-params['centers'][i])*2*math.pi/4096 for c in [params['min_counts'][i]+params['margin_lo'][i],params['max_counts'][i]-params['margin_hi'][i]])
            safe.append((max(lo,angles[0]),min(hi,angles[1])))
        self.limits=safe
        signature=hashlib.sha256((d/'urdf/soarm4.urdf').read_bytes()+json.dumps(self.encoder,sort_keys=True).encode()).hexdigest()
        path=self.declare_parameter('recipe_file',str(Path.home()/'.ros/soarm4/pick_place.json')).value
        self.store=RecipeStore(path,signature,self.names,self.limits)
        self.goal_error=float(params['goal_tolerance_counts'])*2*math.pi/4096
        vp=yaml.safe_load((b/'config/soarm.yaml').read_text())['esp32_bridge']['ros__parameters'];self.hold_diff=vp.get('hold_diff',-300);self.grip_diff=vp.get('grip_diff',-400);self.release_diff=vp.get('release_diff',-50);self.release_timeout=float(vp.get('release_timeout',3.0));self.grip_timeout=float(vp.get('grip_timeout',5.0))
        self.use_vacuum_sensor=bool(self.declare_parameter('use_vacuum_sensor',True).value)
        self.diff=None;self.diff_stamp=0.;self.holding=False;self.hold_stamp=0.;self.carrying=False;self.running=False
        cb=ReentrantCallbackGroup()
        self.scene_client=self.create_client(GetPlanningScene,'/get_planning_scene',callback_group=cb)
        self.verified_plans=None
        self.reuse_plans=bool(self.declare_parameter('reuse_verified_plans',True).value)
        self.pp_pub=self.create_publisher(String,'/soarm4/pick_place_status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Int32,'/vacuum/diff',self.pressure,10,callback_group=cb)
        self.create_subscription(Bool,'/vacuum/holding',self.hold_state,10,callback_group=cb)
        self.vac={k:self.create_client(Trigger,'/vacuum/'+k+('' if self.use_vacuum_sensor else '_timed'),callback_group=cb) for k in ['grip','release']}
        for slot in SLOTS:self.create_service(Trigger,'/soarm4/pick_place/save_'+slot,lambda req,res,s=slot:self.save_slot(s,res),callback_group=cb)
        self.create_service(Trigger,'/soarm4/pick_place/check',lambda req,res:self.process(False,res),callback_group=cb)
        self.create_service(Trigger,'/soarm4/pick_place/run',lambda req,res:self.process(True,res),callback_group=cb)
        self.create_service(Trigger,'/soarm4/pick_place/list',self.describe,callback_group=cb)
        self.create_service(Trigger,'/soarm4/vacuum/grip',lambda req,res:self.manual_vacuum('grip',res),callback_group=cb)
        self.create_service(Trigger,'/soarm4/vacuum/release',lambda req,res:self.manual_vacuum('release',res),callback_group=cb)
        self.route_pub=self.create_publisher(String,'/soarm4/pick_place/route',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.edit_pub=self.create_publisher(String,'/soarm4/pick_place/edit_result',10)
        self.edit_cache=collections.OrderedDict()
        self.create_subscription(String,'/soarm4/pick_place/edit',self.edit_route,10,callback_group=cb)
        self.create_timer(.1,self.watch,callback_group=cb)
        try:self.publish_route()
        except Exception as e:self.pp_status(str(e))
        self.pp_status(('압력 확인 모드 · ' if self.use_vacuum_sensor else '센서 미사용 · 흡착/낙하 감지 불가 · ')+'자세 저장 대기 · '+str(self.store.path))
    def receive(self,msg):
        super().receive(msg)
        if all(n in dict(zip(msg.name,msg.position)) for n in self.names):self.history.append((time.monotonic(),list(self.latest)))
    def pp_status(self,text):self.pp_pub.publish(String(data=text));self.get_logger().info(text)
    def pressure(self,m):self.diff=m.data;self.diff_stamp=time.monotonic()
    def hold_state(self,m):self.holding=m.data;self.hold_stamp=time.monotonic()
    def vacuum_ready(self,require_hold=False):
        if not self.use_vacuum_sensor:return
        if len(self.get_publishers_info_by_topic('/vacuum/diff'))!=1:raise RuntimeError('ESP32 압력 발행자가 없거나 중복입니다')
        if time.monotonic()-self.diff_stamp>.5 or time.monotonic()-self.hold_stamp>.5:raise RuntimeError('압력 피드백 수신이 끊겼습니다')
        if require_hold and (not self.holding or self.diff>self.hold_diff):raise RuntimeError('흡착 유지 실패 · 이동 중단')
    def wait_fresh(self,timeout=1.):
        # Wait only while stationary. Never execute using an old cached position.
        end=time.monotonic()+timeout
        while True:
            if self.stop.is_set():raise RuntimeError(getattr(self,'failure_reason','') or '정지 요청으로 중단됐습니다')
            try:return self.store.validate(self.fresh())
            except RuntimeError:
                if time.monotonic()>=end:raise
                time.sleep(.02)
    def guard(self):
        if self.stop.is_set():raise RuntimeError(getattr(self,'failure_reason','') or '정지 요청으로 중단됐습니다')
        # Static preflight needs a fresh snapshot, not continuous servo-rate updates.
        # During motion or carrying, preserve strict feedback-loss protection.
        if self.active_goal is not None or self.carrying:self.store.validate(self.fresh())
        if self.running:self.vacuum_ready(self.carrying)
    def watch(self):
        if not self.running or self.stop.is_set():return
        try:self.guard()
        except Exception as e:
            self.failure_reason=str(e)
            self.stop.set()
            if self.active_goal:self.active_goal.cancel_goal_async()
            self.pp_status(str(e)+' · 토크/흡착 자동 해제 안 함')
    def await_future(self,f,timeout,checked=True):
        deadline=time.monotonic()+timeout
        while not f.done():
            if checked:self.guard()
            if time.monotonic()>deadline:raise RuntimeError('계획/실행 응답 시간 초과')
            time.sleep(.02)
        if checked:self.guard()
        result=f.result()
        if result is None:raise RuntimeError('서비스/액션 응답 없음')
        return result
    def manual_vacuum(self,command,res):
        if not self.enabled:res.success=False;res.message='실제 제어 비활성 모드';return res
        if not self.lock.acquire(blocking=False):res.success=False;res.message='경로 처리 중입니다. STOP 후 수동 조작하세요';return res
        try:
            if self.use_vacuum_sensor and len(self.get_publishers_info_by_topic('/vacuum/diff'))!=1:raise RuntimeError('ESP32 압력 발행자가 없거나 중복입니다')
            if command=='grip':self.vacuum_ready()
            result=self.call(self.vac[command],Trigger.Request(),max(5.,self.release_timeout+2.,self.grip_timeout+2.))
            res.success=result.success
            if result.success and command=='release':self.carrying=False
            if not self.use_vacuum_sensor:res.message=result.message
            elif command=='grip':res.message=('흡착 ON · 압력 확인: ' if result.success else '펌프 ON 요청 · 압력 미확인, 펌프는 켜져 있을 수 있음: ')+result.message
            else:res.message=('펌프 OFF·밸브 개방 후 닫힘 · 해제 확인: ' if result.success else '해제 확인 실패: ')+result.message
            self.pp_status(res.message)
        except Exception as e:res.success=False;res.message=str(e);self.pp_status(res.message)
        finally:self.lock.release()
        return res
    def route_state(self):
        data=self.store.load()
        state={'use_vacuum_sensor':self.use_vacuum_sensor,'notice':data.get('migration_notice',''),'file':str(self.store.path),'revision':data.get('revision',0),'steps':self.store.route(self.home,data,complete=False)}
        if not self.use_vacuum_sensor:
            state['notice']='센서 미사용 · 명령/시간 기준 실행, 흡착 확인 없음'
            for step in state['steps']:
                if step.get('action')=='grip':step['label']='흡착 ON · 시간 대기 (센서 미사용)'
                elif step.get('action')=='release':step['label']='놓기 · 시간 대기 (센서 미사용)'
        return state
    def publish_route(self):
        state=self.route_state();self.route_pub.publish(String(data=json.dumps(state,ensure_ascii=False)));return state
    def describe(self,req,res):
        try:res.success=True;res.message=json.dumps(self.publish_route(),ensure_ascii=False)
        except Exception as e:res.success=False;res.message=str(e)
        return res
    def capture(self):
        q=self.store.validate(self.fresh());recent=[v for t,v in self.history if time.monotonic()-t<.6]
        if len(recent)<5 or any(max(abs(a-b) for a,b in zip(v,q))>.01 for v in recent):raise RuntimeError('팔을 멈추고 1초 뒤 다시 저장하세요')
        return q
    def save_slot(self,slot,res):
        if not self.lock.acquire(blocking=False):res.success=False;res.message='다른 작업 중에는 저장할 수 없습니다';return res
        try:
            self.store.save(slot,self.capture());self.publish_route();res.success=True;res.message=('흡착' if slot=='pick' else '놓기')+' 위치 저장';self.pp_status(res.message)
        except Exception as e:res.success=False;res.message=str(e);self.pp_status(res.message)
        finally:self.lock.release()
        return res
    def edit_route(self,msg):
        request_id='';locked=False
        try:
            cmd=json.loads(msg.data);request_id=cmd.get('request_id','')
            if not isinstance(request_id,str) or not 1<=len(request_id)<=128:raise ValueError('잘못된 요청 ID')
            if request_id in self.edit_cache:
                self.edit_pub.publish(String(data=self.edit_cache[request_id]));return
            locked=self.lock.acquire(blocking=False)
            if not locked:raise RuntimeError('HOME/픽앤플레이스 처리 중에는 편집할 수 없습니다')
            q=self.capture() if cmd.get('action') in ('save_anchor','insert','replace') else None
            self.store.edit(cmd,q);self.publish_route()
            result={'request_id':request_id,'success':True,'message':'경로 저장 완료 · 홈→흡착→놓기→홈'}
        except Exception as e:result={'request_id':request_id,'success':False,'message':str(e)}
        finally:
            if locked:self.lock.release()
        text=json.dumps(result,ensure_ascii=False);self.edit_cache[request_id]=text
        while len(self.edit_cache)>100:self.edit_cache.popitem(last=False)
        self.edit_pub.publish(String(data=text));self.pp_status(result['message'])
    def make_plan(self,start,target):
        self.guard();self.store.validate(target)
        req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.pipeline_id='ompl';r.planner_id='RRTConnectkConfigDefault';r.num_planning_attempts=5;r.allowed_planning_time=8.;r.max_velocity_scaling_factor=float(self.velocity_scaling);r.max_acceleration_scaling_factor=float(self.acceleration_scaling)
        r.start_state=RobotState();r.start_state.joint_state=JointState(name=self.names,position=start)
        goal=Constraints();goal.joint_constraints=[JointConstraint(joint_name=n,position=q,tolerance_above=1e-6,tolerance_below=1e-6,weight=1.) for n,q in zip(self.names,target)];r.goal_constraints=[goal]
        if not self.plan.wait_for_service(timeout_sec=2):raise RuntimeError('MoveIt 계획 서비스 없음')
        response=self.await_future(self.plan.call_async(req),40).motion_plan_response
        if response.error_code.val!=MoveItErrorCodes.SUCCESS:raise RuntimeError('충돌 회피 경로 계획 실패: '+str(response.error_code.val))
        jt=response.trajectory.joint_trajectory
        if not jt.points or len(jt.joint_names)!=4 or set(jt.joint_names)!=set(self.names):raise RuntimeError('잘못된 계획 경로')
        indices=[jt.joint_names.index(n) for n in self.names]
        for point in jt.points:self.store.validate([point.positions[i] for i in indices])
        if max(abs(start[k]-jt.points[0].positions[i]) for k,i in enumerate(indices))>math.radians(1.0):raise RuntimeError('계획 시작점 불일치')
        if max(abs(target[k]-jt.points[-1].positions[i]) for k,i in enumerate(indices))>.001:raise RuntimeError('계획 목표점 불일치')
        return response.trajectory
    def scene_signature(self):
        if not self.scene_client.wait_for_service(timeout_sec=2):raise RuntimeError('충돌 장면 확인 서비스 없음 · 계획 재사용 금지')
        req=GetPlanningScene.Request()
        # Exclude moving joint state and decorative colors, include all collision-relevant content.
        req.components.components=1|4|8|16|32|64|128|256
        scene=self.await_future(self.scene_client.call_async(req),5).scene
        return scene_digest(message_to_ordereddict(scene))
    def move(self,target,trajectory=None,scene_token=None):
        self.guard();actual=self.wait_fresh()
        if scene_token is not None and self.scene_signature()!=scene_token:
            self.verified_plans=None;raise RuntimeError('장애물/충돌 장면이 바뀌었습니다. 전체 경로를 다시 검사하세요')
        if max(abs(a-b) for a,b in zip(actual,target))<=math.pi/4096:
            self.pp_status('이미 목표 자세 · 불필요한 이동 생략');return
        if trajectory is None or not start_matches(self.names,trajectory.joint_trajectory,actual):
            self.pp_status('현재 시작 자세에서 재계획 중')
            trajectory=self.make_plan(actual,target)
        else:self.pp_status('검증된 경로 재사용 · 시작 자세/충돌 장면 확인')
        self.guard();jt=trajectory.joint_trajectory
        if not start_matches(self.names,jt,self.wait_fresh()):raise RuntimeError('계획 중 실제 자세 변화가 1도를 초과했습니다 · 이동 중단')
        if scene_token is not None and self.scene_signature()!=scene_token:
            self.verified_plans=None;raise RuntimeError('실행 전 충돌 장면 변경 감지 · 이동 중단')
        if not self.execute.wait_for_server(timeout_sec=2):raise RuntimeError('실행 액션 없음')
        if not start_matches(self.names,jt,self.wait_fresh()):raise RuntimeError('실행 직전 시작 자세 변경 · 이동 중단')
        self.guard()
        goal=ExecuteTrajectory.Goal(trajectory=trajectory,controller_names=['arm_controller'])
        # Always obtain acceptance so a simultaneous STOP cannot orphan an accepted action.
        abandoned=threading.Event();pending=self.execute.send_goal_async(goal)
        def late_accept(f):
            if abandoned.is_set():
                try:
                    h=f.result()
                    if h and h.accepted:h.cancel_goal_async()
                except Exception:pass
        pending.add_done_callback(late_accept)
        try:handle=self.await_future(pending,5,checked=False)
        except Exception:
            abandoned.set()
            if pending.done():late_accept(pending)
            raise
        self.active_goal=handle
        if not handle.accepted:raise RuntimeError('제어기 실행 거부')
        if self.stop.is_set():handle.cancel_goal_async()
        t=jt.points[-1].time_from_start;result=self.await_future(handle.get_result_async(),max(30.,t.sec+t.nanosec*1e-9+15.))
        if result.result.error_code.val!=MoveItErrorCodes.SUCCESS:raise RuntimeError('이동 실행 실패: '+str(result.result.error_code.val))
        self.active_goal=None
        if max(abs(a-b) for a,b in zip(self.fresh(),target))>self.goal_error+1e-6:raise RuntimeError('목표 자세 도착 오차 초과')
    def grip(self):
        self.guard();res=self.call(self.vac['grip'],Trigger.Request(),max(5.,self.grip_timeout+2.));self.guard()
        if not res.success:raise RuntimeError('흡착 실패: '+res.message)
        if not self.use_vacuum_sensor:
            self.carrying=True;self.pp_status('흡착 명령/시간 대기 완료 · 센서 확인 없음');return
        deadline=time.monotonic()+1.
        while not (self.holding and self.diff<=self.grip_diff):
            self.guard()
            if time.monotonic()>deadline:raise RuntimeError('흡착 피드백 확인 시간 초과')
            time.sleep(.02)
        deadline=time.monotonic()+.5
        while time.monotonic()<deadline:
            self.guard();self.vacuum_ready(True);time.sleep(.02)
        self.carrying=True;self.pp_status('흡착 유지 확인')
    def release(self):
        self.guard();self.carrying=False;start=time.monotonic();res=self.call(self.vac['release'],Trigger.Request(),max(5.,self.release_timeout+2.));self.guard()
        if not res.success:raise RuntimeError('해제 실패: '+res.message)
        if not self.use_vacuum_sensor:
            self.pp_status('해제 명령/밸브 대기 완료 · 센서 확인 없음');return
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            self.guard()
            if self.diff_stamp>start and self.diff>=self.release_diff and not self.holding:return
            time.sleep(.02)
        raise RuntimeError('밸브 해제 후 잔압 해소가 확인되지 않았습니다')
    def process(self,execute,res):
        if execute and not self.enabled:res.success=False;res.message='실제 실행 비활성 모드';return res
        if not self.lock.acquire(blocking=False):res.success=False;res.message='HOME/경로 작업 처리 중';return res
        self.stop.clear();self.failure_reason=''
        try:
            self.velocity_scaling,self.acceleration_scaling=self.read_rviz_scaling()
            self.pp_status(f'RViz 비율 적용 · 속도 {self.velocity_scaling:.2f} / 가속도 {self.acceleration_scaling:.2f}')
            data=self.store.load(complete=True);steps=self.store.route(self.home,data);motion_steps=[step for step in steps if step['kind']!='action'];self.guard();self.wait_fresh()
            if execute:
                self.vacuum_ready()
                if self.carrying or (self.use_vacuum_sensor and self.holding):raise RuntimeError('이미 흡착 중입니다. 물체를 내려놓고 해제한 뒤 시작하세요')
                self.running=True;self.carrying=False
                held=self.call(self.torque,SetBool.Request(data=True))
                if not held.success:raise RuntimeError(held.message)
            # Reuse only plans validated for this exact route, model/calibration and collision scene.
            scene_token=self.scene_signature()
            key=digest({'recipe':data,'home':self.home,'velocity':self.velocity_scaling,'acceleration':self.acceleration_scaling})
            cache=self.verified_plans if self.reuse_plans else None
            if cache and cache['key']==key and cache['scene']==scene_token:
                plans=cache['plans'];self.pp_status(f'검증된 {len(plans)}개 구간 계획 재사용')
            else:
                self.verified_plans=None;plans=[];start=self.wait_fresh()
                for step in motion_steps:
                    self.pp_status(step['label']+' 경로 사전 검사 중');plans.append(self.make_plan(start,step['q']));start=step['q']
                if self.scene_signature()!=scene_token:raise RuntimeError('사전 검사 중 충돌 장면이 바뀌었습니다. 다시 검사하세요')
                self.verified_plans={'key':key,'scene':scene_token,'plans':plans}
            if not execute:
                # First segment may have been cached at a different measured start.
                if not start_matches(self.names,plans[0].joint_trajectory,self.wait_fresh()):
                    plans=list(plans);plans[0]=self.make_plan(self.wait_fresh(),steps[0]['q'])
                    if self.scene_signature()!=scene_token:raise RuntimeError('검사 중 충돌 장면 변경')
                    self.verified_plans={'key':key,'scene':scene_token,'plans':plans}
                res.success=True;res.message=f'{len(steps)}개 단계 경로 검사 통과 · 이동/펌프 명령 없음';self.pp_status(res.message);return res
            index=0
            def movement(step):
                nonlocal index
                self.pp_status(step['label']+' 이동 중');self.move(step['q'],plans[index],scene_token);index+=1
            run_sequence(steps,movement,self.grip,self.release,self.guard)
            res.success=True;res.message='픽앤플레이스 1회 완료 · 홈 도착, 펌프 OFF/밸브 해제 명령 완료, 토크 유지'+('' if self.use_vacuum_sensor else ' · 센서 확인 없음');self.pp_status(res.message)
        except Exception as e:
            if self.active_goal:
                try:
                    self.await_future(self.active_goal.cancel_goal_async(),3,checked=False)
                    self.await_future(self.active_goal.get_result_async(),5,checked=False)
                except Exception:pass
            res.success=False;res.message=str(e)+' · 자동 재시작/토크 해제/흡착 해제 안 함';self.pp_status(res.message)
        finally:self.running=False;self.active_goal=None;self.lock.release()
        return res

def main():
    rclpy.init();n=PickPlaceManager();ex=MultiThreadedExecutor(num_threads=8);ex.add_node(n)
    try:ex.spin()
    except KeyboardInterrupt:pass
    finally:
        n.stop.set()
        if n.active_goal:n.active_goal.cancel_goal_async()
        ex.shutdown(timeout_sec=5.);n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
