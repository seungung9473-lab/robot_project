"""Six-stop, ten-cycle measured encoder test. Pump/valve are never controlled."""
import argparse,json,sys,time,csv
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.srv import GetMotionPlan,GetStateValidity,GetPlanningScene
from moveit_msgs.msg import Constraints,JointConstraint,PlanningSceneComponents
from moveit_msgs.action import ExecuteTrajectory
from rosidl_runtime_py.convert import message_to_ordereddict
from .tcp_error_test import Test,state
from .cup_goal import Arm,tcp_point,solve,NAMES
from .core import load,matrix
from .motor_limits import MotorLimits
from .tool_clearance import ToolClearance
from .tcp_error_metrics import tcp_errors
from std_msgs.msg import String

class CycleTest(Test):
    def __init__(self,record):
        super().__init__(record);self.arm=Arm();self.motor=MotorLimits();self.motor.intersect(self.arm)
        root=Path(__file__).resolve().parents[1];self.wc=load(root/'config/workcell.yaml');self.W=matrix(self.wc['T_table_world']);self.tcp=tcp_point(self.wc)
        self.clearance=ToolClearance(self.arm,self.wc,self.W,self.tcp);self.active_goal=None
        self.client=ActionClient(self,ExecuteTrajectory,'/execute_trajectory')
        self.last_fine_status=None
        self.create_subscription(String,'/servo/fine_correction_status',self.fine_status,10)
    def fine_status(self,msg):
        try:self.last_fine_status=json.loads(msg.data)
        except ValueError:return
        self.record.event('endpoint_fine_status',report=self.last_fine_status)
    def fresh_wait(self):
        end=time.monotonic()+5
        while time.monotonic()<end:
            before=self.received;rclpy.spin_once(self,timeout_sec=.05)
            if self.received>before:return self.fresh()
        raise RuntimeError('Fresh encoder data unavailable')
    def fk(self,q):return (self.W@self.arm.fk(q)@self.tcp)[:3]
    def valid(self,q):
        if not self.motor.valid(q) or np.any(q<self.arm.lo-1e-8) or np.any(q>self.arm.hi+1e-8) or not self.clearance.check(q)[0]:raise RuntimeError('Motor/URDF limit or 20mm tool clearance rejection')
        req=GetStateValidity.Request();req.group_name='arm';req.robot_state=state(q)
        res=self.call('/check_state_validity',GetStateValidity,req)
        if not res.valid:raise RuntimeError('MoveIt collision: '+str([(c.contact_body_1,c.contact_body_2) for c in res.contacts]))
    def review(self,traj,start,goal):
        jt=traj.joint_trajectory
        if not jt.points or sorted(jt.joint_names)!=sorted(NAMES):raise RuntimeError('Invalid trajectory')
        idx=[jt.joint_names.index(k) for k in NAMES];qs=[np.array(p.positions)[idx] for p in jt.points]
        if np.max(np.abs(qs[0]-start))>.002 or np.max(np.abs(qs[-1]-goal))>.002:raise RuntimeError('Trajectory endpoints mismatch')
        minimum=float('inf')
        for a,b in zip(qs,qs[1:]+[qs[-1]]):
            for u in np.linspace(0,1,max(2,int(np.ceil(np.max(np.abs(a-b))/np.radians(.25)))+1)):
                q=a*(1-u)+b*u;ok,d,_=self.clearance.check(q);minimum=min(minimum,d)
                if not self.motor.valid(q) or np.any(q<self.arm.lo-1e-8) or np.any(q>self.arm.hi+1e-8) or not ok:raise RuntimeError('Path rejected by limits/tool clearance')
        for q in qs:self.valid(q)
        return minimum
    def plan_path(self,start,goal):
        self.valid(start);self.valid(goal);last=None
        for attempt in range(3):
            req=GetMotionPlan.Request();p=req.motion_plan_request;p.group_name='arm';p.pipeline_id='ompl';p.planner_id='RRTConnectkConfigDefault';p.start_state=state(start);p.allowed_planning_time=5.;p.num_planning_attempts=1;p.max_velocity_scaling_factor=.5;p.max_acceleration_scaling_factor=.1
            c=Constraints();c.joint_constraints=[JointConstraint(joint_name=k,position=float(v),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for k,v in zip(NAMES,goal)];p.goal_constraints=[c]
            response=self.call('/plan_kinematic_path',GetMotionPlan,req).motion_plan_response
            if response.error_code.val!=1:last=RuntimeError('MoveIt plan failed '+str(response.error_code.val));continue
            try:distance=self.review(response.trajectory,start,goal)
            except RuntimeError as exc:last=exc;continue
            self.record.event('path_reviewed',minimum_tool_clearance_mm=distance*1000,planning_attempt=attempt+1)
            return response.trajectory
        raise RuntimeError('No reviewed path: '+str(last))
    def execute_path(self,traj,start,goal):
        actual=self.fresh_wait()
        if np.max(np.abs(actual-start))>np.radians(.5):
            traj=self.plan_path(actual,goal);start=actual
        self.review(traj,start,goal);actual=self.fresh_wait()
        if np.max(np.abs(actual-start))>np.radians(.5):raise RuntimeError('Start changed during review')
        if not self.client.wait_for_server(timeout_sec=5):raise RuntimeError('Execution action unavailable')
        self.last_fine_status=None
        g=ExecuteTrajectory.Goal();g.trajectory=traj;f=self.client.send_goal_async(g);rclpy.spin_until_future_complete(self,f,timeout_sec=10)
        if not f.done() or not f.result().accepted:raise RuntimeError('Execution rejected')
        self.active_goal=f.result();f=self.active_goal.get_result_async();t=traj.joint_trajectory.points[-1].time_from_start;duration=t.sec+t.nanosec/1e9
        rclpy.spin_until_future_complete(self,f,timeout_sec=max(30.,duration*2+10))
        if not f.done():raise RuntimeError('Execution timed out')
        if f.result().result.error_code.val!=1:raise RuntimeError('Execution aborted '+str(f.result().result.error_code.val))
        self.active_goal=None
        end=time.monotonic()+.5
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.03)
        samples=[];end=time.monotonic()+.3
        while time.monotonic()<end:
            before=self.received;rclpy.spin_once(self,timeout_sec=.03)
            if self.received>before:samples.append(self.fresh().tolist())
        if not samples:raise RuntimeError('No settled encoder measurements')
        return np.median(samples,axis=0),samples

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true');ap.add_argument('--cycles',type=int,default=10);ap.add_argument('--height-mm',type=float,default=30);args=ap.parse_args()
    if args.height_mm<30 or not 1<=args.cycles<=10:ap.error('Requires height >=30mm, 1..10 cycles')
    sys.path.insert(0,str(Path.home()/'robot_ws/automation/reconstruction'))
    from test_data_recorder import start_trial,atomic_json
    rec=start_trial('six_stop_cycle_encoder_measurement',physical=args.execute,source=__file__);rclpy.init();n=CycleTest(rec);results=[]
    try:
        initial=n.fresh_wait();req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.WORLD_OBJECT_NAMES
        scene=n.call('/get_planning_scene',GetPlanningScene,req).scene;ids={o.id for o in scene.world.collision_objects}
        required={'workcell_table_guard','workcell_turntable','workcell_pad1','workcell_pad2','workcell_power_box'}
        if not required<=ids:raise RuntimeError('Required workcell collision objects missing')
        root=Path(__file__).resolve().parents[1];poses=yaml.safe_load((root/'config/named_poses.yaml').read_text());home=np.array([0,np.pi,n.arm.lo[2],0]);trash=np.array(poses['poses']['trash_drop']['positions_rad'])
        goals={'home':home,'trash':trash};targets={'home':n.fk(home),'trash':n.fk(trash)}
        surfaces={'turntable':n.wc['turntable']['axis_point_table_m'],'left_ID5':n.wc['pads']['pad2']['center_table_m'],'right_ID6':n.wc['pads']['pad1']['center_table_m']}
        for name,p in surfaces.items():
            target=np.array(p)+[0,0,args.height_mm/1000];q=solve(n.arm,n.wc,target,tcp=n.tcp,seeds=30)
            if q is None:raise RuntimeError('No IK for '+name)
            n.valid(q);goals[name]=q;targets[name]=target
        sequence=['turntable','left_ID5','home','right_ID6','home','trash'];cache={};plans=[];start=initial
        # All edges, including trash -> next cycle turntable, reviewed before motion.
        for cycle in range(1,args.cycles+1):
            for stop,name in enumerate(sequence,1):
                goal=goals[name];key=(tuple(np.round(start,8)),name)
                if key not in cache:cache[key]=n.plan_path(start,goal)
                plans.append((cycle,stop,name,start.copy(),goal.copy(),cache[key]));start=goal
        artifact=rec.directory/'cycle_plans.json';atomic_json(artifact,{'sequence':sequence,'cycles':args.cycles,'height_mm':args.height_mm,'velocity':.5,'acceleration':.1,'motor_config_sha256':n.motor.sha256,'goals_rad':{k:v.tolist() for k,v in goals.items()},'targets_table_m':{k:v.tolist() for k,v in targets.items()},'initial_rad':initial.tolist(),'unique_paths':[message_to_ordereddict(x) for x in cache.values()],'pump_valve_controlled':False});rec.attach(artifact)
        print('All cycle paths reviewed:',len(cache),'unique routes',flush=True)
        if not args.execute:rec.finish('planned');print(rec.directory);return 0
        p=rec.directory/'measurements.json';rec.attach(p)
        for cycle,stop,name,start,goal,traj in plans:
            rec.event('segment_start',cycle=cycle,stop=stop,station=name)
            actual,samples=n.execute_path(traj,start,goal)
            row={'cycle':cycle,'stop':stop,'name':name,'target_table_m':targets[name].tolist(),'goal_deg':np.degrees(goal).tolist(),'actual_deg':np.degrees(actual).tolist(),'joint_error_deg':np.degrees(actual-goal).tolist(),'model_tcp_error_mm':((n.fk(actual)-targets[name])*1000).tolist(),'settled_samples':len(samples),'settled_range_deg':np.degrees(np.ptp(samples,axis=0)).tolist()}
            row.update(tcp_errors(n.fk,goal,actual,targets[name]));row['fine_correction_status']=n.last_fine_status
            results.append(row);atomic_json(p,{'completed':len(results),'requested':args.cycles*6,'results':results,'physical_tip_measured':False,'pump_valve_controlled':False});rec.event('measured',measurement=row)
            print('Cycle',cycle,'/',args.cycles,'stop',stop,name,'measured',len(results),flush=True)
        stats={}
        for name in ['all']+list(dict.fromkeys(sequence)):
            rows=results if name=='all' else [x for x in results if x['name']==name];e=np.array([x['joint_error_deg'] for x in rows]);stats[name]={'count':len(rows),'bias_deg':e.mean(0).tolist(),'rmse_deg':np.sqrt((e*e).mean(0)).tolist(),'max_abs_deg':np.abs(e).max(0).tolist()}
        for name,group in stats.items():
            selected=results if name=='all' else [x for x in results if x['name']==name]
            xyz=np.array([x['model_tcp_error_mm'] for x in selected]);norms=np.linalg.norm(xyz,axis=1)
            group.update(tcp_mean_xyz_mm=xyz.mean(0).tolist(),tcp_rmse_xyz_mm=np.sqrt((xyz*xyz).mean(0)).tolist(),tcp_mean_norm_mm=float(norms.mean()),tcp_rms_norm_mm=float(np.sqrt((norms*norms).mean())),tcp_max_norm_mm=float(norms.max()))
        p=rec.directory/'metrics.json';atomic_json(p,stats);rec.attach(p)
        p=rec.directory/'measurements.csv'
        with p.open('w',newline='',encoding='utf-8-sig') as f:
            fields=['cycle','stop','name']+[f'J{i}_{k}_deg' for i in range(1,5) for k in ['goal','actual','error']]+['TCP_'+axis+'_mm' for axis in 'XYZ']+['TCP_distance_mm','tracking_distance_mm']+[f'J{i}_linear_TCP_distance_mm' for i in range(1,5)];writer=csv.DictWriter(f,fields);writer.writeheader()
            for row in results:
                out={k:row[k] for k in ['cycle','stop','name']}
                for i in range(4):
                    for kind,source in [('goal','goal_deg'),('actual','actual_deg'),('error','joint_error_deg')]:out[f'J{i+1}_{kind}_deg']=row[source][i]
                for axis,value in zip('XYZ',row['model_tcp_error_mm']):out['TCP_'+axis+'_mm']=value
                out['TCP_distance_mm']=row['model_tcp_error_norm_mm'];out['tracking_distance_mm']=row['joint_tracking_tcp_error_norm_mm']
                for i,value in enumerate(row['per_joint_linear_tcp_error_norm_mm'],1):out[f'J{i}_linear_TCP_distance_mm']=value
                writer.writerow(out)
        rec.attach(p);rec.finish('complete');print(json.dumps(stats['all'],indent=2));print(rec.directory)
    except BaseException as exc:
        if n.active_goal is not None:
            future=n.active_goal.cancel_goal_async();rclpy.spin_until_future_complete(n,future,timeout_sec=5)
        rec.finish('interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed',str(exc));raise
    finally:n.destroy_node();rclpy.try_shutdown()
