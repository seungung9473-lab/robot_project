"""Local TCP IK, collision checked MoveIt execution, and measured encoder error."""
import argparse,json,time,sys
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan,GetStateValidity
from moveit_msgs.msg import RobotState,Constraints,JointConstraint
from moveit_msgs.action import ExecuteTrajectory
from scipy.optimize import least_squares
from .cup_goal import Arm,tcp_point,NAMES
from .core import load,matrix

class Test(Node):
    def __init__(self,record):
        super().__init__('tcp_error_test');self.record=record;self.q=None;self.received=0
        self.create_subscription(JointState,'/joint_states',self.feedback,10)
    def feedback(self,msg):
        d=dict(zip(msg.name,msg.position))
        if all(n in d for n in NAMES):
            self.q=np.array([d[n] for n in NAMES]);self.received=time.monotonic();self.record.sample('joint_states',position_rad=self.q.tolist())
    def fresh(self):
        if self.q is None or time.monotonic()-self.received>.5:raise RuntimeError('Encoder feedback missing/stale')
        return self.q.copy()
    def call(self,name,typ,req):
        if not hasattr(self,'_service_clients'):self._service_clients={}
        if name not in self._service_clients:self._service_clients[name]=self.create_client(typ,name)
        c=self._service_clients[name]
        if not c.wait_for_service(timeout_sec=5):raise RuntimeError('Service unavailable: '+name)
        f=c.call_async(req);rclpy.spin_until_future_complete(self,f,timeout_sec=45)
        if not f.done() or f.result() is None:raise RuntimeError('Service timeout: '+name)
        return f.result()

def state(q):
    s=RobotState();s.joint_state=JointState(name=NAMES,position=[float(x) for x in q]);return s

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--delta-mm',nargs=3,type=float,default=[0,0,2]);ap.add_argument('--execute',action='store_true');ap.add_argument('--xyz',nargs=3,type=float);ap.add_argument('--velocity',type=float,default=.05);ap.add_argument('--acceleration',type=float,default=.05);a=ap.parse_args()
    if not (0<a.velocity<=1 and 0<a.acceleration<=1):ap.error('Scaling must be in (0,1]')
    sys.path.insert(0,str(Path.home()/'robot_ws/automation/reconstruction'))
    from test_data_recorder import start_trial,atomic_json
    trial=start_trial('tcp_ik_encoder_error',physical=a.execute,source=__file__)
    rclpy.init();n=Test(trial)
    try:
        until=time.monotonic()+10
        while time.monotonic()<until:
            rclpy.spin_once(n,timeout_sec=.1)
            if n.q is not None and time.monotonic()-n.received<.2:break
        start=n.fresh();arm=Arm();wc=load(Path(__file__).resolve().parents[1]/'config/workcell.yaml');W=matrix(wc['T_table_world']);tcp=tcp_point(wc)
        def fk(q):
            T=W@arm.fk(q);T[:3,3]=(T@tcp)[:3];return T
        initial=fk(start);target=np.array(a.xyz) if a.xyz is not None else initial[:3,3]+np.array(a.delta_mm)/1000
        if np.linalg.norm(target-initial[:3,3])>.005:raise RuntimeError('Calibration test limited to 5mm')
        fit=least_squares(lambda q:np.r_[(fk(q)[:3,3]-target)*10,(fk(q)[:3,0]-initial[:3,0])*.3],np.clip(start,arm.lo,arm.hi),bounds=(arm.lo,arm.hi),max_nfev=500)
        goal=fit.x
        if np.linalg.norm(fk(goal)[:3,3]-target)>.0001:raise RuntimeError('IK position residual >0.1mm')
        if np.linalg.norm(fk(goal)[:3,0]-initial[:3,0])>.005:raise RuntimeError('IK orientation residual too large')
        if np.max(np.abs(goal-start))>np.radians(3):raise RuntimeError('Joint change >3deg')
        req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.pipeline_id='ompl';r.planner_id='RRTConnectkConfigDefault';r.start_state=state(start);r.allowed_planning_time=5.;r.num_planning_attempts=1;r.max_velocity_scaling_factor=float(a.velocity);r.max_acceleration_scaling_factor=float(a.acceleration)
        c=Constraints();c.joint_constraints=[JointConstraint(joint_name=k,position=float(v),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for k,v in zip(NAMES,goal)];r.goal_constraints=[c]
        res=n.call('/plan_kinematic_path',GetMotionPlan,req).motion_plan_response
        if res.error_code.val!=1:raise RuntimeError('Planning rejected: '+str(res.error_code.val))
        jt=res.trajectory.joint_trajectory
        if not jt.points:raise RuntimeError('Empty trajectory')
        idx=[jt.joint_names.index(k) for k in NAMES];points=[np.array(p.positions)[idx] for p in jt.points]
        if np.max(np.abs(points[0]-start))>.002 or np.max(np.abs(points[-1]-goal))>.002:raise RuntimeError('Trajectory endpoint mismatch')
        for x,y in zip(points,points[1:]+[points[-1]]):
            for u in np.linspace(0,1,max(2,int(np.ceil(np.max(np.abs(y-x))/np.radians(.25)))+1)):
                q=x*(1-u)+y*u
                if np.max(np.abs(q-start))>np.radians(3):raise RuntimeError('Path leaves small test envelope')
                if np.linalg.norm(fk(q)[:3,3]-initial[:3,3])>.006:raise RuntimeError('Path leaves 6mm TCP envelope')
                v=GetStateValidity.Request();v.group_name='arm';v.robot_state=state(q)
                if not n.call('/check_state_validity',GetStateValidity,v).valid:raise RuntimeError('Path collision')
        out={'start_deg':np.degrees(start).tolist(),'goal_deg':np.degrees(goal).tolist(),'target_table_m':target.tolist(),'initial_table_m':initial[:3,3].tolist(),'physical_tip_measured':False,'executed':False,'velocity_scaling':a.velocity,'acceleration_scaling':a.acceleration}
        trial.event('plan_validated',**out)
        if a.execute:
            # Process newly arrived feedback after synchronous planning/IK work.
            refresh_until=time.monotonic()+.3
            while time.monotonic()<refresh_until:rclpy.spin_once(n,timeout_sec=.02)
            if np.max(np.abs(n.fresh()-start))>np.radians(.3):raise RuntimeError('Start state changed')
            client=ActionClient(n,ExecuteTrajectory,'/execute_trajectory')
            if not client.wait_for_server(timeout_sec=5):raise RuntimeError('Execution unavailable')
            g=ExecuteTrajectory.Goal();g.trajectory=res.trajectory
            f=client.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=10)
            if not f.done() or not f.result().accepted:raise RuntimeError('Execution rejected')
            h=f.result();f=h.get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=40)
            if not f.done():
                cancel=h.cancel_goal_async();rclpy.spin_until_future_complete(n,cancel,timeout_sec=3);raise RuntimeError('Execution timeout; cancellation requested')
            if f.result().result.error_code.val!=1:raise RuntimeError('Execution failed: '+str(f.result().result.error_code.val))
            until=time.monotonic()+1
            while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.1)
            actual=n.fresh();out.update(executed=True,actual_deg=np.degrees(actual).tolist(),joint_error_deg=np.degrees(actual-goal).tolist(),actual_model_tcp_table_m=fk(actual)[:3,3].tolist(),model_tcp_error_mm=((fk(actual)[:3,3]-target)*1000).tolist())
        p=trial.directory/'result.json';atomic_json(p,out);trial.attach(p);trial.finish('complete' if a.execute else 'planned');print(json.dumps(out,indent=2));print(p)
    except Exception as e:
        trial.finish('failed',str(e));print(str(e),file=sys.stderr);return 1
    finally:n.destroy_node();rclpy.shutdown()
    return 0
