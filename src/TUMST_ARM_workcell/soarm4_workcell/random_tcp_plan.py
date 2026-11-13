"""Prepare 50 random TCP/axis IK goals and collision-reviewed routes. No execution."""
import sys, time, argparse
from pathlib import Path
import numpy as np
import rclpy
from scipy.optimize import least_squares
from moveit_msgs.msg import Constraints,JointConstraint
from moveit_msgs.srv import GetMotionPlan,GetStateValidity
from rosidl_runtime_py.convert import message_to_ordereddict
from .tcp_error_test import Test,state
from .cup_goal import Arm,tcp_point,NAMES
from .core import load,matrix
from .tool_clearance import ToolClearance
from .motor_limits import MotorLimits

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--via-home',action='store_true');parser.add_argument('--count',type=int,default=50);parser.add_argument('--seed',type=int,default=20261108);parser.add_argument('--tool-clearance-mm',type=float,default=20.);args=parser.parse_args()
    sys.path.insert(0,str(Path.home()/'robot_ws/automation/reconstruction'))
    from test_data_recorder import start_trial,atomic_json
    rec=start_trial('random_50_tcp_ik_path_planning',physical=False,source=__file__);rclpy.init();n=Test(rec)
    try:
        until=time.monotonic()+10
        while time.monotonic()<until:
            rclpy.spin_once(n,timeout_sec=.1)
            if n.q is not None and time.monotonic()-n.received<.2:break
        initial=n.fresh();start=initial.copy();home=None;arm=Arm();wc=load(Path(__file__).resolve().parents[1]/'config/workcell.yaml');W=matrix(wc['T_table_world']);tcp=tcp_point(wc);rng=np.random.default_rng(args.seed)
        motor=MotorLimits();motor.intersect(arm)
        clearance=ToolClearance(arm,wc,W,tcp,margin=args.tool_clearance_mm/1000)
        if args.tool_clearance_mm<20:raise RuntimeError('Post-collision test requires >=20mm tool clearance')
        def fk(q):
            T=W@arm.fk(q);T[:3,3]=(T@tcp)[:3];return T
        def valid(q):
            if not motor.valid(q) or not clearance.check(q)[0]:return False
            v=GetStateValidity.Request();v.group_name='arm';v.robot_state=state(q);return n.call('/check_state_validity',GetStateValidity,v).valid
        if not valid(start):raise RuntimeError('Current pose rejected by MoveIt')
        initial_home_trajectory=None
        if args.via_home:
            import xml.etree.ElementTree as ET
            srdf=ET.parse(str(Path.home()/'robot_ws/src/TUMST_ARM_moveit_config/config/TUMST_ARM.srdf')).getroot()
            values={j.get('name'):float(j.get('value')) for j in srdf.find("group_state[@name='home']").findall('joint')}
            home=np.array([values[k] for k in NAMES])
            if not valid(home):raise RuntimeError('Home pose rejected by MoveIt')
        def plan_between(a,b):
            req=GetMotionPlan.Request();p=req.motion_plan_request;p.group_name='arm';p.pipeline_id='ompl';p.planner_id='RRTConnectkConfigDefault';p.start_state=state(a);p.num_planning_attempts=1;p.allowed_planning_time=3.;p.max_velocity_scaling_factor=.5;p.max_acceleration_scaling_factor=.1
            c=Constraints();c.joint_constraints=[JointConstraint(joint_name=k,position=float(v),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for k,v in zip(NAMES,b)];p.goal_constraints=[c]
            res=n.call('/plan_kinematic_path',GetMotionPlan,req).motion_plan_response
            if res.error_code.val!=1 or not res.trajectory.joint_trajectory.points:raise RuntimeError('Home route planning failed: '+str(res.error_code.val))
            jt=res.trajectory.joint_trajectory;idx=[jt.joint_names.index(k) for k in NAMES];points=[np.array(t.positions)[idx] for t in jt.points]
            if np.max(np.abs(points[0]-a))>.002 or np.max(np.abs(points[-1]-b))>.002:raise RuntimeError('Route endpoints mismatch')
            for x,y in zip(points,points[1:]+[points[-1]]):
                for u in np.linspace(0,1,max(2,int(np.ceil(np.max(np.abs(y-x))/np.radians(.25)))+1)):
                    q=x*(1-u)+y*u
                    if not motor.valid(q) or not clearance.check(q)[0]:raise RuntimeError('Home path rejected by motor/clearance check')
            if not all(valid(q) for q in points):raise RuntimeError('Home path collision rejection')
            return message_to_ordereddict(res.trajectory)
        if home is not None:
            initial_home_trajectory=plan_between(initial,home);start=home.copy()
        plans=[];attempt=0
        while len(plans)<args.count and attempt<1000:
            attempt+=1;sample=rng.uniform(arm.lo+np.radians(.5),arm.hi-np.radians(.5));target=fk(sample)
            if not valid(sample):continue
            # Random joint sampling generates reachable TCP positions and orientations;
            # solve those Cartesian goals independently, then validate the IK result.
            seed=np.clip(sample+rng.normal(0,.1,4),arm.lo,arm.hi)
            fit=least_squares(lambda q:np.r_[(fk(q)[:3,3]-target[:3,3])*10,(fk(q)[:3,0]-target[:3,0])*.3],seed,bounds=(arm.lo,arm.hi),max_nfev=500)
            goal=fit.x;T=fk(goal)
            if np.linalg.norm(T[:3,3]-target[:3,3])>.0001 or np.linalg.norm(T[:3,0]-target[:3,0])>.005 or not valid(goal):continue
            req=GetMotionPlan.Request();p=req.motion_plan_request;p.group_name='arm';p.pipeline_id='ompl';p.planner_id='RRTConnectkConfigDefault';p.start_state=state(start);p.num_planning_attempts=1;p.allowed_planning_time=3.;p.max_velocity_scaling_factor=.5;p.max_acceleration_scaling_factor=.1
            c=Constraints();c.joint_constraints=[JointConstraint(joint_name=k,position=float(v),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for k,v in zip(NAMES,goal)];p.goal_constraints=[c]
            res=n.call('/plan_kinematic_path',GetMotionPlan,req).motion_plan_response
            if res.error_code.val!=1 or not res.trajectory.joint_trajectory.points:continue
            jt=res.trajectory.joint_trajectory;idx=[jt.joint_names.index(k) for k in NAMES];qs=[np.array(x.positions)[idx] for x in jt.points]
            if np.max(np.abs(qs[0]-start))>.002 or np.max(np.abs(qs[-1]-goal))>.002:continue
            passed=True
            for a,b in zip(qs,qs[1:]+[qs[-1]]):
                for u in np.linspace(0,1,max(2,int(np.ceil(np.max(np.abs(a-b))/np.radians(.25)))+1)):
                    q=a*(1-u)+b*u
                    if not motor.valid(q) or not clearance.check(q)[0]:passed=False;break
                if not passed:break
            if not passed or not all(valid(q) for q in qs):continue
            try:return_trajectory=plan_between(goal,home) if home is not None else None
            except RuntimeError:continue
            plans.append({'return_home_trajectory':return_trajectory,'index':len(plans)+1,'target_table_m':target[:3,3].tolist(),'target_tool_axis_table':target[:3,0].tolist(),'start_rad':start.tolist(),'goal_rad':goal.tolist(),'trajectory':message_to_ordereddict(res.trajectory)})
            start=home.copy() if home is not None else goal
            atomic_json(rec.directory/'random_plans.json',{'motor_config_sha256':motor.sha256,'joint_sampling_lo_rad':arm.lo.tolist(),'joint_sampling_hi_rad':arm.hi.tolist(),'tool_clearance_mm':args.tool_clearance_mm,'tool_clearance_sources':clearance.sources,'via_home':args.via_home,'home_rad':home.tolist() if home is not None else None,'initial_home_trajectory':initial_home_trajectory,'status':'planning_only_with_conservative_tool_clearance','executed':False,'initial_rad':initial.tolist(),'velocity':.5,'acceleration':.1,'sampling':'uniform joint limits -> Cartesian target -> numerical IK; Cartesian distribution not uniform','new_tool_geometry_verified':False,'plans':plans})
            print('Planned',len(plans),'/'+str(args.count)+'; sampled',attempt,flush=True)
        rec.attach(rec.directory/'random_plans.json');rec.finish('planned' if len(plans)==args.count else 'partial');print(rec.directory)
    except BaseException as e:rec.finish('failed',str(e));raise
    finally:n.destroy_node();rclpy.try_shutdown()
