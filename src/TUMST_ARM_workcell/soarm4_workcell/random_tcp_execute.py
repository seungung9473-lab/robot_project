"""Execute prepared random plans with fresh encoder/start/collision guards."""
import argparse,sys,time,json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rosidl_runtime_py.set_message import set_message_fields
from moveit_msgs.msg import RobotTrajectory,Constraints,JointConstraint
from moveit_msgs.srv import GetStateValidity,GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory
from .tcp_error_test import Test,state
from .cup_goal import Arm,tcp_point,NAMES
from .core import load,matrix
from .tool_clearance import ToolClearance
from .motor_limits import MotorLimits

def main():
    ap=argparse.ArgumentParser();ap.add_argument('plan');ap.add_argument('--resume-results');ap.add_argument('--skip-index',type=int,action='append',default=[]);args=ap.parse_args()
    sys.path.insert(0,str(Path.home()/'robot_ws/automation/reconstruction'))
    from test_data_recorder import start_trial,atomic_json
    source=Path(args.plan);data=json.loads(source.read_text())
    if not data.get('via_home') or data.get('tool_clearance_mm',0)<20:raise RuntimeError('Requires home cycles and >=20mm conservative tool clearance')
    if not data['plans']:raise RuntimeError('Requires a reviewed home-cycle plan')
    if not data['plans']:raise RuntimeError('No reviewed plans available')
    previous=[]
    if args.resume_results:
        previous=json.loads(Path(args.resume_results).read_text())['results']
        if len({x['index'] for x in previous})!=len(previous) or len(previous)>=50:raise RuntimeError('Invalid resume indices')
        for old in previous:
            plan=data['plans'][old['index']-1]
            if not np.allclose(old['goal_deg'],np.degrees(plan['goal_rad']),atol=1e-8):raise RuntimeError('Resume plan differs')
    rec=start_trial('random_50_tcp_encoder_execution',physical=True,source=__file__);rec.attach(source);rec.event('tool_model_assumption',new_tool_geometry_verified=False,user_requested_existing_cup_model=True)
    rclpy.init();n=Test(rec);handle=None;results=list(previous)
    if args.resume_results:
        rec.attach(args.resume_results);rec.event('explicit_resume_after_obstacle_removal',previous_completed=len(previous),user_confirmed_obstacle_removed=True)
    try:
        arm=Arm();wc=load(Path(__file__).resolve().parents[1]/'config/workcell.yaml');W=matrix(wc['T_table_world']);tcp=tcp_point(wc)
        motor=MotorLimits();motor.intersect(arm)
        if data.get('motor_config_sha256')!=motor.sha256:raise RuntimeError('Motor configuration mismatch or obsolete plan')
        clearance=ToolClearance(arm,wc,W,tcp,margin=data['tool_clearance_mm']/1000)
        def fk(q):return (W@arm.fk(q)@tcp)[:3]
        def feedback():
            end=time.monotonic()+3
            while time.monotonic()<end:
                before=n.received;rclpy.spin_once(n,timeout_sec=.05)
                if n.received>before:return n.fresh()
            raise RuntimeError('No new encoder feedback')
        client=ActionClient(n,ExecuteTrajectory,'/execute_trajectory')
        if not client.wait_for_server(timeout_sec=8):raise RuntimeError('Execution unavailable')
        completed_indices={x['index'] for x in previous};skip=set(args.skip_index)
        if skip & completed_indices:raise RuntimeError('Cannot skip completed measurements')
        rec.event('skipped_targets',indices=sorted(skip),reason='User requested skipping previously aborted target')
        segments=[0]+[s for i in range(1,len(data['plans'])+1) if i not in skip and i not in completed_indices for s in [2*i-1,2*i]]
        for segment in segments:
            if segment==0:
                row={'index':0,'phase':'initial_home','start_rad':data['initial_rad'],'goal_rad':data['home_rad'],'trajectory':data['initial_home_trajectory'],'target_table_m':fk(data['home_rad']).tolist()}
            else:
                index=(segment-1)//2;deadline=time.monotonic()+180
                while True:
                    data=json.loads(source.read_text())
                    if len(data['plans'])>index:break
                    if time.monotonic()>deadline:raise RuntimeError('Waiting for checked plan timed out')
                    rclpy.spin_once(n,timeout_sec=.1)
                plan=data['plans'][index]
                if segment%2:row=dict(plan,phase='target')
                else:row={'index':plan['index'],'phase':'return_home','start_rad':plan['goal_rad'],'goal_rad':data['home_rad'],'trajectory':plan['return_home_trajectory'],'target_table_m':fk(data['home_rad']).tolist()}
            actual=feedback();start=np.array(row['start_rad']);goal=np.array(row['goal_rad'])
            traj=RobotTrajectory();set_message_fields(traj,row['trajectory'])
            if np.max(np.abs(actual-start))>np.radians(.5):
                # Rebuild the path from measured state, never widen start tolerance.
                if not motor.valid(actual) or not clearance.check(actual)[0]:raise RuntimeError('Measured start invalid')
                req=GetMotionPlan.Request();p=req.motion_plan_request;p.group_name='arm';p.pipeline_id='ompl';p.planner_id='RRTConnectkConfigDefault';p.start_state=state(actual);p.num_planning_attempts=1;p.allowed_planning_time=5.;p.max_velocity_scaling_factor=.5;p.max_acceleration_scaling_factor=.1
                c=Constraints();c.joint_constraints=[JointConstraint(joint_name=k,position=float(v),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for k,v in zip(NAMES,goal)];p.goal_constraints=[c]
                response=n.call('/plan_kinematic_path',GetMotionPlan,req).motion_plan_response
                if response.error_code.val!=1:raise RuntimeError('Measured-start planning failed')
                traj=response.trajectory;start=actual.copy();rec.event('measured_start_path_rebuilt',index=row['index'],phase=row['phase'],start_rad=start.tolist())
            jt=traj.joint_trajectory
            if not jt.points:raise RuntimeError('Empty trajectory')
            idx=[jt.joint_names.index(k) for k in NAMES];qs=[np.array(x.positions)[idx] for x in jt.points]
            if np.max(np.abs(qs[0]-start))>.002 or np.max(np.abs(qs[-1]-goal))>.002:raise RuntimeError('Trajectory endpoint mismatch')
            for a,b in zip(qs,qs[1:]+[qs[-1]]):
                for u in np.linspace(0,1,max(2,int(np.ceil(np.max(np.abs(a-b))/np.radians(.25)))+1)):
                    q=a*(1-u)+b*u
                    if not motor.valid(q) or np.any(q<arm.lo-1e-8) or np.any(q>arm.hi+1e-8):raise RuntimeError('Limit rejection')
                    if not clearance.check(q)[0]:raise RuntimeError('Tool clearance rejection')
            for q in qs:
                req=GetStateValidity.Request();req.group_name='arm';req.robot_state=state(q)
                if not n.call('/check_state_validity',GetStateValidity,req).valid:raise RuntimeError('Collision rejection')
            actual=feedback()
            if np.max(np.abs(actual-start))>np.radians(.5):raise RuntimeError('Start changed during review')
            g=ExecuteTrajectory.Goal();g.trajectory=traj;f=client.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=10)
            if not f.done() or not f.result().accepted:raise RuntimeError('Execution goal rejected')
            handle=f.result();f=handle.get_result_async();duration=jt.points[-1].time_from_start.sec+jt.points[-1].time_from_start.nanosec/1e9
            rclpy.spin_until_future_complete(n,f,timeout_sec=max(30.,duration*2+10))
            if not f.done():raise RuntimeError('Execution timed out')
            if f.result().result.error_code.val!=1:raise RuntimeError('Execution aborted '+str(f.result().result.error_code.val))
            handle=None;end=time.monotonic()+.5
            while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
            actual=feedback();error=np.degrees(actual-goal);target=np.array(row['target_table_m']);e=(fk(actual)-target)*1000
            measurement={'phase':row['phase'],'index':row['index'],'target_table_m':target.tolist(),'goal_deg':np.degrees(goal).tolist(),'actual_deg':np.degrees(actual).tolist(),'joint_error_deg':error.tolist(),'model_tcp_error_mm':e.tolist()}
            rec.event('segment_complete',**measurement)
            if row['phase']!='target':
                print('Home reached after target',row['index'],flush=True);continue
            results.append(measurement)
            p=rec.directory/'results_50.json';atomic_json(p,{'completed':len(results),'requested':50,'skipped_target_indices':sorted(skip),'previous_completed':len(previous),'new_completed':len(results)-len(previous),'results':results,'physical_tip_measured':False,'calibration_changed':False});rec.attach(p);rec.event('motion_complete',**results[-1]);print('Executed',len(results),'/50',flush=True)
        e=np.array([x['joint_error_deg'] for x in results]);t=np.array([x['model_tcp_error_mm'] for x in results]);p=rec.directory/'metrics.json';atomic_json(p,{'completed':len(results),'skipped_target_indices':sorted(skip),'previous_completed':len(previous),'new_completed':len(results)-len(previous),'joint_bias_deg':e.mean(0).tolist(),'joint_rmse_deg':np.sqrt((e*e).mean(0)).tolist(),'joint_max_abs_deg':np.abs(e).max(0).tolist(),'model_tcp_rmse_mm':np.sqrt((t*t).mean(0)).tolist(),'model_tcp_max_norm_mm':float(np.linalg.norm(t,axis=1).max()),'calibration_changed':False,'physical_tip_measured':False,'new_tool_geometry_verified':False});rec.attach(p);rec.finish('complete');print(p)
    except BaseException as exc:
        if handle is not None:
            f=handle.cancel_goal_async();rclpy.spin_until_future_complete(n,f,timeout_sec=5)
        rec.finish('interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed',str(exc));raise
    finally:n.destroy_node();rclpy.try_shutdown()
