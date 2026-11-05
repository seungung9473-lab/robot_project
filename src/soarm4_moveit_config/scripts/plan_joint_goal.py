#!/usr/bin/env python3
"""Request OMPL planning, reject collisions along the path, then preview only."""
import argparse,json,math,sys,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Constraints,JointConstraint,RobotState,DisplayTrajectory,CollisionObject,MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan,GetStateValidity,ApplyPlanningScene

NAMES=['joint1','joint2','joint3','joint4']

class Planner(Node):
    def __init__(self):
        super().__init__('validated_joint_planner',namespace='/soarm4_plan')
        self.plan=self.create_client(GetMotionPlan,'plan_kinematic_path')
        self.valid=self.create_client(GetStateValidity,'check_state_validity')
        self.scene=self.create_client(ApplyPlanningScene,'apply_planning_scene')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.preview=self.create_publisher(DisplayTrajectory,'display_planned_path',qos)
        p=Path(get_package_share_directory('soarm4_moveit_config'));sys.path.insert(0,str(p/'runtime_deps'));sys.path.insert(0,str(p/'audit_tools'))
        from run_scan import ReviewAuditor
        self.audit=ReviewAuditor(p/'audit_model')
    def call(self,client,req,timeout=30):
        if not client.wait_for_service(timeout_sec=timeout):raise RuntimeError('Service unavailable: '+client.srv_name)
        future=client.call_async(req);rclpy.spin_until_future_complete(self,future,timeout_sec=timeout)
        if not future.done() or future.result() is None:raise RuntimeError('Service timeout: '+client.srv_name)
        return future.result()
    def state(self,q):
        state=RobotState();state.joint_state=JointState(name=NAMES,position=[float(v) for v in q]);state.is_diff=False;return state
    def check(self,q,moveit=False):
        if not self.audit.inside_limits(q).all():raise RuntimeError('Joint limits exceeded: '+str(np.degrees(q).tolist()))
        result=self.audit.audit(q)
        if result['outside_coupling_zones'] or result['truncated_unknown'] or result['below_table_plane']:
            pairs=[[self.audit.meshes[i]['name'],self.audit.meshes[j]['name']] for i,j in result['outside_coupling_zones']]
            raise RuntimeError('Detailed collision audit rejected pose: '+str(pairs))
        if moveit:
            req=GetStateValidity.Request();req.robot_state=self.state(q);req.group_name='arm';res=self.call(self.valid,req)
            if not res.valid:raise RuntimeError('MoveIt rejected pose: '+str([(c.contact_body_1,c.contact_body_2) for c in res.contacts]))
    def add_table(self):
        req=ApplyPlanningScene.Request();req.scene.is_diff=True
        table=CollisionObject();table.header.frame_id='world';table.id='table';table.operation=CollisionObject.ADD
        box=SolidPrimitive();box.type=SolidPrimitive.BOX;box.dimensions=[2.,2.,.02]
        pose=Pose();pose.orientation.w=1.;pose.position.z=-.011 # top -1mm avoids fixed mounting surface contact
        table.primitives=[box];table.primitive_poses=[pose];req.scene.world.collision_objects=[table]
        if not self.call(self.scene,req).success:raise RuntimeError('Table scene update failed')
    def run(self,goal,out):
        start=np.array(self.audit.model['poses']['home'],dtype=float);goal=np.radians(goal);self.add_table();self.check(start,True);self.check(goal,True)
        last_error=None
        for attempt in range(3):
            req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.pipeline_id='ompl';r.planner_id='RRTConnectkConfigDefault';r.num_planning_attempts=5;r.allowed_planning_time=8.;r.max_velocity_scaling_factor=.1;r.max_acceleration_scaling_factor=.1;r.start_state=self.state(start)
            c=Constraints()
            for name,value in zip(NAMES,goal):c.joint_constraints.append(JointConstraint(joint_name=name,position=float(value),tolerance_above=.0001,tolerance_below=.0001,weight=1.))
            r.goal_constraints=[c];res=self.call(self.plan,req).motion_plan_response
            if res.error_code.val!=MoveItErrorCodes.SUCCESS:
                last_error=RuntimeError('MoveIt planning failed: '+str(res.error_code.val));self.get_logger().warning('Retrying: '+str(last_error));continue
            trajectory=res.trajectory.joint_trajectory
            if not trajectory.points:raise RuntimeError('Planner returned empty path')
            indices=[trajectory.joint_names.index(n) for n in NAMES];points=[np.array(p.positions)[indices] for p in trajectory.points]
            if np.max(np.abs(points[0]-start))>.001 or np.max(np.abs(points[-1]-goal))>.001:raise RuntimeError('Unexpected trajectory endpoints')
            samples=0
            try:
                for a,b in zip(points[:-1],points[1:]):
                    n=max(1,int(math.ceil(np.max(np.abs(b-a))/math.radians(.5))))
                    for u in np.linspace(0,1,n+1):self.check(a*(1-u)+b*u,True);samples+=1
                if len(points)==1:self.check(points[0]);samples=1
            except RuntimeError as exc:
                last_error=exc;self.get_logger().warning('Retrying: '+str(exc));continue
            display=DisplayTrajectory();display.model_id='soarm4_checked_v5_7';display.trajectory_start=res.trajectory_start;display.trajectory=[res.trajectory]
            # Re-publish long enough for the RViz display to discover this publisher.
            for _ in range(10):self.preview.publish(display);rclpy.spin_once(self,timeout_sec=.1)
            payload={'status':'validated_simulation_plan','execution_enabled':False,'joint_names':NAMES,'start_deg':np.degrees(start).tolist(),'goal_deg':np.degrees(goal).tolist(),'planning_time_s':res.planning_time,'validated_samples':samples,'sample_spacing_max_deg':.5,'points':[{'positions_rad':list(p.positions),'time_s':p.time_from_start.sec+p.time_from_start.nanosec*1e-9} for p in trajectory.points],'note':'Sampled mesh check with documented joint coupling zones; not continuous physical clearance certification.'}
            Path(out).write_text(json.dumps(payload,indent=2));self.get_logger().info('Validated '+str(samples)+' samples; preview only. Saved: '+str(Path(out).resolve()));return payload
        raise RuntimeError('No path passed detailed joint-contact review: '+str(last_error))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--goal-deg',type=float,nargs=4,default=[30,150,-130,0]);parser.add_argument('--output',default='/tmp/soarm4_verified_plan.json');args,rosargs=parser.parse_known_args()
    rclpy.init(args=rosargs);node=None
    try:
        node=Planner();node.run(args.goal_deg,args.output)
    except (RuntimeError,ValueError) as exc:print('PLAN REJECTED: '+str(exc),file=sys.stderr);return 1
    finally:
        if node:node.destroy_node()
        rclpy.try_shutdown()
    return 0
if __name__=='__main__':sys.exit(main())
