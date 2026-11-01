"""Read-only MoveIt validation; requests planning only, never execution."""
import json,time,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer,TransformListener
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPlanningScene,GetStateValidity,GetMotionPlan
from moveit_msgs.msg import PlanningSceneComponents,RobotState,Constraints,JointConstraint
from soarm4_workcell.core import load,collision_geometry,matrix,rotation
from ament_index_python.packages import get_package_share_directory
rclpy.init(args=['--ros-args','-r','/tf:=/soarm4_plan/tf','-r','/tf_static:=/soarm4_plan/tf_static']);n=Node('workcell_runtime_check');buffer=Buffer();listener=TransformListener(buffer,n)
w=Path(get_package_share_directory('soarm4_workcell'));d=load(w/'config/workcell.yaml');p=Path(get_package_share_directory('soarm4_moveit_config'));model=json.loads((p/'audit_model/config/model.json').read_text());names=[j['name'] for j in model['joints']];home=list(map(float,model['poses']['home']));report={'mode':'offline plan-only, no motor interfaces','results':{}}
def call(typ,path,req,timeout=10):
 c=n.create_client(typ,'/soarm4_plan/'+path)
 if not c.wait_for_service(timeout_sec=timeout):raise RuntimeError('Missing '+path)
 f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=timeout)
 if not f.done():raise RuntimeError('Timeout '+path)
 return f.result()
def valid(q):return call(GetStateValidity,'check_state_validity',GetStateValidity.Request(robot_state=RobotState(joint_state=JointState(name=names,position=list(map(float,q)))),group_name='arm'),4)
try:
 req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY|PlanningSceneComponents.ALLOWED_COLLISION_MATRIX;scene=call(GetPlanningScene,'get_planning_scene',req).scene;objs={o.id:o for o in scene.world.collision_objects};expected=collision_geometry(d);assert {v['id'] for v in expected}.issubset(objs)
 for v in expected:
  o=objs[v['id']];assert o.header.frame_id=='world';assert np.allclose(o.primitives[0].dimensions,v['dimensions']);T=np.array(v['T_world_object']);combined=np.eye(4)
  for P in [o.pose,o.primitive_poses[0]]:
   q=P.orientation;M=np.eye(4);M[:3,:3]=rotation([q.x,q.y,q.z,q.w]);M[:3,3]=[P.position.x,P.position.y,P.position.z];combined=combined@M
  assert np.allclose(combined,T)
 report['results']['registered_collision_objects']={k:{'dimensions':list(o.primitives[0].dimensions),'frame':o.header.frame_id} for k,o in objs.items() if k.startswith('workcell_')}
 end=time.monotonic()+8
 while not buffer.can_transform('table','link1',rclpy.time.Time()) and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 tf=buffer.lookup_transform('table','link1',rclpy.time.Time());t=tf.transform.translation;np.testing.assert_allclose([t.x,t.y,t.z],d['robot_registration']['j1_center_table_m'],atol=1e-6);report['results']['tf_j1_table_m']=[t.x,t.y,t.z]
 hv=valid(home);report['results']['home_valid']=hv.valid;report['results']['home_contacts']=[(c.contact_body_1,c.contact_body_2) for c in hv.contacts];print('HOME',hv.valid,report['results']['home_contacts'],flush=True)
 target=None
 for delta in [5,-5,10,-10]:
  q=home.copy();q[0]+=math.radians(delta)
  if valid(q).valid:target=q;break
 if hv.valid and target:
  req=GetMotionPlan.Request();m=req.motion_plan_request;m.group_name='arm';m.pipeline_id='ompl';m.planner_id='RRTConnectkConfigDefault';m.num_planning_attempts=1;m.allowed_planning_time=5.;m.max_velocity_scaling_factor=.1;m.max_acceleration_scaling_factor=.1;m.start_state=RobotState(joint_state=JointState(name=names,position=home));m.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=k,position=float(q),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for k,q in zip(names,target)])]
  plan=call(GetMotionPlan,'plan_kinematic_path',req,30).motion_plan_response;report['results']['plan_error_code']=plan.error_code.val;report['results']['planned_points']=len(plan.trajectory.joint_trajectory.points);report['results']['plan_target_rad']=target;print('PLAN',plan.error_code.val,report['results']['planned_points'],flush=True)
 rng=np.random.default_rng(25);limits=np.array([[float(j['limit']['lower']),float(j['limit']['upper'])] for j in model['joints']]);hit=None
 for i in range(120):
  q=rng.uniform(limits[:,0],limits[:,1]);v=valid(q);pairs=[(c.contact_body_1,c.contact_body_2) for c in v.contacts if 'workcell_' in c.contact_body_1 or 'workcell_' in c.contact_body_2]
  if pairs:hit={'joint_angles_rad':q.tolist(),'valid':v.valid,'contacts':pairs};break
 report['results']['environment_collision_rejected']=hit;print('COLLISION',hit,flush=True)
 services=dict(n.get_service_names_and_types());report['results']['motor_service_names']=[k for k in services if any(s in k for s in ['/servo/','/vacuum/','/turntable/'])];assert not report['results']['motor_service_names']
 assert hv.valid,'HOME invalid in new workcell'
 assert report['results'].get('plan_error_code')==1,'Planning did not pass'
 assert hit and not hit['valid'],'Environment obstacle rejection not confirmed'
 report['status']='passed'
finally:
 out=Path('/home/seungung9473/문서/ChatGPT/라파 대회/soarm4_workcell/validation_runtime.json');out.write_text(json.dumps(report,ensure_ascii=False,indent=2));n.destroy_node();rclpy.try_shutdown()
