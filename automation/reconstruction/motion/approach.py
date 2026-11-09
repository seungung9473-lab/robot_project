import json,time,math,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan,GetStateValidity,ApplyPlanningScene,GetPlanningScene
from moveit_msgs.msg import CollisionObject,PlanningSceneComponents
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import yaml,numpy as np
from scipy.spatial.transform import Rotation
from moveit_msgs.action import ExecuteTrajectory
rclpy.init();n=Node('turntable_central_approach');latest={};names=['joint1','joint2','joint3','joint4']
def cb(m):
 if all(x in m.name for x in names):latest.update(q=[float(m.position[m.name.index(x)]) for x in names],t=time.monotonic())
s=n.create_subscription(JointState,'/joint_states',cb,10)
def fresh():
 end=time.monotonic()+8
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.1)
  if latest and time.monotonic()-latest['t']<.3:return latest['q'][:]
 raise RuntimeError('No fresh motor feedback')
def call(c,req,seconds=45):
 if not c.wait_for_service(timeout_sec=8):raise RuntimeError('Service missing')
 f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=seconds)
 if not f.done():raise RuntimeError('Service timed out')
 return f.result()
def state(q):return RobotState(joint_state=JointState(name=names,position=q))
try:
 # Register an approximate object proxy; no table/cup collision exemptions.
 cfg=yaml.safe_load(Path('/home/seungung9473/robot_ws/src/soarm4_workcell/config/workcell.yaml').read_text());W=np.array(cfg['T_table_world']);T=np.eye(4);T[:3,3]=json.loads(Path(os.environ['SOARM_PAYLOAD_JSON']).read_text())['center_table_m'];T=np.linalg.inv(W)@T;pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,T[:3,3]);quat=Rotation.from_matrix(T[:3,:3]).as_quat();pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,quat)
 co=CollisionObject();co.id='scanned_object_proxy';co.header.frame_id='world';co.operation=CollisionObject.ADD;co.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=json.loads(Path(os.environ['SOARM_PAYLOAD_JSON']).read_text())['dimensions_m'])];co.primitive_poses=[pose];reqscene=ApplyPlanningScene.Request();reqscene.scene.is_diff=True;reqscene.scene.world.collision_objects=[co]
 if not call(n.create_client(ApplyPlanningScene,'/apply_planning_scene'),reqscene).success:raise RuntimeError('Object proxy registration failed')
 scene_req=GetPlanningScene.Request();scene_req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
 registered={x.id for x in call(n.create_client(GetPlanningScene,'/get_planning_scene'),scene_req).scene.world.collision_objects};required={'workcell_table_guard','workcell_turntable','workcell_pad1','workcell_pad2','scanned_object_proxy'}
 if not required<=registered:raise RuntimeError('Missing collision environment '+str(required-registered))
 print('Required collision objects verified',flush=True)
 start=fresh();goal=json.loads(Path(os.environ['SOARM_APPROACH_IK']).read_text())['above'];v=n.create_client(GetStateValidity,'/check_state_validity')
 for label,q in [('start',start),('above',goal)]:
  req=GetStateValidity.Request();req.robot_state=state(q);req.group_name='arm';res=call(v,req);print(label,'valid',res.valid,'contacts',[(c.contact_body_1,c.contact_body_2) for c in res.contacts],flush=True)
  if not res.valid:raise RuntimeError(label+' pose collision/invalid')
 c=n.create_client(GetMotionPlan,'/plan_kinematic_path');req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.pipeline_id='ompl';r.planner_id='RRTConnectkConfigDefault';r.num_planning_attempts=1;r.allowed_planning_time=8.;r.max_velocity_scaling_factor=float(os.environ['SOARM_VELOCITY']);r.max_acceleration_scaling_factor=float(os.environ['SOARM_ACCELERATION']);r.start_state=state(start);r.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=k,position=float(q),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for k,q in zip(names,goal)])]
 res=call(c,req).motion_plan_response;print('Plan result',res.error_code.val,flush=True)
 if res.error_code.val!=1:raise RuntimeError('Planning rejected')
 q=fresh()
 if max(abs(a-b) for a,b in zip(q,start))>math.radians(1):raise RuntimeError('Start moved; refusing execution')
 a=ActionClient(n,ExecuteTrajectory,'/execute_trajectory')
 if not a.wait_for_server(timeout_sec=3):raise RuntimeError('No execution server')
 g=ExecuteTrajectory.Goal();g.trajectory=res.trajectory;f=a.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=5);h=f.result()
 if not h or not h.accepted:raise RuntimeError('Execution refused')
 print('Executing SAM2 suction-candidate preapproach only',flush=True);f=h.get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=120)
 if not f.done():raise RuntimeError('Result timeout; check robot')
 result=f.result().result.error_code.val;print('Execution result',result,'final',fresh(),flush=True)
 if result!=1:raise RuntimeError('Approach execution failed')
finally:n.destroy_node();rclpy.shutdown()
