import json,time,math,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, Constraints, JointConstraint, CollisionObject, AttachedCollisionObject, PlanningSceneComponents, AllowedCollisionEntry
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
import copy
from moveit_msgs.srv import GetMotionPlan,GetStateValidity,ApplyPlanningScene,GetPlanningScene
from moveit_msgs.action import ExecuteTrajectory
import json,yaml,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
m=json.loads(Path('/home/wanjae/robot_ws/src/TUMST_ARM_description/config/model.json').read_text());d=yaml.safe_load(Path('/home/wanjae/robot_ws/src/TUMST_ARM_workcell/config/workcell.yaml').read_text());W=np.array(d['T_table_world']);B=np.eye(4);B[:3,:3]=Rotation.from_euler('z',m['robot_base_yaw']).as_matrix();B[:3,3]=m['robot_base_xyz']
def fk(q):
 T=W@B
 for j,a in zip(m['joints'],q):
  J=np.eye(4);J[:3,3]=j['origin_xyz'];R=np.eye(4);R[:3,:3]=Rotation.from_rotvec(np.array(j['axis'])*a).as_matrix();T=T@J@R
 T[:3,3]+=T[:3,:3]@m['tcp_xyz'];return T

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
def state(q):return RobotState(joint_state=JointState(name=names,position=q),is_diff=True)
vac={};active={'handle':None,'lost':False,'bad':0}
def diff_cb(msg):
 vac.update(value=msg.data,t=time.monotonic());active['bad']=active['bad']+1 if msg.data>-300 else 0
 if active['handle'] is not None and active['bad']>=2 and not active['lost']:
  active['lost']=True;active['handle'].cancel_goal_async();print('VACUUM LOSS: cancelling lift',flush=True)
vsub=n.create_subscription(Int32,'/vacuum/diff',diff_cb,10)
def require_vacuum():
 if not vac or time.monotonic()-vac['t']>.5 or vac['value']>-300:raise RuntimeError('Fresh suction confirmation missing; no lift')
sceneclient=n.create_client(ApplyPlanningScene,'/apply_planning_scene');restore_acm=None

try:
 start=fresh();deadline=time.monotonic()+8.
 while (not vac or time.monotonic()-vac.get('t',0)>.5) and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
 require_vacuum()
 T0=fk(start)
 # Move scanned object from world to attached body, preserving measured world pose.
 O=np.eye(4);O[:3,3]=json.loads(Path(os.environ['SOARM_PAYLOAD_JSON']).read_text())['center_table_m'];L=np.linalg.inv(T0)@O;pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,L[:3,3]);qr=Rotation.from_matrix(L[:3,:3]).as_quat();pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,qr)
 co=CollisionObject();co.id='scanned_object_proxy';co.header.frame_id='cup_tip';co.operation=CollisionObject.ADD;co.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=json.loads(Path(os.environ['SOARM_PAYLOAD_JSON']).read_text())['dimensions_m'])];co.primitive_poses=[pose];attached=AttachedCollisionObject();attached.link_name='cup_tip';attached.object=co;attached.touch_links=['cup_tip','tool_link_cup_visual_link']
 get=GetPlanningScene.Request();get.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX|PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS;current_scene=call(n.create_client(GetPlanningScene,'/get_planning_scene'),get).scene;restore_acm=copy.deepcopy(current_scene.allowed_collision_matrix);acm=copy.deepcopy(restore_acm);existing=[a for a in current_scene.robot_state.attached_collision_objects if a.object.id=='scanned_object_proxy'];attached=existing[0] if existing else attached
 for name in ['scanned_object_proxy','workcell_turntable']:
  if name not in acm.entry_names:
   acm.entry_names.append(name)
   for row in acm.entry_values:row.enabled.append(False)
   acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
 i=acm.entry_names.index('scanned_object_proxy');j=acm.entry_names.index('workcell_turntable');acm.entry_values[i].enabled[j]=True;acm.entry_values[j].enabled[i]=True
 req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True;remove=CollisionObject();remove.id='scanned_object_proxy';remove.operation=CollisionObject.REMOVE;req.scene.world.collision_objects=[];req.scene.robot_state.attached_collision_objects=[attached];req.scene.allowed_collision_matrix=acm
 if not call(sceneclient,req).success:raise RuntimeError('Payload collision model attachment failed')
 print('Payload attached; only support-contact pair allowed during lift',flush=True)
 target=T0[:3,3].copy();target[:]=json.loads(Path(os.environ['SOARM_LIFT_RESULT']).read_text())['tcp_before_table_m']
 def residual(q):
  T=fk(q);return np.r_[(T[:3,3]-target)*10,(T[:3,0]-T0[:3,0])*.3]
 lo=[float(j['limit']['lower']) for j in m['joints']];hi=[float(j['limit']['upper']) for j in m['joints']]
 sol=least_squares(residual,start,bounds=(lo,hi));goal=sol.x.tolist()
 if np.linalg.norm(fk(goal)[:3,3]-target)>.0002:raise RuntimeError('Vertical lift IK failed')
 print('TCP start',T0[:3,3],'target',target,flush=True)
 v=n.create_client(GetStateValidity,'/check_state_validity')
 for label,q in [('start',start),('above',goal)]:
  req=GetStateValidity.Request();req.robot_state=state(q);req.group_name='arm';res=call(v,req);print(label,'valid',res.valid,'contacts',[(c.contact_body_1,c.contact_body_2) for c in res.contacts],flush=True)
  if not res.valid:raise RuntimeError(label+' pose collision/invalid')
 c=n.create_client(GetMotionPlan,'/plan_kinematic_path');req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.pipeline_id='ompl';r.planner_id='RRTConnectkConfigDefault';r.num_planning_attempts=1;r.allowed_planning_time=8.;r.max_velocity_scaling_factor=float(os.environ['SOARM_VELOCITY']);r.max_acceleration_scaling_factor=float(os.environ['SOARM_ACCELERATION']);r.start_state=state(start);r.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=k,position=float(q),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for k,q in zip(names,goal)])]
 res=call(c,req).motion_plan_response;print('Plan result',res.error_code.val,flush=True)
 if res.error_code.val!=1:raise RuntimeError('Planning rejected')
 pts=res.trajectory.joint_trajectory;idx=[pts.joint_names.index(k) for k in names]
 for pt in pts.points:
  p=fk([pt.positions[i] for i in idx])[:3,3]
  if np.linalg.norm(p[:2]-target[:2])>.004 or p[2]<target[2]-.0005 or p[2]>T0[2,3]+.001:raise RuntimeError('Plan deviates from requested vertical lift')
 q=fresh()
 if max(abs(a-b) for a,b in zip(q,start))>math.radians(1):raise RuntimeError('Start moved; refusing execution')
 require_vacuum()
 a=ActionClient(n,ExecuteTrajectory,'/execute_trajectory')
 if not a.wait_for_server(timeout_sec=3):raise RuntimeError('No execution server')
 g=ExecuteTrajectory.Goal();g.trajectory=res.trajectory;f=a.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=5);h=f.result()
 if not h or not h.accepted:raise RuntimeError('Execution refused')
 active['handle']=h
 print('Executing 1cm vertical return, suction maintained',flush=True);f=h.get_result_async();end=time.monotonic()+120
 while not f.done() and time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.05)
  if (not vac or time.monotonic()-vac['t']>.5) and not active['lost']:
   active['lost']=True;h.cancel_goal_async();print('Vacuum feedback stale; cancelling',flush=True)
 active['handle']=None
 if not f.done():raise RuntimeError('Result timeout; check robot')
 result=f.result().result.error_code.val;q=fresh();print('Execution result',result,'final',q,'vacuum',vac,flush=True)
 if result!=1 or active['lost']:raise RuntimeError('Lift did not complete with maintained suction')
 require_vacuum()
 Path(os.environ['SOARM_RETURN_RESULT']).write_text(json.dumps({'success':True,'joint_feedback':q,'tcp_table_m':fk(q)[:3,3].tolist(),'vacuum_diff':vac['value'],'pump_state':'ON holding','payload_attached':True,'time':time.time()}))
finally:
 if restore_acm is not None:
  try:
   req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.allowed_collision_matrix=restore_acm;call(sceneclient,req);print('Temporary support-contact allowance restored',flush=True)
  except Exception as e:print('Support allowance restore failed:',e,flush=True)
 n.destroy_node();rclpy.shutdown()
