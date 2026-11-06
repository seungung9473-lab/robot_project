"""Two reconstructed objects -> robot left/right pads, with attached-body planning."""
import argparse,copy,json,math,os,subprocess,sys,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32,String
from std_srvs.srv import Trigger
from moveit_msgs.msg import RobotState,Constraints,JointConstraint,CollisionObject,AttachedCollisionObject,PlanningSceneComponents,AllowedCollisionEntry
from moveit_msgs.srv import GetMotionPlan,GetStateValidity,ApplyPlanningScene,GetPlanningScene
from moveit_msgs.action import ExecuteTrajectory
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from motion.kinematics import fk,m,W
from placement_geometry import load_context
ROOT=Path('/home/wanjae/robot_ws_mybranch/automation/reconstruction')
p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('--execute',action='store_true');p.add_argument('--resume-contact',action='store_true',help='Only resume first object after recorded grip-before-lift interruption');p.add_argument('--resume-held-small',action='store_true');args=p.parse_args();session=args.session.resolve();plan=json.loads((session/'sort_plan.json').read_text());out=session/('sorting_resume_small' if args.resume_held_small else ('sorting_resume_contact' if args.resume_contact else 'sorting'));out.mkdir(exist_ok=True)
out=session/'sorting_finish';out.mkdir(exist_ok=True)
if args.execute and (out/'execution.json').exists():raise RuntimeError('Execution already attempted; inspect saved state before retry')
ctx=load_context(session);names=['joint'+str(i) for i in range(1,5)];lo=np.array([j['limit']['lower'] for j in m['joints']],float);hi=np.array([j['limit']['upper'] for j in m['joints']],float)
pads={5:np.array([1.0983380219417491,.08356132162842111,.0575]),6:np.array([1.097805614569502,.38441874573139295,.059])}
rclpy.init();node=rclpy.create_node('two_object_sort');latest={};vac={};turn={};holding=False;handle=None;bad=0;lost=False
status={'success':False,'phases':[],'placements':[]}
def joint(msg):
 if all(n in msg.name for n in names):latest.update(q=[float(msg.position[msg.name.index(n)]) for n in names],t=time.monotonic())
def pressure(msg):
 global bad,lost
 vac.update(value=msg.data,t=time.monotonic());bad=bad+1 if msg.data>-200 else 0
 if handle is not None and holding and bad>=2:lost=True;handle.cancel_goal_async()
node.create_subscription(JointState,'/joint_states',joint,10);node.create_subscription(Int32,'/vacuum/diff',pressure,10);node.create_subscription(String,'/turntable/state',lambda msg:turn.update(json.loads(msg.data)),10)
def call(name,req=None,typ=Trigger,timeout=45):
 c=node.create_client(typ,name)
 try:
  if not c.wait_for_service(timeout_sec=8):raise RuntimeError('Missing service '+name)
  f=c.call_async(req or typ.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=timeout)
  if not f.done():raise RuntimeError('Service timeout '+name)
  return f.result()
 finally:node.destroy_client(c)
def fresh():
 end=time.monotonic()+8
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.05)
  if latest and time.monotonic()-latest['t']<.3:return latest['q'][:]
 raise RuntimeError('No fresh actual joints')
def phase(s):
 status['phases'].append({'phase':s,'time':time.time()});(out/'phase.txt').write_text(s)
 if args.execute:(out/'execution.json').write_text(json.dumps(status,ensure_ascii=False,indent=2))
 print(s,flush=True)
def state(q):return RobotState(joint_state=JointState(name=names,position=list(map(float,q))),is_diff=True)
def apply(req):
 if not call('/apply_planning_scene',req,ApplyPlanningScene).success:raise RuntimeError('Scene update failed')
def scene_request():
 req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True;return req

def pose(T):
 p=Pose();p.position.x,p.position.y,p.position.z=map(float,T[:3,3]);p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,Rotation.from_matrix(T[:3,:3]).as_quat());return p

def ik(target,seed,axis=None,payload=None):
 if axis is None:axis=np.array([0.,0.,-1.])
 def error(q):
  T=fk(q)
  if payload is None:pos=T[:3,3]
  else:
   O=T@payload['L'];pos=O[:3,3].copy();pos[2]-=np.sum(np.abs(O[2,:3])*payload['dims'])/2
  return np.r_[(pos-target)*10,(T[:3,0]-axis)*.3]
 candidates=[]
 for seed0 in [np.clip(seed,lo,hi)]+[np.random.default_rng(i).uniform(lo,hi) for i in range(8)]:
  s=least_squares(error,seed0,bounds=(lo,hi),max_nfev=600)
  if np.linalg.norm(error(s.x)[:3])<.002 and np.linalg.norm(error(s.x)[3:])<.001:candidates.append(s.x)
 if not candidates:raise RuntimeError('IK unreachable '+str(target))
 return min(candidates,key=lambda q:np.linalg.norm(q-seed)).tolist()
def vacuum_ok():
 if not vac or time.monotonic()-vac['t']>.5 or vac['value']>-200:raise RuntimeError('Vacuum not confirmed')
def wait_vacuum():
 deadline=time.monotonic()+3;since=time.monotonic();good=0
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.05)
  if vac and vac['t']>since and time.monotonic()-vac['t']<.3 and vac['value']<=-200:
   good+=1;since=vac['t']
   if good>=2:return
 raise RuntimeError('Fresh stable suction confirmation missing')
def acm_for(object_id,support):
 req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX;original=copy.deepcopy(call('/get_planning_scene',req,GetPlanningScene).scene.allowed_collision_matrix);acm=copy.deepcopy(original)
 for name in [object_id,support]:
  if name not in acm.entry_names:
   acm.entry_names.append(name)
   for row in acm.entry_values:row.enabled.append(False)
   acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
 i=acm.entry_names.index(object_id);j=acm.entry_names.index(support);acm.entry_values[i].enabled[j]=acm.entry_values[j].enabled[i]=True
 req=scene_request();req.scene.allowed_collision_matrix=acm;apply(req);return original

def move(goal,label,vertical=False):
 global handle,lost,bad
 start=fresh()
 if vertical and np.linalg.norm(fk(goal)[:3,3]-fk(start)[:3,3])>.011:
  begin=fk(start)[:3,3];end=fk(goal)[:3,3];count=int(np.ceil(np.linalg.norm(end-begin)/.01))
  for step in range(1,count+1):
   target=begin+(end-begin)*step/count;segment=goal if step==count else ik(target,fresh(),axis=fk(goal)[:3,0]);move(segment,label+' '+str(step)+'_of_'+str(count),True)
  return fresh()
 phase(label)
 for q in [start,goal]:
  req=GetStateValidity.Request();req.group_name='arm';req.robot_state=state(q);res=call('/check_state_validity',req,GetStateValidity)
  if not res.valid:raise RuntimeError('Invalid pose '+str([(c.contact_body_1,c.contact_body_2) for c in res.contacts]))
 sc=call('/soarm4/rviz_scaling');sc=json.loads(sc.message) if sc.success else None
 if not sc:raise RuntimeError('No RViz scaling')
 req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.pipeline_id='ompl';r.planner_id='RRTConnectkConfigDefault';r.num_planning_attempts=1;r.allowed_planning_time=8.;r.max_velocity_scaling_factor=float(sc['velocity']);r.max_acceleration_scaling_factor=float(sc['acceleration']);r.start_state=state(start);r.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=float(q),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for n,q in zip(names,goal)])]
 res=call('/plan_kinematic_path',req,GetMotionPlan).motion_plan_response
 if res.error_code.val!=1:raise RuntimeError('Planning failed '+str(res.error_code.val))
 if vertical:
  a=fk(start)[:3,3];b=fk(goal)[:3,3];trajectory=res.trajectory.joint_trajectory;indices=[trajectory.joint_names.index(n) for n in names]
  for point in trajectory.points:
   pos=fk([point.positions[i] for i in indices])[:3,3]
   if np.linalg.norm(pos[:2]-b[:2])>np.linalg.norm(a[:2]-b[:2])+.001 or not min(a[2],b[2])-.001<=pos[2]<=max(a[2],b[2])+.001:raise RuntimeError('Path leaves vertical corridor')
 if max(abs(a-b) for a,b in zip(fresh(),start))>math.radians(1):raise RuntimeError('Actual start changed during planning')
 if holding:vacuum_ok()
 action=ActionClient(node,ExecuteTrajectory,'/execute_trajectory')
 if not action.wait_for_server(timeout_sec=8):raise RuntimeError('Execution unavailable')
 goalmsg=ExecuteTrajectory.Goal();goalmsg.trajectory=res.trajectory;f=action.send_goal_async(goalmsg);rclpy.spin_until_future_complete(node,f,timeout_sec=8);handle=f.result()
 if not handle or not handle.accepted:raise RuntimeError('Execution rejected')
 lost=False;bad=0;f=handle.get_result_async();end=time.monotonic()+120
 while not f.done() and time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.05)
  if holding and (not vac or time.monotonic()-vac['t']>.5):lost=True;handle.cancel_goal_async()
 if not f.done():handle.cancel_goal_async();raise RuntimeError('Execution timeout; cancel requested')
 handle=None
 if f.result().result.error_code.val!=1 or lost:raise RuntimeError('Execution failed or suction lost; holding state retained')
 if holding:vacuum_ok()
 return fresh()

recorder=None
try:
 original=acm_for('sort_object_1','tool_link_cup_visual_link');acm_for('sort_object_1','cup_tip')
 recorder=subprocess.Popen([sys.executable,str(ROOT/'record_suction_diagnosis.py'),str(out)])
 try:
  q=fresh();target=fk(q)[:3,3].copy();target[2]+=.03;move(ik(target,q),'final empty retreat',True)
 finally:
  req=scene_request();req.scene.allowed_collision_matrix=original;apply(req)
 result=call('/soarm4/go_home',timeout=120)
 if not result.success:raise RuntimeError('Home failed '+result.message)
 status.update(success=True,final_joints=fresh(),pump='OFF',holding=False);phase('complete');(out/'execution.json').write_text(json.dumps(status,ensure_ascii=False,indent=2))
finally:
 if recorder:
  (out/'STOP').touch()
  try:recorder.wait(timeout=5)
  except subprocess.TimeoutExpired:recorder.terminate()
 node.destroy_node();rclpy.try_shutdown()
