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
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');p.add_argument('session',type=Path);args=p.parse_args();session=args.session.resolve();out=session/('suction_trial_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(exist_ok=False)
ctx=load_context(session);names=['joint'+str(i) for i in range(1,5)];lo=np.array([j['limit']['lower'] for j in m['joints']],float);hi=np.array([j['limit']['upper'] for j in m['joints']],float)
rclpy.init();node=rclpy.create_node('verified_suction_trial');latest={};vac={};turn={};holding=False;handle=None;bad=0;lost=False
pre_suction_gate=None
status={'success':False,'phases':[],'placements':[],'pressure_settings':{'grip_diff':-200,'hold_diff':-200,'max_feedback_age_s':.5,'bad_consecutive_samples':2}}
def joint(msg):
 if all(n in msg.name for n in names):
  latest.update(q=[float(msg.position[msg.name.index(n)]) for n in names],t=time.monotonic())
  if pre_suction_gate is not None:pre_suction_gate.update(fk(latest['q'])[:3,3])
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
 status['phases'].append({'phase':s,'time':time.time(),'pressure_diff':vac.get('value'),'pressure_age_s':time.monotonic()-vac['t'] if vac else None});(out/'phase.txt').write_text(s)
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

def move(goal,label,vertical=False,pump_target=None,pump_normal=None):
 global handle,lost,bad,pre_suction_gate
 start=fresh()
 if vertical and pump_target is None and np.linalg.norm(fk(goal)[:3,3]-fk(start)[:3,3])>.011:
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
 pump_client=None
 if pump_target is not None:
  from continuous_pre_suction import PreSuctionTrigger
  pump_client=node.create_client(Trigger,'/vacuum/pump_on')
  if not pump_client.wait_for_service(timeout_sec=8):raise RuntimeError('Pre-suction pump service unavailable')
 goalmsg=ExecuteTrajectory.Goal();goalmsg.trajectory=res.trajectory;f=action.send_goal_async(goalmsg);rclpy.spin_until_future_complete(node,f,timeout_sec=8);handle=f.result()
 if not handle or not handle.accepted:raise RuntimeError('Execution rejected')
 if pump_client is not None:
  pre_suction_gate=PreSuctionTrigger(pump_target,pump_normal,lambda:pump_client.call_async(Trigger.Request()))
 lost=False;bad=0;f=handle.get_result_async();end=time.monotonic()+120
 while not f.done() and time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.05)
  if holding and (not vac or time.monotonic()-vac['t']>.5):lost=True;handle.cancel_goal_async()
 if not f.done():handle.cancel_goal_async();raise RuntimeError('Execution timeout; cancel requested')
 handle=None
 if pump_client is not None:
  gate=pre_suction_gate;pre_suction_gate=None
  try:
   if gate.future is None:raise RuntimeError('Pre-suction threshold was not observed; no lift')
   rclpy.spin_until_future_complete(node,gate.future,timeout_sec=3)
   if not gate.future.done() or not gate.future.result().success:raise RuntimeError('Pre-suction pump command failed')
   status['pre_suction']={'threshold_m':.0015,'observed_distance_m':gate.distance,'sensor_gate':False,'motion_stopped_for_trigger':False}
  finally:node.destroy_client(pump_client)
 if f.result().result.error_code.val!=1 or lost:raise RuntimeError('Execution failed or suction lost; holding state retained')
 if holding:vacuum_ok()
 return fresh()

from soarm4_workcell.cup_goal import Arm,solve,tcp_point
from soarm4_workcell.block_measure import measure_now,measure_dir
import yaml
wc=yaml.safe_load((session/'workcell_snapshot.yaml').read_text());arm=Arm();manifest=json.loads((session/'session.json').read_text());candidate_file=Path(json.loads((session/'reconstruction_sam2/processing_status.json').read_text())['candidate_file']);surface=json.loads(candidate_file.read_text());candidate=surface['selected_candidate'];point=np.array(candidate['nominal_contact_position_table_m']);normal=np.array(surface['results'][candidate['camera']]['normal_table']);normal/=np.linalg.norm(normal);object_id='scanned_block';recorder=None;restore_acm=None
try:
 if not surface['results'][candidate['camera']]['geometry_pass']:raise RuntimeError('Suction geometry failed')
 if time.time()-candidate_file.stat().st_mtime>1800:raise RuntimeError('Candidate too old')
 before=measure_dir(session/manifest['views'][0]['path']);now=measure_now(str(out/'fresh_observation'))
 if not before or not now or np.linalg.norm(np.array(before['xy_m'])-now['xy_m'])>.003 or abs(before['z_m']-now['z_m'])>.003:raise RuntimeError('Object moved since scan; no motion')
 q=fresh();deadline=time.monotonic()+5
 while (not turn or not vac) and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
 if not turn or abs(turn['angle_deg'])>1 or turn['zero_count']!=manifest['return_to_zero']['state']['zero_count']:raise RuntimeError('Turntable zero changed')
 if not vac or time.monotonic()-vac['t']>.5 or vac['value']<=-200:raise RuntimeError('Vacuum baseline not clear')
 assert np.linalg.norm(fk(q)[:3,3]-(W@(arm.fk(q)@tcp_point(wc)))[:3])<1e-6
 above=point+normal*.03;qa=solve(arm,wc,above);qc=solve(arm,wc,point)
 if qa is None or qc is None:raise RuntimeError('IK unreachable')
 cloud=np.load(session/'analysis/cloud.npz')['points'];low=np.percentile(cloud,.5,axis=0);high=np.percentile(cloud,99.5,axis=0);low[2]=wc['turntable']['top_height_m'];low[:2]-=.002;high+=.002;dims=high-low;O=np.eye(4);O[:3,3]=(high+low)/2;co=CollisionObject();co.id=object_id;co.header.frame_id='world';co.operation=CollisionObject.ADD;co.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=dims.tolist())];co.primitive_poses=[pose(np.linalg.inv(W)@O)];req=scene_request();req.scene.world.collision_objects=[co];apply(req)
 status.update(surface_point_table_m=point.tolist(),source_candidate=str(candidate_file),before_measurement=before,fresh_measurement=now,scope='3cm approach, contact/grip,10mm lift, return to same support, release, retreat,home; no sorting transfer')
 if not args.execute:phase('preflight_only');raise SystemExit(0)
 env=os.environ.copy();recorder=subprocess.Popen([sys.executable,str(ROOT/'record_suction_diagnosis.py'),str(out)],env=env);deadline=time.monotonic()+10
 while not(out/'READY').exists():
  if time.monotonic()>deadline:raise RuntimeError('Recorder not ready')
  rclpy.spin_once(node,timeout_sec=.05)
 move(qa.tolist(),'approach_3cm')
 restore_acm=acm_for(object_id,'tool_link_cup_visual_link');acm_for(object_id,'cup_tip')
 move(qc.tolist(),'continuous_contact_approach',True,pump_target=point,pump_normal=normal)
 phase('grip_resting');grip=call('/vacuum/grip',timeout=12);status['first_grip']=grip.message
 if not grip.success:
  from grip_retry import retry_targets
  q=fresh();initial_contact=fk(q);status['retries']=[]
  for attempt,target in enumerate(retry_targets(initial_contact[:3,3],normal),1):
   # Keep pump ON. No stale sensor data may authorize another inward step.
   rclpy.spin_once(node,timeout_sec=.05)
   if not vac or time.monotonic()-vac['t']>.5:raise RuntimeError('Pressure feedback stale; no further retry')
   q=fresh();retry_goal=ik(target,q,axis=initial_contact[:3,0])
   record={'attempt':attempt,'step_m':.001,'total_additional_approach_m':attempt*.001,'target_table_m':target.tolist(),'reference':'first failed actual TCP; bounded absolute inward-normal offsets'};status['retries'].append(record)
   move(retry_goal,'grip_retry_'+str(attempt)+'_of_3',True);phase('grip_retry_resting_'+str(attempt));grip=call('/vacuum/grip',timeout=12);record.update(success=grip.success,pressure_result=grip.message)
   if grip.success:break
  if not grip.success:
   released=call('/vacuum/release',timeout=15);status['release_after_failed_retry']=released.message;raise RuntimeError('Grip failed after three1mm retries; no lift')
 holding=True;wait_vacuum();q=fresh();Tcontact=fk(q);L=np.linalg.inv(Tcontact)@O;attached=AttachedCollisionObject();attached.link_name='cup_tip';attached.touch_links=['cup_tip','tool_link_cup_visual_link'];attached.object.id=object_id;attached.object.header.frame_id='cup_tip';attached.object.operation=CollisionObject.ADD;attached.object.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=dims.tolist())];attached.object.primitive_poses=[pose(L)];req=scene_request();req.scene.robot_state.attached_collision_objects=[attached];apply(req);acm_for(object_id,'workcell_turntable')
 target=Tcontact[:3,3]+[0,0,.01];move(ik(target,q,axis=Tcontact[:3,0]),'lift_10mm',True);q=fresh();status['lift']={'tcp_contact_m':Tcontact[:3,3].tolist(),'tcp_lifted_m':fk(q)[:3,3].tolist(),'encoder_height_change_mm':float((fk(q)[2,3]-Tcontact[2,3])*1000),'pressure_diff':vac['value']};phase('holding_10mm')
 end=time.monotonic()+2
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05);vacuum_ok()
 move(ik(Tcontact[:3,3],fresh(),axis=Tcontact[:3,0]),'return_same_surface',True);phase('release_resting');released=call('/vacuum/release',timeout=15);status['release']=released.message
 if not released.success:raise RuntimeError('Release not confirmed')
 holding=False;attached=AttachedCollisionObject();attached.link_name='cup_tip';attached.object.id=object_id;attached.object.operation=CollisionObject.REMOVE;req=scene_request();req.scene.robot_state.attached_collision_objects=[attached];apply(req)
 q=fresh();move(ik(fk(q)[:3,3]+[0,0,.03],q,axis=fk(q)[:3,0]),'empty_retreat',True)
 req=scene_request();req.scene.allowed_collision_matrix=restore_acm;apply(req);restore_acm=None
 phase('home');res=call('/soarm4/go_home',timeout=120)
 if not res.success:raise RuntimeError('Home failed '+res.message)
 status.update(success=True,holding=False,pump='OFF after confirmed release',final_q=fresh());phase('complete')
except BaseException as exc:
 if isinstance(exc,SystemExit) and exc.code==0:raise
 status.update(success=False,error=str(exc),holding=holding,notice='If holding keep suction; no automatic release/drop');phase('failed');raise
finally:
 if restore_acm is not None:
  try:req=scene_request();req.scene.allowed_collision_matrix=restore_acm;apply(req)
  except Exception as exc:print('ACM restore failed',str(exc),flush=True)
 if recorder:
  (out/'STOP').touch()
  try:recorder.wait(timeout=5)
  except subprocess.TimeoutExpired:recorder.terminate()
 node.destroy_node();rclpy.try_shutdown()
