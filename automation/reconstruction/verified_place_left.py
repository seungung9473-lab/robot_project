"""Two reconstructed objects -> robot left/right pads, with attached-body planning."""
import argparse,copy,json,math,os,subprocess,sys,time
from vacuum_thresholds import HELD_DIFF
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
from test_data_recorder import start_trial
p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');p.add_argument('--resume-held',type=Path);p.add_argument('--finish-above',action='store_true',help='After release, retreat and wait above the pad instead of HOME');p.add_argument('--placement-plan',type=Path);p.add_argument('--placement-object');p.add_argument('--inclined',action='store_true');p.add_argument('--pad',type=int,choices=[5,6],default=5);p.add_argument('session',type=Path);args=p.parse_args();session=args.session.resolve();out=session/('placement_left_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(exist_ok=False)
trial=start_trial('physical_pick_place' if args.execute else 'pick_place_preview',physical=args.execute,source=session);trial.attach(out/'execution.json');trial.attach(session/'candidate.json');trial.attach(session/'inclined_context.json');trial.attach(session/'collision_plan_check.json')
packing_path=args.placement_plan or (session/'placement_plan.json');packing=json.loads(packing_path.read_text()) if packing_path.exists() else None
packing_key=args.placement_object
if packing is not None:
 if not packing_key or packing_key not in packing['placements']:raise RuntimeError('Select a reserved placement object before moving')
 if packing.get('pad_id')!=args.pad:raise RuntimeError('Placement reservation pad mismatch')
 if packing_key in packing.get('completed',{}):raise RuntimeError('This placement is already completed')
ctx=load_context(session);names=['joint'+str(i) for i in range(1,5)];lo=np.array([j['limit']['lower'] for j in m['joints']],float);hi=np.array([j['limit']['upper'] for j in m['joints']],float)
rclpy.init();node=rclpy.create_node('verified_suction_trial');latest={};vac={};turn={};holding=False;handle=None;bad=0;lost=False
pre_suction_gate=None
status={'success':False,'phases':[],'placements':[],'pressure_settings':{'grip_diff':HELD_DIFF,'hold_diff':HELD_DIFF,'max_feedback_age_s':.5,'bad_consecutive_samples':2}}
def joint(msg):
 if all(n in msg.name for n in names):
  latest.update(q=[float(msg.position[msg.name.index(n)]) for n in names],t=time.monotonic())
  trial.sample('joint_states',names=names,position_rad=latest['q'])
  if pre_suction_gate is not None:pre_suction_gate.update(fk(latest['q'])[:3,3])
def pressure(msg):
 global bad,lost
 trial.sample('vacuum',diff=int(msg.data))
 vac.update(value=msg.data,t=time.monotonic());bad=bad+1 if msg.data>HELD_DIFF else 0
 if handle is not None and holding and bad>=2:lost=True;handle.cancel_goal_async()
node.create_subscription(JointState,'/joint_states',joint,10);node.create_subscription(Int32,'/vacuum/diff',pressure,10);node.create_subscription(String,'/turntable/state',lambda msg:turn.update(json.loads(msg.data)),10)
state_validity_clients={}

def call(name,req=None,typ=Trigger,timeout=45):
 keep_client=typ is GetStateValidity
 if keep_client:
  if name not in state_validity_clients:state_validity_clients[name]=node.create_client(typ,name)
  c=state_validity_clients[name]
 else:
  c=node.create_client(typ,name)
 try:
  if not c.wait_for_service(timeout_sec=8):raise RuntimeError('Missing service '+name)
  f=c.call_async(req or typ.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=timeout)
  if not f.done():raise RuntimeError('Service timeout '+name)
  return f.result()
 finally:
  if not keep_client:node.destroy_client(c)
def fresh():
 end=time.monotonic()+8
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.05)
  if latest and time.monotonic()-latest['t']<.3:return latest['q'][:]
 raise RuntimeError('No fresh actual joints')
def phase(s):
 trial.event('phase',phase=s,pressure_diff=vac.get('value'),holding=holding)
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
 if not vac or time.monotonic()-vac['t']>.5 or vac['value']>HELD_DIFF:raise RuntimeError('Vacuum not confirmed')
def seal_stable():
 """흡착 판정 뒤 SEAL_STABLE_S 동안 새 압력 값이 모두 HELD_DIFF 이하로 유지되는지 확인(약한 붙음은 들기 전에 거른다)."""
 from seal_stability import SEAL_STABLE_S,evaluate
 samples=[];last=0.;end=time.monotonic()+SEAL_STABLE_S
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.05)
  if vac and vac['t']>last and time.monotonic()-vac['t']<.3:last=vac['t'];samples.append((vac['t'],vac['value']))
 return evaluate(samples,HELD_DIFF)
def wait_vacuum():
 deadline=time.monotonic()+3;since=time.monotonic();good=0
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.05)
  if vac and vac['t']>since and time.monotonic()-vac['t']<.3 and vac['value']<=HELD_DIFF:
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

from grip_retry import SLOW_CONTACT_SCALING
from transit_clearance import RETREAT_UP_M,TRANSIT_MIN_TCP_Z_M,OBSTACLE_PADDING_M,min_tcp_z
def require_empty_cup():
 if holding or not vac or time.monotonic()-vac['t']>.5 or vac['value']<=HELD_DIFF:
  raise RuntimeError('Waiting pose correction requires fresh empty-cup feedback')


def move(goal,label,vertical=False,pump_target=None,pump_normal=None,speed_cap=None,ptp=False,min_tcp_z=None,stop_on_seal=False,require_empty=False):
 global handle,lost,bad,pre_suction_gate
 start=fresh()
 continuous_retreat=vertical and not holding and pump_target is None and label.startswith('empty_retreat_from_pad_ID')
 phase(label)
 for q in [start,goal]:
  req=GetStateValidity.Request();req.group_name='arm';req.robot_state=state(q);res=call('/check_state_validity',req,GetStateValidity)
  if not res.valid:raise RuntimeError('Invalid pose '+str([(c.contact_body_1,c.contact_body_2) for c in res.contacts]))
 sc=call('/soarm4/rviz_scaling');sc=json.loads(sc.message) if sc.success else None
 if not sc:raise RuntimeError('No RViz scaling')
 if speed_cap:sc={'velocity':min(float(sc['velocity']),speed_cap[0]),'acceleration':min(float(sc['acceleration']),speed_cap[1])}  # 느리게 접근: 현재 RViz 비율보다 클 수 없음
 req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.pipeline_id='pilz_industrial_motion_planner' if ptp else 'ompl';r.planner_id='PTP' if ptp else 'RRTConnectkConfigDefault';r.num_planning_attempts=1;r.allowed_planning_time=8.;r.max_velocity_scaling_factor=float(sc['velocity']);r.max_acceleration_scaling_factor=float(sc['acceleration']);r.start_state=state(start);r.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=float(q),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for n,q in zip(names,goal)])]
 from trajectory_cache import compatible,restore
 from types import SimpleNamespace
 cache_file=session/'reviewed_trajectories.json'
 entry=json.loads(cache_file.read_text()).get(label) if cache_file.exists() else None
 reuse=compatible(entry,names,start,goal,sc,start_tolerance_deg=.25 if vertical else 1.,fixed_axis=vertical)
 if reuse:
  trajectory=restore(entry)
  if vertical:
   # Fixed-axis moves need a closer TCP match than the joint tolerance alone.
   first=trajectory.joint_trajectory.points[0].positions
   reuse=np.linalg.norm(fk(first)[:3,3]-fk(start)[:3,3])<=.0005
 if reuse:
  res=SimpleNamespace(trajectory=trajectory)
  status.setdefault('reused_trajectories',[]).append(label)
 elif vertical:
  from normal_contact_trajectory import contact_samples,trajectory_from_samples
  limits=yaml.safe_load((Path.home()/'robot_ws/src/soarm4_moveit_config/config/joint_limits.yaml').read_text())['joint_limits']
  velocities=[float(limits[n]['max_velocity'])*float(sc['velocity']) for n in names]
  accelerations=[float(limits[n]['max_acceleration'])*float(sc['acceleration']) for n in names]
  samples=contact_samples(fk,lo,hi,start,goal,velocities,accelerations)
  res=SimpleNamespace(trajectory=trajectory_from_samples(names,samples))
 else:
  res=call('/plan_kinematic_path',req,GetMotionPlan).motion_plan_response
  if res.error_code.val!=1:raise RuntimeError('Planning failed '+str(res.error_code.val))
 if reuse or vertical or require_empty:
  for sample in res.trajectory.joint_trajectory.points:
   validity=GetStateValidity.Request();validity.group_name='arm';validity.robot_state=state(sample.positions)
   if not call('/check_state_validity',validity,GetStateValidity).valid:
    raise RuntimeError('Trajectory invalid in current scene; stop batch')
 if min_tcp_z is not None:
  _tj=res.trajectory.joint_trajectory;_idx=[_tj.joint_names.index(n) for n in names];_z=min(float(fk([pt.positions[i] for i in _idx])[2,3]) for pt in _tj.points)
  if _z<min_tcp_z:raise RuntimeError('Transit path rejected: tool tip dips to %.0f mm, below %.0f mm clearance'%(_z*1000,min_tcp_z*1000))
 if vertical:
  a=fk(start)[:3,3];b=fk(goal)[:3,3];trajectory=res.trajectory.joint_trajectory;indices=[trajectory.joint_names.index(n) for n in names]
  for point in trajectory.points:
   pos=fk([point.positions[i] for i in indices])[:3,3]
   if vertical:
    line=b-a;length=np.linalg.norm(line);direction=line/max(length,1e-9);along=float((pos-a)@direction);lateral=np.linalg.norm(pos-a-direction*along)
    if lateral>.001 or not -.001<=along<=length+.001:raise RuntimeError('Contact path leaves surface-normal corridor')
    T=fk([point.positions[i] for i in indices]);axis_error=np.degrees(np.arccos(np.clip(T[:3,0]@fk(goal)[:3,0],-1,1)))
    if axis_error>1.:raise RuntimeError('Contact path changes cup orientation')
   elif np.linalg.norm(pos[:2]-b[:2])>np.linalg.norm(a[:2]-b[:2])+.001 or not min(a[2],b[2])-.001<=pos[2]<=max(a[2],b[2])+.001:raise RuntimeError('Path leaves vertical corridor')
 if max(abs(a-b) for a,b in zip(fresh(),start))>math.radians(1):raise RuntimeError('Actual start changed during planning')
 if holding:vacuum_ok()
 action=ActionClient(node,ExecuteTrajectory,'/execute_trajectory')
 if not action.wait_for_server(timeout_sec=8):raise RuntimeError('Execution unavailable')
 pump_client=None
 if pump_target is not None:
  from continuous_pre_suction import PreSuctionTrigger
  pump_client=node.create_client(Trigger,'/vacuum/pump_on')
  if not pump_client.wait_for_service(timeout_sec=8):raise RuntimeError('Pre-suction pump service unavailable')
 # Compare the measured pose against the trajectory actually being dispatched.
 joint_path=res.trajectory.joint_trajectory
 first_positions=dict(zip(joint_path.joint_names,joint_path.points[0].positions))
 actual=fresh()
 if max(abs(q-first_positions[name]) for q,name in zip(actual,names))>=math.radians(1):
  raise RuntimeError('Actual pose no longer matches trajectory start within 1 degree')
 if require_empty:require_empty_cup()
 goalmsg=ExecuteTrajectory.Goal();goalmsg.trajectory=res.trajectory;f=action.send_goal_async(goalmsg);rclpy.spin_until_future_complete(node,f,timeout_sec=8);handle=f.result()
 if not handle or not handle.accepted:raise RuntimeError('Execution rejected')
 if pump_client is not None:
  pre_suction_gate=PreSuctionTrigger(pump_target,pump_normal,lambda:pump_client.call_async(Trigger.Request()))
 lost=False;bad=0;f=handle.get_result_async();end=time.monotonic()+120;sealed_early=False;seal_good=0;seal_last=0.
 while not f.done() and time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.05)
  if holding and (not vac or time.monotonic()-vac['t']>.5):lost=True;handle.cancel_goal_async()
  # 재접근 중 압력이 기준(HELD_DIFF)을 새 값 2개 연속 넘으면 남은 거리를 더 누르지 않고 그 자리에서 멈춘다.
  if stop_on_seal and not sealed_early and vac and vac['t']>seal_last and time.monotonic()-vac['t']<.3:
   seal_last=vac['t'];seal_good=seal_good+1 if vac['value']<=HELD_DIFF else 0
   if seal_good>=2:sealed_early=True;status['stopped_on_seal']={'label':label,'pressure_diff':vac['value']};handle.cancel_goal_async()
 if not f.done():handle.cancel_goal_async();raise RuntimeError('Execution timeout; cancel requested')
 handle=None
 if pump_client is not None:
  gate=pre_suction_gate;pre_suction_gate=None
  try:
   if gate.future is None:raise RuntimeError('Pre-suction threshold was not observed; no lift')
   rclpy.spin_until_future_complete(node,gate.future,timeout_sec=3)
   if not gate.future.done() or not gate.future.result().success:raise RuntimeError('Pre-suction pump command failed')
   status['pre_suction']={'threshold_m':.005,'observed_distance_m':gate.distance,'sensor_gate':False,'motion_stopped_for_trigger':False}
  finally:node.destroy_client(pump_client)
 if (f.result().result.error_code.val!=1 and not sealed_early) or lost:raise RuntimeError('Execution failed or suction lost; holding state retained')
 if holding:vacuum_ok()
 return fresh()

from soarm4_workcell.cup_goal import Arm,solve,tcp_point
from soarm4_workcell.block_measure import measure_now,measure_dir
import yaml
trash_context=json.loads((session/'inclined_context.json').read_text()) if (session/'inclined_context.json').exists() else {};is_trash=trash_context.get('destination')=='trash_drop'
wc=yaml.safe_load((session/'workcell_snapshot.yaml').read_text());arm=Arm();manifest=json.loads((session/'session.json').read_text());candidate_file=Path(json.loads((session/'reconstruction_sam2/processing_status.json').read_text())['candidate_file']);surface=json.loads(candidate_file.read_text());candidate=surface['selected_candidate'];point=np.array(candidate['nominal_contact_position_table_m']);normal=np.array(surface['results'][candidate['camera']]['normal_table']);normal/=np.linalg.norm(normal);object_id='scanned_block';recorder=None;restore_acm=None
# Coupled payload placement: preserve 3-degree radial sag compensation while
# placing the observed object's bottom/centre rather than assuming cup XY is its centre.
from soarm4_workcell.cup_goal import J1_WORLD
from placement_geometry import depth_table
import cv2
j1=(W@np.r_[J1_WORLD,1.])[:2]
payload_pitch=None
def payload_goal(bottom_target,seed,L,dims):
 from payload_placement import place_payload
 tilt=payload_pitch if payload_pitch is not None else math.radians(wc['robot_registration'].get('sag_compensation',{}).get('tool_tilt_away_from_j1_deg',0.))
 result=place_payload(fk,lo,hi,bottom_target,seed,L,dims,j1,tilt)
 if result is None:raise RuntimeError('Payload placement IK unreachable')
 return result.tolist()

def pad_clearance(view,pad):
 md=json.loads((view/'metadata.json').read_text());checks={}
 for cam in ['cam1','cam2']:
  c=md['cameras'][cam];raw=cv2.imread(str(view/c['depth_file']),-1);P,valid=depth_table(raw,.001 if raw.dtype==np.uint16 else 1.,c['camera_info'],wc['cameras'][cam]['T_table_optical']);rr=np.linalg.norm(P[:,:,:2]-np.array(pad['center_table_m'])[:2],axis=2);surface=valid&(rr<.04)&(np.abs(P[:,:,2]-pad['center_table_m'][2])<.012);obstacle=valid&(rr<.048)&(P[:,:,2]>pad['center_table_m'][2]+.012)&(P[:,:,2]<.25);
  if packing is not None:
   from scipy.spatial import ConvexHull
   polygon=np.array(packing['placements'][packing_key]['reserved_polygon_table_m']);hull=ConvexHull(polygon);inside=np.all(P[:,:,:2]@hull.equations[:,:2].T+hull.equations[:,2]<=.002,axis=2)
   surface=valid&inside&(np.abs(P[:,:,2]-pad['center_table_m'][2])<.012);obstacle=obstacle&inside
  checks[cam]={'surface_pixels':int(surface.sum()),'elevated_pixels':int(obstacle.sum())}
 from pad_clearance_policy import judge_pad
 try:status['pad_clearance_mode']=judge_pad(checks,packing is not None)
 finally:status['pad_clearance_counts']=checks
 return checks

stage_start_q=None
def dispose_at_trash():
 global holding,restore_acm
 from trash_destination import load_trash_pose,arrival_confirmed
 taught=load_trash_pose()
 if taught!=trash_context.get('destination_pose'):raise RuntimeError('Taught trash pose changed')
 goal=list(map(float,taught['positions_rad']))
 if any(q<l or q>h for q,l,h in zip(goal,lo,hi)):raise RuntimeError('Trash pose outside joint limits')
 move(goal,'transfer_to_trash_drop')
 # Never vent merely because a plan finished: require measured stable arrival.
 deadline=time.monotonic()+.5
 while time.monotonic()<deadline:
  actual=fresh();vacuum_ok()
  if not arrival_confirmed(actual,goal):raise RuntimeError('Trash release blocked: actual joints not at taught drop pose')
 phase('release_at_trash_drop');status['drop_q_rad']=fresh();status['destination']='trash_drop'
 released=call('/vacuum/release',timeout=15);status['release']=released.message
 if not released.success:raise RuntimeError('Trash release not confirmed')
 holding=False;remove=AttachedCollisionObject();remove.link_name='cup_tip';remove.object.id=object_id;remove.object.operation=CollisionObject.REMOVE;req=scene_request();req.scene.robot_state.attached_collision_objects=[remove];gone=CollisionObject();gone.id=object_id;gone.operation=CollisionObject.REMOVE;req.scene.world.collision_objects=[gone];apply(req)
 req=scene_request();req.scene.allowed_collision_matrix=restore_acm;apply(req);restore_acm=None
 if args.finish_above:
  if stage_start_q is None:raise RuntimeError('Stage start pose unknown; cannot return after trash release')
  move(stage_start_q,'return_to_stage_start_after_trash',ptp=True)  # 폐기 후 이 물체 시작 자세(이전 대기 자세/HOME)로 후퇴
  phase('wait_above_destination')
 else:
  phase('home');res=call('/soarm4/go_home',timeout=120)
  if not res.success:raise RuntimeError('Home failed '+res.message)
 status.update(success=True,holding=False,pump='OFF after confirmed trash release',final_q=fresh(),scope='pick -> taught trash pose -> vent -> wait above' if args.finish_above else 'pick -> taught trash pose -> vent -> home; falling object/bin contents not visually verified');phase('complete')

# An explicitly requested held-object continuation never replays the approach,
# rotates the table, restarts the pump, or estimates a new attachment transform.
if args.resume_held is not None:
 try:
  if not args.execute or not args.inclined:raise RuntimeError('Held resume requires explicit inclined execution')
  previous=args.resume_held.resolve()
  prior=json.loads(previous.read_text())
  if previous.parent.parent!=session or prior.get('success') or not prior.get('holding') or prior.get('pad_id')!=args.pad:raise RuntimeError('Invalid held checkpoint')
  if wc!=yaml.safe_load((Path.home()/'robot_ws/src/soarm4_workcell/config/workcell.yaml').read_text()):raise RuntimeError('Workcell changed during held stop')
  q=fresh();holding=True;wait_vacuum();q=fresh()
  end=time.monotonic()+3
  while not turn and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
  if not turn or abs(turn['angle_deg']-manifest['aligned_turntable_angle_deg'])>1 or turn['zero_count']!=manifest['return_to_zero']['state']['zero_count']:raise RuntimeError('Held table pose changed')
  query=GetPlanningScene.Request();query.components.components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS|PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
  current_scene=call('/get_planning_scene',query,GetPlanningScene).scene
  attachments=[a for a in current_scene.robot_state.attached_collision_objects if a.object.id=='scanned_block']
  if len(attachments)!=1 or attachments[0].link_name!='cup_tip':raise RuntimeError('Exact held object attachment missing')
  attached=attachments[0];co=copy.deepcopy(attached.object)
  if len(co.mesh_poses)!=1 or len(co.meshes)!=1:raise RuntimeError('Expected one observed held mesh')
  pp=co.mesh_poses[0];L=np.eye(4);L[:3,:3]=Rotation.from_quat([pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w]).as_matrix();L[:3,3]=[pp.position.x,pp.position.y,pp.position.z]
  if hasattr(co,'pose'):
   body=co.pose;B=np.eye(4);quat=[body.orientation.x,body.orientation.y,body.orientation.z,body.orientation.w]
   if np.linalg.norm(quat)>1e-9:B[:3,:3]=Rotation.from_quat(quat).as_matrix()
   B[:3,3]=[body.position.x,body.position.y,body.position.z];L=B@L
  cloud=np.load(session/'analysis/cloud.npz')['points'];low=np.percentile(cloud,.5,0);high=np.percentile(cloud,99.5,0);low[2]=wc['turntable']['top_height_m'];low[:2]-=.002;high+=.002;dims=high-low
  from payload_placement import centered_payload_origin
  expected_origin=centered_payload_origin(cloud,(high+low)/2)
  # MoveIt can bake mesh_poses into vertices and return identity poses. Recover
  # the same rigid frame from corresponding canonical hull vertices, never
  # from camera data or an assumed contact. Reject changed geometry/order.
  from collision_geometry import convex_mesh
  canonical=convex_mesh(cloud,expected_origin)
  source_vertices=np.array([[v.x,v.y,v.z] for v in canonical.vertices])
  target_vertices=np.array([[v.x,v.y,v.z] for v in co.meshes[0].vertices])@L[:3,:3].T+L[:3,3]
  if source_vertices.shape!=target_vertices.shape:raise RuntimeError('Held mesh geometry changed')
  x0=source_vertices.mean(0);y0=target_vertices.mean(0);u,_,vt=np.linalg.svd((source_vertices-x0).T@(target_vertices-y0));rr=vt.T@u.T
  if np.linalg.det(rr)<0:vt[-1]*=-1;rr=vt.T@u.T
  tt=y0-rr@x0
  residual=np.max(np.linalg.norm(source_vertices@rr.T+tt-target_vertices,axis=1))
  if residual>1e-6:raise RuntimeError('Held canonical mesh correspondence mismatch')
  L=np.eye(4);L[:3,:3]=rr;L[:3,3]=tt;co.meshes=[canonical];co.mesh_poses=[pose(np.eye(4))];co.pose=pose(np.eye(4));O=fk(q)@L
  status['attachment_rigid_fit_max_error_m']=float(residual)
  if np.linalg.norm(O[:3,3]-expected_origin)>.005:raise RuntimeError('Held object pose differs from stopped contact by >5mm')
  restore_acm=copy.deepcopy(current_scene.allowed_collision_matrix)
  radial=fk(q)[:2,3]-j1;radial/=np.linalg.norm(radial);payload_pitch=math.atan2(float(fk(q)[:2,0]@radial),float(-fk(q)[2,0]))
  pad_key='pad2' if args.pad==5 else 'pad1';pad=wc['pads'][pad_key];pad_nominal=np.array(pad['center_table_m']);pad_target=pad_nominal.copy();pad_target[:2]+=wc.get('place_correction_m',{}).get('pad_ID'+str(args.pad),[0.,0.])
  if packing is not None:raise RuntimeError('Held resume with shared-pad packing is not supported')
  hover=payload_goal(pad_target+[0,0,.03],q,L,dims);place_q=payload_goal(pad_target,hover,L,dims)
  placed=fk(place_q)@L
  if np.degrees(np.arccos(np.clip(placed[2,2],-1,1)))>1:raise RuntimeError('Held placement would tilt payload')
  vertices=np.array([[v.x,v.y,v.z] for v in co.meshes[0].vertices]);placed_vertices=vertices@placed[:3,:3].T+placed[:3,3]
  if np.max(np.linalg.norm(placed_vertices[:,:2]-pad_nominal[:2],axis=1))>pad['diameter_m']/2-.002:raise RuntimeError('Held payload footprint exceeds pad')
  status.update(resumed_from=str(previous),pad_id=args.pad,attachment_transform_preserved=True,scope=f'held contact -> lift -> ID{args.pad} release -> '+('wait above' if args.finish_above else 'home')+'; no camera/table/approach replay')
  acm_for(object_id,'workcell_turntable');q=fresh();begin=fk(q)
  move(ik(begin[:3,3]+[0,0,float(candidate.get('lift_clearance_m',.055))],q,axis=begin[:3,0]),'resume_held_lift',True)
  status['lift']={'encoder_height_change_mm':float((fk(fresh())[2,3]-begin[2,3])*1000)}
  req=scene_request();req.scene.allowed_collision_matrix=restore_acm;apply(req)
  move(hover,'transfer_to_pad_ID'+str(args.pad))
  acm_for(object_id,'workcell_'+pad_key);move(place_q,'lower_payload_to_pad_ID'+str(args.pad),True)
  phase('release_on_pad_ID'+str(args.pad));status['predicted_placed_object_pose_table']=(fk(fresh())@L).tolist();released=call('/vacuum/release',timeout=15);status['release']=released.message
  if not released.success:raise RuntimeError('Release not confirmed')
  holding=False;remove=AttachedCollisionObject();remove.link_name='cup_tip';remove.object.id=object_id;remove.object.operation=CollisionObject.REMOVE;req=scene_request();req.scene.robot_state.attached_collision_objects=[remove];apply(req)
  placed_object=copy.deepcopy(co);placed_object.id='placed_pad_'+str(args.pad)+'_'+out.name;placed_object.header.frame_id='world';placed_object.operation=CollisionObject.ADD;placed_object.mesh_poses=[pose(np.linalg.inv(W)@np.array(status['predicted_placed_object_pose_table']))];req=scene_request();removal=CollisionObject();removal.id=object_id;removal.operation=CollisionObject.REMOVE;req.scene.world.collision_objects=[placed_object,removal];apply(req)
  acm_for(placed_object.id,'tool_link_cup_visual_link');acm_for(placed_object.id,'cup_tip');q=fresh();move(ik(fk(q)[:3,3]+[0,0,RETREAT_UP_M],q,axis=fk(q)[:3,0]),'empty_retreat_from_pad_ID'+str(args.pad),True)
  req=scene_request();req.scene.allowed_collision_matrix=restore_acm;apply(req);restore_acm=None
  if args.finish_above:
   phase('wait_above_destination')
  else:
   phase('home');res=call('/soarm4/go_home',timeout=120)
   if not res.success:raise RuntimeError('Home failed '+res.message)
  status.update(success=True,holding=False,pump='OFF after confirmed release',final_q=fresh());phase('complete')
 except BaseException as exc:
  status.update(success=False,error=str(exc),holding=holding,notice='Keep pump and torque after held error');phase('failed');raise
 finally:
  if restore_acm is not None:
   req=scene_request();req.scene.allowed_collision_matrix=restore_acm;apply(req)
  node.destroy_node();rclpy.try_shutdown()
 raise SystemExit(0)

try:
 if not surface['results'][candidate['camera']]['geometry_pass']:raise RuntimeError('Suction geometry failed')
 if time.time()-candidate_file.stat().st_mtime>1800:raise RuntimeError('Candidate too old')
 if args.inclined:
  if manifest.get('contact_execution_mode')!='precomputed_prediction_only':raise RuntimeError('Prepare prediction-only execution before rotation')
  if not is_trash and not 0<=time.time()-float(manifest['pre_rotation_observation_time'])<180:raise RuntimeError('Pre-rotation destination observation expired')
  before={'mode':'precomputed_prediction_only','post_rotation_camera_used':False}
  now={'contact_point_table_m':point.tolist(),'normal_table':normal.tolist(),'contact_changed':False,'normal_changed':False,'post_rotation_camera_used':False}
  pad_view=session/'pre_rotation_observation/view_000'
 else:
  before=measure_dir(session/manifest['views'][0]['path']);now=measure_now(str(out/'fresh_observation'))
  if not before or not now or np.linalg.norm(np.array(before['xy_m'])-now['xy_m'])>.003 or abs(before['z_m']-now['z_m'])>.003:raise RuntimeError('Object moved since scan; no motion')
 if not is_trash:
  pad_key='pad2' if args.pad==5 else 'pad1';pad=wc['pads'][pad_key];status['pad_clearance']=pad_clearance(pad_view if args.inclined else out/'fresh_observation/view_000',pad)
  pad_nominal=np.array(pad['center_table_m']);pad_target=pad_nominal.copy();pad_target[:2]+=np.asarray(wc.get('place_correction_m',{}).get('pad_ID'+str(args.pad),[0.,0.]));
  if packing is not None:
   if np.linalg.norm(np.array(packing['pad_center_table_m'])-pad_nominal)>.0001:raise RuntimeError('Pad calibration changed; regenerate reservations')
   pad_target[:2]+=np.array(packing['placements'][packing_key]['offset_xy_m']);status['placement_reservation']={'file':str(packing_path),'object':packing_key}
  status.update(pad_id=args.pad,pad_nominal_center_m=pad_nominal.tolist(),commanded_payload_bottom_target_m=pad_target.tolist())
 else:
  from trash_destination import load_trash_pose
  if load_trash_pose()!=trash_context['destination_pose']:raise RuntimeError('Trash pose changed')
  status['destination']='trash_drop'
 q=fresh();deadline=time.monotonic()+5
 while (not turn or not vac) and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
 expected_turn=manifest.get('aligned_turntable_angle_deg',0.) if args.inclined else 0.
 if not turn or abs(turn['angle_deg']-expected_turn)>1 or turn['zero_count']!=manifest['return_to_zero']['state']['zero_count']:raise RuntimeError('Turntable pose or zero changed')
 if not vac or time.monotonic()-vac['t']>.5 or vac['value']<=HELD_DIFF:raise RuntimeError('Vacuum baseline not clear')
 assert np.linalg.norm(fk(q)[:3,3]-(W@(arm.fk(q)@tcp_point(wc)))[:3])<1e-6
 above=point+normal*float(candidate.get('approach_clearance_m',.03))
 if args.inclined:
  predicted=json.loads((session/'inclined_context.json').read_text())['option']
  qc=np.asarray(predicted['contact_q_rad'],dtype=float);qa=np.asarray(predicted['approach_q_rad'],dtype=float)
  desired=np.asarray(candidate['commanded_tool_axis_table'],dtype=float)
  if np.linalg.norm(fk(qc)[:3,3]-point)>.0003 or np.linalg.norm(fk(qa)[:3,3]-above)>.0003:raise RuntimeError('Precomputed joints and stored contact disagree')
 else:qa=solve(arm,wc,above);qc=solve(arm,wc,point)
 if qa is None or qc is None:raise RuntimeError('IK unreachable')
 cloud=np.load(session/'analysis/cloud.npz')['points'];low=np.percentile(cloud,.5,axis=0);high=np.percentile(cloud,99.5,axis=0);low[2]=wc['turntable']['top_height_m'];low[:2]-=.002;high+=.002;dims=high-low;O=np.eye(4);from payload_placement import centered_payload_origin;O[:3,3]=centered_payload_origin(cloud,(high+low)/2) if args.inclined else (high+low)/2;co=CollisionObject();co.id=object_id;co.header.frame_id='world';co.operation=CollisionObject.ADD;co.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=dims.tolist())];co.primitive_poses=[pose(np.linalg.inv(W)@O)];req=scene_request();req.scene.world.collision_objects=[co]
 if args.inclined:
  from collision_geometry import convex_mesh
  observed_mesh=convex_mesh(cloud,O[:3,3]);co.primitives=[];co.primitive_poses=[];co.meshes=[observed_mesh];co.mesh_poses=[pose(np.linalg.inv(W)@O)]
  from moveit_msgs.msg import ObjectColor
  rgb=np.median(np.load(session/'analysis/cloud.npz')['colors_rgb'],axis=0)/255.
  color=ObjectColor();color.id=object_id;color.color.r,color.color.g,color.color.b=map(float,rgb);color.color.a=.001;req.scene.object_colors.append(color)
 if args.inclined:
  current_others=json.loads((session/'inclined_context.json').read_text())['other_objects'];observed_ids={o['id'] for o in current_others};old_request=GetPlanningScene.Request();old_request.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY;old_world=call('/get_planning_scene',old_request,GetPlanningScene).scene.world.collision_objects
  for stale in old_world:
   if stale.id.startswith('inclined_obstacle_object_') and stale.id not in observed_ids:
    removal=CollisionObject();removal.id=stale.id;removal.header.frame_id='world';removal.operation=CollisionObject.REMOVE;req.scene.world.collision_objects.append(removal)
  for other in current_others:
   obj=CollisionObject();obj.id=other['id'];obj.header.frame_id='world';obj.operation=CollisionObject.ADD;obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=other['dims'])];transform=np.eye(4);transform[:3,3]=other['center_table_m'];obj.primitive_poses=[pose(np.linalg.inv(W)@transform)];req.scene.world.collision_objects.append(obj)
   if other.get('mesh_cloud_file'):
    observed=np.load(other['mesh_cloud_file'])['points'];obj.primitives=[];obj.primitive_poses=[];obj.meshes=[convex_mesh(observed,np.array(other['center_table_m']),padding_m=float(other.get('padding_m',OBSTACLE_PADDING_M)))];obj.mesh_poses=[pose(np.linalg.inv(W)@transform)]
   color=ObjectColor();color.id=other['id'];color.color.r,color.color.g,color.color.b=map(float,np.asarray(other.get('rgb_255',[100,100,100]))/255.);color.color.a=.001;req.scene.object_colors.append(color)
 apply(req)
 if args.inclined:
  Tpred=fk(qc);radial=Tpred[:2,3]-j1;radial/=np.linalg.norm(radial);payload_pitch=math.atan2(float(Tpred[:2,0]@radial),float(-Tpred[2,0]))
 if not is_trash:
  predicted_L=np.linalg.inv(fk(qc))@O;place_seed=solve(arm,wc,pad_target+[0,0,float(dims[2])])
  if place_seed is None and packing is not None and packing['placements'][packing_key].get('placement_q_rad'):
   # 계획 단계(tidy_pad_placement.placement_ik)가 이미 통과시킨 놓기 자세를 시드로 쓴다. 기본 시드 IK가 더 엄격해 같은 자리를 거부한 적이 있다(11/10 01:27, 패드 6 오프셋 −22 mm).
   place_seed=np.asarray(packing['placements'][packing_key]['placement_q_rad'],float);status['place_seed_from_reservation']=True
  if place_seed is None:raise RuntimeError('Pad seed IK unreachable')
  predicted_place=payload_goal(pad_target,place_seed,predicted_L,dims);predicted_hover=payload_goal(pad_target+[0,0,.03],predicted_place,predicted_L,dims)
  placed=fk(predicted_place)@predicted_L
  if args.inclined and np.degrees(np.arccos(np.clip(placed[2,2],-1,1)))>1.:raise RuntimeError('Placement would tilt the object; no motion')
  corners=((cloud-O[:3,3]) if args.inclined else np.array([[x,y,z] for x in [-.5,.5] for y in [-.5,.5] for z in [-.5,.5]])*dims);corners=corners@placed[:3,:3].T+placed[:3,3]
  if np.max(np.linalg.norm(corners[:,:2]-pad_nominal[:2],axis=1))>pad['diameter_m']/2-.002:raise RuntimeError('Predicted object footprint extends beyond pad')
  if packing is not None:
   from pad_packing import validate_actual
   validate_actual(packing,packing_key,corners)
  status.update(surface_point_table_m=point.tolist(),source_candidate=str(candidate_file),before_measurement=before,fresh_measurement=now,predicted_pad_place_q_deg=np.degrees(predicted_place).tolist(),scope=f'scan surface pick -> pad ID{args.pad} -> release -> retreat -> '+('wait above' if args.finish_above else 'home'))
 else:
  status.update(surface_point_table_m=point.tolist(),source_candidate=str(candidate_file),before_measurement=before,fresh_measurement=now,destination='trash_drop')
 if not args.execute:phase('preflight_only');raise SystemExit(0)
 if manifest.get('contact_execution_mode')=='precomputed_prediction_only':
  (out/'READY').touch()
 else:
  env=os.environ.copy();recorder=subprocess.Popen([sys.executable,str(ROOT/'record_suction_diagnosis.py'),str(out)],env=env);deadline=time.monotonic()+10
  while not(out/'READY').exists():
   if time.monotonic()>deadline:raise RuntimeError('Recorder not ready')
   rclpy.spin_once(node,timeout_sec=.05)
 stage_start_q=[float(x) for x in fresh()]
 _check=session/'collision_plan_check.json'
 if not _check.exists() or 'transit_q_rad' not in json.loads(_check.read_text()):raise RuntimeError('Reviewed transit pose missing; replan the batch')
 qh=json.loads(_check.read_text())['transit_q_rad']
 move(qh,'transit_above_target',min_tcp_z=TRANSIT_MIN_TCP_Z_M);status['transit_q_rad']=qh
 move(qa.tolist(),'approach_3cm',True)
 restore_acm=acm_for(object_id,'tool_link_cup_visual_link');acm_for(object_id,'cup_tip')
 move(qc.tolist(),'continuous_contact_approach',True,pump_target=point,pump_normal=normal,speed_cap=SLOW_CONTACT_SCALING);status['contact_speed_scaling']={'velocity':SLOW_CONTACT_SCALING[0],'acceleration':SLOW_CONTACT_SCALING[1]}
 phase('grip_resting');t_grip0=time.monotonic();grip=call('/vacuum/grip',timeout=12);status['first_grip']=grip.message
 if grip.success:
  _stable,_detail=seal_stable();status['seal_check_first']=_detail
  if not _stable:grip=type('G',(),{'success':False,'message':'unstable seal '+str(_detail)})()
 if not grip.success:
  # 흡착 실패: 펌프 ON 유지, 첫 흡착 시도부터 1초 동안 압력 확인 -> 그래도 실패면 접촉 TCP에서 법선 방향으로 8mm 더 전진해 1회 재흡착.
  from grip_retry import final_advance_target,SUCTION_WAIT_S,SLOW_5MM_SCALING,FINAL_ADVANCE_M
  q=fresh();initial_contact=fk(q);status['retries']=[];wait_end=t_grip0+SUCTION_WAIT_S;good=0;since=0.
  while time.monotonic()<wait_end and good<2:
   rclpy.spin_once(node,timeout_sec=.05)
   if vac and vac['t']>since and time.monotonic()-vac['t']<.3:
    since=vac['t'];good=good+1 if vac['value']<=HELD_DIFF else 0
  waited={'attempt':0,'wait_s':SUCTION_WAIT_S,'pressure_diff':vac.get('value'),'success':good>=2};status['retries'].append(waited)
  if good>=2:
   _stable,_detail=seal_stable();waited['seal_check']=_detail
   if _stable:grip=type('G',(),{'success':True,'message':f"diff={vac['value']} after waiting"})()
   else:good=0   # 기다려서 붙은 것도 약하면 아래 8mm 재접근으로 넘어간다
  if good<2:
   if not vac or time.monotonic()-vac['t']>.5:raise RuntimeError('Pressure feedback stale; no further advance')
   target=final_advance_target(initial_contact[:3,3],normal)
   q=fresh();retry_goal=ik(target,q,axis=initial_contact[:3,0])
   record={'attempt':1,'step_m':FINAL_ADVANCE_M,'total_additional_approach_m':FINAL_ADVANCE_M,'target_table_m':target.tolist(),'reference':'first actual contact TCP; one 8 mm inward-normal advance after 1 s without suction'};status['retries'].append(record)
   move(retry_goal,'grip_retry_8mm_after_1s',True,speed_cap=SLOW_5MM_SCALING,stop_on_seal=True);_adv=float(np.linalg.norm(fk(fresh())[:3,3]-initial_contact[:3,3])*1000);record['advanced_mm']=_adv;record['stopped_on_seal']=status.pop('stopped_on_seal',None);record['speed_scaling']={'velocity':SLOW_5MM_SCALING[0],'acceleration':SLOW_5MM_SCALING[1]};phase('grip_retry_resting_8mm');grip=call('/vacuum/grip',timeout=12);record.update(success=grip.success,pressure_result=grip.message)
   if grip.success:
    _stable,_detail=seal_stable();record['seal_check']=_detail
    if not _stable:grip=type('G',(),{'success':False,'message':'unstable seal after retry '+str(_detail)})()
  if not grip.success:
   released=call('/vacuum/release',timeout=15);status['release_after_failed_retry']=released.message;raise RuntimeError('Grip failed or seal unstable after 8 mm extra advance; no lift')
 holding=True;wait_vacuum();q=fresh();Tcontact=fk(q)
 if args.inclined:
  radial=Tcontact[:2,3]-j1;radial/=np.linalg.norm(radial);payload_pitch=math.atan2(float(Tcontact[:2,0]@radial),float(-Tcontact[2,0]))
 L=np.linalg.inv(Tcontact)@O;attached=AttachedCollisionObject();attached.link_name='cup_tip';attached.touch_links=['cup_tip','tool_link_cup_visual_link'];attached.object.id=object_id;attached.object.header.frame_id='cup_tip';attached.object.operation=CollisionObject.ADD;attached.object.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=dims.tolist())];attached.object.primitive_poses=[pose(L)]
 if args.inclined:
  attached.object.primitives=[];attached.object.primitive_poses=[];attached.object.meshes=[observed_mesh];attached.object.mesh_poses=[pose(L)]
 req=scene_request();req.scene.robot_state.attached_collision_objects=[attached];apply(req);acm_for(object_id,'workcell_turntable')
 target=Tcontact[:3,3]+[0,0,float(candidate.get('lift_clearance_m',.04))];move(ik(target,q,axis=Tcontact[:3,0]),'lift_clearance_40mm',True);q=fresh();status['lift']={'tcp_contact_m':Tcontact[:3,3].tolist(),'tcp_lifted_m':fk(q)[:3,3].tolist(),'encoder_height_change_mm':float((fk(q)[2,3]-Tcontact[2,3])*1000),'pressure_diff':vac['value']};phase('holding_before_transfer')
 vacuum_ok()
 # Disable the source support exemption before planning the payload transfer.
 req=scene_request();req.scene.allowed_collision_matrix=restore_acm;apply(req)
 if is_trash:
  dispose_at_trash();raise SystemExit(0)
 hover=payload_goal(pad_target+[0,0,.03],fresh(),L,dims);move(hover,'transfer_to_pad_ID'+str(args.pad))
 acm_for(object_id,'workcell_'+pad_key);place_q=payload_goal(pad_target,fresh(),L,dims);move(place_q,f'lower_payload_to_pad_ID{args.pad}',True);phase(f'release_on_pad_ID{args.pad}');status['predicted_placed_object_pose_table']= (fk(fresh())@L).tolist();released=call('/vacuum/release',timeout=15);status['release']=released.message

 if not released.success:raise RuntimeError('Release not confirmed')
 holding=False;attached=AttachedCollisionObject();attached.link_name='cup_tip';attached.object.id=object_id;attached.object.operation=CollisionObject.REMOVE;req=scene_request();req.scene.robot_state.attached_collision_objects=[attached];apply(req)
 placed_object=copy.deepcopy(co);placed_object.id='placed_pad_'+str(args.pad)+'_'+out.name;placed_object.operation=CollisionObject.ADD;placed_pose=pose(np.linalg.inv(W)@np.array(status['predicted_placed_object_pose_table']));placed_object.mesh_poses=[placed_pose] if placed_object.meshes else [];placed_object.primitive_poses=[placed_pose] if placed_object.primitives else [];req=scene_request();removal=CollisionObject();removal.id=object_id;removal.operation=CollisionObject.REMOVE;req.scene.world.collision_objects=[placed_object,removal];apply(req)
 if packing is not None:
  actual_points=(cloud-O[:3,3])@np.array(status['predicted_placed_object_pose_table'])[:3,:3].T+np.array(status['predicted_placed_object_pose_table'])[:3,3]
  packing.setdefault('completed',{})[packing_key]={'collision_id':placed_object.id,'points_table_m':actual_points.tolist(),'execution_file':str(out/'execution.json')};temporary=packing_path.with_suffix('.tmp');temporary.write_text(json.dumps(packing,indent=2)+'\n');temporary.replace(packing_path)
 acm_for(placed_object.id,'tool_link_cup_visual_link');acm_for(placed_object.id,'cup_tip')
 acm_for(object_id,'tool_link_cup_visual_link');acm_for(object_id,'cup_tip')
 q=fresh();move(ik(fk(q)[:3,3]+[0,0,RETREAT_UP_M],q,axis=fk(q)[:3,0]),f'empty_retreat_from_pad_ID{args.pad}',True)
 req=scene_request();req.scene.allowed_collision_matrix=restore_acm;apply(req);restore_acm=None
 if args.finish_above:
  from wait_pose import LIMIT_DEG,check_wait_pose,waiting_pose_errors
  reviewed=json.loads(_check.read_text())
  if not reviewed.get('success') or 'exit_q_rad' not in reviewed:
   raise RuntimeError('Reviewed waiting pose missing; replan the batch')
  expected=reviewed['exit_q_rad']
  actual=fresh()
  errors=waiting_pose_errors(actual,expected)
  status['waiting_pose_error_deg']=errors.tolist()
  if float(errors.max())>LIMIT_DEG:
   require_empty_cup()
   request=GetPlanningScene.Request()
   request.components.components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
   if call('/get_planning_scene',request,GetPlanningScene).scene.robot_state.attached_collision_objects:
    raise RuntimeError('Waiting pose correction blocked: attached object remains in scene')
   move(expected,'correct_empty_wait_pose',min_tcp_z=TRANSIT_MIN_TCP_Z_M,require_empty=True)
   actual=fresh()
   status['waiting_pose_after_correction']=check_wait_pose(actual,expected)
  if float(fk(actual)[2,3])<TRANSIT_MIN_TCP_Z_M:
   raise RuntimeError('Actual waiting pose is below transit clearance')
  phase('wait_above_destination')
 else:
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
