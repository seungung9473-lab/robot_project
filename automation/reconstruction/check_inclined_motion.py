"""Temporarily apply predicted aligned geometry and plan; never execute actions."""
from transit_clearance import RETREAT_UP_M,TRANSIT_MIN_TCP_Z_M,OBSTACLE_PADDING_M,min_tcp_z,transit_height_candidates
import argparse,copy,json,time
from pathlib import Path
import numpy as np,yaml,rclpy
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from rosidl_runtime_py.convert import message_to_ordereddict
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlanningScene,PlanningSceneComponents,CollisionObject,RobotState,Constraints,JointConstraint,AllowedCollisionEntry,AttachedCollisionObject
from moveit_msgs.srv import GetPlanningScene,ApplyPlanningScene,GetMotionPlan,GetStateValidity
from normal_contact_trajectory import contact_samples,trajectory_from_samples
from grip_retry import SLOW_CONTACT_SCALING
from inclined_geometry import solve_axis
from payload_placement import signed_pitch,place_payload
from collision_geometry import convex_mesh
from planning_feedback import calculating
from motion.kinematics import fk,m,W
p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('--placement-plan',type=Path);p.add_argument('--placement-object');p.add_argument('--start-q',nargs=4,type=float);p.add_argument('--finish-above',action='store_true');p.add_argument('--planning-seconds',type=float,default=8.);a=p.parse_args();
if not 0<a.planning_seconds<=8:raise ValueError('Planning budget must be in (0,8]')
s=a.session;ctx=json.loads((s/'inclined_context.json').read_text());option=ctx['option'];trash=ctx.get('destination')=='trash_drop';pad_id=ctx.get('destination_pad_id',5);pad_key='pad2' if pad_id==5 else 'pad1';lo=np.array([j['limit']['lower'] for j in m['joints']],float);hi=np.array([j['limit']['upper'] for j in m['joints']],float);names=['joint'+str(i) for i in range(1,5)]
packing=json.loads(a.placement_plan.read_text()) if a.placement_plan else None
if packing is not None and (packing.get('pad_id')!=pad_id or a.placement_object not in packing['placements']):raise RuntimeError('Invalid placement reservation')
rclpy.init();n=rclpy.create_node('inclined_plan_only');latest={}
n.create_subscription(JointState,'/joint_states',lambda msg:latest.update(q=[msg.position[msg.name.index(k)] for k in names],t=time.monotonic()) if all(k in msg.name for k in names) else None,10)
class EndpointCollision(RuntimeError):
 pass

state_validity_clients={}

def call(path,typ,req):
 if typ is GetMotionPlan:
  motion=req.motion_plan_request
  for label,qstate in [('출발 자세',motion.start_state),('목표 자세',state([j.position for j in motion.goal_constraints[0].joint_constraints]))]:
   vreq=GetStateValidity.Request();vreq.group_name='arm';vreq.robot_state=qstate
   valid=call('/check_state_validity',GetStateValidity,vreq)
   if not valid.valid:raise EndpointCollision(label+' collision '+str([(c.contact_body_1,c.contact_body_2) for c in valid.contacts]))
  goal=[j.position for j in motion.goal_constraints[0].joint_constraints]
  home=m['poses']['home']
  if motion.planner_id=='PTP' or (len(goal)==len(home) and np.max(np.abs(np.asarray(goal)-home))<1e-6):
   motion.pipeline_id='pilz_industrial_motion_planner';motion.planner_id='PTP';motion.num_planning_attempts=1
   reply=raw_plan(req, '홈 복귀 Pilz PTP')
   if reply.motion_plan_response.error_code.val==1:return reply
   print('안전점검 실패: Pilz 홈 경로 거부. 재탐색합니다: RRT-Connect',flush=True)
   motion.pipeline_id='ompl';motion.planner_id='RRTConnectkConfigDefault'
  return raw_plan(req,'RRT-Connect 경로 탐색 및 안전점검')
 keep_client=typ is GetStateValidity
 if keep_client:
  if path not in state_validity_clients:state_validity_clients[path]=n.create_client(typ,path)
  c=state_validity_clients[path]
 else:
  c=n.create_client(typ,path)
 try:
  assert c.wait_for_service(timeout_sec=20),path;f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=30)
  if not f.done():raise RuntimeError('Timeout '+path)
  return f.result()
 finally:
  if not keep_client:n.destroy_client(c)
def raw_plan(req,label):
 c=n.create_client(GetMotionPlan,'/plan_kinematic_path')
 try:
  if not c.wait_for_service(timeout_sec=10):raise RuntimeError('Planning service unavailable')
  with calculating(label):
   f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=30)
   if not f.done() or f.result() is None:raise RuntimeError('Planning response timeout')
   return f.result()
 finally:n.destroy_client(c)

def apply(scene):
 req=ApplyPlanningScene.Request();req.scene=scene;assert call('/apply_planning_scene',ApplyPlanningScene,req).success

def pose(T):
 p=Pose();p.position.x,p.position.y,p.position.z=map(float,T[:3,3]);p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,Rotation.from_matrix(T[:3,:3]).as_quat());return p

def state(q):return RobotState(joint_state=JointState(name=names,position=list(map(float,q))),is_diff=True)
req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY|PlanningSceneComponents.ALLOWED_COLLISION_MATRIX|PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
original=call('/get_planning_scene',GetPlanningScene,req).scene;introduced=[];result={'success':False,'robot_commands_sent':False};cached={}
try:
 scaling_reply=call('/soarm4/rviz_scaling',Trigger,Trigger.Request())
 if not scaling_reply.success:raise RuntimeError('RViz speed unavailable')
 scaling=json.loads(scaling_reply.message);vscale=float(scaling['velocity']);ascale=float(scaling['acceleration'])
 scene=PlanningScene();scene.is_diff=True
 observed_obstacle_ids={o['id'] for o in ctx['other_objects']}
 for stale in original.world.collision_objects:
  if stale.id.startswith('inclined_obstacle_object_') and stale.id not in observed_obstacle_ids:
   removal=CollisionObject();removal.id=stale.id;removal.header.frame_id='world';removal.operation=CollisionObject.REMOVE;scene.world.collision_objects.append(removal)
 P=np.load(s/'analysis/cloud.npz')['points'];center=(np.percentile(P,.5,axis=0)+np.percentile(P,99.5,axis=0))/2;O=np.eye(4);O[:3,3]=center
 obj=CollisionObject();obj.id='scanned_block';obj.header.frame_id='world';obj.operation=CollisionObject.ADD;obj.meshes=[convex_mesh(P,center)];obj.mesh_poses=[pose(np.linalg.inv(W)@O)];scene.world.collision_objects.append(obj);introduced.append(obj.id)
 for other in ctx['other_objects']:
  o=CollisionObject();o.id=other['id'];o.header.frame_id='world';o.operation=CollisionObject.ADD;o.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=other['dims'])];T=np.eye(4);T[:3,3]=other['center_table_m'];o.primitive_poses=[pose(np.linalg.inv(W)@T)]
  if other.get('mesh_cloud_file'):
   obs=np.load(other['mesh_cloud_file'])['points'];o.primitives=[];o.primitive_poses=[];o.meshes=[convex_mesh(obs,np.array(other['center_table_m']),padding_m=float(other.get('padding_m',OBSTACLE_PADDING_M)))];o.mesh_poses=[pose(np.linalg.inv(W)@T)]
  scene.world.collision_objects.append(o);introduced.append(o.id)
 for placed_entry in ctx.get('virtual_placements',[]):
  obj=CollisionObject();obj.id=placed_entry['collision_id'];obj.header.frame_id='world';obj.operation=CollisionObject.ADD
  observed=np.load(placed_entry['cloud_file'])['points'];obj.meshes=[convex_mesh(observed,np.zeros(3))];obj.mesh_poses=[pose(np.linalg.inv(W))];scene.world.collision_objects.append(obj);introduced.append(obj.id)
 apply(scene)
 deadline=time.monotonic()+10
 while (not latest or time.monotonic()-latest['t']>.3) and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
 if not latest:raise RuntimeError('No joint state')
 start=list(a.start_q) if a.start_q is not None else latest['q'];qa=option['approach_q_rad'];qc=option['contact_q_rad']
 if a.finish_above:
  from turntable_wait import review_wait_pose
  manifest=json.loads((s/'session.json').read_text())
  review_wait_pose(n,start,ctx['source_scan'],set(manifest.get('batch_assume_completed',[])))
 # Review the approach from the current stage start at several candidate heights.
 approach=fk(qa);transit_attempts=[]
 result['transit_attempts']=transit_attempts
 limits=yaml.safe_load((Path.home()/'robot_ws/src/TUMST_ARM_moveit_config/config/joint_limits.yaml').read_text())['joint_limits']
 velocities=[float(limits[k]['max_velocity'])*vscale for k in names]
 accelerations=[float(limits[k]['max_acceleration'])*ascale for k in names]
 found_ik=False
 for rise in transit_height_candidates(float(approach[2,3])):
  attempt={'rise_m':rise};transit_attempts.append(attempt)
  solution=solve_axis(fk,lo,hi,approach[:3,3]+[0,0,rise],approach[:3,0],seed=qa)
  if solution is None:
   attempt['error']='IK unavailable';continue
  found_ik=True;qh=np.asarray(solution[0],float)
  req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.pipeline_id='ompl';r.planner_id='RRTConnectkConfigDefault';r.allowed_planning_time=a.planning_seconds;r.num_planning_attempts=1;r.max_velocity_scaling_factor=vscale;r.max_acceleration_scaling_factor=ascale;r.start_state=state(start);r.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=k,position=float(q),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for k,q in zip(names,qh)])]
  try:
   plan=call('/plan_kinematic_path',GetMotionPlan,req).motion_plan_response
  except EndpointCollision as exc:
   attempt['error']=str(exc);continue
  if plan.error_code.val!=1:
   attempt['error']='Approach plan rejected '+str(plan.error_code.val);continue
  path=plan.trajectory.joint_trajectory
  indices=[list(path.joint_names).index(k) for k in names]
  if not path.points:
   attempt['error']='Approach plan rejected: empty trajectory';continue
  transit_z=min_tcp_z(fk,[[pt.positions[i] for i in indices] for pt in path.points])
  if transit_z<TRANSIT_MIN_TCP_Z_M:
   attempt['error']='Transit path rejected: tool tip dips to %.0f mm, below %.0f mm clearance'%(transit_z*1000,TRANSIT_MIN_TCP_Z_M*1000)
   continue
  try:
   descent_profile=contact_samples(fk,lo,hi,qh,qa,velocities,accelerations)
   descent=descent_profile[1]
  except RuntimeError as exc:
   attempt['error']='Transit descent unavailable: '+str(exc);continue
  for q in descent:
   request=GetStateValidity.Request();request.group_name='arm';request.robot_state=state(q)
   valid=call('/check_state_validity',GetStateValidity,request)
   if not valid.valid:
    attempt['error']='Transit descent collision '+str([(c.contact_body_1,c.contact_body_2) for c in valid.contacts])
    break
  else:
   attempt['selected']=True
   result.update(start_q_rad=list(map(float,start)),transit_q_rad=qh.tolist(),
                 transit_rise_m=rise,transit_min_tcp_z_m=transit_z,
                 approach_plan_error=plan.error_code.val,transit_descent_samples=len(descent))
   cached['transit_above_target']={'joint_names':list(plan.trajectory.joint_trajectory.joint_names),'start':list(map(float,start)),'goal':list(map(float,qh)),'scaling':scaling,'trajectory':message_to_ordereddict(plan.trajectory)}
   cached['approach_3cm']={'profile':'fixed_axis','joint_names':names,'start':qh.tolist(),'goal':list(map(float,qa)),
                         'scaling':scaling,'trajectory':message_to_ordereddict(trajectory_from_samples(names,descent_profile))}
   break
 else:
  if not found_ik:raise RuntimeError('Transit pose above target IK unavailable at every candidate height')
  errors=[a['error'] for a in transit_attempts if a['error']!='IK unavailable']
  raise RuntimeError('No safe transit height: '+' | '.join(dict.fromkeys(errors)))
 # Only intended cup/object contact is exempt. Other-object/table collisions remain active.
 acm=copy.deepcopy(original.allowed_collision_matrix)
 for key in ['scanned_block','cup_tip','tool_link_cup_visual_link']:
  if key not in acm.entry_names:
   acm.entry_names.append(key)
   for row in acm.entry_values:row.enabled.append(False)
   acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
 for key in ['cup_tip','tool_link_cup_visual_link']:
  i=acm.entry_names.index(key);j=acm.entry_names.index('scanned_block');acm.entry_values[i].enabled[j]=True;acm.entry_values[j].enabled[i]=True
 scene=PlanningScene();scene.is_diff=True;scene.allowed_collision_matrix=acm;apply(scene)
 cfg=limits
 contact_scaling={'velocity':min(vscale,SLOW_CONTACT_SCALING[0]),
                  'acceleration':min(ascale,SLOW_CONTACT_SCALING[1])}
 contact_profile=contact_samples(fk,lo,hi,qa,qc,
     [float(cfg[k]['max_velocity'])*contact_scaling['velocity'] for k in names],
     [float(cfg[k]['max_acceleration'])*contact_scaling['acceleration'] for k in names])
 times,positions,_,_=contact_profile
 for q in positions:
  req=GetStateValidity.Request();req.group_name='arm';req.robot_state=state(q);valid=call('/check_state_validity',GetStateValidity,req)
  if not valid.valid:raise RuntimeError('Contact collision '+str([(c.contact_body_1,c.contact_body_2) for c in valid.contacts]))
 result.update(contact_samples=len(positions),contact_duration_s=float(times[-1]))
 cached['continuous_contact_approach']={'profile':'fixed_axis','joint_names':names,'start':list(map(float,qa)),
     'goal':list(map(float,qc)),'scaling':contact_scaling,
     'trajectory':message_to_ordereddict(trajectory_from_samples(names,contact_profile))}

 # Keep the actual inclined pickup pitch while carrying the body upright.
 wc=yaml.safe_load((s/'workcell_snapshot.yaml').read_text());P=np.load(s/'analysis/cloud.npz')['points'];low=np.percentile(P,.5,0);high=np.percentile(P,99.5,0);low[2]=wc['turntable']['top_height_m'];low[:2]-=.002;high+=.002;dims=high-low;O=np.eye(4);from payload_placement import centered_payload_origin;O[:3,3]=centered_payload_origin(P,(high+low)/2);L=np.linalg.inv(fk(qc))@O;j1=(W@np.r_[[-.0177,.0177,.1598],1])[:3];pitch=signed_pitch(fk(qc),j1);target=None if trash else np.array(wc['pads'][pad_key]['center_table_m'])
 if not trash:target[:2]+=wc['place_correction_m'].get('pad_ID'+str(pad_id),[0.,0.])
 if trash:
  from trash_destination import load_trash_pose
  taught=load_trash_pose()
  if taught!=ctx['destination_pose']:raise RuntimeError('Trash pose changed since preparation')
  placed=np.array(taught['positions_rad'],float)
  if np.any(placed<lo) or np.any(placed>hi):raise RuntimeError('Trash pose outside joint limits')
  hover=placed
 else:
  if packing is not None:target[:2]+=np.array(packing['placements'][a.placement_object]['offset_xy_m'])
  placed=place_payload(fk,lo,hi,target,qc,L,dims,j1,pitch)
  if placed is None:raise RuntimeError('Placement IK unavailable')
  if np.degrees(np.arccos(np.clip((fk(placed)@L)[2,2],-1,1)))>1:raise RuntimeError('Payload would not stay upright')
  hover=place_payload(fk,lo,hi,target+[0,0,.03],placed,L,dims,j1,pitch)
  if hover is None:raise RuntimeError('Placement hover IK unavailable')
 lifted=solve_axis(fk,lo,hi,fk(qc)[:3,3]+[0,0,float(option.get('lift_clearance_m',.04))],fk(qc)[:3,0],seed=qc)
 if lifted is None:raise RuntimeError('40mm vertical lift IK unavailable')
 attached=AttachedCollisionObject();attached.link_name='cup_tip';attached.touch_links=['cup_tip','tool_link_cup_visual_link'];attached.object.id='scanned_block';attached.object.header.frame_id='cup_tip';attached.object.operation=CollisionObject.ADD;attached.object.meshes=[convex_mesh(P,O[:3,3])];attached.object.mesh_poses=[pose(L)]
 scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True;scene.robot_state.attached_collision_objects=[attached];scene.allowed_collision_matrix=original.allowed_collision_matrix;apply(scene)
 # Check the complete attached-payload lift, including its starting state.
 lift_acm=copy.deepcopy(original.allowed_collision_matrix)
 for key in ['scanned_block','workcell_turntable']:
  if key not in lift_acm.entry_names:
   lift_acm.entry_names.append(key)
   for row in lift_acm.entry_values:row.enabled.append(False)
   lift_acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(lift_acm.entry_names)))
 i=lift_acm.entry_names.index('scanned_block');j=lift_acm.entry_names.index('workcell_turntable');lift_acm.entry_values[i].enabled[j]=lift_acm.entry_values[j].enabled[i]=True
 scene=PlanningScene();scene.is_diff=True;scene.allowed_collision_matrix=lift_acm;apply(scene)
 lift_count=0;lift_seed=np.asarray(qc)
 for fraction in np.linspace(0,1,56):
  target_lift=fk(qc)[:3,3]+[0,0,float(option.get('lift_clearance_m',.04))*fraction]
  solution=solve_axis(fk,lo,hi,target_lift,fk(qc)[:3,0],seed=lift_seed,attempts=2)
  if solution is None:raise RuntimeError('Attached lift intermediate IK unavailable')
  lift_seed=solution[0];request=GetStateValidity.Request();request.group_name='arm';request.robot_state=state(lift_seed);valid=call('/check_state_validity',GetStateValidity,request)
  if not valid.valid:raise RuntimeError('Attached lift collision at '+str(round(float(fraction),3))+': '+str([(c.contact_body_1,c.contact_body_2) for c in valid.contacts]))
  lift_count+=1
 result['attached_lift_samples']=lift_count
 scene=PlanningScene();scene.is_diff=True;scene.allowed_collision_matrix=original.allowed_collision_matrix;apply(scene)
 validity_request=GetStateValidity.Request();validity_request.group_name='arm';validity_request.robot_state=state(lifted[0]);lift_valid=call('/check_state_validity',GetStateValidity,validity_request)
 result['lifted_payload_state_valid']=lift_valid.valid;result['lifted_payload_contacts']=[(c.contact_body_1,c.contact_body_2) for c in lift_valid.contacts]
 if not trash:
  # Validate descent while only the carried body may touch the destination pad.
  placed_body=fk(placed)@L;corners=(P-O[:3,3])@placed_body[:3,:3].T+placed_body[:3,3];pad=np.array(wc['pads'][pad_key]['center_table_m'])
  if np.max(np.linalg.norm(corners[:,:2]-pad[:2],axis=1))>wc['pads'][pad_key]['diameter_m']/2-.002:raise RuntimeError('Payload footprint exceeds pad')
  if packing is not None:
   from pad_packing import validate_actual
   validate_actual(packing,a.placement_object,corners)
  acm=copy.deepcopy(original.allowed_collision_matrix)
  for key in ['scanned_block','workcell_'+pad_key]:
   if key not in acm.entry_names:
    acm.entry_names.append(key)
    for row in acm.entry_values:row.enabled.append(False)
    acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
  i=acm.entry_names.index('scanned_block');j=acm.entry_names.index('workcell_'+pad_key);acm.entry_values[i].enabled[j]=acm.entry_values[j].enabled[i]=True
  scene=PlanningScene();scene.is_diff=True;scene.allowed_collision_matrix=acm;apply(scene)
  for endpoint,label in [(placed,'Placement'),(hover,'Placement hover')]:
   vr=GetStateValidity.Request();vr.group_name='arm';vr.robot_state=state(endpoint)
   vv=call('/check_state_validity',GetStateValidity,vr)
   if not vv.valid:raise RuntimeError(label+' collision '+str([(c.contact_body_1,c.contact_body_2) for c in vv.contacts]))
  scene=PlanningScene();scene.is_diff=True;scene.allowed_collision_matrix=original.allowed_collision_matrix;apply(scene)
 req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.pipeline_id='ompl';r.planner_id='RRTConnectkConfigDefault';r.allowed_planning_time=a.planning_seconds;r.num_planning_attempts=1;r.max_velocity_scaling_factor=vscale;r.max_acceleration_scaling_factor=ascale;r.start_state=state(lifted[0]);r.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=k,position=float(q),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for k,q in zip(names,hover)])]
 transfer=call('/plan_kinematic_path',GetMotionPlan,req).motion_plan_response;result['transfer_plan_error']=transfer.error_code.val
 if transfer.error_code.val!=1:raise RuntimeError('Payload transfer plan rejected '+str(transfer.error_code.val))
 cached['transfer_to_trash_drop' if trash else 'transfer_to_pad_ID'+str(pad_id)]={'joint_names':list(transfer.trajectory.joint_trajectory.joint_names),'start':lifted[0].tolist(),'goal':hover.tolist(),'scaling':scaling,'trajectory':message_to_ordereddict(transfer.trajectory)}
 if trash:
  # Simulate confirmed release: payload falls into the user-taught bin, not onto a pad.
  detached=AttachedCollisionObject();detached.link_name='cup_tip';detached.object.id='scanned_block';detached.object.operation=CollisionObject.REMOVE
  remove=CollisionObject();remove.id='scanned_block';remove.operation=CollisionObject.REMOVE
  scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True;scene.robot_state.attached_collision_objects=[detached];scene.world.collision_objects=[remove];scene.allowed_collision_matrix=original.allowed_collision_matrix;apply(scene)
  if True:  # trash always plans the empty return to this stage's start pose (HOME or previous wait pose)
   request=GetMotionPlan.Request();mr=request.motion_plan_request;mr.group_name='arm';mr.pipeline_id='ompl';mr.planner_id='RRTConnectkConfigDefault';mr.allowed_planning_time=a.planning_seconds;mr.num_planning_attempts=1;mr.max_velocity_scaling_factor=vscale;mr.max_acceleration_scaling_factor=ascale;mr.start_state=state(placed);mr.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=k,position=float(q),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for k,q in zip(names,start)])]
   if trash:
    mr.pipeline_id='pilz_industrial_motion_planner';mr.planner_id='PTP'
   response=call('/plan_kinematic_path',GetMotionPlan,request).motion_plan_response
   if response.error_code.val!=1:raise RuntimeError('Trash empty return path rejected')
   return_label='return_to_stage_start_after_trash' if a.finish_above else 'return_to_initial_home'
   cached[return_label]={'joint_names':list(response.trajectory.joint_trajectory.joint_names),'start':placed.tolist(),'goal':list(map(float,start)),'scaling':scaling,'trajectory':message_to_ordereddict(response.trajectory)}
  result['exit_q_rad']=list(map(float,start))
  result.update(success=True,destination='trash_drop',trash_goal_q_rad=placed.tolist(),object_origin_table=O.tolist(),predicted_placed_object_pose_table=None,finish_above=a.finish_above,empty_retreat_and_return_checked=True,scope='approach/contact/attached lift/trash transfer/empty return reviewed; no physical release performed; bin interior/fall not modeled')
  (s/'reviewed_trajectories.json').write_text(json.dumps(cached,indent=2));raise SystemExit(0)
 scene=PlanningScene();scene.is_diff=True;scene.allowed_collision_matrix=acm;apply(scene)
 tt,qq,_,_=contact_samples(fk,lo,hi,hover,placed,[float(cfg[k]['max_velocity'])*vscale for k in names],[float(cfg[k]['max_acceleration'])*ascale for k in names])
 for q in qq:
  req=GetStateValidity.Request();req.group_name='arm';req.robot_state=state(q);valid=call('/check_state_validity',GetStateValidity,req)
  if not valid.valid:raise RuntimeError('Placement collision '+str([(c.contact_body_1,c.contact_body_2) for c in valid.contacts]))
 # Review empty retreat and return to the original home/start after release.
 detached=AttachedCollisionObject();detached.link_name='cup_tip';detached.object.id='scanned_block';detached.object.operation=CollisionObject.REMOVE
 placed_world=CollisionObject();placed_world.id='scanned_block';placed_world.header.frame_id='world';placed_world.operation=CollisionObject.ADD;placed_world.meshes=[convex_mesh(P,O[:3,3])];placed_world.mesh_poses=[pose(np.linalg.inv(W)@placed_body)]
 for key in ['scanned_block','cup_tip','tool_link_cup_visual_link']:
  if key not in acm.entry_names:
   acm.entry_names.append(key)
   for row in acm.entry_values:row.enabled.append(False)
   acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
 for key in ['cup_tip','tool_link_cup_visual_link']:
  i=acm.entry_names.index(key);j=acm.entry_names.index('scanned_block');acm.entry_values[i].enabled[j]=acm.entry_values[j].enabled[i]=True
 scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True;scene.robot_state.attached_collision_objects=[detached];scene.world.collision_objects=[placed_world];scene.allowed_collision_matrix=acm;apply(scene)
 retreat=solve_axis(fk,lo,hi,fk(placed)[:3,3]+[0,0,RETREAT_UP_M],fk(placed)[:3,0],seed=placed)
 if retreat is None:raise RuntimeError('Empty retreat IK unavailable')
 for label,begin,end in [('empty_retreat_from_pad_ID'+str(pad_id),placed,retreat[0])]+([] if a.finish_above else [('return_to_initial_home',retreat[0],start)]):
  if label=='return_to_initial_home':
   scene=PlanningScene();scene.is_diff=True;scene.allowed_collision_matrix=original.allowed_collision_matrix;apply(scene)
  request=GetMotionPlan.Request();mr=request.motion_plan_request;mr.group_name='arm';mr.pipeline_id='ompl';mr.planner_id='RRTConnectkConfigDefault';mr.allowed_planning_time=a.planning_seconds;mr.num_planning_attempts=1;mr.max_velocity_scaling_factor=vscale;mr.max_acceleration_scaling_factor=ascale;mr.start_state=state(begin);mr.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=k,position=float(q),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for k,q in zip(names,end)])]
  if trash or label=='return_to_initial_home':
   mr.pipeline_id='pilz_industrial_motion_planner';mr.planner_id='PTP'
  response=call('/plan_kinematic_path',GetMotionPlan,request).motion_plan_response
  if response.error_code.val!=1:raise RuntimeError(label+' plan rejected '+str(response.error_code.val))
  cached[label]={'joint_names':list(response.trajectory.joint_trajectory.joint_names),'start':list(map(float,begin)),'goal':list(map(float,end)),'scaling':scaling,'trajectory':message_to_ordereddict(response.trajectory)}
 result['empty_retreat_and_return_checked']=not a.finish_above
 result['exit_q_rad']=retreat[0].tolist() if a.finish_above else list(map(float,start))
 result['finish_above']=a.finish_above
 result.update(predicted_placed_object_pose_table=placed_body.tolist(),object_origin_table=O.tolist(),success=True,placement_samples=len(qq),placement_q_rad=placed.tolist(),scope='approach/contact/full attached lift/transfer/placement/empty retreat/return planning; physical grip/release still untested')
 if result['success']:(s/'reviewed_trajectories.json').write_text(json.dumps(cached,indent=2))
except Exception as exc:
 result['error']=str(exc);print('안전점검 실패: '+str(exc),flush=True)
finally:
 # Detach first, then query again: detaching may create a world object.
 query_current=GetPlanningScene.Request();query_current.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY|PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
 current_scene=call('/get_planning_scene',GetPlanningScene,query_current).scene
 if any(o.object.id=='scanned_block' for o in current_scene.robot_state.attached_collision_objects):
  detached=AttachedCollisionObject();detached.link_name='cup_tip';detached.object.id='scanned_block';detached.object.operation=CollisionObject.REMOVE
  cleanup=PlanningScene();cleanup.is_diff=True;cleanup.robot_state.is_diff=True;cleanup.robot_state.attached_collision_objects=[detached];apply(cleanup)
 current_scene=call('/get_planning_scene',GetPlanningScene,query_current).scene
 original_ids={o.id for o in original.world.collision_objects}
 cleanup=PlanningScene();cleanup.is_diff=True
 for obj in current_scene.world.collision_objects:
  if obj.id not in introduced or obj.id in original_ids:continue
  removal=CollisionObject();removal.id=obj.id;removal.header.frame_id='world';removal.operation=CollisionObject.REMOVE;cleanup.world.collision_objects.append(removal)
 if cleanup.world.collision_objects:apply(cleanup)
 restore=PlanningScene();restore.is_diff=True;restore.allowed_collision_matrix=original.allowed_collision_matrix;restore.robot_state.is_diff=True;restore.robot_state.attached_collision_objects=list(original.robot_state.attached_collision_objects);restore.world.collision_objects=list(original.world.collision_objects);apply(restore)
 (s/'collision_plan_check.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));n.destroy_node();rclpy.try_shutdown()
