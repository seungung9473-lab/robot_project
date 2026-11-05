import json,time,math,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState,Constraints,JointConstraint,DisplayTrajectory
from moveit_msgs.srv import GetMotionPlan,GetStateValidity
from moveit_msgs.action import ExecuteTrajectory
from std_srvs.srv import Trigger
from sag_compensation import compensated_axis
from placement_geometry import contact_target
import json,yaml,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
m=json.loads(Path('/home/seungung9473/robot_ws/src/soarm4_description/config/model.json').read_text());d=yaml.safe_load(Path('/home/seungung9473/robot_ws/src/soarm4_workcell/config/workcell.yaml').read_text());W=np.array(d['T_table_world']);B=np.eye(4);B[:3,:3]=Rotation.from_euler('z',m['robot_base_yaw']).as_matrix();B[:3,3]=m['robot_base_xyz']
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
def state(q):return RobotState(joint_state=JointState(name=names,position=q))
ROOT=Path('/home/seungung9473/robot_ws/automation/reconstruction')
settings=json.loads((ROOT/'contact_settings.json').read_text())
candidate_path=Path(os.environ.get('SOARM_SUCTION_CANDIDATES',str(ROOT/'multiview/20261101_194509_024461/reconstruction_sam2/suction_candidates.json')))
candidate_report=json.loads(candidate_path.read_text());candidate=candidate_report['selected_candidate']
nominal_contact=candidate.get('nominal_contact_position_table_m',candidate['position_table_m'])
extra_z=float(candidate.get('execution_contact_z_adjustment_m',0.))
if not np.isfinite(extra_z) or not -.002<=extra_z<=0:raise RuntimeError('Invalid local contact correction')
effective_offset=float(settings['contact_z_offset_m'])+extra_z
contact_z=float(nominal_contact[2])+effective_offset

try:
 start=fresh();T0=fk(start);target=contact_target(nominal_contact,effective_offset)
 if np.linalg.norm(T0[:2,3]-np.asarray(nominal_contact)[:2])>.005:raise RuntimeError('Above-candidate XY drift exceeds5mm; no descent')
 relative_step=float(os.environ.get('SOARM_CONTACT_RELATIVE_STEP_M','0'))
 if relative_step:
  if not np.isfinite(relative_step) or not 0<relative_step<=.001:raise RuntimeError('Relative contact step exceeds1mm')
  normal=np.asarray(candidate_report['results'][candidate['camera']]['normal_table'],dtype=float);normal/=np.linalg.norm(normal)
  if normal[2]<0:normal=-normal
  if normal[2]<np.cos(np.deg2rad(15)):raise RuntimeError('Surface normal not suitable for short contact retry')
  target=T0[:3,3]-normal*relative_step;contact_z=float(target[2]);target_axis=T0[:3,0].copy();sag_info={'mode':'relative approach from actual encoder-derived TCP','step_m':relative_step,'normal_table':normal.tolist(),'orientation_preserved':True,'old_goal_offset_not_accumulated':True}
 else:target_axis,sag_info=compensated_axis(fk,start)
 print('Sag compensation',json.dumps(sag_info),flush=True)
 if T0[2,3]<contact_z-.0005:raise RuntimeError('Already below expected surface; no downward motion')
 def residual(q):
  T=fk(q);return np.r_[(T[:3,3]-target)*10,(T[:3,0]-target_axis)*.3]
 lo=[float(j['limit']['lower']) for j in m['joints']];hi=[float(j['limit']['upper']) for j in m['joints']]
 sol=least_squares(residual,start,bounds=(lo,hi));goal=sol.x.tolist()
 print('current q',start,'TCP',T0[:3,3],'target',target,'IK error mm',np.linalg.norm(fk(goal)[:3,3]-target)*1000,flush=True)
 if np.linalg.norm(fk(goal)[:3,3]-target)>.0002:raise RuntimeError('Contact pose IK failed')
 if np.arccos(np.clip(np.dot(fk(goal)[:3,0],target_axis),-1,1))>math.radians(.2):raise RuntimeError('Compensated tool orientation unreachable')
 print('TCP start',T0[:3,3],'target',target,flush=True)
 v=n.create_client(GetStateValidity,'/check_state_validity')
 for label,q in [('start',start),('above',goal)]:
  req=GetStateValidity.Request();req.robot_state=state(q);req.group_name='arm';res=call(v,req);print(label,'valid',res.valid,'contacts',[(c.contact_body_1,c.contact_body_2) for c in res.contacts],flush=True)
  if not res.valid:raise RuntimeError(label+' pose collision/invalid')
 rviz=call(n.create_client(Trigger,'/soarm4/rviz_scaling'),Trigger.Request(),8)
 if not rviz.success:raise RuntimeError('RViz scaling unavailable')
 scaling=json.loads(rviz.message);velocity=float(scaling['velocity']);acceleration=float(scaling['acceleration'])
 if not (0<velocity<=1 and 0<acceleration<=1):raise RuntimeError('Invalid RViz scaling')
 print('Nominal contact Z',nominal_contact[2],'offset',effective_offset,'contact target Z',contact_z,'RViz ratios',velocity,acceleration,flush=True)
 c=n.create_client(GetMotionPlan,'/plan_kinematic_path');req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.pipeline_id='ompl';r.planner_id='RRTConnectkConfigDefault';r.num_planning_attempts=1;r.allowed_planning_time=8.;r.max_velocity_scaling_factor=velocity;r.max_acceleration_scaling_factor=acceleration;r.start_state=state(start);r.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=k,position=float(q),tolerance_above=.0001,tolerance_below=.0001,weight=1.) for k,q in zip(names,goal)])]
 res=call(c,req).motion_plan_response;print('Plan result',res.error_code.val,flush=True)
 if res.error_code.val!=1:raise RuntimeError('Planning rejected')
 pts=res.trajectory.joint_trajectory;idx=[pts.joint_names.index(k) for k in names]
 for pt in pts.points:
  p=fk([pt.positions[i] for i in idx])[:3,3]
  xy_distance=np.linalg.norm(T0[:2,3]-target[:2]);allowed_xy=xy_distance+.001
  if np.linalg.norm(p[:2]-target[:2])>allowed_xy or p[2]<target[2]-.0005 or p[2]>T0[2,3]+.001:raise RuntimeError('Contact path leaves candidate corridor')
 q=fresh()
 if max(abs(a-b) for a,b in zip(q,start))>math.radians(1):raise RuntimeError('Start moved; refusing execution')
 a=ActionClient(n,ExecuteTrajectory,'/execute_trajectory')
 if not a.wait_for_server(timeout_sec=3):raise RuntimeError('No execution server')
 g=ExecuteTrajectory.Goal();g.trajectory=res.trajectory;f=a.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=5);h=f.result()
 if not h or not h.accepted:raise RuntimeError('Execution refused')
 print('Executing level-cup contact approach',flush=True);f=h.get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=120)
 if not f.done():raise RuntimeError('Result timeout; check robot')
 result=f.result().result.error_code.val;final=fresh();print('Execution result',result,'final',final,flush=True)
 if relative_step:
  report={'start_tcp_table_m':T0[:3,3].tolist(),'target_tcp_table_m':target.tolist(),'final_tcp_table_m':fk(final)[:3,3].tolist(),'requested_gap_reduction_m':relative_step,'encoder_estimated_gap_reduction_m':float((T0[:3,3]-fk(final)[:3,3])@normal),'not_direct_optical_gap_measurement':True}
  print('Relative contact result',json.dumps(report),flush=True)
  if os.environ.get('SOARM_CONTACT_STEP_RESULT'):Path(os.environ['SOARM_CONTACT_STEP_RESULT']).write_text(json.dumps(report,indent=2))
 if result!=1:raise RuntimeError('Contact execution failed')
finally:n.destroy_node();rclpy.shutdown()
