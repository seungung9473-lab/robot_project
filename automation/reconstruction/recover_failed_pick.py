"""After confirmed release, retreat along the attempted face normal and home."""
import json,time,sys,copy
from vacuum_thresholds import HELD_DIFF
from pathlib import Path
import numpy as np,rclpy,yaml
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from moveit_msgs.msg import PlanningScene,PlanningSceneComponents,RobotState,AllowedCollisionEntry,RobotTrajectory
from moveit_msgs.srv import GetPlanningScene,ApplyPlanningScene,GetStateValidity
from moveit_msgs.action import ExecuteTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from inclined_geometry import solve_axis
from normal_contact_trajectory import contact_samples
from motion.kinematics import fk,m
s=Path(sys.argv[1]);failed=json.loads((s/'execution.json').read_text());assert not failed['holding'] and 'release_after_failed_retry' in failed
prep=s.parent;normal=np.array(json.loads((prep/'candidate.json').read_text())['selected_candidate']['normal_table']);lo=np.array([j['limit']['lower'] for j in m['joints']],float);hi=np.array([j['limit']['upper'] for j in m['joints']],float);names=['joint'+str(i) for i in range(1,5)]
rclpy.init();n=rclpy.create_node('released_pick_retreat');latest={};vac={}
n.create_subscription(JointState,'/joint_states',lambda msg:latest.update(q=[msg.position[msg.name.index(k)] for k in names],t=time.monotonic()) if all(k in msg.name for k in names) else None,10);n.create_subscription(Int32,'/vacuum/diff',lambda msg:vac.update(v=msg.data,t=time.monotonic()),10)
def call(path,typ,req,timeout=30):
 c=n.create_client(typ,path)
 try:
  assert c.wait_for_service(timeout_sec=15),path;f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=timeout)
  if not f.done():raise RuntimeError('Timeout '+path)
  return f.result()
 finally:n.destroy_client(c)
def apply(scene):
 req=ApplyPlanningScene.Request();req.scene=scene;assert call('/apply_planning_scene',ApplyPlanningScene,req).success

def state(q):return RobotState(joint_state=JointState(name=names,position=list(map(float,q))),is_diff=True)
start=time.monotonic()
while time.monotonic()-start<3:rclpy.spin_once(n,timeout_sec=.05)
assert latest and vac and time.monotonic()-latest['t']<.3 and time.monotonic()-vac['t']<.5 and vac['v']>HELD_DIFF,'Release/fresh state required'
q=latest['q'];T=fk(q);sol=solve_axis(fk,lo,hi,T[:3,3]+normal*.015,T[:3,0],seed=q);assert sol is not None,'No retreat IK'
req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX;original=call('/get_planning_scene',GetPlanningScene,req).scene.allowed_collision_matrix;acm=copy.deepcopy(original)
for key in ['scanned_block','cup_tip','tool_link_cup_visual_link']:
 if key not in acm.entry_names:
  acm.entry_names.append(key)
  for row in acm.entry_values:row.enabled.append(False)
  acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
for key in ['cup_tip','tool_link_cup_visual_link']:
 i=acm.entry_names.index(key);j=acm.entry_names.index('scanned_block');acm.entry_values[i].enabled[j]=acm.entry_values[j].enabled[i]=True
scene=PlanningScene();scene.is_diff=True;scene.allowed_collision_matrix=acm;apply(scene)
result={'success':False}
try:
 cfg=yaml.safe_load((Path.home()/'robot_ws/src/soarm4_moveit_config/config/joint_limits.yaml').read_text())['joint_limits'];ts,qs,vs,acc=contact_samples(fk,lo,hi,q,sol[0],[float(cfg[k]['max_velocity'])*.1 for k in names],[float(cfg[k]['max_acceleration'])*.1 for k in names]);traj=RobotTrajectory();traj.joint_trajectory.joint_names=names
 for t,qi,v,a in zip(ts,qs,vs,acc):
  req=GetStateValidity.Request();req.group_name='arm';req.robot_state=state(qi);check=call('/check_state_validity',GetStateValidity,req);assert check.valid,str([(c.contact_body_1,c.contact_body_2) for c in check.contacts]);pt=JointTrajectoryPoint();pt.positions=list(map(float,qi));pt.velocities=list(map(float,v));pt.accelerations=list(map(float,a));ns=int(round(t*1e9));pt.time_from_start.sec=ns//10**9;pt.time_from_start.nanosec=ns%10**9;traj.joint_trajectory.points.append(pt)
 for _ in range(5):rclpy.spin_once(n,timeout_sec=.02)
 assert time.monotonic()-latest['t']<.3 and np.max(np.abs(np.array(latest['q'])-q))<np.deg2rad(1),'Start changed'
 action=ActionClient(n,ExecuteTrajectory,'/execute_trajectory');assert action.wait_for_server(timeout_sec=15);goal=ExecuteTrajectory.Goal();goal.trajectory=traj;f=action.send_goal_async(goal);rclpy.spin_until_future_complete(n,f,timeout_sec=10);h=f.result();assert h and h.accepted;f=h.get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=120)
 if not f.done():h.cancel_goal_async();raise RuntimeError('Retreat timeout')
 assert f.result().result.error_code.val==1,'Retreat execution failed';result['retreat_success']=True
finally:
 scene=PlanningScene();scene.is_diff=True;scene.allowed_collision_matrix=original;apply(scene)
if result.get('retreat_success'):
 home=call('/soarm4/go_home',Trigger,Trigger.Request(),120);result.update(success=home.success,home=home.message)
(s/'recovery.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));n.destroy_node();rclpy.try_shutdown()
