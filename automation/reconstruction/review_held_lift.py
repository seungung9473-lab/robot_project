"""Read-only lift collision review. No action goals or scene mutations."""
import json,time,sys
from vacuum_thresholds import HELD_DIFF
from pathlib import Path
import numpy as np,rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from motion.kinematics import fk,m
from inclined_geometry import solve_axis
rclpy.init();n=rclpy.create_node('held_lift_review');names=['joint'+str(i) for i in range(1,5)];latest={};vac={}
n.create_subscription(JointState,'/joint_states',lambda x:latest.update(q=[x.position[x.name.index(k)] for k in names],t=time.monotonic()) if all(k in x.name for k in names) else None,10);n.create_subscription(Int32,'/vacuum/diff',lambda x:vac.update(value=x.data,t=time.monotonic()),10)
end=time.monotonic()+5
while (not latest or not vac) and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
if not latest or not vac or vac['value']>HELD_DIFF:raise RuntimeError('Fresh held-object confirmation missing')
q=np.array(latest['q']);T=fk(q);lo=np.array([j['limit']['lower'] for j in m['joints']],float);hi=np.array([j['limit']['upper'] for j in m['joints']],float);client=n.create_client(GetStateValidity,'/check_state_validity');assert client.wait_for_service(timeout_sec=5);report=[];seed=q
for mm in [0,1,2,3,5,8,10,15,20,30,40,55,65]:
 sol=solve_axis(fk,lo,hi,T[:3,3]+[0,0,mm/1000],T[:3,0],seed=seed)
 if sol is None:report.append({'lift_mm':mm,'ik':False});continue
 seed=sol[0];req=GetStateValidity.Request();req.group_name='arm';req.robot_state=RobotState(joint_state=JointState(name=names,position=seed.tolist()),is_diff=True);f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5);res=f.result();row={'lift_mm':mm,'ik':True,'valid':res.valid,'q':seed.tolist(),'contacts':[{'a':c.contact_body_1,'b':c.contact_body_2,'depth_m':c.depth} for c in res.contacts]};report.append(row);print(json.dumps(row),flush=True)
Path(sys.argv[1]).write_text(json.dumps({'start_q':q.tolist(),'start_tcp':T.tolist(),'pressure_diff':vac['value'],'samples':report},indent=2));n.destroy_node();rclpy.shutdown()
