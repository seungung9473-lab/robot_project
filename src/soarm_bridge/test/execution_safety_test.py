#!/usr/bin/env python3
"""Exercise real ROS action callbacks against a simulated servo bus only."""
import sys,time,threading,math,json
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import Trigger
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P));import soarm_bridge.servo_bridge as module
class FakeBus:
 def __init__(self,*a):self.positions={i:2047 for i in range(1,5)};self.targets=dict(self.positions);self.torques={i:0 for i in range(1,5)};self.failed=set();self.commands=[];self.corrupt_hold=False
 def ping(self,i):return True
 def read_state(self,i):
  if i in self.failed:return None
  if self.torques[i]:self.positions[i]=self.targets[i]
  return self.positions[i],0,12.,25
 def read(self,i,a,n):return bytes([self.torques[i]]) if a==40 else (int(0 if self.corrupt_hold else self.targets[i]).to_bytes(2,'little') if a==42 else bytes(n))
 def torque(self,i,on):self.torques[i]=int(on)
 def sync_goal(self,goals,acc=10):
  self.commands.append((time.monotonic(),dict(goals)))
  for i,(c,v) in goals.items():self.targets[i]=c
 def close(self):pass
module.STSBus=FakeBus
rclpy.init(args=['--ros-args','--params-file',str(P/'config/soarm.yaml'),'-p','read_only:=false']);bridge=module.ServoBridge();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(bridge);thread=threading.Thread(target=executor.spin,daemon=True);thread.start();clientnode=rclpy.create_node('servo_execution_validation');client=ActionClient(clientnode,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory');stop=clientnode.create_client(Trigger,'/servo/stop')
def wait(f,timeout=5):
 rclpy.spin_until_future_complete(clientnode,f,timeout_sec=timeout)
 assert f.done(),'Timed out';return f.result()
def make(delta=1.,duration=1.):
 g=FollowJointTrajectory.Goal();g.trajectory.joint_names=list(bridge.names);q=[bridge.c2r(i,c) for i,c in enumerate(bridge.pos)];target=list(q);target[0]+=math.radians(delta)
 a=JointTrajectoryPoint();a.positions=q;b=JointTrajectoryPoint();b.positions=target;b.time_from_start.sec=int(duration);b.time_from_start.nanosec=int((duration-int(duration))*1e9);g.trajectory.points=[a,b];return g
result={}
try:
 assert client.wait_for_server(timeout_sec=5);time.sleep(.15)
 bridge.bus.corrupt_hold=True;h=wait(client.send_goal_async(make()));assert h.accepted;r=wait(h.get_result_async());assert r.status==6 and not any(bridge.bus.torques.values());result['bad_initial_hold_blocks_torque_enable']='PASS';bridge.bus.corrupt_hold=False;time.sleep(.1)
 h=wait(client.send_goal_async(make()));assert h.accepted;r=wait(h.get_result_async());assert r.result.error_code==0;radians=bridge.c2r(0,bridge.pos[0]);assert abs(math.degrees(radians)-1)<.4;result['valid_small_execution']='PASS'
 bridge.validate_trajectory(make(10.,5.));result['over_5_degrees_within_urdf_accepted']='PASS'
 for name,change in [('too_fast',lambda g:(setattr(g.trajectory.points[1].time_from_start,'sec',0),setattr(g.trajectory.points[1].time_from_start,'nanosec',1000000))),('outside_limits',lambda g:setattr(g.trajectory.points[1],'positions',[g.trajectory.points[1].positions[0],g.trajectory.points[1].positions[1],g.trajectory.points[1].positions[2]-.001,g.trajectory.points[1].positions[3]])),('duplicate_joints',lambda g:setattr(g.trajectory,'joint_names',['joint1']*4)),('bad_time',lambda g:setattr(g.trajectory.points[1].time_from_start,'sec',0))]:
  g=make();change(g);before=len(bridge.bus.commands);h=wait(client.send_goal_async(g));assert not h.accepted,name;assert len(bridge.bus.commands)==before;result[name+'_rejected']='PASS'
 h=wait(client.send_goal_async(make(-.5,2.)));assert h.accepted;time.sleep(.2);wait(h.cancel_goal_async());r=wait(h.get_result_async());assert r.status==5,r.status;before=len(bridge.bus.commands);time.sleep(.1);assert len(bridge.bus.commands)<=before+1;result['cancel_stops_future_commands']='PASS'
 time.sleep(.1);h=wait(client.send_goal_async(make(1.,2.)));assert h.accepted;time.sleep(.2);assert stop.wait_for_service(timeout_sec=2);assert wait(stop.call_async(Trigger.Request())).success;r=wait(h.get_result_async());assert r.status==6;assert all(bridge.bus.torques.values());result['stop_service_holds_without_torque_drop']='PASS'
 time.sleep(.1);h=wait(client.send_goal_async(make(.5,2.)));assert h.accepted;time.sleep(.2);bridge.bus.failed={2};r=wait(h.get_result_async());assert r.status==6;result['feedback_loss_aborts']='PASS';bridge.bus.failed=set();time.sleep(.15)
 for _,goals in bridge.bus.commands:
  for sid,(count,speed) in goals.items():
   i=sid-1;assert bridge.cmin[i]<=count<=bridge.cmax[i];assert 1<=speed<=bridge.vmax
 result['all_transmitted_goals_within_encoder_bounds']='PASS';print(json.dumps(result,indent=2));(P/'validation_execution_simulation.json').write_text(json.dumps(result,indent=2))
finally:
 bridge.stop_event.set();executor.shutdown(timeout_sec=3);thread.join(timeout=3);clientnode.destroy_node();bridge.destroy_node();rclpy.shutdown()
