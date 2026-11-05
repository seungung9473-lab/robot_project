"""Explicit full cycle: home -> stopped scan -> Gemini/SAM2 -> QR/date routes -> reviewed motion."""
import argparse,json,os,subprocess,sys,time,uuid,fcntl
from pathlib import Path
R=Path(__file__).resolve().parent
from test_data_recorder import start_trial

def control_preflight():
 import rclpy
 from rcl_interfaces.srv import GetParameters
 from std_msgs.msg import Int32
 rclpy.init();n=rclpy.create_node('expiry_control_preflight');vac={};n.create_subscription(Int32,'/vacuum/diff',lambda m:vac.update(value=m.data,t=time.monotonic()),10)
 try:
  for node in ['servo_bridge','esp32_bridge','xl330_turntable']:
   c=n.create_client(GetParameters,'/'+node+'/get_parameters')
   if not c.wait_for_service(timeout_sec=10):raise RuntimeError(node+' is not connected')
   req=GetParameters.Request();req.names=['read_only'];f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=10)
   if not f.done() or f.result() is None or len(f.result().values)!=1 or f.result().values[0].type!=1:raise RuntimeError(node+' mode is unknown')
   if f.result().values[0].bool_value:raise RuntimeError(node+' is read-only; enable control before starting a physical cycle')
   n.destroy_client(c)
  end=time.monotonic()+3
  while (not vac or time.monotonic()-vac['t']>.5) and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
  if not vac or time.monotonic()-vac['t']>.5 or vac['value']<=-200:raise RuntimeError('Fresh empty-cup confirmation missing; no automatic home/scan')
 finally:n.destroy_node();rclpy.shutdown()

def main():
 p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');p.add_argument('--reference-date');a=p.parse_args()
 if not a.execute:print('No movement. --execute: home, 0..330deg stopped scan, Gemini/SAM2, QR expiry ranking, trash/ID5/ID6, home.');return
 trial=start_trial('expiry_cycle',physical=True)
 if os.environ.get('ROS_DOMAIN_ID')!='73':raise RuntimeError('Use ROS_DOMAIN_ID=73')
 if a.reference_date:
  from datetime import date
  a.reference_date=date.fromisoformat(a.reference_date).isoformat()
 control_preflight();out=R/'expiry_cycles'/('cycle_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);out.mkdir(parents=True);state={'status':'starting','robot_motion_requested':True};log=out/'cycle.json';trial.attach(log)
 def stage(name):trial.event('stage',stage=name);state['stage']=name;log.write_text(json.dumps(state,ensure_ascii=False,indent=2));print(name,flush=True)
 def run(*cmd):subprocess.run(list(map(str,cmd)),check=True)
 lock=(R/'scan_pick_cycle.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 try:
  stage('home');run(sys.executable,Path.home()/'robot_ws/src/soarm4_moveit_config/scripts/go_home.py')
  stage('scan_12_stopped_angles');run('ros2','run','soarm_turntable','turntable_scan','scan','--angles',*range(0,360,30),'--timeout',60,'--output',out/'scans')
  scans=[s for s in (out/'scans').iterdir() if (s/'session.json').exists()]
  if len(scans)!=1:raise RuntimeError('Ambiguous fresh scan')
  state['session']=str(scans[0]);stage('Gemini_SAM2_QR_and_complete_route_review')
  # Batch planner/executor take the same lock; release only after the scan is complete.
  fcntl.flock(lock,fcntl.LOCK_UN)
  extra=[] if not a.reference_date else ['--reference-date',a.reference_date]
  run(sys.executable,R/'run_expiry_sort.py',scans[0],'--recognize','--read-dates','--execute',*extra)
  report=json.loads((scans[0]/'expiry_route_report.json').read_text());state['unresolved_objects']=[o['object'] for o in report['objects'] if o['status']=='unknown'];state['status']='complete_with_unreadable_objects' if state['unresolved_objects'] else 'complete';stage(state['status'])
 except BaseException as exc:state.update(status='stopped',error=str(exc));stage('stopped; no automatic release of held object');raise
 finally:fcntl.flock(lock,fcntl.LOCK_UN);lock.close()
if __name__=='__main__':main()
