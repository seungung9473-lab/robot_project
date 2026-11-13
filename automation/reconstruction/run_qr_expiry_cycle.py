"""Explicit full cycle: home -> stopped scan -> Gemini/SAM2 -> QR/date routes -> reviewed motion."""
import argparse,json,os,subprocess,sys,time,uuid,fcntl
from vacuum_thresholds import HELD_DIFF
from pathlib import Path
R=Path(__file__).resolve().parent
from test_data_recorder import start_trial

def control_preflight(*, zero_pressure=False):
 import rclpy
 from rcl_interfaces.srv import GetParameters
 from std_msgs.msg import Int32
 from std_srvs.srv import Trigger
 rclpy.init();n=rclpy.create_node('expiry_control_preflight');vac={}
 samples=[]

 def receive_pressure(message):
  vac.update(value=message.data,t=time.monotonic())
  if zero_pressure:
   samples.append((vac['t'],message.data))

 def vacuum_command(name):
  client=n.create_client(Trigger,'/vacuum/'+name)
  try:
   if not client.wait_for_service(timeout_sec=5):
    raise RuntimeError(f'진공 {name} 서비스 연결 안 됨')
   future=client.call_async(Trigger.Request())
   rclpy.spin_until_future_complete(n,future,timeout_sec=5)
   response=future.result() if future.done() else None
   if response is None or not response.success:
    raise RuntimeError(f'진공 {name} 실패: '+(response.message if response else '응답 없음'))
  finally:
   n.destroy_client(client)

 n.create_subscription(Int32,'/vacuum/diff',receive_pressure,10)
 try:
  for node in ['servo_bridge','esp32_bridge','xl330_turntable']:
   c=n.create_client(GetParameters,'/'+node+'/get_parameters')
   if not c.wait_for_service(timeout_sec=10):raise RuntimeError(node+' is not connected. 먼저 ros2 launch soarm_pipeline desktop_ready.launch.py allow_execution:=true 로 장치를 준비하세요.')
   req=GetParameters.Request();req.names=['read_only'];f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=10)
   if not f.done() or f.result() is None or len(f.result().values)!=1 or f.result().values[0].type!=1:raise RuntimeError(node+' mode is unknown')
   if f.result().values[0].bool_value:raise RuntimeError(node+' is read-only; enable control before starting a physical cycle')
   n.destroy_client(c)
  end=time.monotonic()+3
  while (not vac or time.monotonic()-vac['t']>.5) and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
  if not vac or time.monotonic()-vac['t']>.5 or vac['value']<=HELD_DIFF:raise RuntimeError('Fresh empty-cup confirmation missing; no automatic home/scan')
  if zero_pressure:
   vacuum_command('off')
   vacuum_command('zero')
   after=time.monotonic()+.4
   deadline=after+5
   while time.monotonic()<deadline:
    rclpy.spin_once(n,timeout_sec=.05)
    recent=[value for received,value in samples if received>after][-3:]
    if len(recent)==3 and all(abs(value)<=50 for value in recent):
     return recent
   raise RuntimeError(f"압력 영점 확인 실패 · 마지막 diff={vac.get('value', '수신 없음')}")
 finally:n.destroy_node();rclpy.shutdown()

def main():
 p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');p.add_argument('--reference-date');a=p.parse_args()
 if not a.execute:print('No movement. --execute: home, 0..330deg stopped scan, Gemini/SAM2, QR expiry ranking, trash/ID5/ID6, home.');return
 trial=start_trial('expiry_cycle',physical=True)
 if os.environ.get('ROS_DOMAIN_ID')!='73':raise RuntimeError('Use ROS_DOMAIN_ID=73')
 if a.reference_date:
  from datetime import date
  a.reference_date=date.fromisoformat(a.reference_date).isoformat()
 from desktop_runtime import require_cuda_profile, gpu_python
 runtime=require_cuda_profile();os.environ['SOARM_SAM2_DEVICE']='cuda';os.environ['SOARM_SAM2_PYTHON']=gpu_python();trial.event('sam2_runtime',**runtime);print('CUDA/FP32: '+runtime['gpu_name'],flush=True)
 control_preflight();out=R/'expiry_cycles'/('cycle_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);out.mkdir(parents=True);state={'status':'starting','robot_motion_requested':True};log=out/'cycle.json';trial.attach(log)
 def stage(name):trial.event('stage',stage=name);state['stage']=name;log.write_text(json.dumps(state,ensure_ascii=False,indent=2));print(name,flush=True)
 def run(*cmd):subprocess.run(list(map(str,cmd)),check=True)
 lock=(R/'scan_pick_cycle.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 try:
  from scan_scene_reset import clear_previous_scan_scene
  stage('clear_previous_turntable_and_pad_scans')
  clear_previous_scan_scene(trial.directory);trial.attach(trial.directory/'previous_scan_reset.json');trial.attach(trial.directory/'previous_scan_reset_before.json')
  stage('home');run(sys.executable,Path.home()/'robot_ws/src/TUMST_ARM_moveit_config/scripts/go_home.py')
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
