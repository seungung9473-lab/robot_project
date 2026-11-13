"""Execute only a fully reviewed sequence; stop on any stage failure.

Contact points/angles are never refitted or visually checked after rotation.
"""
import argparse,json,sys,subprocess,time,fcntl,math
from vacuum_thresholds import HELD_DIFF
from pathlib import Path
import numpy as np
from batch_search import require_complete
from batch_inputs import fingerprints
from turntable_wait import review_wait_pose
from wait_pose import check_wait_pose
R=Path(__file__).resolve().parent
from test_data_recorder import start_trial

def main():
 ap=argparse.ArgumentParser();ap.add_argument('plan',type=Path);ap.add_argument('--execute',action='store_true');a=ap.parse_args();path=a.plan.resolve();plan=json.loads(path.read_text());require_complete(plan)
 if not a.execute:print('Validated complete batch; no movement. Add --execute to run.');return
 source=Path(plan['source_scan']);trial=start_trial('batch_execution',physical=True,source=source);trial.attach(path)
 from expiry_policy import validate_route
 routes={o['object']:o for o in json.loads((source/'color_sort_plan.json').read_text())['objects']}
 for stage in plan['stages']:validate_route(routes[stage['object']])
 lock=(R/'scan_pick_cycle.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 if plan['input_fingerprints']!=fingerprints(source):raise RuntimeError('Scan/configuration/executor changed; plan the complete batch again')
 if time.time()-plan['created_time']>1800 or time.time()-(source/'session.json').stat().st_mtime>1800:raise RuntimeError('Batch or scan expired; rescan/replan')
 # Re-read placed objects before any actuator command; changed occupancy invalidates reservations.
 from pad_occupancy import read_occupancy,validate_reserved
 from motion import kinematics
 reservation_paths={stage['placement_plan'] for stage in plan['stages'] if stage.get('placement_plan')}
 if reservation_paths:
  occupancy=read_occupancy(kinematics.W)
  for reservation_path in reservation_paths:validate_reserved(json.loads(Path(reservation_path).read_text()),occupancy)
 import rclpy
 from sensor_msgs.msg import JointState
 from std_msgs.msg import Int32
 from soarm_turntable.scan import Scanner
 from soarm4_workcell.camera_calib import capture
 rclpy.init();guard=rclpy.create_node('batch_initial_guard');feedback={};names=['joint'+str(i) for i in range(1,5)]
 guard.create_subscription(JointState,'/joint_states',lambda m:feedback.update(q=[m.position[m.name.index(k)] for k in names],qt=time.monotonic()) if all(k in m.name for k in names) else None,10);guard.create_subscription(Int32,'/vacuum/diff',lambda m:feedback.update(diff=m.data,pt=time.monotonic()),10)
 try:
  end=time.monotonic()+15
  while time.monotonic()<end:
   rclpy.spin_once(guard,timeout_sec=.05)
   if 'q' in feedback and 'diff' in feedback and time.monotonic()-feedback['qt']<.3 and time.monotonic()-feedback['pt']<.5:break
  if 'q' not in feedback or 'diff' not in feedback or time.monotonic()-feedback.get('qt',0)>.3 or time.monotonic()-feedback.get('pt',0)>.5:raise RuntimeError('Fresh initial joint/pressure feedback missing')
  if feedback['diff']<=HELD_DIFF:raise RuntimeError('Possible held object; no automatic batch start')
  if np.max(abs(np.array(feedback['q'])-[0,math.pi,-math.pi,0]))>math.radians(1):raise RuntimeError('Start the batch from the verified home pose')
 finally:guard.destroy_node();rclpy.shutdown()
 plan['execution_performed']=True;plan['execution_status']='running'
 def save():path.write_text(json.dumps(plan,ensure_ascii=False,indent=2))
 save()
 try:
  for index,stage in enumerate(plan['stages']):
   validate_route(routes[stage['object']])
   prep=Path(stage['prepared_session']);manifest=json.loads((prep/'session.json').read_text());completed_path=source/'completed_placements.json';completed=json.loads(completed_path.read_text()) if completed_path.exists() else {}
   expected=set(plan['already_completed'])|{x['object'] for x in plan['stages'][:index]}
   if set(completed)!=expected:raise RuntimeError('Actual completed objects differ from reviewed batch')
   # Prior placements must match the virtual obstacles used for all-stage review.
   for earlier in plan['stages'][:index]:
    if earlier.get('destination')=='trash_drop':continue
    actual_prep=Path(completed[earlier['object']]['prepared_session']);results=list(actual_prep.glob('placement_left_*/execution.json'))
    if len(results)!=1:raise RuntimeError('Ambiguous prior placement result')
    actual=json.loads(results[0].read_text());T=np.asarray(actual['predicted_placed_object_pose_table']);expected_T=np.asarray(earlier['placement']['pose_table'])
    angular=math.acos(np.clip((np.trace(T[:3,:3]@expected_T[:3,:3].T)-1)/2,-1,1))
    if np.linalg.norm(T[:3,3]-expected_T[:3,3])>.003 or angular>math.radians(1):raise RuntimeError('Prior placed pose changed; stop before next stage')
   trial.event('stage_start',number=index+1,object=stage['object'],pad_id=stage['pad_id'],destination=stage.get('destination','pad'))
   print('BATCH STAGE',index+1,stage['object'],'-> ID',stage['pad_id'],flush=True)
   home_recheck=False
   if stage.get('destination')!='trash_drop':
    import shutil
    import yaml
    from batch_completion import go_home_empty
    from pad_clearance_policy import PadObstructed,judge_pad,pad_observation
    view=Path(capture(str(prep/'pre_rotation_observation'),1)[0])
    wc=yaml.safe_load((prep/'workcell_snapshot.yaml').read_text())
    pad=wc['pads']['pad2' if stage['pad_id']==5 else 'pad1']
    reservation=stage.get('placement_plan')
    polygon=json.loads(Path(reservation).read_text())['placements'][stage['object']]['reserved_polygon_table_m'] if reservation else None
    try:
     judge_pad(pad_observation(view,pad,wc,polygon),bool(reservation))
    except PadObstructed:
     print(f'패드 {stage["pad_id"]} 돌출 감지 · 빈 컵 확인 후 HOME에서 재검사',flush=True)
     home=go_home_empty()
     if not home['success']:raise RuntimeError('패드 재검사 HOME 실패: '+home['message'])
     home_view=Path(capture(str(prep/'home_pad_recheck'),1)[0])
     judge_pad(pad_observation(home_view,pad,wc,polygon),False)
     view.rename(prep/'pre_home_pad_observation')
     shutil.copytree(home_view,view)
     home_recheck=True
     trial.event('pad_rechecked_at_home',object=stage['object'],pad_id=stage['pad_id'])
     print('HOME 패드 재검사 통과 · 실제 시작 자세 기준으로 이동 경로 재검증',flush=True)
   manifest.update(contact_execution_mode='precomputed_prediction_only',pre_rotation_observation_time=time.time(),pad_rechecked_at_home=home_recheck);(prep/'session.json').write_text(json.dumps(manifest,indent=2))
   rclpy.init();scanner=Scanner()
   try:
    end=time.monotonic()+15
    while (not scanner.arm or not scanner.states) and time.monotonic()<end:scanner.spin()
    end=time.monotonic()+1
    while time.monotonic()<end:scanner.spin()
    scanner.arm_reference=list(scanner.arm[1]);
    if plan.get('finish_above'):
     expected_q=[0,math.pi,-math.pi,0] if home_recheck else stage['start_q_rad']
     offset=check_wait_pose(scanner.arm[1],expected_q)
     if offset['replan_first_move']:
      print('대기 자세가 검토 시작 자세와 최대 %.1f° 다름: 이 단계 첫 이동은 현재 자세에서 다시 계획·충돌검사합니다'%offset['max_deg'],flush=True)
      trial.event('wait_pose_offset_replan',stage=index+1,object=stage['object'],max_deg=offset['max_deg'],per_joint_deg=offset['per_joint_deg'])
     review_wait_pose(scanner,scanner.arm[1],source,set(completed),require_empty=True)
    state=scanner.state();scanner.reference_zero=state['zero_count'];scanner.reference_stop=state['stop_generation'];scanner.guard_arm()
    if state['zero_count']!=manifest['return_to_zero']['state']['zero_count']:raise RuntimeError('Turntable zero changed')
    scanner.move(float(stage['angle_deg']),60);scanner.wait_stable()
   except BaseException:scanner.stop();raise
   finally:scanner.destroy_node();rclpy.shutdown()
   command=[sys.executable,str(R/'verified_place_left.py'),str(prep),'--inclined','--pad',str(stage['pad_id'] if stage['pad_id'] is not None else 5),'--execute']
   if stage.get('placement_plan'):command+=['--placement-plan',stage['placement_plan'],'--placement-object',stage['object']]
   if plan.get('finish_above'):command+=['--finish-above']
   subprocess.run(command,check=True)
   completed[stage['object']]={'pad_id':stage['pad_id'],'destination':stage.get('destination','pad'),'prepared_session':str(prep),'completed_time':time.time()};temporary=completed_path.with_suffix('.tmp');temporary.write_text(json.dumps(completed,indent=2));temporary.replace(completed_path);stage['execution_status']='complete';save()
  from batch_completion import finish_batch_home
  finish_batch_home(plan,save=save)
  plan['execution_status']='complete';save()
 except BaseException as exc:
  plan.update(execution_status='stopped',error=str(exc),notice='No automatic retry/restart/release after a stage error');save();raise
if __name__=='__main__':main()
