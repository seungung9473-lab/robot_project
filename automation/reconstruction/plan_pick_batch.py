"""Plan order/angles for all requested objects, including future placed obstacles.

Default is planning only. No motor, pump, home or turntable goal is sent.
"""
import argparse,json,subprocess,sys,time,uuid,fcntl,signal
from pathlib import Path
import numpy as np
from batch_search import search_all,require_complete,invariant_source_overlap,SearchBudgetExceeded
R=Path(__file__).resolve().parent
from test_data_recorder import start_trial
from planning_feedback import calculating,geometry_key,reservation_choices,PlanningEnvironment

def main():
 ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);ap.add_argument('--objects',nargs='+',required=True);ap.add_argument('--execute',action='store_true');ap.add_argument('--pad-plan',type=Path,action='append',default=[]);ap.add_argument('--lift-clearance',type=float,default=.055);a=ap.parse_args();s=a.session.resolve();trial_record=start_trial('batch_planning',physical=a.execute,source=s)
 if len(set(a.objects))!=len(a.objects):raise ValueError('Duplicate requested object')
 report=json.loads((s/'inclined_pick_plan.json').read_text());routes={o['object']:o for o in json.loads((s/'color_sort_plan.json').read_text())['objects']};completed=json.loads((s/'completed_placements.json').read_text()) if (s/'completed_placements.json').exists() else {}
 pending=[k for k in a.objects if k not in completed]
 from expiry_policy import validate_route
 for k in pending:
  if k in routes:validate_route(routes[k])
  if k not in routes or (routes[k].get('destination')!='trash_drop' and routes[k].get('destination_pad_id') not in [5,6]):raise ValueError('Unassigned object '+k)
 out=s/('batch_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);out.mkdir();lock=(R/'scan_pick_cycle.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 from pad_occupancy import read_occupancy,validate_reserved
 from tidy_pad_placement import reserve_pad,StaticPlacementRejected
 from motion import kinematics
 from planning_feedback import calculating
 pad_plans={json.loads(p.read_text())['pad_id']:p.resolve() for p in a.pad_plan}
 required_pads={routes[k]['destination_pad_id'] for k in pending if routes[k].get('destination')!='trash_drop'}
 occupancy=read_occupancy(kinematics.W) if required_pads else None
 for pad in required_pads:
  keys=[k for k in pending if routes[k].get('destination_pad_id')==pad]
  if pad not in pad_plans:
   with calculating('패드 '+str(pad)+' 기존 점유 회피 예약'):
    pad_plans[pad]=reserve_pad(s,keys,pad,kinematics,occupancy=occupancy)
  reservation=json.loads(pad_plans[pad].read_text())
  if not all(k in reservation['placements'] for k in keys):raise ValueError('Pad reservation does not cover all requested objects')
  # A stale or legacy reservation cannot bypass the current placed-obstacle check.
  validate_reserved(reservation,occupancy)
 failures={};pick_failures=set();exit_poses={};environment=PlanningEnvironment(s)
 def evaluate(key,done,placed):
  entry=next(x for x in report['objects'] if x['object']==key)
  trash=routes[key].get('destination')=='trash_drop';reservation=None if trash else pad_plans.get(routes[key]['destination_pad_id'])
  candidates=reservation_choices(entry,reservation,out)
  # 접힘이 적은 각도부터: 시작 자세에서 접근 자세까지 관절 이동(J3/J4 가중)이 작은 후보를 먼저 검토한다.
  from transit_clearance import travel_cost
  import math as _math
  _start0=exit_poses.get(tuple(done)) or [0.,_math.pi,-_math.pi,0.]
  candidates=sorted(candidates,key=lambda c:travel_cost(_start0,c[1]['approach_q_rad']))
  for number,(index,opt,reservation) in enumerate(candidates,1):
   reservation_data=json.loads(reservation.read_text()) if reservation else None
   start_q=exit_poses.get(tuple(done));snapshot={'environment_digest':environment.snapshot(),'batch_start_q':start_q}
   token=geometry_key(key,index,done,placed,reservation_data,snapshot)
   pick_token=(key,index,tuple(map(float,start_q)) if start_q is not None else None,tuple(done))
   if pick_token in pick_failures:
    print(f'재탐색: {key} 후보 {index} · 집기 접근 자세가 불가능해 같은 집기 후보의 다른 놓기 위치는 건너뜀',flush=True)
    yield {'success':False,'review':{'object':key,'option':index,'error':'pick-side IK unavailable (skipped)','pick_side_skip':True}};continue
   if token in failures:
    review={**failures[token],'memo_reused':True}
    print(f'재탐색: {key} 후보 {index} · 동일 조건의 기존 검사 결과 재사용 (시간 초과는 불가능 판정 아님)',flush=True)
    trial_record.event('candidate_memo_hit',**review)
    yield {'success':False,'review':review};continue
   print(f'계산중: {key} 후보 {number}/{len(candidates)} · 각도 {opt["turntable_angle_deg"]:.2f}° · 완료 가정 {list(done)}',flush=True)
   placement_args=[] if reservation is None else ['--placement-plan',str(reservation),'--placement-object',key]
   trial=out/(key+'_'+str(index)+'_'+uuid.uuid4().hex[:6]);trial.mkdir();vp=trial/'virtual_placements.json';vp.write_text(json.dumps(list(placed),indent=2))
   cmd=[sys.executable,str(R/'run_inclined_pick.py'),str(s),'--object',key,'--option-index',str(index),'--pad',str(routes[key]['destination_pad_id'] if not trash else 5),'--lift-clearance',str(a.lift_clearance),'--virtual-placements',str(vp)]
   if trash:cmd+=['--destination','trash_drop']
   cmd+=placement_args
   for old in done:cmd+=['--assume-placed',old]
   with calculating(key+' 물체·배치 기하 계산'):
    result=subprocess.run(cmd,capture_output=True,text=True)
   (trial/'prepare.log').write_text(result.stdout+result.stderr)
   if result.returncode:
    yield {'success':False,'review':{'object':key,'option':index,'error':'prepare failed','log':str(trial/'prepare.log')}};continue
   prep=Path(json.loads(result.stdout)['prepared_session']);log=trial/'collision.log'
   manifest_path=prep/'session.json';manifest=json.loads(manifest_path.read_text());manifest['batch_assume_completed']=list(done);manifest_path.write_text(json.dumps(manifest,indent=2))
   chain_args=['--finish-above']+([] if start_q is None else ['--start-q',*map(str,start_q)])
   for budget in (1.,8.):
    print(f'계산중: {key} 안전점검 · 경로당 탐색 한도 {budget:g}초',flush=True)
    with log.open('a') as stream, calculating(key+' 접촉·들기·배치 안전점검'):
     process=subprocess.Popen([sys.executable,'-u',str(R/'check_inclined_motion.py'),str(prep),*placement_args,*chain_args,'--planning-seconds',str(budget)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
     try:
      for line in process.stdout:
       stream.write(line);stream.flush();print(line,end='',flush=True)
      returncode=process.wait()
     except BaseException:
      process.send_signal(signal.SIGINT)
      try:process.wait(timeout=40)
      except subprocess.TimeoutExpired:raise RuntimeError('안전점검 중단 후 장면 정리 확인 실패; 자동 재시작 금지')
      raise
    check=json.loads((prep/'collision_plan_check.json').read_text()) if (prep/'collision_plan_check.json').exists() else {'success':False,'error':'No check result'}
    if returncode==0 and check.get('success'):break
    # A failed search is not proof of impossibility. Only search failures get the longer budget.
    if budget==8. or not any(part in str(check.get('error') or '') for part in ('plan rejected','path rejected')):break
    print('안전점검 실패: 짧은 경로 탐색에서 경로 미확보. 재탐색합니다: 최대 8초',flush=True)
   review={'object':key,'option':index,'angle_deg':opt['turntable_angle_deg'],'previously_placed':list(done),'success':bool(returncode==0 and check.get('success')),'error':check.get('error'),'log':str(log)};print(json.dumps(review,ensure_ascii=False),flush=True);trial_record.event('candidate_review',**review)
   if not review['success']:
    failures[token]=review
    if 'Transit pose above target IK unavailable' in str(review.get('error') or ''):pick_failures.add(pick_token)  # 놓는 자리와 무관한 집기쪽 실패
    print('안전점검 실패: '+str(review['error'])+' · 재탐색합니다: 다른 놓기 위치·자세·순서',flush=True)
    yield {'success':False,'review':review}
    if invariant_source_overlap(review.get('error')):break
    continue
   exit_poses[tuple(done)+(key,)]=check['exit_q_rad']
   if trash:
    yield {'success':True,'object':key,'pad_id':None,'destination':'trash_drop','option_index':index,'angle_deg':opt['turntable_angle_deg'],'prepared_session':str(prep),'placement_plan':None,'placement':{'object':key,'destination':'trash_drop','removed_from_workcell':True},'review':review,'start_q_rad':check['start_q_rad'],'exit_q_rad':check['exit_q_rad']};continue
   cloud=np.load(prep/'analysis/cloud.npz')['points'];O=np.array(check['object_origin_table']);T=np.array(check['predicted_placed_object_pose_table']);placed_cloud=(cloud-O[:3,3])@T[:3,:3].T+T[:3,3];path=trial/'placed_cloud.npz';np.savez_compressed(path,points=placed_cloud)
   placement={'object':key,'collision_id':'batch_virtual_'+key,'cloud_file':str(path),'pose_table':T.tolist()}
   yield {'success':True,'object':key,'pad_id':routes[key]['destination_pad_id'],'option_index':index,'angle_deg':opt['turntable_angle_deg'],'prepared_session':str(prep),'placement_plan':str(reservation) if reservation else None,'placement':placement,'review':review,'start_q_rad':check['start_q_rad'],'exit_q_rad':check['exit_q_rad']}
 try:
  stages,trials=search_all(pending,list(completed),[],evaluate,limit=300,time_limit=180.)
 except SearchBudgetExceeded as exc:
  stages=None;trials=exc.trials
  print('안전점검 실패: 탐색 예산 소진 (경로 불가능 확정 아님)',flush=True)
 finally:environment.close()
 plan={'version':1,'source_scan':str(s),'created_time':time.time(),'requested_objects':pending,'already_completed':list(completed),'all_stages_ready':stages is not None,'stages':stages or [],'trials':trials,'execution_performed':False,'post_rotation_camera_verification':False,'return_home_between_stages':False,'finish_above':True}
 from batch_inputs import fingerprints
 plan['input_fingerprints']=fingerprints(s)
 path=out/'batch_plan.json';trial_record.attach(path);path.write_text(json.dumps(plan,ensure_ascii=False,indent=2));print('BATCH_PLAN',path,flush=True);fcntl.flock(lock,fcntl.LOCK_UN)
 if stages is None:raise SystemExit('안전점검 실패: 탐색 예산 내 전체 안전 경로 미확보. 집기 실행 없음')
 require_complete(plan)
 if a.execute:subprocess.run([sys.executable,str(R/'execute_pick_batch.py'),str(path),'--execute'],check=True)
if __name__=='__main__':main()
