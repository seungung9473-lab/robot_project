"""Plan order/angles for all requested objects, including future placed obstacles.

Default is planning only. No motor, pump, home or turntable goal is sent.
"""
import argparse,json,subprocess,sys,time,uuid,fcntl
from pathlib import Path
import numpy as np
from batch_search import search_all,require_complete,invariant_source_overlap
R=Path(__file__).resolve().parent
from test_data_recorder import start_trial

def main():
 ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);ap.add_argument('--objects',nargs='+',required=True);ap.add_argument('--execute',action='store_true');ap.add_argument('--pad-plan',type=Path,action='append',default=[]);ap.add_argument('--lift-clearance',type=float,default=.055);a=ap.parse_args();s=a.session.resolve();trial_record=start_trial('batch_planning',physical=a.execute,source=s)
 if len(set(a.objects))!=len(a.objects):raise ValueError('Duplicate requested object')
 report=json.loads((s/'inclined_pick_plan.json').read_text());routes={o['object']:o for o in json.loads((s/'color_sort_plan.json').read_text())['objects']};completed=json.loads((s/'completed_placements.json').read_text()) if (s/'completed_placements.json').exists() else {}
 pending=[k for k in a.objects if k not in completed]
 from expiry_policy import validate_route
 for k in pending:
  if k in routes:validate_route(routes[k])
  if k not in routes or (routes[k].get('destination')!='trash_drop' and routes[k].get('destination_pad_id') not in [5,6]):raise ValueError('Unassigned object '+k)
 pad_plans={json.loads(p.read_text())['pad_id']:p.resolve() for p in a.pad_plan}
 for pad in {routes[k]['destination_pad_id'] for k in pending if routes[k].get('destination')!='trash_drop'}:
  keys=[k for k in pending if routes[k]['destination_pad_id']==pad]
  if len(keys)>1 and pad not in pad_plans:
   subprocess.run([sys.executable,str(R/'plan_batch_pad.py'),str(s),str(pad)],check=True)
   pad_plans[pad]=s/('pad_'+str(pad)+'_placement_plan.json')
  if pad in pad_plans and not all(k in json.loads(pad_plans[pad].read_text())['placements'] for k in keys):raise ValueError('Pad reservation does not cover all requested objects')
 out=s/('batch_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);out.mkdir();lock=(R/'scan_pick_cycle.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 def evaluate(key,done,placed):
  entry=next(x for x in report['objects'] if x['object']==key);choices=[opt for face in entry['matched_faces'] for opt in face['alignment_options']]
  for index,opt in enumerate(choices):
   trash=routes[key].get('destination')=='trash_drop';reservation=None if trash else pad_plans.get(routes[key]['destination_pad_id'])
   if reservation and index!=json.loads(reservation.read_text())['placements'][key].get('pickup_option_index',index):continue
   placement_args=[] if reservation is None else ['--placement-plan',str(reservation),'--placement-object',key]
   trial=out/(key+'_'+str(index)+'_'+uuid.uuid4().hex[:6]);trial.mkdir();vp=trial/'virtual_placements.json';vp.write_text(json.dumps(list(placed),indent=2))
   cmd=[sys.executable,str(R/'run_inclined_pick.py'),str(s),'--object',key,'--option-index',str(index),'--pad',str(routes[key]['destination_pad_id'] if not trash else 5),'--lift-clearance',str(a.lift_clearance),'--virtual-placements',str(vp)]
   if trash:cmd+=['--destination','trash_drop']
   cmd+=placement_args
   for old in done:cmd+=['--assume-placed',old]
   result=subprocess.run(cmd,capture_output=True,text=True);(trial/'prepare.log').write_text(result.stdout+result.stderr)
   if result.returncode:
    yield {'success':False,'review':{'object':key,'option':index,'error':'prepare failed','log':str(trial/'prepare.log')}};continue
   prep=Path(json.loads(result.stdout)['prepared_session']);log=trial/'collision.log'
   with log.open('w') as stream:run=subprocess.run([sys.executable,str(R/'check_inclined_motion.py'),str(prep),*placement_args],stdout=stream,stderr=subprocess.STDOUT)
   check=json.loads((prep/'collision_plan_check.json').read_text()) if (prep/'collision_plan_check.json').exists() else {'success':False,'error':'No check result'}
   review={'object':key,'option':index,'angle_deg':opt['turntable_angle_deg'],'previously_placed':list(done),'success':bool(run.returncode==0 and check.get('success')),'error':check.get('error'),'log':str(log)};print(json.dumps(review,ensure_ascii=False),flush=True);trial_record.event('candidate_review',**review)
   if not review['success']:
    yield {'success':False,'review':review}
    if invariant_source_overlap(review.get('error')):break
    continue
   if trash:
    yield {'success':True,'object':key,'pad_id':None,'destination':'trash_drop','option_index':index,'angle_deg':opt['turntable_angle_deg'],'prepared_session':str(prep),'placement_plan':None,'placement':{'object':key,'destination':'trash_drop','removed_from_workcell':True},'review':review};continue
   cloud=np.load(prep/'analysis/cloud.npz')['points'];O=np.array(check['object_origin_table']);T=np.array(check['predicted_placed_object_pose_table']);placed_cloud=(cloud-O[:3,3])@T[:3,:3].T+T[:3,3];path=trial/'placed_cloud.npz';np.savez_compressed(path,points=placed_cloud)
   placement={'object':key,'collision_id':'batch_virtual_'+key,'cloud_file':str(path),'pose_table':T.tolist()}
   yield {'success':True,'object':key,'pad_id':routes[key]['destination_pad_id'],'option_index':index,'angle_deg':opt['turntable_angle_deg'],'prepared_session':str(prep),'placement_plan':str(reservation) if reservation else None,'placement':placement,'review':review}
 stages,trials=search_all(pending,list(completed),[],evaluate)
 plan={'version':1,'source_scan':str(s),'created_time':time.time(),'requested_objects':pending,'already_completed':list(completed),'all_stages_ready':stages is not None,'stages':stages or [],'trials':trials,'execution_performed':False,'post_rotation_camera_verification':False,'return_home_between_stages':True}
 from batch_inputs import fingerprints
 plan['input_fingerprints']=fingerprints(s)
 path=out/'batch_plan.json';trial_record.attach(path);path.write_text(json.dumps(plan,ensure_ascii=False,indent=2));print('BATCH_PLAN',path,flush=True);fcntl.flock(lock,fcntl.LOCK_UN)
 if stages is None:raise SystemExit('No complete sequence found; no robot movement was executed')
 require_complete(plan)
 if a.execute:subprocess.run([sys.executable,str(R/'execute_pick_batch.py'),str(path),'--execute'],check=True)
if __name__=='__main__':main()
