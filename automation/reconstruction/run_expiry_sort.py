"""Gemini instances + QR expiry observations -> local calendar arithmetic -> all-object motion review.

No movement by default. --read-dates decodes per-object QR codes from the stopped scan images.
--execute starts only a completely reviewed sequence; never a background watcher.
"""
import argparse,json,subprocess,sys,time,shutil
from pathlib import Path
from expiry_policy import classify,today_seoul,rank_destinations
from trash_destination import load_trash_pose
R=Path(__file__).resolve().parent
from test_data_recorder import start_trial

def route_dates(session,policy=None):
 s=Path(session).resolve();policy=policy or json.loads((R/'expiry_sort_policy.json').read_text());today=today_seoul()
 evidence=json.loads((s/'expiry_observations.json').read_text())
 if evidence.get('source_scan')!=str(s):raise ValueError('Date observations belong to another scan')
 if time.time()-evidence.get('read_at',0)>1800:raise ValueError('Date observations expired; read fresh labels')
 source=s/'color_sort_plan.json';base=s/'color_sort_plan.before_expiry.json'
 if not base.exists():shutil.copy2(source,base)
 routes=json.loads(base.read_text());by_key={o['object']:o for o in routes['objects']};decisions=[];eligible=[];seen=set()
 completed=json.loads((s/'completed_placements.json').read_text()) if (s/'completed_placements.json').exists() else {}
 for item in evidence['objects']:
  key=item['object']
  if key not in by_key or key in seen:raise ValueError('Unknown/duplicate date identity')
  seen.add(key);decision=classify(item['observations'],today);decisions.append({'object':key,**decision});route=by_key[key];route['expiry_decision']=decision
  if decision['status']=='unknown':route.update(destination=None,destination_pad_id=None,route_status='date_uncertain_keep_as_obstacle');continue
  destination=decision['destination'];route.update(destination=destination,destination_pad_id=decision['destination_pad_id'],route_status='assigned_by_expiry_date')
  if key not in completed:eligible.append(key)
 from datetime import date
 reference=today if policy.get('reference_date','today')=='today' else date.fromisoformat(policy['reference_date'])
 rank_destinations(decisions,reference)
 for decision in decisions:
  if decision['status']=='not_expired':by_key[decision['object']].update(destination=decision['destination'],destination_pad_id=decision['destination_pad_id'],expiry_decision={k:v for k,v in decision.items() if k!='object'})
 # An object omitted by the reader must not inherit a color-based route.
 for key,route in by_key.items():
  if key not in seen:route.update(destination=None,destination_pad_id=None,route_status='date_not_read_keep_as_obstacle')
 if any(o.get('destination')=='trash_drop' for o in by_key.values()):load_trash_pose()
 report={'today':today.isoformat(),'timezone':'Asia/Seoul','policy':policy,'objects':decisions,'eligible_objects':eligible,'robot_commands_sent':False};(s/'expiry_route_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));routes['routing_policy']='expiry date overrides all color routes';source.write_text(json.dumps(routes,ensure_ascii=False,indent=2));return report

def main():
 ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);ap.add_argument('--read-dates',action='store_true');ap.add_argument('--recognize',action='store_true',help='Gemini object identification and SAM2 preprocessing; uses configured API');ap.add_argument('--reference-date',help='YYYY-MM-DD comparison date; expired status always uses real today');ap.add_argument('--execute',action='store_true');a=ap.parse_args();s=a.session.resolve();trial=start_trial('expiry_recognition_and_sort',physical=a.execute,source=s)
 for name in ['gemini_detection.json','color_sort_plan.json','expiry_observations.json','expiry_route_report.json','completed_placements.json']:trial.attach(s/name)
 manifest=json.loads((s/'session.json').read_text())
 if manifest['status']!='complete' or time.time()-(s/'session.json').stat().st_mtime>1800:raise RuntimeError('Fresh completed reconstructed scan required')
 if a.recognize:
  if (s/'objects').exists():raise RuntimeError('Object instances already exist; use a new scan for re-recognition')
  from prepare_expiry_instances import prepare_instances
  prepare_instances(s)
 if a.read_dates:
  from read_expiry_labels import read_dates
  read_dates(s)
 policy=json.loads((R/'expiry_sort_policy.json').read_text())
 if a.reference_date:
  from datetime import date
  policy['reference_date']=date.fromisoformat(a.reference_date).isoformat()
 report=route_dates(s,policy);print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
 if not report['eligible_objects']:print('No unambiguous pending date targets; no movement');return
 cmd=[sys.executable,str(R/'plan_pick_batch.py'),str(s),'--objects',*report['eligible_objects']]
 if a.execute:cmd.append('--execute')
 subprocess.run(cmd,check=True)
 trial.finish('partial' if any(o['status']=='unknown' for o in report['objects']) else ('complete' if a.execute else 'planned'))
if __name__=='__main__':main()
