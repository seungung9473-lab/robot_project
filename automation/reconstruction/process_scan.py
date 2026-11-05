"""Process one explicit scan session. Offline only; never moves robot or pump."""
import argparse,json,subprocess,sys
from pathlib import Path
r=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('--reuse-masks',action='store_true');a=p.parse_args();session=a.session.resolve();manifest=json.loads((session/'session.json').read_text())
if manifest['status']!='complete' or len(manifest['views'])!=12:raise RuntimeError('Requires a complete12-view scan')
returned=manifest.get('return_to_zero',{})
if returned.get('status')!='complete' or abs(returned.get('state',{}).get('angle_deg',999))>1:raise RuntimeError('Return to0 not confirmed')
angles=[v['angle_deg'] for v in manifest['views']]
if abs(angles[0])>1 or any(abs((b-a)+30)>1 for a,b in zip(angles,angles[1:])):raise RuntimeError('Unexpected reference or angular spacing')
def run(*args):subprocess.run([str(x) for x in args],check=True)
if not a.reuse_masks:
 run(sys.executable,r/'prepare_sam2_prompts.py','--session',session)
 run(r/'.sam2_cpu/bin/python',r/'apply_sam2.py','--session',session)
out=session/'reconstruction_sam2'
run(sys.executable,r/'reconstruct.py',session,'--direction',1,'--reference-angle',0,'--mask-root',session/'sam2','--output',out)
run(sys.executable,r/'find_suction_surface.py','--folder',out)
c=json.loads((out/'suction_candidates.json').read_text());selected=c.get('selected_candidate',{});metrics=c.get('results',{}).get(selected.get('camera'),{});ready=bool(selected and metrics.get('geometry_pass',False))
status={'session':str(session),'candidate_file':str(out/'suction_candidates.json'),'geometry_pass':ready,'robot_execution_performed':False,'requires_fresh_feedback_ik_collision_vacuum_checks':True}
(out/'processing_status.json').write_text(json.dumps(status,indent=2)+'\n');print(json.dumps(status,indent=2),flush=True)

run(sys.executable,r/'refine_reference_view.py',session)
