"""One authorized cycle: home,12-view scan, reconstruction, candidate, pick demo."""
import argparse,json,os,subprocess,sys,datetime,fcntl
from pathlib import Path
r=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true',help='Physically move robot/turntable and operate vacuum');p.add_argument('--measured-surface-z',type=float,help='Optional measured table-frame surface height, current verification object only');a=p.parse_args()
steps=['robot home','scan0,-30,...,-330 degrees and return0','SAM2 + RGB-D reconstruction','suction surface and geometry checks','IK/collision/vacuum-checked10mm lift, return and release']
if not a.execute:
 print('\n'.join(steps));print('No hardware action. Add --execute only with object and surrounding area prepared.');sys.exit(0)
if os.environ.get('ROS_DOMAIN_ID')!='73':raise RuntimeError('Set actual hardware ROS_DOMAIN_ID=73')
lock=(r/'scan_pick_cycle.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
runs=r/'pipeline_runs';runs.mkdir(exist_ok=True);log=runs/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.json');state={'steps':steps,'status':'starting'}
def save(stage):state['stage']=stage;log.write_text(json.dumps(state,indent=2)+'\n');print(stage,flush=True)
def run(*args):subprocess.run([str(x) for x in args],check=True)
try:
 save('home');run(sys.executable,Path.home()/'robot_ws/src/soarm4_moveit_config/scripts/go_home.py')
 root=r/'multiview';before=set(root.iterdir());save('scan');run('ros2','run','soarm_turntable','turntable_scan','scan','--angles',*[str(-30*i) for i in range(12)],'--output',root)
 fresh=[d for d in set(root.iterdir())-before if (d/'session.json').exists()]
 if len(fresh)!=1:raise RuntimeError('Cannot identify unique new scan')
 session=fresh[0];state['session']=str(session);save('reconstruct_and_generate_surface');run(sys.executable,r/'process_scan.py',session)
 save('pick');extra=[] if a.measured_surface_z is None else ['--measured-surface-z',str(a.measured_surface_z)];run(sys.executable,r/'execute_scan_pick.py',session,'--execute',*extra);state['status']='complete';save('complete')
except BaseException as exc:
 state.update(status='failed',error=str(exc));save('stopped');raise
finally:fcntl.flock(lock,fcntl.LOCK_UN);lock.close()
