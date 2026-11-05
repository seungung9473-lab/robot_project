"""Execute one validated scan candidate, lift10mm, return and release. Hardware."""
import argparse,json,os,subprocess,sys,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import AttachedCollisionObject,CollisionObject
from motion.kinematics import fk,m
from placement_geometry import load_context
r=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('--execute',action='store_true');p.add_argument('--trial-name',default=None);p.add_argument('--measured-surface-z',type=float,help='Optional explicitly measured surface height in table metres, recorded separately from depth');a=p.parse_args();session=a.session.resolve();folder=session/'reconstruction_sam2';status_file=folder/'processing_status.json';processing=json.loads(status_file.read_text()) if status_file.exists() else {};candidate_file=Path(processing.get('candidate_file',str(folder/'suction_candidates.json')));surface_folder=candidate_file.parent;report=json.loads(candidate_file.read_text());candidate=report['selected_candidate'];camera=candidate['camera'];metrics=report['results'][camera];manifest=json.loads((session/'session.json').read_text());cloud_report=json.loads((folder/'report.json').read_text())
context=load_context(session);axis=np.array(context['axis_point_table_m']);surface=np.array(candidate['nominal_contact_position_table_m'])
if np.linalg.norm(surface[:2]-axis[:2])>context['support_radius_m']-.001:raise RuntimeError('Suction point outside supported disc; no motion')
if not metrics['geometry_pass']:raise RuntimeError('Suction geometry rejected')
if abs(cloud_report['reference_angle_deg'])>1 or manifest['return_to_zero']['status']!='complete':raise RuntimeError('Candidate rotation frame does not match0')
if time.time()-candidate_file.stat().st_mtime>1800:raise RuntimeError('Candidate too old; rescan')
other_camera='cam1' if camera=='cam2' else 'cam2';other=report['results'].get(other_camera,{})
other_points=np.load(surface_folder/(other_camera+'_top_surface.npz'))['points'];check_point=np.array(metrics['candidate_point_reference_table_m']);cross=float(cKDTree(other_points).query(check_point)[0]);plane_error=abs(float(check_point@other['normal_table']+other['plane_d']));normal_error=float(np.degrees(np.arccos(np.clip(np.dot(metrics['normal_table'],other['normal_table']),-1,1))))
if cross>.003 or plane_error>.003 or normal_error>5:raise RuntimeError('Same suction point not confirmed by both cameras')
trial_name=a.trial_name or ('pick_trial_measured_height' if a.measured_surface_z is not None else 'pick_trial')
if Path(trial_name).name!=trial_name:raise RuntimeError('Trial name must be one directory name')
out=session/trial_name;out.mkdir(exist_ok=True)
if (out/'execution_status.json').exists() and a.execute:raise RuntimeError('Trial already exists; choose a new --trial-name')
if a.measured_surface_z is not None:
 if not np.isfinite(a.measured_surface_z) or abs(a.measured_surface_z-candidate['nominal_contact_position_table_m'][2])>.002:raise RuntimeError('Measured height disagrees with reconstruction by more than2mm')
 report['measured_height_override']={'depth_surface_z_m':candidate['nominal_contact_position_table_m'][2],'measured_surface_z_m':a.measured_surface_z,'scope':'current known60mm verification block only; not automatic height correction for other objects'}
 candidate['nominal_contact_position_table_m'][2]=float(a.measured_surface_z);candidate['contact_target_table_m'][2]=float(a.measured_surface_z)+float(json.loads((r/'contact_settings.json').read_text())['contact_z_offset_m']);candidate_file=out/'candidate_with_measured_height.json';candidate_file.write_text(json.dumps(report,indent=2)+'\n')
execution_contact_settings=json.loads((r/'contact_settings.json').read_text());candidate['contact_z_offset_m']=float(execution_contact_settings['contact_z_offset_m']);candidate['contact_target_table_m']=list(candidate['nominal_contact_position_table_m']);candidate['contact_target_table_m'][2]+=candidate['contact_z_offset_m'];report['contact_settings']=execution_contact_settings
if a.measured_surface_z is not None:candidate_file.write_text(json.dumps(report,indent=2)+'\n')
points=np.load(folder/(camera+'_object.npz'))['points'];low=np.percentile(points,.5,axis=0);high=np.percentile(points,99.5,axis=0);low[2]=context['top_height_m'];high+=.002;low[:2]-=.002;dims=high-low;payload={'center_table_m':((high+low)/2).tolist(),'dimensions_m':dims.tolist()};(out/'payload.json').write_text(json.dumps(payload,indent=2))
if not (.03<dims[0]<.12 and .03<dims[1]<.12 and .02<dims[2]<.15):raise RuntimeError('Unexpected object extent')
rclpy.init();node=rclpy.create_node('scan_candidate_pick_coordinator');latest={}
sub=node.create_subscription(JointState,'/joint_states',lambda msg:latest.update(q=[float(msg.position[msg.name.index('joint'+str(i))]) for i in range(1,5)],t=time.monotonic()) if all('joint'+str(i) in msg.name for i in range(1,5)) else None,10)
def call(service,req=None,typ=Trigger,timeout=10):
 client=node.create_client(typ,service)
 try:
  if not client.wait_for_service(timeout_sec=4):raise RuntimeError('Missing '+service)
  f=client.call_async(req or typ.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=timeout)
  if not f.done():raise RuntimeError('Timeout '+service)
  return f.result()
 finally:node.destroy_client(client)
def fresh():
 end=time.monotonic()+8
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.05)
  if latest and time.monotonic()-latest['t']<.3:return latest['q']
 raise RuntimeError('No fresh actual joints')
def turntable_zero():
 from std_msgs.msg import String
 data={};s=node.create_subscription(String,'/turntable/state',lambda msg:data.update(json.loads(msg.data)),10);end=time.monotonic()+5
 try:
  while time.monotonic()<end:
   rclpy.spin_once(node,timeout_sec=.05)
   if data:break
  if not data or abs(data['angle_deg'])>1 or data['hardware_error'] or data['zero_count']!=manifest['return_to_zero']['state']['zero_count']:raise RuntimeError('Turntable state changed after scan')
 finally:node.destroy_subscription(s)
recorder=None;holding=False;status={'candidate':candidate,'same_point_cross_camera_distance_mm':cross*1000,'cross_camera_plane_error_mm':plane_error*1000,'cross_camera_normal_error_deg':normal_error,'phases':[]}
def phase(name):
 (out/'phase.txt').write_text(name);status['phases'].append(name);(out/'execution_status.json').write_text(json.dumps(status,indent=2))
def run(file):subprocess.run([sys.executable,str(file)],check=True,env=env)
try:
 turntable_zero();start=fresh();sc=call('/soarm4/rviz_scaling')
 if not sc.success:raise RuntimeError(sc.message)
 scaling=json.loads(sc.message);env=os.environ.copy();env.update(SOARM_SUCTION_CANDIDATES=str(candidate_file),SOARM_APPROACH_IK=str(out/'approach_ik.json'),SOARM_PAYLOAD_JSON=str(out/'payload.json'),SOARM_LIFT_RESULT=str(out/'lift_result.json'),SOARM_RETURN_RESULT=str(out/'return_result.json'),SOARM_VELOCITY=str(float(scaling['velocity'])),SOARM_ACCELERATION=str(float(scaling['acceleration'])))
 target=np.array(candidate['nominal_contact_position_table_m']);target[2]+=.03
 lo=[float(j['limit']['lower']) for j in m['joints']];hi=[float(j['limit']['upper']) for j in m['joints']]
 def error(q):
  t=fk(q);return np.r_[(t[:3,3]-target)*10,(t[:3,0]-[0,0,-1])*.3]
 solutions=[least_squares(error,np.clip(start,lo,hi),bounds=(lo,hi),max_nfev=600)]
 for seed in range(10):solutions.append(least_squares(error,np.random.default_rng(seed).uniform(lo,hi),bounds=(lo,hi),max_nfev=600))
 valid=[s for s in solutions if np.linalg.norm(fk(s.x)[:3,3]-target)<.0002 and np.linalg.norm(fk(s.x)[:3,0]-[0,0,-1])<.003]
 if not valid:raise RuntimeError('Candidate unreachable')
 sol=min(valid,key=lambda s:np.linalg.norm(s.x-start));(out/'approach_ik.json').write_text(json.dumps({'above':sol.x.tolist()}))
 status.update(scaling=scaling,approach_target_table_m=target.tolist(),contact_settings=json.loads((r/'contact_settings.json').read_text()),sag_settings=json.loads((r/'sag_compensation.json').read_text()))
 if not a.execute:phase('validated_without_motion');print('Preflight complete; no motion');sys.exit(0)
 phase('approach');recorder=subprocess.Popen([sys.executable,str(r/'record_suction_diagnosis.py'),str(out)],env=env)
 end=time.monotonic()+8
 while not(out/'READY').exists():
  if time.monotonic()>end:raise RuntimeError('Recording cameras not ready')
  rclpy.spin_once(node,timeout_sec=.05)
 run(r/'motion/approach.py');phase('contact_moving');run(r/'execute_contact.py');phase('contact_no_vacuum');time.sleep(1)
 phase('suction_resting');grip=call('/vacuum/grip');status['grip_response']=grip.message
 if not grip.success:
  release=call('/vacuum/release');status['release_response']=release.message;raise RuntimeError('Grip rejected; no lift')
 holding=True;time.sleep(1);phase('lifting');run(r/'motion/lift.py');phase('holding');time.sleep(2);phase('returning');run(r/'motion/return.py');phase('released');release=call('/vacuum/release');status['release_response']=release.message
 if not release.success:raise RuntimeError('Release not confirmed')
 holding=False;req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True;attached=AttachedCollisionObject();attached.link_name='cup_tip';attached.object.id='scanned_object_proxy';attached.object.operation=CollisionObject.REMOVE;req.scene.robot_state.attached_collision_objects=[attached]
 if not call('/apply_planning_scene',req,ApplyPlanningScene).success:raise RuntimeError('Payload scene detach failed')
 time.sleep(1);status['success']=True;phase('complete');print('Pick demonstration complete; returned and released')
except BaseException as exc:
 if isinstance(exc,SystemExit) and exc.code==0:raise
 status.update(success=False,error=str(exc),holding=holding,notice='If holding, retain pump and inspect physical state before further commands');phase('failed');raise
finally:
 if recorder:
  (out/'STOP').touch()
  try:recorder.wait(timeout=5)
  except subprocess.TimeoutExpired:recorder.terminate()
 node.destroy_node();rclpy.try_shutdown()
