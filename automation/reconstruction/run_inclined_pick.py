"""Prepare inclined pick; --execute explicitly enables home/table/pick/place motion."""
import argparse,json,subprocess,sys,time,fcntl,uuid
from vacuum_thresholds import HELD_DIFF
from pathlib import Path
import numpy as np,yaml
from inclined_geometry import rotate_surface
R=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('--execute',action='store_true');p.add_argument('--destination',choices=['pad','trash_drop'],default='pad');p.add_argument('--object',default=None);p.add_argument('--assume-placed',action='append',default=[]);p.add_argument('--virtual-placements',type=Path);p.add_argument('--option-index',type=int);p.add_argument('--lift-clearance',type=float);p.add_argument('--placement-plan',type=Path);p.add_argument('--placement-object');p.add_argument('--pad',type=int,choices=[5,6],default=5);a=p.parse_args();s=a.session.resolve();report=json.loads((s/'inclined_pick_plan.json').read_text());wc=yaml.safe_load((s/'workcell_snapshot.yaml').read_text());current=yaml.safe_load((Path.home()/'robot_ws/src/TUMST_ARM_workcell/config/workcell.yaml').read_text())
if wc!=current:raise RuntimeError('Workcell changed since scan; rescan')
options=[]
for obj in report['objects']:
 if a.object is not None and obj['object']!=a.object:continue
 for face in obj['matched_faces']:
  for option in face['alignment_options']:options.append((obj,face,option))
if a.execute and (a.assume_placed or a.virtual_placements):raise RuntimeError('Virtual placement flags are planning-only')
if not options:raise RuntimeError('No two-camera confirmed, reachable inclined surface')
if a.placement_plan is not None:
 reserved=json.loads(a.placement_plan.read_text())['placements'][a.placement_object or a.object]
 a.option_index=reserved.get('pickup_option_index',a.option_index)
if a.option_index is not None:
 if a.object is None:raise RuntimeError('Explicit object required for indexed pickup option')
 obj,face,option=options[a.option_index]
else:
 obj,face,option=max(options,key=lambda x:(x[2]['point_table_m'][2],min(x[1]['face']['boundary_clearance_mm'],x[1]['other_camera_face']['boundary_clearance_mm']),-x[1]['cross_camera_normal_deg']))
if a.lift_clearance is not None:
 if not .01<=a.lift_clearance<=.10:raise RuntimeError('Lift clearance outside 10..100mm')
 option=dict(option,lift_clearance_m=float(a.lift_clearance))

out=s/('inclined_prepared_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);out.mkdir();(out/'analysis').mkdir();(out/'reconstruction_sam2').mkdir();(out/'workcell_snapshot.yaml').write_text(yaml.safe_dump(wc));(out/'spatial_context.json').write_text((s/'spatial_context.json').read_text())
completed_file=s/'completed_placements.json';completed=json.loads(completed_file.read_text()) if completed_file.exists() else {}
if obj['object'] in completed:raise RuntimeError('Object already placed; do not execute again')
angle=option['turntable_angle_deg'];axis=np.array(wc['turntable']['axis_point_table_m']);other=[]
for entry in report['objects']:
 if entry['object'] in completed or entry['object'] in a.assume_placed:continue
 data=np.load(Path(entry['session'])/'analysis/cloud.npz');P=data['points'];from scipy.spatial.transform import Rotation
 rot=Rotation.from_euler('z',angle,degrees=True).as_matrix();Q=(P-axis)@rot.T+axis
 if entry['object']==obj['object']:
  np.savez_compressed(out/'analysis/cloud.npz',points=Q,colors_rgb=data['colors_rgb'])
 else:
  (out/'obstacles').mkdir(exist_ok=True);mesh_file=out/'obstacles'/(entry['object']+'.npz');np.savez_compressed(mesh_file,points=Q);low=np.percentile(Q,.5,axis=0)-.003;high=np.percentile(Q,99.5,axis=0)+.003;low[2]=wc['turntable']['top_height_m'];other.append({'id':'inclined_obstacle_'+entry['object'],'mesh_cloud_file':str(mesh_file),'padding_m':.001,'center_table_m':((low+high)/2).tolist(),'dims':(high-low).tolist(),'rgb_255':np.median(data['colors_rgb'],axis=0).tolist()})
point=option['point_table_m'];normal=option['normal_table'];candidate={'camera':'cam1','nominal_contact_position_table_m':point,'position_table_m':point,'normal_table':normal,'commanded_tool_axis_table':option['commanded_tool_axis_table'],'approach_clearance_m':option.get('approach_clearance_m',.03),'lift_clearance_m':option.get('lift_clearance_m',.04),'approved_for_automatic_execution':False}
# Global shape uses both cameras; any local-only contact refinement is explicitly recorded. No hidden
# vertical offset is copied into the new normal-based contact target.
candidate['local_contact_verification']=face.get('local_contact_verification')
(out/'candidate.json').write_text(json.dumps({'selected_candidate':candidate,'results':{'cam1':{'geometry_pass':True,'normal_table':normal}},'cross_camera':face},indent=2));(out/'reconstruction_sam2/processing_status.json').write_text(json.dumps({'candidate_file':str(out/'candidate.json')}))
manifest=json.loads((s/'session.json').read_text());manifest.update(aligned_turntable_angle_deg=angle,source_scan=str(s));(out/'session.json').write_text(json.dumps(manifest,indent=2));(out/'inclined_context.json').write_text(json.dumps({'selected_object':obj['object'],'destination_pad_id':a.pad,'turntable_angle_deg':angle,'other_objects':other,'option':option,'source_scan':str(s),'virtual_placements':json.loads(a.virtual_placements.read_text()) if a.virtual_placements else [],'simulated_aligned_scene':not a.execute},indent=2))
if a.destination=='trash_drop':
 from trash_destination import load_trash_pose
 destination=load_trash_pose();context_path=out/'inclined_context.json';context=json.loads(context_path.read_text());context.update(destination='trash_drop',destination_pose=destination,destination_pad_id=None);context_path.write_text(json.dumps(context,indent=2))
print(json.dumps({'prepared_session':str(out),'object':obj['object'],'turntable_angle_deg':angle,'surface_tilt_deg':face['face']['tilt_deg'],'execute':a.execute},indent=2),flush=True)
if not a.execute:sys.exit(0)
if time.time()-(s/'session.json').stat().st_mtime>1800:raise RuntimeError('Scan older than30min; rescan before physical execution')
lock=(R/'scan_pick_cycle.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
def run(*cmd):subprocess.run(list(map(str,cmd)),check=True)
# Do not home/rotate while an object may already be held.
import rclpy
from std_msgs.msg import Int32
rclpy.init();check_node=rclpy.create_node('inclined_start_guard');vac={}
check_node.create_subscription(Int32,'/vacuum/diff',lambda msg:vac.update(value=msg.data,t=time.monotonic()),10)
try:
 deadline=time.monotonic()+15
 while not vac and time.monotonic()<deadline:rclpy.spin_once(check_node,timeout_sec=.05)
 if not vac or time.monotonic()-vac['t']>.5:raise RuntimeError('No fresh vacuum feedback before home')
 if vac['value']<=HELD_DIFF:raise RuntimeError('Possible held object: no home or table rotation')
finally:check_node.destroy_node();rclpy.try_shutdown()
# Check future aligned scene without actuating anything before home/table motion.
placement_args=[] if a.placement_plan is None else ['--placement-plan',a.placement_plan.resolve(),'--placement-object',a.placement_object or obj['object']]
run(sys.executable,R/'check_inclined_motion.py',out,*placement_args)
checked=json.loads((out/'collision_plan_check.json').read_text())
if not checked['success']:raise RuntimeError('Inclined approach/contact plan failed: '+checked.get('error','unknown'))
# Capture the destination before rotation; no camera use after rotation.
from soarm4_workcell.camera_calib import capture
capture(str(out/'pre_rotation_observation'),1)
manifest.update(contact_execution_mode='precomputed_prediction_only',pre_rotation_observation_time=time.time())
(out/'session.json').write_text(json.dumps(manifest,indent=2))
# Home completes before table motion; Scanner guards against arm motion during rotation.
run(sys.executable,Path.home()/'robot_ws/src/TUMST_ARM_moveit_config/scripts/go_home.py')
import rclpy
from soarm_turntable.scan import Scanner
rclpy.init();n=Scanner()
try:
 deadline=time.monotonic()+15
 while (not n.arm or not n.states) and time.monotonic()<deadline:n.spin()
 warmup=time.monotonic()+3
 while time.monotonic()<warmup:n.spin()
 n.arm_reference=list(n.arm[1]);state=n.state();n.reference_zero=state['zero_count'];n.reference_stop=state['stop_generation'];n.guard_arm()
 if state['zero_count']!=manifest['return_to_zero']['state']['zero_count']:raise RuntimeError('Turntable zero changed')
 n.move(angle,60);n.wait_stable()
except BaseException:
 n.stop();raise
finally:n.destroy_node();rclpy.try_shutdown()
# Execute the precomputed contact without post-rotation imaging or face fitting;
# trajectory plans are always fresh; pump triggers while moving at1.5mm.
run(sys.executable,R/'verified_place_left.py',out,'--inclined','--pad',str(a.pad),'--execute',*placement_args)

completed[obj['object']]={'pad_id':a.pad if a.destination=='pad' else None,'destination':a.destination,'prepared_session':str(out),'completed_time':time.time()}
temporary=completed_file.with_suffix('.tmp');temporary.write_text(json.dumps(completed,indent=2));temporary.replace(completed_file)
