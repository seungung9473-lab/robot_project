import ast
import numpy as np
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from planning_feedback import calculating, geometry_key, reservation_choices, retry_safe
import tidy_pad_placement  # Load numerical dependencies before subprocess mocks.
import tidy_demo

class PlanningUpdateTests(unittest.TestCase):
 def test_progress_heartbeat_stops_after_action(self):
  rows=[]
  with calculating('物체',lambda text,**kw:rows.append(text),interval=.01):time.sleep(.035)
  count=len(rows);time.sleep(.03)
  self.assertGreaterEqual(count,2);self.assertEqual(len(rows),count)
 def test_memo_key_changes_for_pose_speed_geometry_and_reservation(self):
  with tempfile.TemporaryDirectory() as tmp:
   cloud=Path(tmp)/'cloud';cloud.write_text('a')
   placement={'object':'a','cloud_file':str(cloud),'pose_table':[1]}
   base=geometry_key('b',0,['a'],[placement],{'offset':[0]},'scene1')
   for option,done,shapes,reservation,environment in [(1,['a'],[placement],{'offset':[0]},'scene1'),(0,[],[placement],{'offset':[0]},'scene1'),(0,['a'],[{**placement,'pose_table':[2]}],{'offset':[0]},'scene1'),(0,['a'],[placement],{'offset':[1]},'scene1'),(0,['a'],[placement],{'offset':[0]},'scene2')]:
    self.assertNotEqual(base,geometry_key('b',option,done,shapes,reservation,environment))
   cloud.write_text('b');self.assertNotEqual(base,geometry_key('b',0,['a'],[placement],{'offset':[0]},'scene1'))
 def test_alternative_reservations_do_not_overwrite_primary_or_multi_object(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);p=root/'plan.json';primary={'pickup_option_index':0,'offset_xy_m':[0,0]};alt={'pickup_option_index':0,'offset_xy_m':[.002,0]}
   plan={'placements':{'a':primary},'alternatives':{'a':[alt]}}
   p.write_text(json.dumps(plan));original=p.read_text()
   entry={'object':'a','matched_faces':[{'alignment_options':[{'turntable_angle_deg':0}]}]}
   variants=reservation_choices(entry,p,root);self.assertEqual(len(variants),2);self.assertEqual(p.read_text(),original)
   self.assertEqual(json.loads(variants[1][2].read_text())['placements']['a'],alt)
   plan['placements']['b']=primary;p.write_text(json.dumps(plan));self.assertEqual(len(reservation_choices(entry,p,root)),1)
 def test_retry_denied_after_any_execution_or_incomplete_report(self):
  with tempfile.TemporaryDirectory() as tmp:
   s=Path(tmp);self.assertFalse(retry_safe(s));b=s/'batch_1';b.mkdir();p=b/'batch_plan.json'
   for report,expected in [({},False),({'all_stages_ready':False,'execution_performed':False},True),({'all_stages_ready':True,'execution_performed':False},False),({'all_stages_ready':False,'execution_performed':True},False)]:
    p.write_text(json.dumps(report));self.assertEqual(retry_safe(s),expected)
   p.write_text(json.dumps({'all_stages_ready':False,'execution_performed':False}));(s/'completed_placements.json').write_text('{}');self.assertFalse(retry_safe(s))
 def test_cli_rescan_cap_and_safe_default(self):
  with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(tidy_demo.main([]),0)
  with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):tidy_demo.main(['--max-rescans','2'])
 def test_batch_rescan_is_bounded_and_never_retries_executor_failure(self):
  for planning_failure in [True,False]:
   with self.subTest(planning_failure=planning_failure),tempfile.TemporaryDirectory() as tmp:
    root=Path(tmp);code=root/'automation/reconstruction';code.mkdir(parents=True);er=root/'automation/er2';er.mkdir(parents=True)
    directory=root/'trial';directory.mkdir();trial=Mock(directory=directory);counts={'scan':0,'plan':0}
    def popen(command,**kwargs):
     command=list(map(str,command));status=0
     if command[:4]==['ros2','run','soarm_turntable','turntable_scan']:
      counts['scan']+=1;session=Path(command[command.index('--output')+1])/'fresh';session.mkdir(parents=True)
      (session/'session.json').write_text(json.dumps({'status':'complete','views':[{}]*12}))
      (session/'color_sort_plan.json').write_text(json.dumps({'objects':[]}))
     elif 'plan_pick_batch.py' in command[1]:
      counts['plan']+=1;session=Path(command[2]);b=session/'batch_1';b.mkdir()
      (b/'batch_plan.json').write_text(json.dumps({'all_stages_ready':not planning_failure,'execution_performed':not planning_failure,'trials':[]}));status=1
     return Mock(stdout=iter(['child log\n']),wait=Mock(return_value=status))
    fake_torch=types.SimpleNamespace(cuda=types.SimpleNamespace(is_available=lambda:True,get_device_name=lambda:'GPU'))
    modules={'test_data_recorder':types.SimpleNamespace(start_trial=lambda *a,**kw:trial),'torch':fake_torch,
     'run_qr_expiry_cycle':types.SimpleNamespace(control_preflight=lambda:None),'prepare_expiry_instances':types.SimpleNamespace(prepare_instances=lambda s:None),
     'read_expiry_labels':types.SimpleNamespace(read_dates=lambda s:None),'run_expiry_sort':types.SimpleNamespace(route_dates=lambda s:{'eligible_objects':['a'],'objects':[]}),
     'motion.kinematics':types.SimpleNamespace()}
    with patch.dict(sys.modules,modules),patch.dict(tidy_demo.os.environ,{'ROS_DOMAIN_ID':'73'}),patch.object(tidy_demo,'select_backend',return_value=root),patch.object(tidy_demo,'understand',return_value=({'operation':'expiry_sort','scope':'turntable','reason':'정리'},{})),patch.object(tidy_demo,'LiveTelemetry'),patch('pad_occupancy.clear_confirmed_empty_pads',return_value={'success':True}),patch.object(tidy_demo,'rescan_preflight') as guard,patch.object(tidy_demo.subprocess,'Popen',side_effect=popen),contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
     self.assertEqual(tidy_demo.main(['--execute']),1)
    self.assertEqual(counts['scan'],2 if planning_failure else 1);self.assertEqual(counts['plan'],counts['scan']);self.assertEqual(guard.call_count,1 if planning_failure else 0)
 def test_home_ptp_before_rrt_and_no_execution_on_both_failure(self):
  # Exercise actual go_home method using mocked ROS interfaces; never sends hardware commands.
  tree=ast.parse((Path(__file__).resolve().parents[3]/'src/soarm4_moveit_config/scripts/home_manager.py').read_text())
  method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='go_home')
  import math
  class Request:
   def __init__(self):self.motion_plan_request=types.SimpleNamespace()
  class Constraints:
   def __init__(self):self.joint_constraints=[]
  class State:
   pass
  env={'GetMotionPlan':types.SimpleNamespace(Request=Request),'RobotState':State,'Constraints':Constraints,'JointState':lambda **kw:kw,'JointConstraint':lambda **kw:kw,'SetBool':types.SimpleNamespace(Request=lambda **kw:kw),'MoveItErrorCodes':types.SimpleNamespace(SUCCESS=1),'math':math}
  exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'home_manager.py','exec'),env)
  for codes in [(1,),(-1,1),(-1,-1)]:
   node=Mock();node.enabled=True;node.lock=threading.Lock();node.stop=threading.Event();node.names=['a'];node.home=[1.];node.limits=[(-2,2)];node.active_goal=None;node.read_rviz_scaling.return_value=(.3,.4);node.fresh.return_value=[0.]
   attempts=[];remaining=iter(codes)
   def call(client,request,*args):
    if client is node.torque:return types.SimpleNamespace(success=True)
    r=request.motion_plan_request;attempts.append((r.pipeline_id,r.planner_id))
    # Empty trajectory ends function before any action execution, even for a successful plan.
    return types.SimpleNamespace(motion_plan_response=types.SimpleNamespace(error_code=types.SimpleNamespace(val=next(remaining)),trajectory=types.SimpleNamespace(joint_trajectory=types.SimpleNamespace(points=[]))))
   node.call.side_effect=call;res=types.SimpleNamespace()
   env['go_home'](node,None,res)
   self.assertEqual(attempts[0],('pilz_industrial_motion_planner','PTP'));self.assertEqual(len(attempts),len(codes));node.execute.send_goal_async.assert_not_called()
 def test_collision_endpoints_reject_before_expensive_planning(self):
  import numpy as np
  root=Path(__file__).resolve().parents[1]
  source=root/'check_inclined_motion.py'
  if not source.exists():source=Path(__file__).parent/'check_inclined_motion.py'
  tree=ast.parse(source.read_text());method=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='call')
  validity_type=types.SimpleNamespace(Request=lambda:types.SimpleNamespace())
  plan_type=object();future=Mock();future.done.return_value=True;future.result.return_value=types.SimpleNamespace(valid=False,contacts=[types.SimpleNamespace(contact_body_1='placed_pad',contact_body_2='scanned_block')])
  client=Mock();client.wait_for_service.return_value=True;client.call_async.return_value=future
  node=Mock();node.create_client.return_value=client;raw=Mock()
  env={'GetMotionPlan':plan_type,'GetStateValidity':validity_type,'state':lambda q:q,'n':node,'rclpy':Mock(),'raw_plan':raw,'m':{'poses':{'home':[0,0,0,0]}},'np':np}
  exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'checker.py','exec'),env)
  request=types.SimpleNamespace(motion_plan_request=types.SimpleNamespace(start_state=[1,0,0,0],goal_constraints=[types.SimpleNamespace(joint_constraints=[types.SimpleNamespace(position=0) for _ in range(4)])]))
  with self.assertRaisesRegex(RuntimeError,'collision'):env['call']('/plan_kinematic_path',plan_type,request)
  raw.assert_not_called()

 def test_static_failure_marker_allows_only_unexecuted_retry(self):
  with tempfile.TemporaryDirectory() as tmp:
   s=Path(tmp);marker=s/'pre_execution_safety_failure.json'
   marker.write_text(json.dumps({'kind':'static_placement_rejection','execution_performed':False}));self.assertTrue(retry_safe(s))
   marker.write_text(json.dumps({'kind':'static_placement_rejection','execution_performed':True}));self.assertFalse(retry_safe(s))
   marker.write_text(json.dumps({'kind':'static_placement_rejection','execution_performed':False}));(s/'completed_placements.json').write_text('{}');self.assertFalse(retry_safe(s))

if __name__=='__main__':unittest.main()
