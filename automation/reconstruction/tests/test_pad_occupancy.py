import copy
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pad_occupancy import snapshot, placed_obstacles, free_offsets, blockers, validate_reserved
from tidy_pad_placement import reserve_pad, StaticPlacementRejected, jointly_choose
from pad_packing import validate_actual
from planning_feedback import retry_safe


def square(x=0,y=0,half=.008):
 return np.array([[x-half,y-half],[x+half,y-half],[x+half,y+half],[x-half,y+half]])

def occupancy(poly):
 return {'version':1,'geometry_digest':'same','obstacles':[{'id':'placed_pad_6_old','polygon_table_m':poly.tolist(),'z_min_m':.1,'z_max_m':.2}]}

def pose(x=0,y=0,z=0):
 return {'position':{'x':x,'y':y,'z':z},'orientation':{'x':0.,'y':0.,'z':0.,'w':1.}}

class OccupancyTests(unittest.TestCase):
 def test_outer_mesh_and_world_transforms_all_applied(self):
  mesh={'vertices':[{'x':x,'y':y,'z':z} for x in (-.01,.01) for y in (-.02,.02) for z in (0,.03)]}
  obj={'id':'placed_pad_6_old','header':{'frame_id':'world'},'pose':pose(.1,.2,0),'meshes':[mesh],'mesh_poses':[pose(.01,.02,.1)]}
  W=np.eye(4);W[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix();W[:3,3]=[1,2,3]
  result=placed_obstacles([obj],W)[0];p=np.array(result['polygon_table_m'])
  np.testing.assert_allclose((p.min(0)+p.max(0))/2,[.78,2.11]);self.assertAlmostEqual(result['z_min_m'],3.1)
 def test_unknown_frame_and_missing_shape_fail_closed(self):
  obj={'id':'placed_pad_5_bad','header':{'frame_id':'other'},'pose':pose()}
  with self.assertRaises(ValueError):placed_obstacles([obj],np.eye(4))
  obj['header']['frame_id']='world'
  with self.assertRaises(ValueError):placed_obstacles([obj],np.eye(4))
 def test_static_workcell_is_not_mistaken_for_placed_payload(self):
  self.assertEqual(placed_obstacles([{'id':'workcell_pad1'}],np.eye(4)),[])
 def test_search_finds_free_space_beyond_old_four_mm_range(self):
  ticks=np.arange(-.05,.0501,.002);grid=np.array([(x,y) for x in ticks for y in ticks])
  occ=occupancy(square(half=.009));free=free_offsets(square(half=.006),grid,[0,0],.05,occ)
  self.assertGreater(len(free),0);self.assertTrue(np.all(np.max(np.abs(free),axis=1)>.004))
  for offset in free:self.assertEqual(blockers(square(half=.006)+offset,occ),[])
 def test_completely_occupied_pad_has_no_candidate(self):
  occ=occupancy(square(half=.06));grid=np.array([[x,y] for x in np.arange(-.05,.051,.002) for y in np.arange(-.05,.051,.002)])
  self.assertEqual(len(free_offsets(square(half=.006),grid,[0,0],.05,occ)),0)
 def test_no_mesh_shrink_or_gap_reduction(self):
  occ=occupancy(square(half=.008));self.assertTrue(blockers(square(x=.017,half=.008),occ))
  self.assertFalse(blockers(square(x=.019,half=.008),occ))
 def test_old_and_changed_reservations_are_rejected_before_path_search(self):
  plan={'pad_id':6,'placements':{'a':{'reserved_polygon_table_m':square().tolist()}}}
  with self.assertRaisesRegex(RuntimeError,'예약 충돌'):validate_reserved(plan,occupancy(square()))
  plan['placements']['a']['reserved_polygon_table_m']=square(x=.03).tolist();plan['occupancy_geometry_digest']='old'
  with self.assertRaisesRegex(RuntimeError,'점유 모델 변경'):validate_reserved(plan,occupancy(square()))
 def test_final_actual_shape_rechecked_against_occupied_geometry(self):
  p=np.c_[square(),np.zeros(4)];plan={'pad_center_table_m':[0,0,0],'radius_m':.05,'edge_margin_m':.002,'object_gap_m':.002,'placements':{'a':{}},'occupied':occupancy(square())['obstacles']}
  with self.assertRaisesRegex(ValueError,'existing placed'):validate_actual(plan,'a',p)
 def test_joint_reservation_backtracks_to_nonoverlapping_pair(self):
  def candidate(x):return {'reserved_polygon_table_m':square(x=x).tolist()}
  result=jointly_choose({'a':[candidate(0),candidate(-.025)],'b':[candidate(0)]})
  self.assertIsNotNone(result);self.assertEqual(result['a']['reserved_polygon_table_m'],square(x=-.025).tolist())
 def test_occupied_failure_disables_rescan(self):
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp)/'pre_execution_safety_failure.json';p.write_text(json.dumps({'kind':'static_placement_rejection','execution_performed':False,'retryable':False}))
   self.assertFalse(retry_safe(tmp))
 def test_reservation_pipeline_with_occupied_pad(self):
  for full in [False,True]:
   with self.subTest(full=full),tempfile.TemporaryDirectory() as tmp:
    s=Path(tmp);obj=s/'object/analysis';obj.mkdir(parents=True)
    cloud=np.array([[.2+x,.2+y,.12+z] for x in (-.005,.005) for y in (-.005,.005) for z in (-.005,.005)])
    np.savez(obj/'cloud.npz',points=cloud)
    wc={'turntable':{'axis_point_table_m':[0,0,0],'top_height_m':.1},'pads':{'pad1':{'center_table_m':[0,0,.1],'diameter_m':.1}},'place_correction_m':{}}
    (s/'workcell_snapshot.yaml').write_text(yaml.safe_dump(wc))
    report={'objects':[{'object':'a','session':str(obj.parent),'matched_faces':[{'alignment_options':[{'turntable_angle_deg':0.,'contact_q_rad':[.2,.2,.2,0]}]}]}]}
    (s/'inclined_pick_plan.json').write_text(json.dumps(report))
    R=Rotation.from_euler('y',90,degrees=True).as_matrix()
    def fk(q):
     T=np.eye(4);T[:3,:3]=R;T[:3,3]=q[:3];return T
    def ik(fk,lo,hi,target,seed,local,dims,j1,pitch):
     q=np.array(seed,float);q[:3]=target+[0,0,dims[2]/2]-R@local[:3,3];return q
    kin=types.SimpleNamespace(fk=fk,W=np.eye(4),m={'joints':[{'limit':{'lower':-3,'upper':3}}]*4})
    occ=occupancy(square(half=.06 if full else .009))
    with patch('tidy_pad_placement.placement_ik',side_effect=ik):
     if full:
      with self.assertRaises(StaticPlacementRejected) as error:reserve_pad(s,['a'],6,kin,occupancy=occ)
      self.assertFalse(error.exception.retryable);self.assertFalse((s/'pad_6_placement_plan.json').exists())
     else:
      result=reserve_pad(s,['a'],6,kin,occupancy=occ);plan=json.loads(result.read_text());validate_reserved(plan,occ)
      self.assertGreater(np.max(np.abs(plan['placements']['a']['offset_xy_m'])),.004)
      self.assertEqual(plan['edge_margin_m'],.002);self.assertEqual(plan['object_gap_m'],.002)
      for alternative in plan['alternatives']['a']:self.assertFalse(blockers(alternative['reserved_polygon_table_m'],occ))

 def test_frontend_occupied_failure_is_not_converted_to_rescan(self):
  import ast
  import tidy_demo
  from unittest.mock import Mock
  source=Path(tidy_demo.__file__).read_text();tree=ast.parse(source)
  reserve=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='reserve')
  with tempfile.TemporaryDirectory() as tmp:
   session=Path(tmp);reader=Mock(return_value=occupancy(square()))
   planner=Mock(side_effect=StaticPlacementRejected('패드 6 점유',retryable=False,reason='occupied_pad',blocked_ids=['placed_old']))
   env={'occupancy_cache':[],'read_occupancy':reader,'kin':types.SimpleNamespace(W=np.eye(4)),
        'reserve_pad':planner,'session':session,'StaticPlacementRejected':StaticPlacementRejected,
        'BatchSafetyFailure':tidy_demo.BatchSafetyFailure,'json':json}
   exec(compile(ast.fix_missing_locations(ast.Module(body=[reserve],type_ignores=[])),'tidy_demo.py','exec'),env)
   with self.assertRaises(RuntimeError) as error:env['reserve'](['a'],6)
   self.assertNotIsInstance(error.exception,tidy_demo.BatchSafetyFailure)
   self.assertFalse(retry_safe(session));self.assertEqual(reader.call_count,1)

if __name__=='__main__':unittest.main()
