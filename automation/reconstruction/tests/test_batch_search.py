import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from batch_search import search_all,require_complete,invariant_source_overlap
from trajectory_cache import compatible

class BatchSearchTests(unittest.TestCase):
 def test_only_rigid_source_overlap_is_pruned(self):
  self.assertTrue(invariant_source_overlap("Attached lift collision at 0.0: [('inclined_obstacle_object_3', 'scanned_block')]"))
  self.assertFalse(invariant_source_overlap("Attached lift collision at 0.1: [('inclined_obstacle_object_3', 'scanned_block')]"))
  self.assertFalse(invariant_source_overlap("Attached lift collision at 0.0: [('tool_link', 'scanned_block')]"))
  self.assertFalse(invariant_source_overlap("Attached lift collision at 0.0: [('inclined_obstacle_object_3', 'scanned_block'), ('link2','link3')]"))
 def test_blocking_object_is_placed_first(self):
  contexts=[]
  def evaluate(key,done,placed):
   contexts.append((key,done,placed));yield {'success':not(key=='gray' and 'green' not in done),'object':key,'placement':{'object':key},'review':{'object':key}}
  stages,_=search_all(['gray','green'],[],[],evaluate)
  self.assertEqual([s['object'] for s in stages],['green','gray'])
  self.assertIn(('gray',('green',),({'object':'green'},)),contexts)
 def test_placed_obstacle_can_block_the_remaining_stage(self):
  def evaluate(key,done,placed):
   yield {'success':not bool(placed),'object':key,'placement':key}
  stages,_=search_all(['a','b'],[],[],evaluate);self.assertIsNone(stages)
 def test_no_partial_batch_execution(self):
  with self.assertRaises(RuntimeError):require_complete({'all_stages_ready':False,'requested_objects':['a','b'],'stages':[{'object':'a'}]})
  with self.assertRaises(RuntimeError):require_complete({'all_stages_ready':True,'requested_objects':['a','b'],'stages':[{'object':'a'},{'object':'a'}]})
 def test_alternative_angle_after_dead_end(self):
  def evaluate(key,done,placed):
   if key=='a':
    for angle in [0,90]:yield {'success':True,'object':key,'placement':angle,'review':{'angle':angle}}
   elif done:yield {'success':placed[-1]==90,'object':key,'placement':180}
  stages,_=search_all(['a','b'],[],[],evaluate);self.assertEqual(stages[0]['placement'],90)
 def test_search_budget_never_returns_partial_route(self):
  def evaluate(key,done,placed):
   for i in range(5):yield {'success':False}
  with self.assertRaises(RuntimeError):search_all(['a'],[],[],evaluate,limit=2)
 def test_cache_requires_matching_pose_and_speed(self):
  sc={'velocity':.1,'acceleration':.2};e={'joint_names':['j'],'start':[0.],'goal':[1.],'scaling':sc}
  self.assertTrue(compatible(e,['j'],[.001],[1.001],sc))
  self.assertFalse(compatible(e,['j'],[.02],[1.],sc))
  self.assertFalse(compatible(e,['j'],[0.],[1.],{'velocity':.5,'acceleration':.2}))
  self.assertFalse(compatible(e,['other'],[0.],[1.],sc))
if __name__=='__main__':unittest.main()
