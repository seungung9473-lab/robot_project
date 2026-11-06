import copy,json,tempfile,unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from moveit_msgs.msg import PlanningScene,CollisionObject,AttachedCollisionObject
from pad_occupancy import reset_empty_pad_scene,stale_pad_ids
class EmptyPadResetTests(unittest.TestCase):
 def scene(self,ids):
  s=PlanningScene()
  for key in ids:
   obj=CollisionObject();obj.id=key;s.world.collision_objects.append(obj)
  return s
 def test_only_previous_models_on_five_six_removed(self):
  original=['placed_pad_5_old','placed_pad_6_old','workcell_pad1','workcell_pad2','workcell_table_guard','placed_pad_7_keep','batch_virtual_keep']
  current=self.scene(original);removed=[]
  def apply(diff):
   self.assertTrue(diff.is_diff)
   for obj in diff.world.collision_objects:
    self.assertEqual(obj.operation,CollisionObject.REMOVE);removed.append(obj.id)
   current.world.collision_objects=[o for o in current.world.collision_objects if o.id not in removed]
  with tempfile.TemporaryDirectory() as tmp:
   report=reset_empty_pad_scene(lambda:copy.deepcopy(current),apply,tmp)
   self.assertTrue(report['success']);self.assertEqual(set(removed),{'placed_pad_5_old','placed_pad_6_old'})
   self.assertEqual(set(report['retained_ids']),set(original)-set(removed));self.assertFalse(report['motor_commands_sent'])
   self.assertTrue((Path(tmp)/'empty_pad_reset_before.json').exists())
 def test_second_reset_is_noop(self):
  scene=self.scene(['workcell_pad1']);calls=[]
  with tempfile.TemporaryDirectory() as tmp:
   report=reset_empty_pad_scene(lambda:copy.deepcopy(scene),calls.append,tmp)
   self.assertEqual(report['removed_ids'],[]);self.assertEqual(calls,[])
 def test_attached_payload_prevents_clear(self):
  scene=self.scene(['placed_pad_5_old']);scene.robot_state.attached_collision_objects=[AttachedCollisionObject()];calls=[]
  with tempfile.TemporaryDirectory() as tmp,self.assertRaisesRegex(RuntimeError,'부착'):
   reset_empty_pad_scene(lambda:copy.deepcopy(scene),calls.append,tmp)
  self.assertEqual(calls,[])
 def test_unremoved_objects_fail_closed(self):
  scene=self.scene(['placed_pad_5_old'])
  with tempfile.TemporaryDirectory() as tmp,self.assertRaisesRegex(RuntimeError,'초기화 확인 실패'):
   reset_empty_pad_scene(lambda:copy.deepcopy(scene),lambda diff:None,tmp)
if __name__=='__main__':unittest.main()
