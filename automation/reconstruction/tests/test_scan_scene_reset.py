import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import pytest
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from scan_scene_reset import reset_previous_scan_scene, scan_object_ids

def world(*ids):
    s=PlanningScene()
    for key in ids:
        o=CollisionObject();o.id=key;s.world.collision_objects.append(o)
    return s

def test_removes_only_prior_scan_and_preserves_workcell_and_unknown_obstacle(tmp_path):
    s=world('workcell_turntable','workcell_pad1','workcell_pad2','workcell_table_guard','workcell_power_box','foreign_obstacle','scanned_block','inclined_obstacle_object_1','batch_virtual_object_2','placed_pad_5_old','placed_pad_6_old')
    cleared=[]
    def apply(diff):
        ids={o.id for o in diff.world.collision_objects}
        assert all(o.operation==CollisionObject.REMOVE for o in diff.world.collision_objects)
        s.world.collision_objects=[o for o in s.world.collision_objects if o.id not in ids]
    out=reset_previous_scan_scene(lambda:s,apply,tmp_path,stop_display=lambda:[123],clear_markers=lambda:cleared.append(True))
    assert out['success'] and cleared==[True]
    assert set(out['retained_ids'])=={'workcell_turntable','workcell_pad1','workcell_pad2','workcell_table_guard','workcell_power_box','foreign_obstacle'}
    assert len(out['removed_ids'])==5

def test_attached_object_prevents_any_reset(tmp_path):
    s=world('scanned_block');s.robot_state.attached_collision_objects=[AttachedCollisionObject()]
    def forbidden(*args):raise AssertionError('No reset operation allowed')
    with pytest.raises(RuntimeError,match='부착'):
        reset_previous_scan_scene(lambda:s,forbidden,tmp_path,stop_display=forbidden,clear_markers=forbidden)

def test_failed_removal_blocks_new_cycle(tmp_path):
    s=world('scanned_block')
    with pytest.raises(RuntimeError,match='확인 실패'):
        reset_previous_scan_scene(lambda:s,lambda _:None,tmp_path,stop_display=lambda:[],clear_markers=lambda:None)
