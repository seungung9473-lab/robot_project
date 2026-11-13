import math,sys
from pathlib import Path
import pytest
R=Path(__file__).resolve().parents[3]/'automation/reconstruction'
sys.path.insert(0,str(R))
from wait_pose import check_wait_pose,LIMIT_DEG,EXACT_DEG

BASE=[-0.546,0.990,-1.059,-1.378]

def shifted(deg,joint=2):
    q=list(BASE);q[joint]+=math.radians(deg);return q

def test_exact_start_reuses_reviewed_trajectory():
    r=check_wait_pose(BASE,BASE)
    assert r['max_deg']==0 and not r['replan_first_move']

def test_small_offset_within_cache_tolerance_does_not_replan():
    assert not check_wait_pose(shifted(.2),BASE)['replan_first_move']

def test_offset_between_exact_and_limit_replans_first_move():
    r=check_wait_pose(shifted(4.2),BASE)
    assert r['replan_first_move'] and abs(r['max_deg']-4.2)<1e-6
    assert r['per_joint_deg'][2]>4.1

def test_reported_wrist_offset_requires_replanning():
    assert check_wait_pose(shifted(2.4),BASE)['replan_first_move']
    assert check_wait_pose(shifted(3.16,joint=3),BASE)['replan_first_move']

def test_offset_over_limit_is_refused_and_names_joint():
    with pytest.raises(RuntimeError,match='joint 4'):
        check_wait_pose(shifted(5.5,joint=3),BASE)

def test_limit_and_cache_tolerance_values():
    assert LIMIT_DEG==5.0 and EXACT_DEG==1.0

def test_executor_uses_check_and_logs_replan():
    text=(R/'execute_pick_batch.py').read_text()
    assert 'check_wait_pose(' in text and 'wait_pose_offset_replan' in text
    assert "radians(1):raise RuntimeError('Actual waiting pose" not in text

def test_three_and_a_half_degrees_on_the_wrist_is_not_refused():
    r=check_wait_pose(shifted(3.5,joint=3),BASE)   # 23:42 실행에서 실제로 나온 어긋남
    assert abs(r['max_deg']-3.5)<1e-6

def test_cached_trajectory_checks_the_actual_first_waypoint():
    from trajectory_cache import compatible

    names = ['joint1', 'joint2', 'joint3', 'joint4']
    scaling = {'velocity': .8, 'acceleration': .4}
    path = {'joint_names': names, 'points': [{'positions': BASE}]}
    entry = {'joint_names': names, 'start': BASE, 'goal': BASE,
             'scaling': scaling, 'trajectory': {'joint_trajectory': path}}
    assert compatible(entry, names, shifted(.2), BASE, scaling)
    assert not compatible(entry, names, shifted(3.16, joint=3), BASE, scaling)
    path['points'][0]['positions'] = shifted(3.16, joint=3)
    assert not compatible(entry, names, BASE, BASE, scaling)



def test_generic_cached_retreat_is_not_reused_as_fixed_axis_motion():
    from trajectory_cache import compatible

    names = ['joint1', 'joint2', 'joint3', 'joint4']
    scaling = {'velocity': .8, 'acceleration': .4}
    entry = {'joint_names': names, 'start': BASE, 'goal': BASE, 'scaling': scaling,
             'trajectory': {'joint_trajectory': {'joint_names': names,
                                                  'points': [{'positions': BASE}]}}}
    assert compatible(entry, names, BASE, BASE, scaling)
    assert not compatible(entry, names, BASE, BASE, scaling, fixed_axis=True)
    entry['profile'] = 'fixed_axis'
    assert compatible(entry, names, BASE, BASE, scaling, fixed_axis=True)
