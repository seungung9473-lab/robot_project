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
    r=check_wait_pose(shifted(2.4),BASE)
    assert r['replan_first_move'] and abs(r['max_deg']-2.4)<1e-6
    assert r['per_joint_deg'][2]>2.3

def test_offset_over_limit_is_refused_and_names_joint():
    with pytest.raises(RuntimeError,match='joint 4'):
        check_wait_pose(shifted(5.5,joint=3),BASE)

def test_limit_and_cache_tolerance_values():
    assert LIMIT_DEG==5.0 and EXACT_DEG==.25

def test_executor_uses_check_and_logs_replan():
    text=(R/'execute_pick_batch.py').read_text()
    assert 'check_wait_pose(' in text and 'wait_pose_offset_replan' in text
    assert "radians(1):raise RuntimeError('Actual waiting pose" not in text

def test_three_and_a_half_degrees_on_the_wrist_is_replanned_not_refused():
    r=check_wait_pose(shifted(3.5,joint=3),BASE)   # 23:42 실행에서 실제로 나온 어긋남
    assert r['replan_first_move'] and abs(r['max_deg']-3.5)<1e-6
