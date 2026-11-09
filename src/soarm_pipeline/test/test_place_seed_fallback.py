from pathlib import Path
R=Path(__file__).resolve().parents[3]/'automation/reconstruction'

def test_reservation_pose_is_used_when_default_seed_ik_fails():
    t=(R/'verified_place_left.py').read_text()
    a=t.index("place_seed=solve(arm,wc,pad_target");b=t.index("raise RuntimeError('Pad seed IK unreachable')")
    mid=t[a:b]
    assert "placement_q_rad" in mid and "place_seed_from_reservation" in mid and "place_seed is None" in mid

def test_reservations_store_the_planned_placement_pose():
    assert "'placement_q_rad':q.tolist()" in (R/'tidy_pad_placement.py').read_text()
