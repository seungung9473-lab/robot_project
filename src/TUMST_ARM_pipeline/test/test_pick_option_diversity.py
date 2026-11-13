import sys
from pathlib import Path
R=Path(__file__).resolve().parents[3]/'automation/reconstruction'
sys.path.insert(0,str(R))
from tidy_pad_placement import alternative_candidates,MAX_PER_PICKUP_OPTION

def c(i,o):return {'id':i,'pickup_option_index':o}

def test_other_pickup_options_come_before_more_slots_of_the_same_option():
    cands=[c(0,0),c(1,0),c(2,0),c(3,1),c(4,1),c(5,2)]
    alt=alternative_candidates(cands,cands[0])
    assert [a['pickup_option_index'] for a in alt][:2]==[1,2]
    assert cands[0] not in alt and len(alt)==5

def test_single_option_still_returns_its_other_slots():
    cands=[c(i,0) for i in range(6)]
    assert [a['id'] for a in alternative_candidates(cands,cands[0])]==[1,2,3,4,5]

def test_per_option_cap_leaves_room_for_other_options():
    assert 1<=MAX_PER_PICKUP_OPTION<32//2

def test_batch_planner_skips_same_pick_after_transit_ik_failure():
    t=(R/'plan_pick_batch.py').read_text()
    assert "Transit pose above target IK unavailable" in t and 'pick_failures' in t
    assert t.index('pick_token in pick_failures')<t.index("print(f'계산중: {key} 후보")
