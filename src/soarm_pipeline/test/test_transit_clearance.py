import re,sys
from pathlib import Path
R=Path(__file__).resolve().parents[3]/'automation/reconstruction'
sys.path.insert(0,str(R))
from transit_clearance import RETREAT_UP_M
def test_retreat_clears_object_tops():
    # turntable top z=59 mm, objects up to ~99 mm; wait tip must be well above them
    assert RETREAT_UP_M>=0.08
def test_execution_and_planning_use_same_retreat():
    for name in ('verified_place_left.py','check_inclined_motion.py'):
        text=(R/name).read_text()
        assert 'RETREAT_UP_M' in text
        for line in text.splitlines():
            if 'empty_retreat_from_pad_ID' in line and 'ik(' in line or 'solve_axis(' in line and 'placed' in line and '[0,0,' in line:
                assert '[0,0,.03]' not in line,(name,line[:120])

def test_trash_route_returns_to_stage_start_in_execution_and_planning():
    ex=(R/'verified_place_left.py').read_text();pl=(R/'check_inclined_motion.py').read_text()
    assert "return_to_stage_start_after_trash" in ex and "ptp=True" in ex and "stage_start_q=[float(x) for x in fresh()]" in ex
    assert "result['exit_q_rad']=list(map(float,start))" in pl
    assert "placed.tolist() if a.finish_above else list(map(float,start))" not in pl  # old trash exit (stay at drop pose) is gone

def test_neighbour_padding_is_wider_but_below_pad_gap():
    from transit_clearance import OBSTACLE_PADDING_M
    # 이웃 물체 여유는 기존 1 mm보다 크고, 패드 위 실제 물체 간격(2+2.5+2.5=7 mm)보다 작아야 한다
    assert .001<OBSTACLE_PADDING_M<=.005
    for name in ('verified_place_left.py','check_inclined_motion.py'):
        text=(R/name).read_text()
        assert "other.get('padding_m',OBSTACLE_PADDING_M)" in text
        assert "other.get('padding_m',.001)" not in text
