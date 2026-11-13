import math,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'automation/reconstruction'))
from transit_clearance import travel_cost,min_tcp_z,TRANSIT_UP_M,TRANSIT_MIN_TCP_Z_M
R=Path(__file__).resolve().parents[3]/'automation/reconstruction'
HOME=[0,math.pi,-math.pi,0]
def test_less_folding_costs_less():
    near=[0.1,math.pi-.1,-math.pi+.2,.3];far=[0.1,math.pi-.1,-2.8+.0,1.2]
    assert travel_cost(HOME,near)<travel_cost(HOME,far)
def test_cost_wraps_angles():
    assert travel_cost([0,0,0,0],[2*math.pi-.01,0,0,0])<.05
def test_min_tcp_z_uses_lowest_sample():
    import numpy as np
    fk=lambda q:np.array([[0,0,0,0],[0,0,0,0],[0,0,0,q[0]],[0,0,0,1]],float)
    assert min_tcp_z(fk,[[.2],[.12],[.3]])==.12
def test_transit_constants_clear_objects():
    assert TRANSIT_UP_M>=.08 and TRANSIT_MIN_TCP_Z_M>=.059+.040
def test_planning_and_execution_wired():
    p=(R/'check_inclined_motion.py').read_text();e=(R/'verified_place_left.py').read_text();b=(R/'plan_pick_batch.py').read_text()
    assert "transit_q_rad" in p and "Transit path rejected" in p and "'transit_above_target'" in p
    assert "move(qh,'transit_above_target',min_tcp_z=TRANSIT_MIN_TCP_Z_M)" in e and "move(qa.tolist(),'approach_3cm',True)" in e
    assert "travel_cost(_start0" in b
