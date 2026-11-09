import sys
from pathlib import Path
import pytest
R=Path(__file__).resolve().parents[3]/'automation/reconstruction'
sys.path.insert(0,str(R))
from pad_clearance_policy import judge_pad,MIN_SURFACE_PIXELS,MAX_ELEVATED_PIXELS

def checks(s1,e1,s2,e2):
    return {'cam1':{'surface_pixels':s1,'elevated_pixels':e1},'cam2':{'surface_pixels':s2,'elevated_pixels':e2}}

def test_pad_seen_and_empty_is_observed():
    assert judge_pad(checks(500,0,20,0),True)=='observed'
    assert judge_pad(checks(500,0,20,0),False)=='observed'

def test_object_on_pad_always_stops_even_with_start_check():
    with pytest.raises(RuntimeError,match='object seen on pad'):
        judge_pad(checks(500,MAX_ELEVATED_PIXELS,0,0),True)
    with pytest.raises(RuntimeError,match='object seen on pad'):
        judge_pad(checks(0,0,0,MAX_ELEVATED_PIXELS+10),True)

def test_hidden_pad_falls_back_to_start_of_batch_check():
    assert judge_pad(checks(MIN_SURFACE_PIXELS-1,0,3,0),True)=='start_of_batch_check'

def test_hidden_pad_without_start_check_still_stops_and_reports_counts():
    with pytest.raises(RuntimeError,match=r'floor 5 px'):
        judge_pad(checks(5,0,0,0),False)

def test_thresholds_unchanged_and_executor_uses_policy():
    assert MIN_SURFACE_PIXELS==100 and MAX_ELEVATED_PIXELS==40
    text=(R/'verified_place_left.py').read_text()
    assert 'judge_pad(checks,packing is not None)' in text
    assert "<100 or any(c['elevated_pixels']>=40" not in text
