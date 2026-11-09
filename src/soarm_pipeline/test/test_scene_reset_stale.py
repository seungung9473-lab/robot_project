import sys
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
R=Path(__file__).resolve().parents[3]/'automation/reconstruction'
sys.path.insert(0,str(R))
from scan_scene_reset import stale_attached

def att(oid,link='cup_tip'):return NS(object=NS(id=oid),link_name=link)

def test_nothing_attached_needs_no_sensor():
    assert stale_attached([],None)==[]

def test_stale_scan_block_is_removed_only_when_sensor_says_not_holding():
    assert stale_attached([att('scanned_block')],lambda:True)==[('scanned_block','cup_tip')]
    for answer in (False,None):
        with pytest.raises(RuntimeError,match='부착 물체'):stale_attached([att('scanned_block')],lambda a=answer:a)
    with pytest.raises(RuntimeError):stale_attached([att('scanned_block')],None)

def test_other_attached_objects_always_stop():
    with pytest.raises(RuntimeError):stale_attached([att('scanned_block'),att('something_else')],lambda:True)
    with pytest.raises(RuntimeError):stale_attached([att('tool_extra','tool_link')],lambda:True)

def test_reset_function_and_caller_use_the_check():
    text=(R/'scan_scene_reset.py').read_text()
    assert 'stale_attached(before.robot_state.attached_collision_objects, not_holding)' in text
    assert "not_holding=not_holding" in (R/'pad_occupancy.py').read_text()

def test_pad_occupancy_scene_query_retries_once_before_failing():
    text=(R/'pad_occupancy.py').read_text()
    i=text.index('def read_occupancy')
    block=text[i:i+2600]
    assert 'for attempt in range(2)' in block and '2회 시도' in block
