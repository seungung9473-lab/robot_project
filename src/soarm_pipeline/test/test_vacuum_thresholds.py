import re,sys
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[3]
R=ROOT/'automation/reconstruction'
sys.path.insert(0,str(R))
from vacuum_thresholds import HELD_DIFF,RELEASE_CONFIRM_DIFF

def test_threshold_values_and_ordering():
    assert HELD_DIFF==-100 and RELEASE_CONFIRM_DIFF==-50
    assert HELD_DIFF<RELEASE_CONFIRM_DIFF   # 붙음 기준이 해제 확인 기준보다 더 낮아야 한다(사이에 여유)

def test_bridge_params_use_same_threshold():
    p=yaml.safe_load((ROOT/'src/soarm_bridge/config/soarm.yaml').read_text())
    params=next(v for k,v in p.items() if 'esp32' in k)['ros__parameters']
    assert params['hold_diff']==HELD_DIFF and params['grip_diff']==HELD_DIFF and params['release_diff']==RELEASE_CONFIRM_DIFF

def test_no_stale_literal_threshold_left_in_scripts():
    pattern=re.compile(r'(<=|>)\s*-200\b|diff.{0,12}-200\b')
    bad=[f.name for f in R.glob('*.py') if pattern.search(f.read_text())]
    assert not bad,bad

def test_scripts_import_the_shared_constant():
    for name in ('verified_place_left.py','verified_suction_trial.py','execute_two_object_sort.py','execute_pick_batch.py','turntable_wait.py','batch_completion.py'):
        text=(R/name).read_text()
        assert 'from vacuum_thresholds import HELD_DIFF' in text,name

def test_taught_trash_pose_calibration_hashes_are_current():
    """soarm.yaml 등 보정 파일을 바꾸면 폐기 위치 가드가 시연을 멈춘다 — 바꿀 때마다 이 테스트가 먼저 알려 준다."""
    from trash_destination import load_trash_pose
    load_trash_pose()
