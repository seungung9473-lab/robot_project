import sys
from pathlib import Path
R=Path(__file__).resolve().parents[3]/'automation/reconstruction'
sys.path.insert(0,str(R))
from seal_stability import evaluate,SEAL_STABLE_S,MIN_SAMPLES
from vacuum_thresholds import HELD_DIFF

def window(values):return [(i*.1,v) for i,v in enumerate(values)]

def test_deep_and_steady_seal_is_stable():
    ok,d=evaluate(window([-300,-310,-305,-298,-300,-302,-301,-299,-300,-300]),HELD_DIFF);assert ok and d['worst']==-298

def test_seal_that_only_just_crosses_or_dips_back_is_not_stable():
    assert not evaluate(window([-156,-155,-152,-120,-90,-60,-40,-30,-29,-29]),HELD_DIFF)[0]       # 22:32 실행처럼 풀리는 경우
    assert not evaluate(window([-300]*9+[-100]),HELD_DIFF)[0]                                  # 한 번만 올라와도 불합격
    assert evaluate(window([HELD_DIFF]*10),HELD_DIFF)[0]                                        # 정확히 기준값은 합격

def test_too_few_samples_is_not_stable():
    ok,d=evaluate(window([-400]*(MIN_SAMPLES-1)),HELD_DIFF);assert not ok and 'too few' in d['reason']

def test_check_is_wired_into_all_three_grip_paths():
    text=(R/'verified_place_left.py').read_text()
    assert text.count('seal_stable()')>=4   # 정의 1 + 첫 흡착·기다린 뒤·재접근 후 3
    assert "status['seal_check_first']" in text and "waited['seal_check']" in text and "record['seal_check']" in text
    assert SEAL_STABLE_S==1.0
