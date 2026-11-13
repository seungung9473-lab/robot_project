import sys
from datetime import date
from pathlib import Path
R=Path(__file__).resolve().parents[3]/'automation/reconstruction'
sys.path.insert(0,str(R))
from expiry_policy import is_expired,classify,rank_destinations,validate_route,DISCARD_ON_EXPIRY_DATE
import json

def obs(text,n=3):return [{'evidence_id':f'view_00{i}/cam1','raw_text':text,'date_text':text,'confidence':.95,'source':'printed_numeric'} for i in range(1,n+1)]

def test_expiry_day_itself_is_discarded():
    assert DISCARD_ON_EXPIRY_DATE
    today=date(2026,11,10)
    assert is_expired(date(2026,11,9),today) and is_expired(date(2026,11,10),today)
    assert not is_expired(date(2026,11,11),today)

def test_printed_date_equal_to_today_goes_to_trash_and_next_day_does_not():
    today=date(2026,11,10)
    same=classify(obs('11.10'),today);nxt=classify(obs('11.11'),today)
    assert same['status']=='expired' and same['destination']=='trash_drop' and same['remaining_days']==0
    assert nxt['status']=='not_expired'

def test_five_dates_on_the_rehearsal_day_no_longer_stop():
    # 연습일 11/10: 11/10(당일)이 폐기로 바뀌어 만료 안 된 날짜가 11/15, 11/25 두 종류뿐이다
    today=date(2026,11,10);D=['2026-11-01','2026-11-05','2026-11-10','2026-11-15','2026-11-25']
    dec=[{'object':str(i),'status':'expired' if is_expired(date.fromisoformat(d),today) else 'not_expired','expiry_date':d} for i,d in enumerate(D)]
    out=rank_destinations(dec,today);assert [o.get('pad_label') for o in out]==[None,None,None,'할인','정규']

def test_route_check_uses_the_same_rule_and_policy_file_matches():
    route={'expiry_decision':{'status':'expired','expiry_date':'2026-11-10','today':'2026-11-10'},'destination':'trash_drop'}
    validate_route(route,date(2026,11,10))          # 당일 폐기 판정이 실행 직전 검사와 어긋나지 않는다
    assert json.loads((R/'expiry_sort_policy.json').read_text())['valid_on_expiry_date'] is False
