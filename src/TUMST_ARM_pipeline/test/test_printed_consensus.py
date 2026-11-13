import sys
from datetime import date
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'automation/reconstruction'))
from expiry_policy import printed_consensus
T=date(2026,11,9)
def o(i,d,c=.9):return {'source':'printed_numeric','evidence_id':f'v{i}','date_text':d,'confidence':c}
def test_single_same_status_outlier_ignored():
    assert printed_consensus([o(i,'12.30') for i in range(8)]+[o(20,'11.12')],T)==date(2026,12,30)
def test_outlier_changing_expired_status_blocks():
    assert printed_consensus([o(i,'12.30') for i in range(8)]+[o(20,'11.01')],T) is None
def test_two_outliers_or_weak_majority_block():
    assert printed_consensus([o(i,'12.30') for i in range(8)]+[o(20,'11.12'),o(21,'11.13')],T) is None
    assert printed_consensus([o(i,'12.30') for i in range(3)]+[o(20,'11.12')],T) is None
def test_same_evidence_conflict_blocks():
    assert printed_consensus([o(1,'11.15'),o(1,'11.16')]+[o(i,'11.15') for i in range(2,6)],T) is None
