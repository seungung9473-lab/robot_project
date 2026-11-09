from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]

def test_scanner_has_refresh_that_does_not_guard_and_still_fails_on_real_loss():
    t=(ROOT/'src/soarm_turntable/soarm_turntable/scan.py').read_text()
    body=t[t.index('def refresh_arm'):t.index('def await_future')]
    assert 'self.guard_arm()' not in body and "'스캔 중 로봇팔 피드백 유실'" in body and '_scan_executor.spin_once' in body

def test_wait_pose_review_refreshes_before_spinning():
    t=(ROOT/'automation/reconstruction/turntable_wait.py').read_text()
    assert t.index('node.refresh_arm()')<t.index('node.spin(.05)')
