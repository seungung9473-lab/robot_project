from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock,patch
import pytest
from soarm_turntable.scan import Scanner

def fake(arrive=2):
 c=[1.];n=SimpleNamespace(states=deque(),state=Mock());calls=[0]
 def spin():
  c[0]+=.1;calls[0]+=1
  if arrive is not None and calls[0]>=arrive:n.states.append((c[0],{}))
 n.spin=Mock(side_effect=spin);return n,c

def test_wait_for_fresh_initial_state():
 n,c=fake()
 with patch('soarm_turntable.scan.time.monotonic',side_effect=lambda:c[0]),patch('soarm_turntable.scan.stable_states',return_value=True):Scanner.wait_stable(n,1.)
 assert n.spin.call_count==2;n.state.assert_called_once()

def test_stale_state_cannot_pass():
 n,c=fake(None);n.states.append((0.,{}))
 with patch('soarm_turntable.scan.time.monotonic',side_effect=lambda:c[0]),patch('soarm_turntable.scan.stable_states',return_value=True) as stable:
  with pytest.raises(RuntimeError,match='새 상태'):Scanner.wait_stable(n,.3)
 n.state.assert_not_called();stable.assert_not_called()

def test_hardware_error_still_aborts():
 n,c=fake(1);n.state.side_effect=RuntimeError('hardware error')
 with patch('soarm_turntable.scan.time.monotonic',side_effect=lambda:c[0]):
  with pytest.raises(RuntimeError,match='hardware error'):Scanner.wait_stable(n)

def test_arm_guard_error_still_aborts():
 n,c=fake();n.spin.side_effect=RuntimeError('arm moved')
 with patch('soarm_turntable.scan.time.monotonic',side_effect=lambda:c[0]):
  with pytest.raises(RuntimeError,match='arm moved'):Scanner.wait_stable(n)

def test_persistent_executor_and_arm_guard():
 n=SimpleNamespace(_scan_executor=Mock(),guard_arm=Mock())
 with patch('soarm_turntable.scan.rclpy.spin_once',side_effect=AssertionError()):Scanner.spin(n,.01)
 n._scan_executor.spin_once.assert_called_once_with(timeout_sec=.01);n.guard_arm.assert_called_once()
