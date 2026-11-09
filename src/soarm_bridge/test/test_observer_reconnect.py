import ast
from pathlib import Path
import termios
import types
import unittest
from unittest.mock import Mock,patch
source=Path(__file__).resolve().parents[1]/'soarm_bridge/servo_bridge.py'
module=ast.parse(source.read_text());method=next(n for n in ast.walk(module) if isinstance(n,ast.FunctionDef) and n.name=='reconnect_observer')
class ObserverRecoveryTests(unittest.TestCase):
 def make(self,readonly):
  env={'time':types.SimpleNamespace(monotonic=lambda:10.),'termios':termios,'STSBus':Mock()}
  exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'observer.py','exec'),env)
  node=Mock(read_only=readonly,next_observer_reconnect=0.,port_name='/dev/servo_bus',baud_rate=1000000,n=4)
  return node,env
 def test_control_mode_never_reconnects(self):
  node,env=self.make(False);self.assertFalse(env['reconnect_observer'](node));node.bus.close.assert_not_called();env['STSBus'].assert_not_called()
 def test_observer_reconnects_reads_only_without_torque_or_goal(self):
  node,env=self.make(True);self.assertTrue(env['reconnect_observer'](node));env['STSBus'].assert_called_once_with('/dev/servo_bus',1000000)
  self.assertEqual(node.pos,[None]*4);self.assertEqual(node.last_feedback_time,0.);self.assertEqual(env['STSBus'].return_value.mock_calls,[])
 def test_retries_throttled_during_usb_absence(self):
  node,env=self.make(True);env['STSBus'].side_effect=OSError('unplugged');self.assertFalse(env['reconnect_observer'](node));self.assertFalse(env['reconnect_observer'](node));self.assertEqual(env['STSBus'].call_count,1)
 def test_termios_close_error_does_not_kill_observer(self):
  node,env=self.make(True);node.bus.close.side_effect=termios.error(5,'I/O error');self.assertTrue(env['reconnect_observer'](node))
if __name__=='__main__':unittest.main()
