import unittest
from unittest.mock import patch
from j3_integral_tune import program_integral
class FakeBus:
    def __init__(self,torque=0,fail=False):
        self.mem={21:32,22:32,23:0,24:0,25:0,26:0,27:0,40:torque,55:1};self.writes=[];self.fail=fail
    def read(self,sid,addr,n):return bytes(self.mem[addr+i] for i in range(n))
    def write(self,sid,addr,data):
        self.writes.append((sid,addr,data[0]))
        if self.fail and addr==23 and data[0]==1:self.fail=False;return
        self.mem[addr]=data[0]
class J3IntegralTests(unittest.TestCase):
    @patch('j3_integral_tune.time.sleep')
    def test_only_i_and_integral_limit_and_lock_written(self,_):
        b=FakeBus();self.assertEqual(program_integral(b,[32,32,0,0,0,0,0]),[32,32,1,0,4,0,0]);self.assertEqual(b.mem[55],1)
        self.assertTrue(all(sid==3 and addr in [23,25,55] for sid,addr,_ in b.writes))
    def test_torque_on_refused_before_any_write(self):
        b=FakeBus(torque=1)
        with self.assertRaises(RuntimeError):program_integral(b,[32,32,0,0,0,0,0])
        self.assertFalse(b.writes)
    @patch('j3_integral_tune.time.sleep')
    def test_failed_write_rolls_back_and_relocks(self,_):
        b=FakeBus(fail=True)
        with self.assertRaises(RuntimeError):program_integral(b,[32,32,0,0,0,0,0])
        self.assertEqual([b.mem[i] for i in range(21,28)],[32,32,0,0,0,0,0]);self.assertEqual(b.mem[55],1)

    @patch('j3_integral_tune.time.sleep')
    def test_restore_disables_i_before_removing_limit(self,_):
        b=FakeBus();b.mem[23]=1;b.mem[25]=4
        self.assertEqual(program_integral(b,[32,32,1,0,4,0,0],0,0),[32,32,0,0,0,0,0])
        writes=[(a,v) for _,a,v in b.writes if a in [23,25]]
        self.assertEqual(writes,[(23,0),(25,0)]);self.assertEqual(b.mem[55],1)
