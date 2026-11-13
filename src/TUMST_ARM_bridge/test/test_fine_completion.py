import threading
import unittest
from types import SimpleNamespace
from soarm_bridge.servo_bridge import ServoBridge

class FineCompletionTests(unittest.TestCase):
    def bridge(self,pos,update=None,stable=True):
        reports=[];writes=[]
        b=SimpleNamespace(pos=[pos],cmin=[0],cmax=[4095],mlo=[0],mhi=[0],ids=[1],settle_speed=30,acc=50,fine_tol=1,fine_max=4,fine_max_joint=[4],fine_lower=[0],fine_upper=[4095],stop_event=threading.Event())
        b.get_parameter=lambda _:SimpleNamespace(value=5)
        b.wait_still=lambda _:stable
        b.feedback_fresh=lambda:True
        b.hold_here=lambda:None
        b.report_fine=lambda *args:reports.append(args)
        def write(goals,acc):
            writes.append(goals)
            if update:b.pos[0]=update(b.pos[0])
        b.bus=SimpleNamespace(sync_goal=write)
        return b,reports,writes
    def goal(self):return SimpleNamespace(is_cancel_requested=False,abort=lambda:None,canceled=lambda:None)
    def test_saturation_stops_without_repeating_same_command(self):
        b,reports,writes=self.bridge(990)
        self.assertTrue(ServoBridge.fine_correct(b,[0],[1000],self.goal()))
        self.assertEqual(len(writes),1);self.assertEqual(writes[0][1][0],1004)
        self.assertEqual(reports[-1][3],'command_bound_reached')
    def test_reaching_tolerance_is_reported(self):
        b,reports,writes=self.bridge(998,lambda _:1000)
        self.assertTrue(ServoBridge.fine_correct(b,[0],[1000],self.goal()))
        self.assertEqual(reports[-1][3],'within_tolerance')
    def test_iteration_cap_is_not_unbounded_hunting(self):
        b,reports,writes=self.bridge(1002,lambda p:998 if p==1002 else 1002)
        self.assertTrue(ServoBridge.fine_correct(b,[0],[1000],self.goal()))
        self.assertEqual(len(writes),5);self.assertEqual(reports[-1][3],'iteration_limit')
        self.assertTrue(all(abs(w[1][0]-1000)<=4 for w in writes))
    def test_unsettled_motor_gets_no_new_command(self):
        b,reports,writes=self.bridge(990,stable=False)
        self.assertTrue(ServoBridge.fine_correct(b,[0],[1000],self.goal()))
        self.assertFalse(writes);self.assertEqual(reports[-1][3],'not_settled')
    def test_cancel_is_not_reported_as_success(self):
        b,reports,writes=self.bridge(990,stable=False);g=self.goal();g.is_cancel_requested=True
        self.assertFalse(ServoBridge.fine_correct(b,[0],[1000],g));self.assertFalse(writes)

    def test_missing_encoder_aborts_without_command(self):
        b,reports,writes=self.bridge(None)
        self.assertFalse(ServoBridge.fine_correct(b,[0],[1000],self.goal()))
        self.assertFalse(writes)
