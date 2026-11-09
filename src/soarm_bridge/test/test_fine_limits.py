import math,tempfile,unittest
from pathlib import Path
from soarm_bridge.fine_limits import correction_limits
from soarm_bridge.servo_bridge import fine_goals

class CorrectionLimitTests(unittest.TestCase):
    def test_joint_specific_caps_preserve_other_joints(self):
        self.assertEqual(fine_goals([1000,1000],[1010,1010],[996,996],[0,0],[4095,4095],1,[8,4]),[992,996])
    def test_urdf_can_be_tighter_than_motor_bounds(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'r.urdf';p.write_text('<robot><joint name="j"><limit lower="-1.0" upper="1.0"/></joint></robot>')
            lo,hi=correction_limits(p,['j'],[2000],[-1],[.043],[0],[4095])
            for c in range(lo[0],hi[0]+1):
                q=-(c-2000)*2*math.pi/4096;q=q/(1+.043) if q>0 else q
                self.assertGreaterEqual(q,-1.);self.assertLessEqual(q,1.)
            self.assertGreater(lo[0],0);self.assertLess(hi[0],4095)
    def test_motor_bound_is_never_extended(self):
        p=Path(__file__).resolve().parents[2]/'soarm4_description/urdf/soarm4.urdf'
        lo,hi=correction_limits(p,['joint1','joint2','joint3','joint4'],[2078,4095,0,1990],[-1]*4,[.043,0,0,0],[974,1934,0,774],[3357,4095,2047,3159])
        self.assertEqual(hi[2],2047);self.assertLess(hi[3],3159)
        for a,b in zip(lo,[974,1934,0,774]):self.assertGreaterEqual(a,b)
        for a,b in zip(hi,[3357,4095,2047,3159]):self.assertLessEqual(a,b)
    def test_missing_cap_is_rejected(self):
        with self.assertRaises(ValueError):fine_goals([1000,1000],[990,990],[1000,1000],[0,0],[4095,4095],1,[8])
