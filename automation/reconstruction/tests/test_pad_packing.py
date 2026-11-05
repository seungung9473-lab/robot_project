import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pad_packing import reserve,separated,validate_actual

def rectangle(x,y):return np.array([[a,b,0] for a in [-x/2,x/2] for b in [-y/2,y/2]])
class PackingTests(unittest.TestCase):
 def test_joint_reservation_leaves_second_space(self):
  p=reserve({'green1':rectangle(.03,.04),'green2':rectangle(.03,.04)},[1.,2.,.059],.05)
  a=np.array(p['placements']['green1']['reserved_polygon_table_m']);b=np.array(p['placements']['green2']['reserved_polygon_table_m'])
  self.assertTrue(separated(a,b,.002))
  for k in p['placements']:validate_actual(p,k,np.c_[p['placements'][k]['reserved_polygon_table_m'],np.zeros(4)])
 def test_oversize_not_shrunk(self):
  with self.assertRaises(ValueError):reserve({'a':rectangle(.11,.03)},[0,0,0],.05)
 def test_actual_rotation_or_overlap_rejected(self):
  p=reserve({'a':rectangle(.03,.04),'b':rectangle(.03,.04)},[0,0,0],.05)
  with self.assertRaises(ValueError):validate_actual(p,'a',np.c_[p['placements']['b']['reserved_polygon_table_m'],np.zeros(4)])
if __name__=='__main__':unittest.main()
