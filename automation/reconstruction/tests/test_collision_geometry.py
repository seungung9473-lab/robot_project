import unittest,sys
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from collision_geometry import padded_hull_points

class CollisionGeometryTests(unittest.TestCase):
 def setUp(self):
  self.points=np.array([[x,y,z] for x in [-.02,.02] for y in [-.03,.03] for z in [0.,.05]])
 def test_margin_encloses_every_observation(self):
  expanded=padded_hull_points(self.points,.001);hull=ConvexHull(expanded)
  self.assertLessEqual(np.max(self.points@hull.equations[:,:3].T+hull.equations[:,3]),1e-10)
  np.testing.assert_allclose(expanded.min(0),self.points.min(0)-.001)
  np.testing.assert_allclose(expanded.max(0),self.points.max(0)+.001)
 def test_cannot_shrink_geometry(self):
  for margin in [-.001,float('nan'),float('inf'),.011]:
   with self.assertRaises(ValueError):padded_hull_points(self.points,margin)
 def test_nonfinite_measurements_rejected(self):
  points=self.points.copy();points[0,0]=float('nan')
  with self.assertRaises(ValueError):padded_hull_points(points,.001)

if __name__=='__main__':unittest.main()
