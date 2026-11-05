import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from inclined_geometry import patch,alignment_angles,rotate_surface,compensated_axis,solve_axis
from continuous_pre_suction import PreSuctionTrigger
class InclinedGeometryTests(unittest.TestCase):
 def surface(self,hole=False):
  u=np.array([1,0,-1])/np.sqrt(2);v=np.array([0,1,0]);n=np.cross(u,v);n=n/np.linalg.norm(n)
  x,y=np.meshgrid(np.arange(-.03,.0301,.001),np.arange(-.03,.0301,.001));xy=np.c_[x.ravel(),y.ravel()]
  if hole:xy=xy[np.linalg.norm(xy,axis=1)>.006]
  center=np.array([1.25,.5,.12]);P=center+xy[:,0,None]*u+xy[:,1,None]*v
  return P,n,center
 def test_inclined_patch_and_hole(self):
  P,n,c=self.surface();d=-c@n;r=patch(P,n,d,center_point=c)
  self.assertTrue(r['geometry_pass']);self.assertAlmostEqual(r['tilt_deg'],45)
  P,n,c=self.surface(True);r=patch(P,n,-c@n,center_point=c)
  self.assertFalse(r['geometry_pass'])
 def test_small_surface_variation_within_2_5mm(self):
  x,y=np.meshgrid(np.arange(-.03,.0301,.001),np.arange(-.03,.0301,.001))
  z=np.where(np.indices(x.shape).sum(axis=0)%2==0, .0023,- .0023)
  P=np.c_[x.ravel(),y.ravel(),z.ravel()]+[1.25,.5,.12]
  result=patch(P,[0,0,1],-.12,center_point=[1.25,.5,.12])
  self.assertTrue(result['geometry_pass']);self.assertAlmostEqual(result['patch_rms_mm'],2.3)
  P[:,2]=.12+np.where(z.ravel()>0,.003,-.003)
  self.assertIsNone(patch(P,[0,0,1],-.12,center_point=[1.25,.5,.12]))
 def test_edge_margin_stays_3mm(self):
  x,y=np.meshgrid(np.arange(-.014,.0141,.001),np.arange(-.02,.0201,.001))
  P=np.c_[x.ravel(),y.ravel(),np.full(x.size,.12)]
  result=patch(P,[0,0,1],-.12,center_point=[0,0,.12])
  self.assertFalse(result['geometry_pass']);self.assertAlmostEqual(result['edge_margin_mm'],3.)
 def test_narrow_valid_face_is_not_lost_to_grid_rounding(self):
  x,y=np.meshgrid(np.linspace(-.0156,.0156,33),np.linspace(-.02,.02,41))
  P=np.c_[x.ravel(),y.ravel(),np.full(x.size,.12)]
  result=patch(P,[0,0,1],-.12)
  self.assertTrue(result['geometry_pass']);self.assertGreaterEqual(result['boundary_clearance_mm'],15.5)
 def test_vertical_faces_including_y_normal(self):
  x,y=np.meshgrid(np.arange(-.03,.0301,.001),np.arange(-.03,.0301,.001))
  c=np.array([1.25,.5,.12])
  for n in ([1,0,0],[-1,0,0],[0,1,0],[0,-1,0]):
   n=np.array(n,float);u=np.cross(np.eye(3)[np.argmin(np.abs(n))],n);v=np.cross(n,u)
   P=c+x.ravel()[:,None]*u+y.ravel()[:,None]*v
   r=patch(P,n,-c@n,center_point=c)
   self.assertTrue(r['geometry_pass']);self.assertAlmostEqual(r['tilt_deg'],90)
   self.assertIsNone(patch(P,n,-c@n,max_tilt=65,center_point=c))
 def test_over_90_degrees_rejected(self):
  P,n,c=self.surface()
  self.assertIsNone(patch(P,-n,c@n,center_point=c))
 def test_rotation_uses_axis_and_rotates_normal(self):
  p,n=rotate_surface([1.3,.5,.12],[1,0,0],[1.2,.5,.06],90)
  np.testing.assert_allclose(p,[1.2,.6,.12]);np.testing.assert_allclose(n,[0,1,0],atol=1e-12)
 def test_alignment_radial(self):
  p=[1.25,.51,.12];n=np.array([.7,.1,.7]);n/=np.linalg.norm(n);axis=[1.251,.514,.059];j1=[1.4678,.522,.1608]
  roots=alignment_angles(p,n,axis,j1);self.assertGreaterEqual(len(roots),2)
  for angle in roots:
   point,normal=rotate_surface(p,n,axis,angle);r=point[:2]-np.array(j1[:2]);self.assertLess(abs(np.cross(normal[:2],r)),1e-7)
 def test_pump_no_pressure_gate(self):
  commands=[]
  def send():commands.append('on');return object()
  gate=PreSuctionTrigger([0,0,0],[0,0,1],send)
  for z in [.03,.002,.0016]:gate.update([0,0,z])
  self.assertFalse(commands)
  for z in [.0015,.001,0]:gate.update([0,0,z])
  self.assertEqual(commands,['on'])
 def test_normal_trajectory_is_continuous_and_bounded(self):
  from normal_contact_trajectory import contact_samples
  from motion.kinematics import fk,m
  from inclined_geometry import solve_axis
  lo=np.array([j['limit']['lower'] for j in m['joints']],float);hi=np.array([j['limit']['upper'] for j in m['joints']],float)
  q=np.array([.030248949,1.06026478,-2.76197526,.99846911]);T=fk(q);axis=T[:3,0];target=T[:3,3]-.03*axis;above=solve_axis(fk,lo,hi,target,axis,seed=q)
  self.assertIsNotNone(above)
  ts,qs,vs,acc=contact_samples(fk,lo,hi,above[0],q,[.1]*4,[.15]*4)
  self.assertTrue(np.all(np.diff(ts)>0));self.assertLessEqual(np.abs(vs).max(),.1);self.assertLessEqual(np.abs(acc).max(),.15)
  np.testing.assert_allclose(vs[[0,-1]],0)
  for qi in qs:
   M=fk(qi);line=T[:3,3]-target;along=(M[:3,3]-target)@line/(line@line);lateral=M[:3,3]-target-along*line
   self.assertLess(np.linalg.norm(lateral),.0003)
 def test_actual_arm_ik_and_unreachable(self):
  from motion.kinematics import fk,m
  lo=np.array([j['limit']['lower'] for j in m['joints']],float);hi=np.array([j['limit']['upper'] for j in m['joints']],float)
  q=lo*.4+hi*.6;T=fk(q);sol=solve_axis(fk,lo,hi,T[:3,3],T[:3,0],seed=q,attempts=2)
  self.assertIsNotNone(sol);self.assertIsNone(solve_axis(fk,lo,hi,[5,5,5],[0,0,-1],seed=q,attempts=2))
if __name__=='__main__':unittest.main()
