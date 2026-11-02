from pathlib import Path
import unittest
import numpy as np
from soarm4_workcell.core import load,matrix,quaternion,rotation,translated,collision_geometry
P=Path(__file__).resolve().parents[1]/'config/workcell.yaml'
class WorkcellTests(unittest.TestCase):
    def setUp(self):self.d=load(P);self.T=matrix(self.d['T_table_world'])
    def test_reference_corner_matches_stl_bounds(self):
        # Model's +90deg orientation: world(+x,-y) lower enclosure corner maps table min/min.
        corner=np.array([-.1177,.0575,0.,1.]);actual=self.T@corner
        r=self.d['robot_registration'];np.testing.assert_allclose(actual[:3],[*(np.add(r['enclosure_corner_table_m'][:2],r['xy_contact_correction_m'])),r['global_z_lift_m']],atol=1e-7)
    def test_j1_offset_is_retained(self):
        p=np.array([-.0177,.0177,.1598,1.]);np.testing.assert_allclose((self.T@p)[:3],self.d['robot_registration']['j1_center_table_m'],atol=1e-7)
    def test_rotation_quaternion_roundtrip(self):
        for angle in [0,.2,np.pi/2,np.pi,3*np.pi/2]:
            R=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
            np.testing.assert_allclose(rotation(quaternion(R)),R,atol=1e-12)
    def test_scene_geometry_uses_inverse_world_transform(self):
        g={v['id']:v for v in collision_geometry(self.d)};self.assertEqual(len(g),5)
        for name,center,radius,height in [('workcell_turntable',[1.251,.514],.075,.059),('workcell_pad1',[1.246,.667],.05,.0575),('workcell_pad2',[1.252,.367],.05,.059)]:
            v=g[name];np.testing.assert_allclose(v['dimensions'],[height,radius],atol=1e-8);p=self.T@np.array(v['T_world_object']);np.testing.assert_allclose(p[:3,3],[*center,height/2],atol=1e-8)
        box=g['workcell_power_box'];np.testing.assert_allclose(box['dimensions'],[.25,.55,.3],atol=1e-9);np.testing.assert_allclose((self.T@np.array(box['T_world_object']))[:3,3],[1.775,.125,.15],atol=1e-9)
        table=g['workcell_table_guard'];top=(self.T@np.array(table['T_world_object']))[2,3]+table['dimensions'][2]/2;self.assertAlmostEqual(top,-.001)
    def test_camera_extrinsics_right_handed(self):
        for c in self.d['cameras'].values():
            T=matrix(c['T_table_optical']);np.testing.assert_allclose(T@np.linalg.inv(T),np.eye(4),atol=1e-12)
    def test_invalid_transform_rejected(self):
        T=np.eye(4);T[0,0]=-1
        with self.assertRaises(ValueError):matrix(T)
if __name__=='__main__':unittest.main()
