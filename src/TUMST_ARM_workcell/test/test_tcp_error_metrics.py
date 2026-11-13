import unittest
import numpy as np
from soarm4_workcell.tcp_error_metrics import tcp_errors
class TcpErrorTests(unittest.TestCase):
    def test_zero_joint_error_keeps_ik_residual_separate(self):
        m=tcp_errors(lambda q:np.array([1,2,3.]),np.zeros(4),np.zeros(4),[1,2,2.999])
        self.assertAlmostEqual(m['model_tcp_error_norm_mm'],1)
        self.assertEqual(m['joint_tracking_tcp_error_norm_mm'],0)
    def test_linear_arm_exactly_decomposes(self):
        A=np.array([[1,2,0,0],[0,0,3,0],[0,0,0,4.]])
        goal=np.zeros(4);actual=np.array([.001,.002,.003,.004]);m=tcp_errors(lambda q:A@q,goal,actual,[0,0,0])
        np.testing.assert_allclose(m['model_tcp_error_mm'],[5,9,16])
        np.testing.assert_allclose(np.sum(m['per_joint_linear_tcp_error_mm'],axis=0),m['model_tcp_error_mm'],atol=1e-8)
    def test_nonlinear_remainder_is_reported(self):
        fk=lambda q:np.array([np.cos(q[0]),np.sin(q[0]),0.])
        m=tcp_errors(fk,np.zeros(4),[.1,0,0,0],[1,0,0])
        self.assertGreater(np.linalg.norm(m['linearization_remainder_mm']),4)
        np.testing.assert_allclose(np.array(m['joint_tracking_tcp_error_mm'])+m['ik_position_residual_mm'],m['model_tcp_error_mm'])
    def test_nan_input_rejected(self):
        with self.assertRaises(ValueError):tcp_errors(lambda q:q[:3],[0,0,0,0],[float('nan'),0,0,0],[0,0,0])
