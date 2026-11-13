from pathlib import Path
import math
import unittest
import numpy as np
from soarm4_workcell.core import load, matrix
from soarm4_workcell.cup_goal import Arm, solve, tcp_point, named_targets, J1_WORLD

P = Path(__file__).resolve().parents[1] / 'config/workcell.yaml'


class CupGoalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.wc = load(P); cls.arm = Arm(); cls.T = matrix(cls.wc['T_table_world']); cls.tcp = tcp_point(cls.wc)

    def tip_and_axis(self, q):
        M = self.T @ self.arm.fk(q); return (M @ self.tcp)[:3], M[:3, :3] @ [1, 0, 0]

    def test_tip_reaches_target_with_sag_tilt_away_from_j1(self):
        p = np.array(named_targets(self.wc)['pad_ID5']) + [0, 0, 0.005]; q = solve(self.arm, self.wc, p)
        tip, axis = self.tip_and_axis(q); np.testing.assert_allclose(tip, p, atol=1e-4)
        tilt = self.wc['robot_registration']['sag_compensation']['tool_tilt_away_from_j1_deg']
        self.assertAlmostEqual(math.degrees(math.acos(-axis[2])), tilt, delta=0.05)
        j1 = (self.T @ np.r_[J1_WORLD, 1])[:2]; self.assertGreater(np.dot(axis[:2], p[:2] - j1), 0)  # 컵 끝이 J1 반대쪽

    def test_zero_tilt_points_straight_down(self):
        p = np.array(named_targets(self.wc)['turntable']) + [0, 0, 0.03]; q = solve(self.arm, self.wc, p, tilt_deg=0.0)
        tip, axis = self.tip_and_axis(q); np.testing.assert_allclose(tip, p, atol=1e-4); np.testing.assert_allclose(axis, [0, 0, -1], atol=1e-4)

    def test_matches_values_verified_on_robot_20261103(self):
        # 실물에서 컵 중심 0.7~1.4 mm로 확인한 관절각 (턴테이블·ID5·ID6 위 5 mm)
        got = {k: np.degrees(solve(self.arm, self.wc, np.array(v) + [0, 0, 0.005])) for k, v in named_targets(self.wc).items()}
        for k, v in {'turntable': [2.24, 49.92, -102.23, -34.69], 'pad_ID6': [-33.07, 29.4, -61.78, -54.63], 'pad_ID5': [35.66, 29.0, -60.25, -55.75]}.items():
            np.testing.assert_allclose(got[k], v, atol=0.05, err_msg=k)


if __name__ == '__main__':
    unittest.main()
