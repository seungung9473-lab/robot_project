import math
import unittest
from types import SimpleNamespace
from soarm_bridge.servo_bridge import fine_goals, ServoBridge


class FineGoalsTests(unittest.TestCase):
    lo, hi = [0, 0], [4095, 4095]

    def test_all_within_tolerance_returns_none(self):
        self.assertIsNone(fine_goals([2000, 1000], [2002, 999], [2000, 1000], self.lo, self.hi, 2, 20))

    def test_short_joint_gets_overshoot_other_keeps_command(self):
        # J1 6 count 모자람(데드밴드) → 명령을 6 더, J2는 그대로
        self.assertEqual(fine_goals([2000, 1000], [1994, 1001], [2000, 1000], self.lo, self.hi, 2, 20), [2006, 1000])

    def test_repeated_correction_accumulates_from_last_command(self):
        self.assertEqual(fine_goals([2000, 1000], [1997, 1000], [2006, 1000], self.lo, self.hi, 2, 20), [2009, 1000])

    def test_correction_is_capped_and_clamped(self):
        self.assertEqual(fine_goals([2000, 10], [1950, 30], [2000, 10], self.lo, [4095, 4095], 2, 20), [2020, 0])


class TightCorrectionTests(unittest.TestCase):
    def test_two_count_error_is_now_corrected(self):
        self.assertEqual(fine_goals([1000], [1002], [1000], [0], [4095], 1, 4), [998])
    def test_one_count_error_does_not_chase_quantization(self):
        self.assertIsNone(fine_goals([1000], [1001], [1000], [0], [4095], 1, 4))
    def test_commands_bounded_even_with_large_residual(self):
        self.assertEqual(fine_goals([1000], [900], [1003], [0], [4095], 1, 4), [1004])
    def test_effective_limit_cannot_be_crossed(self):
        self.assertEqual(fine_goals([1000], [1005], [1000], [999], [2000], 1, 4), [999])


class PositiveGainTests(unittest.TestCase):
    b = SimpleNamespace(sign=[-1], center=[2078], cmin=[974], cmax=[3357], mlo=[0], mhi=[0], pos_gain=[0.043])

    def test_positive_side_commands_more_and_reports_true_angle(self):
        r = math.radians(35.66); c, _ = ServoBridge.r2c(self.b, 0, r)
        self.assertAlmostEqual(math.degrees(-(c - 2078) * 2 * math.pi / 4096), 35.66 * 1.043, delta=0.1)  # 모터는 더 돎
        self.assertAlmostEqual(math.degrees(ServoBridge.c2r(self.b, 0, c)), 35.66, delta=0.1)          # 보고는 실제 각도

    def test_negative_side_unchanged(self):
        r = math.radians(-33.07); c, _ = ServoBridge.r2c(self.b, 0, r)
        self.assertAlmostEqual(math.degrees(ServoBridge.c2r(self.b, 0, c)), -33.07, delta=0.1)
        self.assertEqual(c, round(2078 + 33.07 / 360 * 4096))


if __name__ == '__main__':
    unittest.main()
