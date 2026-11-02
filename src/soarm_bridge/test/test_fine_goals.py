import unittest
from soarm_bridge.servo_bridge import fine_goals


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


if __name__ == '__main__':
    unittest.main()
