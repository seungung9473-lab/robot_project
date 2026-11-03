import json
import tempfile
import unittest
from pathlib import Path
from soarm4_workcell.experiment import ExperimentLog, xy_error_mm, STAGES


class ExperimentTests(unittest.TestCase):
    def make(self):
        d = tempfile.mkdtemp(); return ExperimentLog('t', root=d, snapshot=lambda: {'j1_center': 2078}), Path(d)

    def test_xy_error_is_signed_mm(self):
        e = xy_error_mm([1.0, 0.5], [1.003, 0.496]); self.assertEqual((e['dx_mm'], e['dy_mm']), (3.0, -4.0)); self.assertEqual(e['err_mm'], 5.0)

    def test_trials_are_numbered_and_snapshot_saved(self):
        log, _ = self.make(); a = log.new_trial(note='a'); b = log.new_trial(note='b'); self.assertEqual((a, b), (1, 2))
        self.assertEqual(log.records()[0]['config'], {'j1_center': 2078})

    def test_place_stage_computes_error_automatically(self):
        log, _ = self.make(); t = log.new_trial(); log.add(t, 'S5_place', target_xy_m=[1.251, 0.367], placed_xy_m=[1.2512, 0.3731])
        r = [x for x in log.records() if x['stage'] == 'S5_place'][0]; self.assertAlmostEqual(r['dy_mm'], 6.1, places=1)

    def test_unknown_stage_rejected(self):
        log, _ = self.make(); t = log.new_trial()
        with self.assertRaises(ValueError): log.add(t, 'S9_nope', x=1)

    def test_summary_mean_and_std_over_trials(self):
        log, _ = self.make()
        for dy in (2.0, 4.0, 6.0):
            t = log.new_trial(); log.add(t, 'S5_place', target_xy_m=[0, 0], placed_xy_m=[0, dy / 1000])
        s = log.summary()['S5_place']['stats']['dy_mm']; self.assertEqual((s['n'], s['mean'], s['std']), (3, 4.0, 2.0))

    def test_csv_export_has_one_row_per_stage_record(self):
        log, d = self.make(); t = log.new_trial(); log.add(t, 'S3_grip', first_diff=-404, success=True); p = log.export_csv()
        rows = p.read_text().splitlines(); self.assertEqual(len(rows), 2); self.assertIn('first_diff', rows[0])


if __name__ == '__main__':
    unittest.main()
