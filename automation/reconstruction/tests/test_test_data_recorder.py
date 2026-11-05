import json,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from test_data_recorder import Trial,import_existing,refresh_index

class RecorderTests(unittest.TestCase):
    def test_physical_result_and_throttled_signals(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);result=root/'execution.json';result.write_text(json.dumps({'success':True,'release':'diff=1'}))
            t=Trial('physical_pick_place',True,root,root,install_hooks=False);t.attach(result)
            for _ in range(30):t.sample('vacuum',diff=-250)
            t.sample('joint_states',position_rad=[0,3.14,-3.14,0]);t.event('phase',phase='home');t.finish()
            r=json.loads((t.directory/'summary.json').read_text());self.assertEqual(r['outcome'],'complete')
            samples=[json.loads(x) for x in (t.directory/'telemetry.jsonl').read_text().splitlines()]
            self.assertEqual(len(samples),2);self.assertTrue((root/'index.csv').exists())
            self.assertIn('diff=1',(t.directory/'results.json').read_text())

    def test_failure_is_not_success_and_finish_is_idempotent(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);p=root/'result.json';p.write_text(json.dumps({'success':False,'error':'pressure lost'}))
            t=Trial('trial',root=root,install_hooks=False);t.attach(p);t.finish();t.finish()
            r=json.loads((t.directory/'summary.json').read_text());self.assertEqual(r['outcome'],'failed');self.assertEqual(r['error'],'pressure lost')

    def test_no_result_not_claimed_success(self):
        with tempfile.TemporaryDirectory() as folder:
            t=Trial('trial',root=folder,install_hooks=False);t.finish()
            self.assertEqual(json.loads((t.directory/'summary.json').read_text())['outcome'],'finished_without_verified_result')

    def test_history_idempotent_preserves_missing_telemetry(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);p=root/'old.json';p.write_text(json.dumps({'success':True}))
            self.assertTrue(import_existing(p,'physical_execution',root));self.assertFalse(import_existing(p,'physical_execution',root));refresh_index(root)
            r=json.loads(next((root/'runs').glob('*/summary.json')).read_text());self.assertTrue(r['retrospective']);self.assertTrue(r['telemetry_not_reconstructed']);self.assertIsNone(r['started_at'])

if __name__=='__main__':unittest.main()
