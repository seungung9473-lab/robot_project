"""Desktop-profile checks; no ROS, hardware, network or GPU inference."""
import ast,json,os,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch,Mock
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'automation/reconstruction'))
from desktop_runtime import require_cuda_profile,gpu_python

class DesktopRuntimeTests(unittest.TestCase):
 def fake(self,available=True):return SimpleNamespace(cuda=SimpleNamespace(is_available=lambda:available,get_device_name=lambda _: 'test GPU'),__version__='test',version=SimpleNamespace(cuda='test'))
 def config(self,folder,precision='fp32',batch=1):
  p=Path(folder)/'config.json';p.write_text(json.dumps({'precision':precision,'batch_size':batch}));return p
 def test_auto_without_cuda_does_not_fallback(self):
  with self.assertRaisesRegex(RuntimeError,'no CPU fallback'):require_cuda_profile('auto',torch_module=self.fake(False))
 def test_cpu_rejected_even_when_cuda_is_available(self):
  with self.assertRaisesRegex(RuntimeError,'CPU fallback'):require_cuda_profile('cpu',torch_module=self.fake())
 def test_validated_precision_only(self):
  with tempfile.TemporaryDirectory() as f:
   with self.assertRaisesRegex(RuntimeError,'FP32'):require_cuda_profile('cuda',torch_module=self.fake(),config_path=self.config(f,'bf16'))
 def test_boolean_is_not_a_batch_size(self):
  with tempfile.TemporaryDirectory() as f:
   with self.assertRaisesRegex(RuntimeError,'batch'):require_cuda_profile('cuda',torch_module=self.fake(),config_path=self.config(f,batch=True))
 def test_actual_runtime_metadata(self):
  with tempfile.TemporaryDirectory() as f:
   data=require_cuda_profile('cuda',torch_module=self.fake(),config_path=self.config(f));self.assertEqual(data['gpu_name'],'test GPU');self.assertFalse(data['cpu_fallback'])
 def test_venv_path_not_resolved_to_system_python(self):
  with tempfile.TemporaryDirectory() as f,patch.dict(os.environ,{},clear=True):
   p=Path(f)/'automation/reconstruction/.sam2_gpu/bin/python';p.parent.mkdir(parents=True);p.symlink_to(sys.executable);self.assertEqual(gpu_python(f),str(p))
 def test_missing_gpu_python_is_explicit(self):
  with tempfile.TemporaryDirectory() as f,patch.dict(os.environ,{},clear=True):
   with self.assertRaisesRegex(RuntimeError,'CUDA Python missing'):gpu_python(f)
 def test_default_backend_is_current_workspace(self):
  s=(ROOT/'run_tidy_demo.sh').read_text();self.assertIn('SOARM_DEMO_BACKEND:-$workspace',s);self.assertNotIn('${workspace}_mybranch',s)
 def test_gpu_preflight_precedes_hardware_preflight(self):
  tree=ast.parse((ROOT/'automation/reconstruction/run_qr_expiry_cycle.py').read_text());main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main');calls={}
  for n in ast.walk(main):
   if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id in ('require_cuda_profile','control_preflight'):calls[n.func.id]=n.lineno
  self.assertLess(calls['require_cuda_profile'],calls['control_preflight'])
 def test_ros_entry_uses_gpu_interpreter_for_execution(self):
  s=(ROOT/'src/soarm_pipeline/scripts/expiry_sort.py').read_text();self.assertIn("if '--execute' in sys.argv[1:]",s);self.assertIn('runtime=gpu_python(entry.parents[2])',s)

 def test_disconnected_hardware_does_not_call_gemini(self):
  import tidy_demo,contextlib,io
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);(root/'automation/reconstruction').mkdir(parents=True);folder=root/'record';folder.mkdir();trial=Mock(directory=folder)
   with patch.object(tidy_demo,'select_backend',return_value=root),patch('test_data_recorder.start_trial',return_value=trial),patch('desktop_runtime.require_cuda_profile',return_value={'device':'cuda','gpu_name':'mock GPU'}),patch('desktop_runtime.gpu_python',return_value='/fake/cuda/python'),patch('run_qr_expiry_cycle.control_preflight',side_effect=RuntimeError('servo offline')),patch.object(tidy_demo,'understand') as api,patch.dict(os.environ,{'ROS_DOMAIN_ID':'73'}),contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
    self.assertEqual(tidy_demo.main(['--execute']),1);api.assert_not_called()
   result=json.loads((folder/'demo_result.json').read_text());self.assertEqual(result['error'],'servo offline');self.assertEqual(result['stages'][0]['status'],'stopped')

if __name__=='__main__':unittest.main()
