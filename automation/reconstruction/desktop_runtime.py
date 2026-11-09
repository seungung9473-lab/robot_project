"""Desktop CUDA profile. No ROS or actuator commands; no CPU fallback."""
import json,os,sys
from pathlib import Path

def gpu_python(workspace=None):
    root=Path(workspace) if workspace is not None else Path(__file__).resolve().parents[2]
    path=Path(os.environ.get('SOARM_SAM2_PYTHON',str(root/'automation/reconstruction/.sam2_gpu/bin/python'))).expanduser()
    if not path.is_file() or not os.access(path,os.X_OK):
        raise RuntimeError('CUDA Python missing: '+str(path)+'; install .sam2_gpu or set SOARM_SAM2_PYTHON')
    # Keep the venv symlink path; resolve() would select system Python and lose its environment.
    return str(path.absolute())

def require_cuda_profile(device=None, *, torch_module=None, config_path=None):
    requested=device if device is not None else os.environ.get('SOARM_SAM2_DEVICE','cuda')
    if requested not in ('cuda','auto'):
        raise RuntimeError('Desktop requires CUDA/FP32; CPU fallback is disabled')
    if torch_module is None:
        import torch as torch_module
    if not torch_module.cuda.is_available():
        raise RuntimeError('CUDA unavailable; stopped before robot movement, no CPU fallback')
    path=Path(config_path) if config_path else Path(__file__).with_name('sam2_gpu_config.json')
    config=json.loads(path.read_text())
    if config.get('precision')!='fp32':raise RuntimeError('Only validated FP32 SAM2 is enabled')
    batch=config.get('batch_size')
    if type(batch) is not int or batch<1:raise RuntimeError('Invalid SAM2 batch size')
    name=torch_module.cuda.get_device_name(0)
    if not name:raise RuntimeError('GPU identity unavailable')
    return {'device':'cuda','precision':'fp32','batch_size':batch,'gpu_name':name,
            'torch_version':torch_module.__version__,'cuda_version':torch_module.version.cuda,
            'python':sys.executable,'cpu_fallback':False}
