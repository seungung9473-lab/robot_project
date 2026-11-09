"""Offline SAM2.1 inference; one model and image encoding for all object prompts."""
import argparse, hashlib, os, contextlib, json
from pathlib import Path
import torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from sam2_shared import run_sessions

def main():
    parser=argparse.ArgumentParser();group=parser.add_mutually_exclusive_group()
    group.add_argument('--session',type=Path);group.add_argument('--sessions',type=Path,nargs='+')
    parser.add_argument('--device',choices=['auto','cpu','cuda'],default=os.environ.get('SOARM_SAM2_DEVICE','cuda'))
    parser.add_argument('--review',action='store_true',help='Write optional JPG mask review images in GPU mode')
    args=parser.parse_args();root=Path(__file__).resolve().parent
    sessions=args.sessions or [args.session or root/'multiview/20261101_194509_024461']
    if len({p.resolve() for p in sessions})!=len(sessions):raise ValueError('Duplicate SAM2 session')
    torch.set_num_threads(min(6,os.cpu_count() or 2));checkpoint=root/'checkpoints/sam2.1_hiera_tiny.pt'
    from desktop_runtime import require_cuda_profile
    runtime=require_cuda_profile(args.device,torch_module=torch)
    device='cuda'
    predictor=SAM2ImagePredictor(build_sam2('configs/sam2.1/sam2.1_hiera_t.yaml',str(checkpoint),device=device,apply_postprocessing=False))
    if device=='cuda':
        from sam2_gpu import run_gpu
        config=json.loads((root/'sam2_gpu_config.json').read_text())
        if config.get('precision')!='fp32':raise RuntimeError('Only validated FP32 SAM2 is enabled')
        ctx=contextlib.nullcontext()
        with torch.inference_mode(),ctx:stats=run_gpu(sessions,predictor,config['batch_size'],args.review)
        print(json.dumps({**runtime,**stats}))
        for session in sessions:
            path=Path(session)/'sam2/manifest.json';payload=json.loads(path.read_text());payload['runtime']=runtime;path.write_text(json.dumps(payload,indent=2))
    else:
        with torch.inference_mode():run_sessions(sessions,predictor,hashlib.sha256(checkpoint.read_bytes()).hexdigest())

if __name__=='__main__':main()
