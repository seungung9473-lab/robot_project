"""Offline SAM2.1 inference; one model and image encoding for all object prompts."""
import argparse,hashlib,os
from pathlib import Path
import torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from sam2_shared import run_sessions

def main():
    parser=argparse.ArgumentParser();group=parser.add_mutually_exclusive_group()
    group.add_argument('--session',type=Path);group.add_argument('--sessions',type=Path,nargs='+')
    args=parser.parse_args();root=Path(__file__).resolve().parent
    sessions=args.sessions or [args.session or root/'multiview/20261101_194509_024461']
    if len({p.resolve() for p in sessions})!=len(sessions):raise ValueError('Duplicate SAM2 session')
    torch.set_num_threads(min(6,os.cpu_count() or 2));checkpoint=root/'checkpoints/sam2.1_hiera_tiny.pt'
    predictor=SAM2ImagePredictor(build_sam2('configs/sam2.1/sam2.1_hiera_t.yaml',str(checkpoint),device='cpu',apply_postprocessing=False))
    with torch.inference_mode():run_sessions(sessions,predictor,hashlib.sha256(checkpoint.read_bytes()).hexdigest())

if __name__=='__main__':main()
