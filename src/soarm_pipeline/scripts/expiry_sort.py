#!/usr/bin/python3
"""ROS entry point for the workspace-local pipeline. No execution by default."""
import os
import sys
from pathlib import Path

def find_entry():
    for parent in Path(__file__).resolve().parents:
        entry=parent/'automation/reconstruction/run_qr_expiry_cycle.py'
        if entry.is_file():return entry
    raise RuntimeError('Workspace automation/reconstruction/run_qr_expiry_cycle.py missing')

if __name__=='__main__':
    os.execv(sys.executable,[sys.executable,str(find_entry()),*sys.argv[1:]])
