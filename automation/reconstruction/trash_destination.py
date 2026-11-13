"""Saved joint target, calibration identity and arrival gate for trash release."""
import hashlib,math
from pathlib import Path
import numpy as np,yaml
POSE_FILE=Path.home()/'robot_ws/src/TUMST_ARM_workcell/config/named_poses.yaml'

def load_trash_pose(path=POSE_FILE):
 data=yaml.safe_load(Path(path).read_text())['poses']['trash_drop'];q=np.asarray(data['positions_rad'],float)
 if data['joint_names']!=['joint1','joint2','joint3','joint4'] or q.shape!=(4,) or not np.isfinite(q).all():raise ValueError('Invalid taught trash joints')
 for p,digest in data['calibration_hashes'].items():
  if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=digest:raise ValueError('Taught trash calibration changed: '+p)
 return data

def arrival_confirmed(actual,target,tolerance_deg=1.):
 a=np.asarray(actual,float);b=np.asarray(target,float)
 return a.shape==b.shape==(4,) and np.isfinite(a).all() and np.isfinite(b).all() and np.max(np.abs(a-b))<=math.radians(tolerance_deg)
