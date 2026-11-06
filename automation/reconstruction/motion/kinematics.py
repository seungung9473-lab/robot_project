import json,yaml,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
m=json.loads(Path('/home/wanjae/robot_ws/src/soarm4_description/config/model.json').read_text());d=yaml.safe_load(Path('/home/wanjae/robot_ws/src/soarm4_workcell/config/workcell.yaml').read_text());W=np.array(d['T_table_world']);B=np.eye(4);B[:3,:3]=Rotation.from_euler('z',m['robot_base_yaw']).as_matrix();B[:3,3]=m['robot_base_xyz']
def fk(q):
 T=W@B
 for j,a in zip(m['joints'],q):
  J=np.eye(4);J[:3,3]=j['origin_xyz'];R=np.eye(4);R[:3,:3]=Rotation.from_rotvec(np.array(j['axis'])*a).as_matrix();T=T@J@R
 T[:3,3]+=T[:3,:3]@m['tcp_xyz'];return T
