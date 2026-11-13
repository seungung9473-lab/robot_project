import json,yaml,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
m=json.loads(Path('/home/wanjae/robot_ws/src/TUMST_ARM_description/config/model.json').read_text());d=yaml.safe_load(Path('/home/wanjae/robot_ws/src/TUMST_ARM_workcell/config/workcell.yaml').read_text());W=np.array(d['T_table_world']);B=np.eye(4);B[:3,:3]=Rotation.from_euler('z',m['robot_base_yaw']).as_matrix();B[:3,3]=m['robot_base_xyz']
def joint_frames(q):
    """Base and successive joint frames in table coordinates."""
    T = W @ B
    frames = [T]
    for joint, angle in zip(m['joints'], q):
        transform = np.eye(4)
        transform[:3, 3] = joint['origin_xyz']
        transform[:3, :3] = Rotation.from_rotvec(np.array(joint['axis']) * angle).as_matrix()
        T = T @ transform
        frames.append(T)
    return frames


def fk(q):
    T = joint_frames(q)[-1]
    T[:3, 3] += T[:3, :3] @ m['tcp_xyz']
    return T
