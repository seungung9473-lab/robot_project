"""Reuse reviewed trajectories only for matching measured starts, goals and speed."""
import math,numpy as np

from wait_pose import EXACT_DEG

def compatible(entry,names,start,goal,scaling,tolerance_deg=.25,start_tolerance_deg=EXACT_DEG,*,fixed_axis=False):
    if not entry or entry['joint_names']!=list(names):return False
    if fixed_axis and entry.get('profile') != 'fixed_axis':return False
    if any(abs(float(entry['scaling'][k])-float(scaling[k]))>1e-9 for k in ['velocity','acceleration']):return False
    trajectory = entry['trajectory']['joint_trajectory']
    if trajectory['joint_names'] != list(names) or not trajectory['points']:
        return False
    first = trajectory['points'][0]['positions']
    limit = math.radians(min(start_tolerance_deg, EXACT_DEG))
    return (np.max(np.abs(np.asarray(entry['start'])-start)) < limit
            and np.max(np.abs(np.asarray(first)-start)) < limit
            and np.max(np.abs(np.asarray(entry['goal'])-goal))<=math.radians(tolerance_deg))

def restore(entry):
    from moveit_msgs.msg import RobotTrajectory
    from rosidl_runtime_py.set_message import set_message_fields
    trajectory=RobotTrajectory();set_message_fields(trajectory,entry['trajectory']);return trajectory
