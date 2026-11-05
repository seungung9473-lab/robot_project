"""Reuse reviewed trajectories only for matching measured starts, goals and speed."""
import math,numpy as np

def compatible(entry,names,start,goal,scaling,tolerance_deg=.25):
    if not entry or entry['joint_names']!=list(names):return False
    if any(abs(float(entry['scaling'][k])-float(scaling[k]))>1e-9 for k in ['velocity','acceleration']):return False
    return (np.max(np.abs(np.asarray(entry['start'])-start))<=math.radians(tolerance_deg)
            and np.max(np.abs(np.asarray(entry['goal'])-goal))<=math.radians(tolerance_deg))

def restore(entry):
    from moveit_msgs.msg import RobotTrajectory
    from rosidl_runtime_py.set_message import set_message_fields
    trajectory=RobotTrajectory();set_message_fields(trajectory,entry['trajectory']);return trajectory
