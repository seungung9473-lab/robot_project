"""Reuse reviewed trajectories only for matching measured starts, goals and speed."""
import math,numpy as np

from wait_pose import EXACT_DEG

def compatible(entry,names,start,goal,scaling,tolerance_deg=.25,start_tolerance_deg=EXACT_DEG):
    if not entry or entry['joint_names']!=list(names):return False
    if any(abs(float(entry['scaling'][k])-float(scaling[k]))>1e-9 for k in ['velocity','acceleration']):return False
    # 시작 자세는 wait_pose.EXACT_DEG(3.5°)까지 검토한 경로를 그대로 쓴다(2026-11-10 사용자 결정). 목표는 계속 .25° 이내여야 한다.
    return (np.max(np.abs(np.asarray(entry['start'])-start))<=math.radians(start_tolerance_deg)
            and np.max(np.abs(np.asarray(entry['goal'])-goal))<=math.radians(tolerance_deg))

def restore(entry):
    from moveit_msgs.msg import RobotTrajectory
    from rosidl_runtime_py.set_message import set_message_fields
    trajectory=RobotTrajectory();set_message_fields(trajectory,entry['trajectory']);return trajectory
