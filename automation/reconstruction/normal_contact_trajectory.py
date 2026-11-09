"""Continuous surface-normal contact trajectory, joint/acceleration bounded."""
import numpy as np
from inclined_geometry import solve_axis

def contact_samples(fk,lo,hi,start,goal,velocity,acceleration,samples=81):
    start=np.asarray(start);goal=np.asarray(goal);a=fk(start);b=fk(goal);axis=b[:3,0]
    if np.degrees(np.arccos(np.clip(a[:3,0]@axis,-1,1)))>1:raise RuntimeError('Approach axis differs from contact axis')
    u=np.linspace(0,1,samples);s=10*u**3-15*u**4+6*u**5;positions=[start];seed=start
    for fraction in s[1:-1]:
        result=solve_axis(fk,lo,hi,a[:3,3]+fraction*(b[:3,3]-a[:3,3]),axis,seed=seed,attempts=0)
        if result is None:raise RuntimeError('No fixed-axis IK along contact line')
        seed=result[0];positions.append(seed)
    positions.append(goal);q=np.array(positions);v=np.gradient(q,u,axis=0,edge_order=2);v[[0,-1]]=0;acc=np.gradient(v,u,axis=0,edge_order=2);acc[[0,-1]]=0
    vmax=np.asarray(velocity,float);amax=np.asarray(acceleration,float)
    if np.any(vmax<=0) or np.any(amax<=0):raise ValueError('Positive velocity/acceleration limits required')
    duration=max(.5,float(np.max(np.abs(v)/vmax)),float(np.sqrt(np.max(np.abs(acc)/amax))))*1.15
    return u*duration,q,v/duration,acc/duration**2



def trajectory_from_samples(names, samples):
    """Convert a checked fixed-axis profile to the trajectory sent to MoveIt."""
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint

    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(names)
    for time_s, q, velocity, acceleration in zip(*samples):
        point = JointTrajectoryPoint()
        point.positions = list(map(float, q))
        point.velocities = list(map(float, velocity))
        point.accelerations = list(map(float, acceleration))
        ns = round(float(time_s) * 1_000_000_000)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 1_000_000_000)
        trajectory.joint_trajectory.points.append(point)
    return trajectory
