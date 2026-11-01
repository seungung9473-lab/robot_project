#!/usr/bin/env python3
"""
simple_pick.py — MoveIt 없이 soarm_bridge 만으로 '자세 이동 → 잡기 → 이동 → 놓기' 해보는 예제
  먼저:  ros2 launch soarm_bridge bringup.launch.py
  실행:  python3 simple_pick.py
관절 각도(rad)는 URDF 기준: 0 = 팔을 앞으로 수평으로 쭉 편 자세, J2/J3/J4 + = 위로 듦
아래 자세 값은 예시라 실제 물체 위치에 맞게 바꿔서 쓸 것 (처음엔 손을 12V 스위치에!)
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']
POSES = {                       # [J1, J2, J3, J4] rad  ← 예시값
    'ready': [0.0, 1.9, -2.3, -0.5],
    'above': [0.0, 1.2, -1.6, -1.1],
    'pick':  [0.0, 1.0, -1.6, -0.9],
    'place': [1.2, 1.2, -1.6, -1.1],
}


class SimplePick(Node):
    def __init__(self):
        super().__init__('simple_pick')
        self.ac = ActionClient(self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
        self.grip = self.create_client(Trigger, 'vacuum/grip')
        self.release = self.create_client(Trigger, 'vacuum/release')

    def move(self, name, sec=2.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        p = JointTrajectoryPoint(positions=POSES[name], time_from_start=Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9)))
        goal.trajectory.points = [p]
        self.ac.wait_for_server()
        gh = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh)
        res = gh.result().get_result_async()
        rclpy.spin_until_future_complete(self, res)
        ok = res.result().result.error_code == 0
        self.get_logger().info(f'move {name}: {"OK" if ok else "FAIL"}')
        return ok

    def call(self, cli):
        cli.wait_for_service()
        f = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, f)
        return f.result().success, f.result().message


def main():
    rclpy.init()
    n = SimplePick()
    n.move('ready')
    n.move('above')
    n.move('pick', 1.0)
    ok, msg = n.call(n.grip)
    n.get_logger().info(f'grip: {ok} ({msg})')
    n.move('above', 1.0)
    if ok:
        n.move('place')
        print('release:', n.call(n.release))
    n.move('ready')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
