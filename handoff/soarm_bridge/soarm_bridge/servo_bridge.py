#!/usr/bin/env python3
"""
servo_bridge — STS 서보 버스 ↔ ROS 2

  발행  /joint_states            (sensor_msgs/JointState, rad)  30 Hz
        /servo/status            (std_msgs/Float32MultiArray)   [전압V, 온도°C] x 관절 수
        /turntable/angle         (std_msgs/Float64, deg)        턴테이블 서보 쓸 때만
  구독  /turntable/goto          (std_msgs/Float64, deg 0~360)
  서비스 /servo/torque            (std_srvs/SetBool)             true=힘 줌, false=힘 뺌(팔 떨어짐 주의)
  액션  <action_name>            (control_msgs/FollowJointTrajectory)  MoveIt 이 여기로 궤적을 보냄

변환: rad = sign * (count - center) * 2π/4096   (config/soarm.yaml 의 servo_map)
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64
from std_srvs.srv import SetBool

from soarm_bridge.sts_bus import STSBus

C2R = 2 * math.pi / 4096


class ServoBridge(Node):
    def __init__(self):
        super().__init__('servo_bridge')
        p = self.declare_parameter
        port = p('port', '/dev/servo_bus').value
        baud = p('baud', 1_000_000).value
        self.names = list(p('joint_names', ['joint1', 'joint2', 'joint3', 'joint4']).value)
        self.ids = list(p('ids', [1, 2, 3, 4]).value)
        self.center = list(p('centers', [2047, 4095, 0, 2047]).value)
        self.sign = list(p('signs', [-1, -1, -1, -1]).value)
        self.cmin = list(p('min_counts', [860, 1879, 0, 660]).value)
        self.cmax = list(p('max_counts', [3472, 4095, 2047, 3273]).value)
        self.mlo = list(p('margin_lo', [40, 10, 40, 40]).value)
        self.mhi = list(p('margin_hi', [40, 40, 10, 40]).value)
        self.acc = p('acc', 30).value
        self.vmin = p('min_speed', 50).value
        self.vmax = p('max_speed', 1500).value
        self.tol = p('goal_tolerance_counts', 25).value
        self.settle = p('settle_timeout', 3.0).value
        rate = p('rate_hz', 30.0).value
        action_name = p('action_name', 'arm_controller/follow_joint_trajectory').value
        self.tt_id = p('turntable_id', 0).value              # 0 = 턴테이블 서보 없음
        self.tt_speed = p('turntable_speed', 600).value

        self.bus = STSBus(port, baud)
        self.n = len(self.ids)
        self.pos = [None] * self.n
        self.vel = [0.0] * self.n
        self.volt = [0.0] * self.n
        self.temp = [0.0] * self.n
        self.torque_on = False
        self.exec_lock = threading.Lock()

        cb = ReentrantCallbackGroup()
        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_st = self.create_publisher(Float32MultiArray, 'servo/status', 10)
        self.create_service(SetBool, 'servo/torque', self.srv_torque, callback_group=cb)
        self.create_timer(1.0 / rate, self.tick, callback_group=cb)
        self.action = ActionServer(self, FollowJointTrajectory, action_name, self.execute,
                                   goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=cb)
        if self.tt_id:
            self.pub_tt = self.create_publisher(Float64, 'turntable/angle', 10)
            self.create_subscription(Float64, 'turntable/goto', self.on_tt_goto, 10, callback_group=cb)
            self.tt_deg = None

        missing = [sid for sid in self.ids + ([self.tt_id] if self.tt_id else []) if not self.bus.ping(sid)]
        if missing:
            self.get_logger().error(f'응답 없는 서보 ID: {missing} (12V 전원 / 배선 / ID 확인)')
        self.get_logger().info(f'servo_bridge 시작: {port}, 관절 {self.names}, 액션 "{action_name}"')

    # ---------------- 변환
    def c2r(self, i, c):
        return self.sign[i] * (c - self.center[i]) * C2R

    def r2c(self, i, r):
        c = self.center[i] + self.sign[i] * r / C2R
        lo, hi = self.cmin[i] + self.mlo[i], self.cmax[i] - self.mhi[i]
        return int(round(min(max(c, lo), hi))), (c < lo - 1 or c > hi + 1)

    # ---------------- 주기 읽기
    def tick(self):
        for i, sid in enumerate(self.ids):
            st = self.bus.read_state(sid)
            if st:
                c, spd, v, t = st
                self.pos[i] = c
                self.vel[i] = self.sign[i] * spd * C2R
                self.volt[i], self.temp[i] = v, float(t)
        if None in self.pos:
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.names
        js.position = [self.c2r(i, c) for i, c in enumerate(self.pos)]
        js.velocity = list(self.vel)
        self.pub_js.publish(js)
        self.pub_st.publish(Float32MultiArray(data=[x for i in range(self.n) for x in (self.volt[i], self.temp[i])]))
        if self.tt_id:
            c = self.bus.read_pos(self.tt_id)
            if c is not None:
                self.tt_deg = c * 360.0 / 4096
                self.pub_tt.publish(Float64(data=self.tt_deg))

    # ---------------- 토크
    def set_torque(self, on):
        if on and None not in self.pos:
            # 힘 주기 전에 목표 = 현재 위치 (예전 목표값으로 갑자기 튀는 것 방지)
            self.bus.sync_goal({sid: (self.pos[i], self.vmin) for i, sid in enumerate(self.ids)}, self.acc)
            time.sleep(0.02)
        for sid in self.ids:
            self.bus.torque(sid, on)
        self.torque_on = on

    def srv_torque(self, req, res):
        if not req.data and self.exec_lock.locked():
            res.success, res.message = False, '궤적 실행 중에는 토크를 끌 수 없음'
            return res
        self.set_torque(req.data)
        res.success, res.message = True, 'torque ' + ('ON' if req.data else 'OFF (팔 받치기!)')
        return res

    # ---------------- 궤적 실행
    def on_goal(self, goal):
        names = list(goal.trajectory.joint_names)
        if not set(names) <= set(self.names) or not goal.trajectory.points:
            self.get_logger().warn(f'거절: 모르는 관절 {names}')
            return GoalResponse.REJECT
        if self.exec_lock.locked():
            self.get_logger().warn('거절: 이미 다른 궤적 실행 중')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, gh):
        with self.exec_lock:
            traj = gh.request.trajectory
            idx = [self.names.index(nm) for nm in traj.joint_names]
            result = FollowJointTrajectory.Result()
            if not self.torque_on:
                self.set_torque(True)
            prev = list(self.pos)
            t_prev = 0.0
            t0 = time.monotonic()
            clamped = False
            for pt in traj.points:
                if gh.is_cancel_requested:
                    self.hold_here()
                    gh.canceled()
                    result.error_code = 0
                    return result
                t_pt = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                dt = max(t_pt - t_prev, 0.02)
                goals = {}
                tgt = list(prev)
                for k, i in enumerate(idx):
                    c, cl = self.r2c(i, pt.positions[k])
                    clamped |= cl
                    spd = abs(c - prev[i]) / dt * 1.1
                    goals[self.ids[i]] = (c, int(min(max(spd, self.vmin), self.vmax)))
                    tgt[i] = c
                self.bus.sync_goal(goals, self.acc)
                prev, t_prev = tgt, t_pt
                while time.monotonic() - t0 < t_pt:            # 다음 점 시간까지 대기
                    if gh.is_cancel_requested:
                        break
                    time.sleep(0.005)
                fb = FollowJointTrajectory.Feedback()
                fb.joint_names = [self.names[i] for i in idx]
                fb.actual.positions = [self.c2r(i, self.pos[i]) for i in idx]
                fb.desired.positions = list(pt.positions)
                gh.publish_feedback(fb)
            if clamped:
                self.get_logger().warn('일부 목표가 관절 범위 밖이라 잘림 (clamp)')
            t_end = time.monotonic()
            while time.monotonic() - t_end < self.settle:         # 도착 확인
                if all(abs(self.pos[i] - prev[i]) <= self.tol for i in idx):
                    gh.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result
                time.sleep(0.02)
            gh.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = '목표 위치 도착 못 함 (걸림/과부하/전원 확인)'
            self.get_logger().error(result.error_string)
            return result

    def hold_here(self):
        self.bus.sync_goal({sid: (self.pos[i], self.vmin) for i, sid in enumerate(self.ids)}, self.acc)

    # ---------------- 턴테이블 (STS3215, ID = turntable_id)
    def on_tt_goto(self, msg):
        deg = msg.data % 360.0
        c = int(round(deg * 4096 / 360.0)) % 4096
        self.bus.torque(self.tt_id, True)
        self.bus.sync_goal({self.tt_id: (c, self.tt_speed)}, self.acc)
        self.get_logger().info(f'turntable → {deg:.1f}°')

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ServoBridge()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
