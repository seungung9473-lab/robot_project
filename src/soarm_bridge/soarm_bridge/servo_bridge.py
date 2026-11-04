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
from std_srvs.srv import SetBool, Trigger

from soarm_bridge.sts_bus import STSBus
from soarm_bridge.trajectory_speed import planned_speed

C2R = 2 * math.pi / 4096


def fine_goals(final, pos, cmd, cmin, cmax, tol, max_extra):
    """도착 후 미세 보정 목표(count). 모든 관절이 final에서 tol 이내면 None.
    모자란 관절은 '지난 명령 + 모자란 만큼'(final에서 최대 max_extra까지), 나머지는 지난 명령 유지."""
    err = [f - p for f, p in zip(final, pos)]
    if all(abs(e) <= tol for e in err):return None
    out = []
    for f, e, c, lo, hi in zip(final, err, cmd, cmin, cmax):
        g = c + e if abs(e) > tol else c
        out.append(int(round(min(max(g, f - max_extra, lo), f + max_extra, hi))))
    return out


class ServoBridge(Node):
    def __init__(self):
        super().__init__('servo_bridge')
        p = self.declare_parameter
        self.read_only = bool(p('read_only', False).value)
        port = p('port', '/dev/servo_bus').value
        baud = p('baud', 1_000_000).value
        self.names = list(p('joint_names', ['joint1', 'joint2', 'joint3', 'joint4']).value)
        self.ids = list(p('ids', [1, 2, 3, 4]).value)
        self.center = list(p('centers', [2047, 4095, 0, 1990]).value)
        self.sign = list(p('signs', [-1, -1, -1, -1]).value)
        self.cmin = list(p('min_counts', [860, 1879, 0, 660]).value)
        self.cmax = list(p('max_counts', [3472, 4095, 2047, 3273]).value)
        self.mlo = list(p('margin_lo', [40, 10, 40, 40]).value)
        self.mhi = list(p('margin_hi', [40, 40, 10, 40]).value)
        # 관절이 + 쪽에서 실제로 덜 도는 비율 (튜브·케이블 당김+유격 보정). 0 = 보정 없음
        self.pos_gain = list(p('positive_gain', [0.0, 0.0, 0.0, 0.0]).value)
        self.acc = p('acc', 50).value
        self.vmin = p('min_speed', 1).value
        self.settle_speed = p('settle_speed', 30).value
        self.vmax = p('max_speed', 400).value
        if not (1<=self.vmin<=self.vmax and 1<=self.settle_speed<=self.vmax):raise ValueError('Invalid speed bounds')
        self.tol = p('goal_tolerance_counts', 15).value
        self.settle = p('settle_timeout', 3.0).value
        # 도착 후 미세 보정: 데드밴드 때문에 모자라게 선 관절에 모자란 만큼 더 보내기 (0회 = 끔)
        p('fine_correction_iterations', 3)
        self.fine_tol = p('fine_tolerance_counts', 2).value
        self.fine_max = p('fine_max_counts', 20).value
        rate = p('rate_hz', 30.0).value
        self.stream_hz = p('stream_hz', 50.0).value          # 궤적 따라갈 때 목표 보내는 주기
        self.lookahead = p('lookahead', 0.15).value          # 목표를 이만큼(초) 앞선 지점으로 → 서보가 중간에 안 멈춤
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
        self.poll_lock = threading.Lock()
        self.feedback_timeout=float(p('feedback_timeout',0.5).value)
        if not math.isfinite(self.feedback_timeout) or self.feedback_timeout<=0:raise ValueError('Invalid feedback_timeout')
        self.get_logger().info(f'엔코더 피드백 허용 시간: {self.feedback_timeout:.3f}초')
        self.last_feedback_time = 0.0
        self.last_missing_ids = []
        self.stop_event = threading.Event()

        cb = ReentrantCallbackGroup()
        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_st = self.create_publisher(Float32MultiArray, 'servo/status', 10)
        self.create_service(SetBool, 'servo/torque', self.srv_torque, callback_group=cb)
        self.create_service(Trigger, 'servo/toggle_torque', self.srv_toggle_torque, callback_group=cb)
        self.create_service(Trigger, 'servo/stop', self.srv_stop, callback_group=cb)
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
        r = self.sign[i] * (c - self.center[i]) * C2R
        return r / (1 + self.pos_gain[i]) if r > 0 else r          # 모터가 더 돈 만큼 실제 팔 각도로 환산

    def r2c(self, i, r):
        if r > 0:r = r * (1 + self.pos_gain[i])                   # + 쪽에서 팔이 덜 도는 만큼 모터를 더 돌림
        c = self.center[i] + self.sign[i] * r / C2R
        lo, hi = self.cmin[i] + self.mlo[i], self.cmax[i] - self.mhi[i]
        return int(round(min(max(c, lo), hi))), (c < lo - 1 or c > hi + 1)

    # ---------------- 주기 읽기
    def tick(self):
        if not self.poll_lock.acquire(blocking=False):return
        try:self.poll()
        finally:self.poll_lock.release()

    def poll(self):
        for i, sid in enumerate(self.ids):
            try:st = self.bus.read_state(sid)
            except (RuntimeError,OSError):st=None
            if st and 0<=st[0]<=4095:
                c, spd, v, t = st
                self.pos[i] = c
                self.vel[i] = self.sign[i] * spd * C2R
                self.volt[i], self.temp[i] = v, float(t)
            else:self.pos[i]=None
        missing=[sid for sid,c in zip(self.ids,self.pos) if c is None]
        if missing:
            if missing!=self.last_missing_ids:
                self.get_logger().error(f'엔코더 응답 유실 IDs={missing}; 위치 발행 중단; 최근 status={getattr(self.bus,"last_errors",{})}')
            self.last_missing_ids=missing
            return
        if self.last_missing_ids:
            self.get_logger().info(f'엔코더 응답 복구 IDs={self.last_missing_ids}')
            self.last_missing_ids=[]
        self.last_feedback_time=time.monotonic()
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
        if self.read_only:raise RuntimeError('Read-only bridge')
        if on and None not in self.pos:
            # 힘 주기 전에 목표 = 현재 위치 (예전 목표값으로 갑자기 튀는 것 방지)
            self.bus.sync_goal({sid: (self.pos[i], self.vmin) for i, sid in enumerate(self.ids)}, self.acc)
            time.sleep(0.02)
        for sid in self.ids:
            self.bus.torque(sid, on)
        self.torque_on = on

    def srv_torque(self, req, res):
        if self.read_only:
            res.success=False;res.message='읽기 전용 연결 점검 중';return res
        if not req.data and self.exec_lock.locked():
            res.success, res.message = False, '궤적 실행 중에는 토크를 끌 수 없음'
            return res
        self.set_torque(req.data)
        res.success, res.message = True, 'torque ' + ('ON' if req.data else 'OFF (팔 받치기!)')
        return res

    def srv_toggle_torque(self, req, res):
        if getattr(self,'read_only',False):
            res.success=False;res.message='읽기 전용 모드에서는 토크를 변경하지 않습니다';return res
        if not self.exec_lock.acquire(blocking=False):
            res.success=False;res.message='이동 중입니다. STOP 후 정지가 완료되면 다시 누르세요';return res
        try:
            states=[]
            for sid in self.ids:
                state=self.bus.read(sid,40,1)
                if state is None or len(state)!=1 or state[0] not in (0,1):
                    raise RuntimeError(f'ID {sid}: 토크 상태 읽기 실패')
                states.append(state[0])
            on=not any(states)
            if on:
                positions=[]
                for sid in self.ids:
                    st=self.bus.read_state(sid)
                    if st is None or not 0<=st[0]<=4095:
                        raise RuntimeError(f'ID {sid}: 현재 위치 읽기 실패')
                    positions.append(st[0])
                self.bus.sync_goal({sid:(positions[i],max(1,self.vmin)) for i,sid in enumerate(self.ids)},self.acc)
                for i,sid in enumerate(self.ids):
                    data=self.bus.read(sid,42,2)
                    if data is None or len(data)!=2 or int.from_bytes(data,'little')!=positions[i]:
                        raise RuntimeError(f'ID {sid}: 현재 위치 유지 목표 검증 실패')
            for sid in self.ids:
                self.bus.torque(sid,on)
                state=self.bus.read(sid,40,1)
                if state is None or len(state)!=1 or state[0]!=int(on):
                    raise RuntimeError(f'ID {sid}: 토크 변경 확인 실패')
            self.torque_on=on
            res.success=True;res.message='현재 자세 고정 · 토크 ON' if on else '토크 해제 · 팔을 받쳐 주세요'
        except (RuntimeError,OSError) as exc:
            res.success=False;res.message=str(exc)
        finally:self.exec_lock.release()
        self.get_logger().info(res.message)
        return res

    def feedback_fresh(self):
        return None not in self.pos and time.monotonic()-self.last_feedback_time<=self.feedback_timeout

    def srv_stop(self,req,res):
        self.stop_event.set();res.success=True;res.message='정지 요청 · 토크 유지';return res

    # ---------------- 궤적 실행
    def on_goal(self, goal):
        if self.read_only:return GoalResponse.REJECT
        names = list(goal.trajectory.joint_names)
        if not set(names) <= set(self.names) or not goal.trajectory.points:
            self.get_logger().warn(f'거절: 모르는 관절 {names}')
            return GoalResponse.REJECT
        if self.exec_lock.locked():
            self.get_logger().warn('거절: 이미 다른 궤적 실행 중')
            return GoalResponse.REJECT
        self.stop_event.clear()
        return GoalResponse.ACCEPT

    def execute(self, gh):
        """궤적 스트리밍: 50 Hz 로 '지금+lookahead' 시점의 보간 위치를 목표로 보냄.
        점마다 목표를 주면 서보가 매 점에서 감속-정지-재가속 → 천천히 움직일 때 달달 떨림. 그래서 바꿈."""
        with self.exec_lock:
            traj = gh.request.trajectory
            idx = [self.names.index(nm) for nm in traj.joint_names]
            result = FollowJointTrajectory.Result()
            if not self.feedback_fresh():
                gh.abort();result.error_code=FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED;return result
            if not self.torque_on:
                self.set_torque(True)
            # 시간표 T[k], 관절별 목표 Q[j][k] (rad). 첫 점 = 지금 위치(t=0)
            T = [0.0]
            Q = [[self.c2r(i, self.pos[i])] for i in idx]
            V = [[None] for i in idx]
            for pt in traj.points:
                t_pt = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                if t_pt <= T[-1] + 1e-4:                      # 같은 시각 점(t=0 시작점 등)은 덮어씀
                    for j in range(len(idx)):
                        Q[j][-1] = pt.positions[j]
                        V[j][-1] = pt.velocities[j] if len(pt.velocities)==len(idx) else None
                    continue
                T.append(t_pt)
                for j in range(len(idx)):
                    Q[j].append(pt.positions[j])
                    V[j].append(pt.velocities[j] if len(pt.velocities)==len(idx) else None)
            t_end = T[-1]

            def sample(t):
                t = min(max(t, 0.0), t_end)
                k = 0
                while k < len(T) - 2 and T[k + 1] < t:
                    k += 1
                if len(T) == 1:
                    return [q[0] for q in Q], [0.0] * len(idx)
                a = (t - T[k]) / max(T[k + 1] - T[k], 1e-6)
                pos = [q[k] + (q[k + 1] - q[k]) * a for q in Q]
                slopes = [(q[k + 1] - q[k]) / max(T[k + 1] - T[k], 1e-6) for q in Q]
                vel = [planned_speed(T,V[j],k,a,slopes[j]) for j in range(len(idx))]
                return pos, vel

            clamped = False
            period = 1.0 / self.stream_hz
            t0 = time.monotonic()
            n = 0
            while True:
                if gh.is_cancel_requested or self.stop_event.is_set():
                    self.hold_here()
                    if gh.is_cancel_requested:gh.canceled()
                    else:gh.abort()
                    result.error_code = 0
                    return result
                if not self.feedback_fresh():
                    self.get_logger().error('엔코더 통신 유실: 궤적 중단')
                    gh.abort();result.error_code=FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED;return result
                t = time.monotonic() - t0
                done = t >= t_end
                ta = t_end if done else t + self.lookahead
                pos, vel = sample(ta)
                goals = {}
                for j, i in enumerate(idx):
                    c, cl = self.r2c(i, pos[j])
                    clamped |= cl
                    spd = abs(vel[j]) / C2R * 1.05                # 궤적 속도 그대로 (count/s)
                    if done:
                        spd = max(spd, self.settle_speed)
                    goals[self.ids[i]] = (c, int(min(max(spd, max(1,self.vmin)), self.vmax)))   # 0 은 '최고속'이라 금지
                self.bus.sync_goal(goals, self.acc)
                if n % 5 == 0 or done:
                    fb = FollowJointTrajectory.Feedback()
                    fb.joint_names = [self.names[i] for i in idx]
                    fb.actual.positions = [self.c2r(i, self.pos[i]) for i in idx]
                    fb.desired.positions = sample(t)[0]
                    gh.publish_feedback(fb)
                n += 1
                if done:
                    break
                time.sleep(max(0.0, t0 + n * period - time.monotonic()))
            final = [self.r2c(i, Q[j][-1])[0] for j, i in enumerate(idx)]
            if clamped:
                self.get_logger().warn('일부 목표가 관절 범위 밖이라 잘림 (clamp)')
            t_fin = time.monotonic()
            while time.monotonic() - t_fin < self.settle:         # 도착 확인
                if gh.is_cancel_requested or self.stop_event.is_set():
                    self.hold_here()
                    if gh.is_cancel_requested:gh.canceled()
                    else:gh.abort()
                    result.error_code=0;return result
                if not self.feedback_fresh():
                    gh.abort();result.error_code=FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED;return result
                if all(abs(self.pos[i] - final[j]) <= self.tol for j, i in enumerate(idx)):
                    if not self.fine_correct(idx, final, gh):
                        result.error_code=0;return result
                    gh.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result
                time.sleep(0.02)
            gh.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = '목표 위치 도착 못 함 (걸림/과부하/전원 확인)'
            self.get_logger().error(result.error_string)
            return result

    def wait_still(self, gh, timeout=1.5, window=0.25):
        """엔코더가 window초 동안 안 변하면 True. 취소/정지면 False."""
        end = time.monotonic() + timeout; last = list(self.pos); since = time.monotonic()
        while time.monotonic() < end:
            if gh.is_cancel_requested or self.stop_event.is_set() or not self.feedback_fresh():return False
            time.sleep(0.03)
            if self.pos != last:last = list(self.pos); since = time.monotonic()
            elif time.monotonic() - since >= window:return True
        return True

    def fine_correct(self, idx, final, gh):
        """도착 후 데드밴드로 모자라게 선 관절에 '목표 + 모자란 만큼'을 다시 보내 목표에 맞춘다.
        취소/정지면 hold 후 False (gh 상태는 여기서 정함)."""
        cmd = list(final)
        for _ in range(self.get_parameter('fine_correction_iterations').value):  # 실행 중 ros2 param set 으로 바꿀 수 있음
            if not self.wait_still(gh):break
            goals = fine_goals(final, [self.pos[i] for i in idx], cmd, [self.cmin[i] for i in idx], [self.cmax[i] for i in idx], self.fine_tol, self.fine_max)
            if goals is None:return True
            cmd = goals
            self.bus.sync_goal({self.ids[i]: (cmd[j], self.settle_speed) for j, i in enumerate(idx)}, self.acc)
        else:
            self.wait_still(gh);return True
        if gh.is_cancel_requested or self.stop_event.is_set():
            self.hold_here()
            if gh.is_cancel_requested:gh.canceled()
            else:gh.abort()
            return False
        return True

    def hold_here(self):
        if self.read_only:return
        if not self.feedback_fresh():return
        self.bus.sync_goal({sid: (self.pos[i], self.vmin) for i, sid in enumerate(self.ids)}, self.acc)

    # ---------------- 턴테이블 (STS3215, ID = turntable_id)
    def on_tt_goto(self, msg):
        if self.read_only:return
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
