"""블록 하나를 집어서 옮겨 놓는 한 사이클을 한 프로세스로 실행한다 (실물 로봇을 움직임: 펌프·모터).
- 속도·가속도 배율 인자(--scale)
- 3 cm 위 → 바로 접촉으로 내려가는 등 중간 정지 줄임 (이동 6번)
- 검증된 경로를 저장해 두고 같은 구간·같은 장면이면 계획·세부 검사를 건너뜀 (~/robot_ws/data/plan_cache)
- 운반 중 압력 감시: diff가 -200보다 약해지면 /servo/stop
- 단계별 결과를 실험 기록(ExperimentLog)에 남김
사용 전 필요: live.launch.py(allow_execution:=true) + 진공 브리지 + 카메라 2대 + scene.launch.py. 실행 전에 사용자 확인을 받을 것.

    python3 -m soarm4_workcell.pickplace_cycle --place pad_ID5 --scale 0.2 --log speed
"""
import argparse, hashlib, json, math, threading, time
from pathlib import Path
import numpy as np
import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import ApplyPlanningScene
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from .core import load, matrix
from .cup_goal import Arm, solve
from .block_measure import measure_now
from .experiment import ExperimentLog

CONFIG = Path(__file__).resolve().parents[1] / 'config/workcell.yaml'
NAMES = ['joint1', 'joint2', 'joint3', 'joint4']
CACHE = Path.home() / 'robot_ws/data/plan_cache'
HOLD = -300; LOST = -200


class Cycle(Node):
    def __init__(self, scale, cache_dir=CACHE, use_cache=True):
        super().__init__('pickplace_cycle'); self.scale = scale; self.use_cache = use_cache; self.cache_dir = Path(cache_dir); self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.wc = load(CONFIG); self.arm = Arm(); self.q = None; self.diff = None; self.holding = False; self.carrying = False; self.lost = False; self.scene = 'free'; self.timing = []
        self.create_subscription(JointState, '/joint_states', self._js, 10); self.create_subscription(Int32, '/vacuum/diff', self._diff, 10); self.create_subscription(Bool, '/vacuum/holding', lambda m: setattr(self, 'holding', m.data), 10)
        self.mg = ActionClient(self, MoveGroup, '/move_action'); self.ex = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.scene_srv = self.create_client(ApplyPlanningScene, '/apply_planning_scene'); self.srv = {k: self.create_client(Trigger, v) for k, v in
                                                                                                        {'grip': '/vacuum/grip', 'release': '/vacuum/release', 'off': '/vacuum/off', 'stop': '/servo/stop'}.items()}
        self.spin_thread = threading.Thread(target=lambda: rclpy.spin(self), daemon=True); self.spin_thread.start()
        # 연결(발견)이 끝나기 전에 부르면 첫 호출이 시간 초과로 실패할 수 있어서, 시작할 때 전부 기다린다
        for name, c in (('MoveGroup(/move_action)', self.mg), ('ExecuteTrajectory(/execute_trajectory)', self.ex)):
            if not c.wait_for_server(timeout_sec=20): raise RuntimeError(name + ' 연결 안 됨')
        for name, c in [('/apply_planning_scene', self.scene_srv)] + [(k, v) for k, v in self.srv.items()]:
            if not c.wait_for_service(timeout_sec=20): raise RuntimeError(str(name) + ' 서비스 연결 안 됨')
        t = time.time()
        while (self.q is None or self.diff is None) and time.time() - t < 10: time.sleep(0.05)
        if self.q is None or self.diff is None: raise RuntimeError('/joint_states 또는 /vacuum/diff 신호가 없습니다')

    def _js(self, m): d = dict(zip(m.name, m.position)); self.q = [math.degrees(d[n]) for n in NAMES]

    def _diff(self, m):
        self.diff = m.data
        if self.carrying and m.data > LOST and not self.lost: self.lost = True; self.get_logger().error(f'흡착 풀림 감지 diff={m.data} → 팔 정지'); self.srv['stop'].call_async(Trigger.Request())

    def wait(self, fut, timeout, what=''):
        t = time.time()
        while not fut.done():
            if time.time() - t > timeout: raise RuntimeError('응답 시간 초과' + (' (' + what + ')' if what else ''))
            time.sleep(0.01)
        return fut.result()

    # ---------- 장면
    def apply(self, scene):
        r = ApplyPlanningScene.Request(); r.scene = scene; r.scene.is_diff = True; return self.wait(self.scene_srv.call_async(r), 15, '장면 갱신').success

    def block_box(self, on, xy=None, ztop=None, side=0.074):
        o = CollisionObject(); o.id = 'scanned_block'; o.header.frame_id = 'world'
        if on:
            top = self.wc['turntable']['top_height_m']; h = ztop - top + 0.004; c = (matrix(self.wc['T_table_world']) @ np.r_[xy[0], xy[1], top + h / 2, 1])[:3]
            b = SolidPrimitive(); b.type = b.BOX; b.dimensions = [side, side, h]; p = Pose(); p.orientation.w = 1.0; p.position.x, p.position.y, p.position.z = map(float, c); o.primitives = [b]; o.primitive_poses = [p]; o.operation = CollisionObject.ADD
        else: o.operation = CollisionObject.REMOVE
        from moveit_msgs.msg import PlanningScene
        s = PlanningScene(); s.world.collision_objects = [o]; return self.apply(s)

    def attach(self, on, length=0.0608, width=0.075):
        from moveit_msgs.msg import PlanningScene
        a = AttachedCollisionObject(); a.link_name = 'tool_link'; a.touch_links = ['tool_link', 'tool_link_print_visual_link', 'tool_link_alu_visual_link', 'tool_link_cup_visual_link']
        o = a.object; o.id = 'held_block'; o.header.frame_id = 'tool_link'
        if on:
            b = SolidPrimitive(); b.type = b.BOX; b.dimensions = [length, width, width]; p = Pose(); p.orientation.w = 1.0; p.position.x = 0.1157 + length / 2 - 0.002; o.primitives = [b]; o.primitive_poses = [p]; o.operation = CollisionObject.ADD
        else: o.operation = CollisionObject.REMOVE
        s = PlanningScene(); s.robot_state.is_diff = True; s.robot_state.attached_collision_objects = [a]; ok = self.apply(s)
        if not on:
            w = CollisionObject(); w.id = 'held_block'; w.header.frame_id = 'world'; w.operation = CollisionObject.REMOVE; s2 = PlanningScene(); s2.world.collision_objects = [w]; self.apply(s2)
        return ok

    # ---------- 이동 (계획 저장·재사용)
    def _key(self, goal):
        return hashlib.sha1(json.dumps([[round(g, 2) for g in goal], round(self.scale, 3), self.scene]).encode()).hexdigest()[:16]

    def _cached(self, goal):
        p = self.cache_dir / (self._key(goal) + '.json')
        if not (self.use_cache and p.exists()): return None
        d = json.loads(p.read_text())
        if max(abs(a - b) for a, b in zip(d['start'], self.q)) > 0.5: return None
        tr = ExecuteTrajectory.Goal().trajectory; set_message_fields(tr, d['trajectory']); return tr

    def move(self, goal, label):
        t0 = time.time(); tr = self._cached(goal); cached = tr is not None; t_plan = 0.0
        if tr is None:
            g = MoveGroup.Goal(); r = g.request; r.group_name = 'arm'; r.pipeline_id = 'ompl'; r.planner_id = 'RRTConnectkConfigDefault'; r.num_planning_attempts = 5; r.allowed_planning_time = 10.0
            r.max_velocity_scaling_factor = self.scale; r.max_acceleration_scaling_factor = self.scale; r.start_state.is_diff = True; cs = Constraints()
            for n, v in zip(NAMES, goal): cs.joint_constraints.append(JointConstraint(joint_name=n, position=math.radians(v), tolerance_above=0.002, tolerance_below=0.002, weight=1.0))
            r.goal_constraints = [cs]; g.planning_options.plan_only = True; start = list(self.q)
            h = self.wait(self.mg.send_goal_async(g), 20, label + ' 계획 요청')
            if not h.accepted: raise RuntimeError(label + ': 계획 요청 거부')
            res = self.wait(h.get_result_async(), 60, label + ' 계획 결과').result
            if res.error_code.val != MoveItErrorCodes.SUCCESS: raise RuntimeError(f'{label}: 계획 실패 (코드 {res.error_code.val})')
            tr = res.planned_trajectory; t_plan = time.time() - t0
            if self.use_cache: (self.cache_dir / (self._key(goal) + '.json')).write_text(json.dumps({'start': start, 'goal': goal, 'scale': self.scale, 'scene': self.scene, 'trajectory': message_to_ordereddict(tr)}))
        t1 = time.time(); e = ExecuteTrajectory.Goal(); e.trajectory = tr; h = self.wait(self.ex.send_goal_async(e), 20, label + ' 실행 요청')
        if not h.accepted: raise RuntimeError(label + ': 실행 요청 거부')
        er = self.wait(h.get_result_async(), 120, label + ' 실행 결과').result
        if er.error_code.val != MoveItErrorCodes.SUCCESS: raise RuntimeError(f'{label}: 실행 실패 (코드 {er.error_code.val})')
        self.timing.append({'move': label, 'cached': cached, 'plan_s': round(t_plan, 1), 'exec_s': round(time.time() - t1, 1)}); return self.timing[-1]

    def goal_for(self, xyz, clearance):
        q = solve(self.arm, self.wc, np.array([xyz[0], xyz[1], xyz[2] + clearance]))
        if q is None: raise RuntimeError(f'관절 해 없음 {xyz} +{clearance}')
        return [float(v) for v in np.degrees(q)]

    def vac(self, name, timeout=10):
        return self.wait(self.srv[name].call_async(Trigger.Request()), timeout)

    def run(self, pick_name, place_name, log=None, note='', block_h=None):
        wc = self.wc; top = wc['turntable']['top_height_m']; T0 = time.time()
        targets = {'turntable': wc['turntable']['axis_point_table_m'][:2], 'pad_ID5': wc['pads']['pad2']['center_table_m'][:2], 'pad_ID6': wc['pads']['pad1']['center_table_m'][:2]}
        trial = log.new_trial(note=note or f'{pick_name} → {place_name}, 배율 {self.scale}', scale=self.scale, pick=pick_name, place=place_name) if log else None
        m0 = measure_now(str(Path.home() / 'robot_ws/data/block_measure_tmp'))
        if not m0: raise RuntimeError('블록을 못 찾음')
        pick = [m0['xy_m'][0], m0['xy_m'][1], m0['z_m']]
        if block_h:  # 스캔으로 정한 블록 높이를 쓰고, 집는 면 높이도 (놓인 스테이션 높이 + 블록 높이)로 고정 → 한 장 측정의 높이 오차 제거, 계획 재사용 가능
            st = {'turntable': (wc['turntable']['axis_point_table_m'][:2], top), 'pad_ID5': (wc['pads']['pad2']['center_table_m'][:2], wc['pads']['pad2']['center_table_m'][2]), 'pad_ID6': (wc['pads']['pad1']['center_table_m'][:2], wc['pads']['pad1']['center_table_m'][2])}
            near = min(st, key=lambda k: np.hypot(pick[0] - st[k][0][0], pick[1] - st[k][0][1])); pick[2] = st[near][1] + block_h; pick_name = near
        h = pick[2] - top; print(f'블록 측정 {pick}, 높이 {h*1000:.1f} mm, 카메라 차이 {m0.get("cam_gap_mm")} mm', flush=True)
        if log: log.add(trial, 'S2_recon', source='red-threshold+depth', pickup_xyz_m=pick, cam_gap_mm=m0.get('cam_gap_mm'), block_height_mm=round(h * 1000, 1), block_yaw_deg=m0['yaw_deg'])
        try:
            self.scene = 'block_on_surface'; self.block_box(True, pick[:2], pick[2]); self.move(self.goal_for(pick, 0.03), '집기 위치 3cm 위로')
            self.block_box(False); self.scene = 'free'; self.move(self.goal_for(pick, 0.0), '윗면에 닿게 내리기')
            g = self.vac('grip', 12); ok = g.success
            import re; mm = re.search(r'diff=(-?\d+)', g.message); first = int(mm.group(1)) if mm else self.diff  # 서비스가 판정한 순간의 값
            if log: log.add(trial, 'S3_grip', first_diff=first, success=ok, message=g.message)
            if not ok: raise RuntimeError('흡착 실패 ' + g.message)
            self.carrying = True; self.move(self.goal_for(pick, 0.03), '3cm 들어올리기')
            self.scene = 'carry'; self.attach(True); pt = targets[place_name]; corr = wc.get('place_correction_m', {}).get(place_name, [0.0, 0.0]); place_top = [pt[0] + corr[0], pt[1] + corr[1], top + h]; dmin = []  # 목표(pt)는 그대로, 보내는 위치에만 보정
            self.move(self.goal_for(place_top, 0.03), '놓을 곳 위로 운반'); dmin.append(self.diff)
            self.attach(False); self.scene = 'free'; self.move(self.goal_for(place_top, 0.001), '내려놓기'); dmin.append(self.diff)
            if log: log.add(trial, 'S4_carry', diff_at_place=self.diff, diff_after_carry=dmin[0], lost_grip=self.lost)
            self.carrying = False; self.vac('release', 12); time.sleep(0.5); self.vac('off', 5); released = self.diff
            self.move(self.goal_for(place_top, 0.03), '3cm 올리기')
        except Exception:
            print('!! 중단:', 'carrying' if self.carrying else '', flush=True); raise
        m1 = measure_now(str(Path.home() / 'robot_ws/data/block_measure_tmp')); total = round(time.time() - T0, 1)
        moves = sum(t['plan_s'] + t['exec_s'] for t in self.timing)
        if log:
            log.add(trial, 'S5_place', correction_m=[round(float(c), 5) for c in corr], target_xy_m=[round(float(pt[0]), 5), round(float(pt[1]), 5)], placed_xy_m=m1['xy_m'] if m1 else None, cam_gap_mm=(m1 or {}).get('cam_gap_mm'), block_yaw_deg=(m1 or {}).get('yaw_deg'), released_diff=released)
            log.add(trial, 'S6_cycle', success=bool(m1) and not self.lost, total_s=total, moves_s=round(moves, 1), n_moves=len(self.timing), cached_moves=sum(t['cached'] for t in self.timing), timing=self.timing, scale=self.scale)
        return {'trial': trial, 'total_s': total, 'moves_s': round(moves, 1), 'placed': m1, 'target': [float(pt[0]), float(pt[1])], 'timing': self.timing}


def main(argv=None):
    ap = argparse.ArgumentParser(description='집기·옮기기 한 사이클 (실물 로봇이 움직임)')
    ap.add_argument('--pick', default='auto', help='auto = 카메라로 지금 블록 위치를 찾음 (이름은 기록용)'); ap.add_argument('--place', required=True, choices=['turntable', 'pad_ID5', 'pad_ID6'])
    ap.add_argument('--scale', type=float, default=0.1); ap.add_argument('--log', default='speed'); ap.add_argument('--no-cache', action='store_true'); ap.add_argument('--block-h-mm', type=float, default=None, help='블록 높이(mm). 주면 측정 높이 대신 이 값을 씀'); ap.add_argument('--note', default='')
    a = ap.parse_args(argv)
    if not 0.05 <= a.scale <= 0.6: ap.error('--scale 은 0.05~0.6')
    rclpy.init(); n = Cycle(a.scale, use_cache=not a.no_cache)
    try:
        r = n.run(a.pick, a.place, ExperimentLog(a.log), a.note, block_h=(a.block_h_mm / 1000 if a.block_h_mm else None)); print(json.dumps({k: v for k, v in r.items() if k != 'timing'}, ensure_ascii=False)); [print(' ', t) for t in r['timing']]
    finally:
        rclpy.try_shutdown(); n.spin_thread.join(timeout=3); n.destroy_node()


if __name__ == '__main__': main()
