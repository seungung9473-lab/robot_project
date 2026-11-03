"""컵이 아래를 보는 자세로 테이블 좌표 목표 위에 가는 관절각 계산 (처짐 보정 포함). 계산만, ROS/모터 없음.

workcell.yaml의 로봇 배치(T_table_world)와 robot_registration.sag_compensation.tool_tilt_away_from_j1_deg를 쓴다.
처짐 보정: 컵 끝이 로봇 쪽으로 처지는 만큼 컵 축을 J1 반대쪽으로 미리 기울여 목표를 잡는다 (11/3 실측 3°).

사용:
    python3 -m soarm4_workcell.cup_goal                         # 턴테이블·패드 위 5 mm
    python3 -m soarm4_workcell.cup_goal --clearance 0.03        # 3 cm 위
    python3 -m soarm4_workcell.cup_goal --xyz 1.20 0.50 0.06    # 임의 점 (테이블 m, z = 면 높이)
결과 관절각(°)은 RViz Joints 또는 plan_joint_goal.py --goal-deg 로 계획·실행한다.
"""
import argparse, json, math
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from .core import load, matrix

CONFIG = Path(__file__).resolve().parents[1] / 'config/workcell.yaml'
NAMES = ['joint1', 'joint2', 'joint3', 'joint4']
HOME = np.array([0.0, math.pi, -math.pi, 0.0])
J1_WORLD = np.array([-0.0177, 0.0177, 0.1598])   # URDF joint1 축 위치 (world)


def urdf_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('soarm4_description')) / 'urdf/soarm4.urdf'
    except Exception:
        return Path(__file__).resolve().parents[2] / 'soarm4_description/urdf/soarm4.urdf'


def _origin(o):
    x = [float(v) for v in (o.get('xyz') or '0 0 0').split()]; a, b, c = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
    Rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    Ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    Rz = np.array([[math.cos(c), -math.sin(c), 0], [math.sin(c), math.cos(c), 0], [0, 0, 1]])
    M = np.eye(4); M[:3, :3] = Rz @ Ry @ Rx; M[:3, 3] = x; return M


def _axis_angle(axis, q):
    k = np.asarray(axis, float); K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    M = np.eye(4); M[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K; return M


class Arm:
    """URDF world → tool_link 정기구학."""
    def __init__(self, urdf=None):
        root = ET.parse(str(urdf or urdf_path())).getroot(); by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.chain = []; link = 'tool_link'
        while link in by_child:j = by_child[link]; self.chain.append(j); link = j.find('parent').get('link')
        self.chain.reverse()
        lim = {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper'))) for j in root.findall('joint') if j.get('type') == 'revolute'}
        self.lo = np.array([lim[n][0] for n in NAMES]); self.hi = np.array([lim[n][1] for n in NAMES])

    def fk(self, q):
        M = np.eye(4); qd = dict(zip(NAMES, q))
        for j in self.chain:
            M = M @ _origin(j.find('origin'))
            if j.get('type') == 'revolute':M = M @ _axis_angle([float(v) for v in j.find('axis').get('xyz').split()], qd[j.get('name')])
        return M


def tcp_point(wc):
    m = json.loads((urdf_path().parents[1] / 'config/model.json').read_text()); return np.r_[m['tcp_xyz'], 1.0]


def solve(arm, wc, target_table, tilt_deg=None, tcp=None, seeds=60):
    """컵 끝을 target_table(m)에 두고 컵 축을 아래(+J1 반대쪽으로 tilt_deg)로 하는 관절각(rad). 해가 없으면 None. HOME에 가까운 해를 고른다."""
    T = matrix(wc['T_table_world']); Ti = np.linalg.inv(T); tcp = tcp_point(wc) if tcp is None else tcp
    if tilt_deg is None:tilt_deg = wc['robot_registration'].get('sag_compensation', {}).get('tool_tilt_away_from_j1_deg', 0.0)
    j1 = (T @ np.r_[J1_WORLD, 1])[:2]; u = np.asarray(target_table[:2], float) - j1; u = u / np.linalg.norm(u); t = math.radians(tilt_deg)
    axis_w = T[:3, :3].T @ np.r_[math.sin(t) * u, -math.cos(t)]; goal = (Ti @ np.r_[target_table, 1])[:3]; best = None
    for seed in np.random.default_rng(0).uniform(arm.lo, arm.hi, (seeds, 4)):
        f = lambda q: np.r_[(arm.fk(q) @ tcp)[:3] - goal, 0.05 * (arm.fk(q)[:3, :3] @ [1, 0, 0] - axis_w)]
        s = least_squares(f, seed, bounds=(arm.lo, arm.hi))
        if s.cost < 1e-10 and (best is None or np.abs(s.x - HOME).sum() < np.abs(best - HOME).sum()):best = s.x
    return best


def named_targets(wc):
    out = {'turntable': list(wc['turntable']['axis_point_table_m'])}
    for p in wc['pads'].values():out[f"pad_ID{p['marker_id']}"] = list(p['center_table_m'])
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description='컵 아래 자세 목표 관절각 (처짐 보정 포함, 계산만)')
    ap.add_argument('--workcell', default=str(CONFIG)); ap.add_argument('--clearance', type=float, default=0.005, help='면 위 높이 (m)')
    ap.add_argument('--xyz', type=float, nargs=3, help='임의 목표 면 (테이블 m)'); ap.add_argument('--tilt', type=float, help='처짐 보정 각 (기본: workcell 값)')
    ap.add_argument('--json'); a = ap.parse_args(argv)
    wc = load(Path(a.workcell)); arm = Arm(); targets = {'point': a.xyz} if a.xyz else named_targets(wc); out = {}
    for name, p in targets.items():
        q = solve(arm, wc, np.array([p[0], p[1], p[2] + a.clearance]), a.tilt)
        out[name] = None if q is None else np.degrees(q).round(2).tolist()
        print(f'{name}: 면 {[round(v, 4) for v in p]} + {a.clearance} m → 관절(°) {out[name] if q is not None else "해 없음"}')
    if a.json:Path(a.json).write_text(json.dumps(out, indent=1))
    return 0


if __name__ == '__main__':raise SystemExit(main())
