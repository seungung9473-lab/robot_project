"""집기·옮기기 실험 기록. 시험 한 번(trial)마다 단계(stage)별 측정값과 그때의 설정을 JSONL로 쌓고, 단계별 통계를 낸다.
ROS·모터·펌프 접근 없음 (기록만).

저장: <root>/<실험이름>/trials.jsonl  (한 줄 = 한 번의 시험 기록), 요약은 summary.csv로 내보냄
단계(STAGES)는 EXPERIMENTS_KO.md 참고.

사용:
    from soarm4_workcell.experiment import ExperimentLog
    log = ExperimentLog('pickplace')                      # 기본 ~/robot_ws/data/experiments/pickplace
    t = log.new_trial(note='턴테이블 → ID5')              # 설정 스냅샷(J1 영점, gain, 기준 등)이 자동으로 붙음
    log.add(t, 'S5_place', target_xy_m=[1.251, 0.367], placed_xy_m=[1.2512, 0.3731])   # err_mm 자동 계산
    python3 -m soarm4_workcell.experiment summary ~/robot_ws/data/experiments/pickplace
"""
import argparse, csv, datetime, hashlib, json, math, subprocess
from pathlib import Path
import numpy as np
import yaml

ROOT = Path.home() / 'robot_ws/data/experiments'
REPO = Path(__file__).resolve().parents[3]
STAGES = {
    'S1_pose': '컵 위치 정확도 (빈 컵을 목표 위 5 mm에 세우고 실측)',
    'S2_recon': '스캔·복원 (Gemini 박스 → SAM2 마스크 → 점군 → 흡착면 선택)',
    'S3_grip': '흡착 (컵을 닿게 한 뒤 펌프 ON, 압력)',
    'S4_carry': '운반 (들어 올려 옮기는 동안 압력 유지)',
    'S5_place': '내려놓기 (카메라로 잰 놓인 위치 오차)',
    'S6_cycle': '전체 한 사이클 (집기→옮기기→놓기→HOME)',
}


def config_snapshot(repo=REPO):
    """결과에 영향을 주는 설정: J1 영점, 보정 gain, 흡착 기준, 처짐 각, 카메라 보정 출처 등."""
    snap = {}
    try:
        b = yaml.safe_load((repo / 'src/soarm_bridge/config/soarm.yaml').read_text())
        s = b['servo_bridge']['ros__parameters']; v = b['esp32_bridge']['ros__parameters']
        snap.update(j1_center=s['centers'][0], positive_gain=s.get('positive_gain'), fine_correction_iterations=s.get('fine_correction_iterations', 3),
                    grip_diff=v['grip_diff'], hold_diff=v['hold_diff'], release_diff=v['release_diff'])
        w = yaml.safe_load((repo / 'src/soarm4_workcell/config/workcell.yaml').read_text()); r = w['robot_registration']
        snap.update(sag_tilt_deg=r.get('sag_compensation', {}).get('tool_tilt_away_from_j1_deg'), xy_correction_m=r.get('xy_contact_correction_m'),
                    cam_sources={k: Path(c.get('source', '')).parent.name for k, c in w['cameras'].items()},
                    workcell_sha=hashlib.sha256((repo / 'src/soarm4_workcell/config/workcell.yaml').read_bytes()).hexdigest()[:10])
    except Exception as e:
        snap['config_error'] = str(e)[:100]
    try:
        snap['git'] = subprocess.run(['git', '-C', str(repo), 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        pass
    return snap


def xy_error_mm(target_xy, actual_xy):
    d = (np.asarray(actual_xy, float) - np.asarray(target_xy, float)) * 1e3
    return {'dx_mm': round(float(d[0]), 2), 'dy_mm': round(float(d[1]), 2), 'err_mm': round(float(math.hypot(*d)), 2)}


class ExperimentLog:
    def __init__(self, name, root=ROOT, snapshot=config_snapshot):
        self.dir = Path(root) / name; self.dir.mkdir(parents=True, exist_ok=True); self.path = self.dir / 'trials.jsonl'; self.snapshot = snapshot

    def records(self):
        return [json.loads(l) for l in self.path.read_text().splitlines() if l.strip()] if self.path.exists() else []

    def new_trial(self, note='', **extra):
        """새 시험 번호를 만들고 설정 스냅샷을 첫 기록으로 남긴다. 번호(trial id)를 돌려준다."""
        ids = [r['trial'] for r in self.records()]; t = (max(ids) + 1) if ids else 1
        self._write({'trial': t, 'stage': 'config', 'note': note, 'config': self.snapshot(), **extra}); return t

    def add(self, trial, stage, **fields):
        """단계 기록 추가. target_xy_m과 placed_xy_m(또는 actual_xy_m)이 있으면 오차(mm)를 계산해서 넣는다."""
        if stage not in STAGES: raise ValueError(f'알 수 없는 단계 {stage}; 가능: {list(STAGES)}')
        a = fields.get('placed_xy_m', fields.get('actual_xy_m'))
        if 'target_xy_m' in fields and a is not None: fields = {**xy_error_mm(fields['target_xy_m'], a), **fields}
        self._write({'trial': trial, 'stage': stage, **fields})

    def _write(self, rec):
        rec = {'time': datetime.datetime.now().isoformat(timespec='seconds'), **rec}
        with open(self.path, 'a') as f: f.write(json.dumps(rec, ensure_ascii=False) + '\n')

    def summary(self):
        """단계별 시험 수와 숫자 항목의 평균·표준편차·최대. 시험 수가 1이면 표준편차는 None."""
        out = {}
        for stage in STAGES:
            rows = [r for r in self.records() if r['stage'] == stage]
            if not rows: continue
            keys = sorted({k for r in rows for k, v in r.items() if isinstance(v, (int, float)) and not isinstance(v, bool) and k not in ('trial',)})
            stats = {}
            for k in keys:
                v = np.array([r[k] for r in rows if isinstance(r.get(k), (int, float))], float)
                stats[k] = {'n': int(len(v)), 'mean': round(float(v.mean()), 2), 'std': round(float(v.std(ddof=1)), 2) if len(v) > 1 else None, 'max_abs': round(float(np.abs(v).max()), 2)}
            out[stage] = {'trials': sorted({r['trial'] for r in rows}), 'stats': stats}
        return out

    def export_csv(self):
        """평평하게 펴서 summary.csv로 저장 (엑셀로 열기 좋게). 반환: 경로."""
        rows = [r for r in self.records() if r['stage'] != 'config']; cols = ['trial', 'stage', 'time']
        for r in rows:
            for k, v in r.items():
                if k not in cols and not isinstance(v, (dict, list)): cols.append(k)
        p = self.dir / 'summary.csv'
        with open(p, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore'); w.writeheader(); [w.writerow(r) for r in rows]
        return p


def main(argv=None):
    ap = argparse.ArgumentParser(description='실험 기록 요약'); ap.add_argument('cmd', choices=['summary', 'csv', 'stages']); ap.add_argument('dir', nargs='?', default=str(ROOT / 'pickplace'))
    a = ap.parse_args(argv)
    if a.cmd == 'stages':
        for k, v in STAGES.items(): print(f'{k}: {v}')
        return 0
    log = ExperimentLog(Path(a.dir).name, root=Path(a.dir).parent)
    if a.cmd == 'csv': print(log.export_csv()); return 0
    s = log.summary(); print(f'시험 {len({r["trial"] for r in log.records()})}번, 기록 {len(log.records())}줄')
    for stage, d in s.items():
        print(f'\n[{stage}] {STAGES[stage]}  (시험 {d["trials"]})')
        for k, st in d['stats'].items(): print(f'  {k}: n={st["n"]} 평균 {st["mean"]} 표준편차 {st["std"]} 최대|값| {st["max_abs"]}')
    return 0


if __name__ == '__main__': raise SystemExit(main())
