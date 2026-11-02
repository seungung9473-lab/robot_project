from soarm_turntable.scan_core import stability_problem, stable_states


def st(**kw):
    s = {'hardware_error': 0, 'moving': False, 'velocity_raw': 0, 'position_count': 1000}
    s.update(kw)
    return s


def run(n=8, dt=.1, **kw):
    return [(i * dt, st(**kw)) for i in range(n)]


def test_still_is_stable():
    states = run()
    assert stable_states(states, states[-1][0] + .05)


def test_slow_poll_is_sample_problem_not_motion():
    # 표본 구간이 .3s로 짧음(통신 지연): 움직임 증거가 아니므로 'sample'
    states = run(n=3, dt=.15)
    kind, _ = stability_problem(states, states[-1][0] + .05)
    assert kind == 'sample'


def test_stale_state_is_sample_problem():
    states = run()
    kind, _ = stability_problem(states, states[-1][0] + .5)
    assert kind == 'sample'


def test_real_motion_is_reported():
    for kw in ({'moving': True}, {'velocity_raw': 5}, {'hardware_error': 1}):
        states = run(**kw)
        kind, reason = stability_problem(states, states[-1][0] + .05)
        assert kind == 'motion' and reason


def test_position_drift_is_motion():
    states = run()
    states[-1] = (states[-1][0], st(position_count=1005))
    kind, _ = stability_problem(states, states[-1][0] + .05)
    assert kind == 'motion'
