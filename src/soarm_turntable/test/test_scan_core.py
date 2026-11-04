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


def test_capture_gate_requires_full_extra_stop_interval():
    from soarm_turntable.scan_core import StationaryCaptureGate
    gate=StationaryCaptureGate(1.)
    assert not gate.update(None,10.)
    assert not gate.update(None,10.99)
    assert gate.update(None,11.)


def test_capture_gate_restarts_dwell_after_status_gap():
    from soarm_turntable.scan_core import StationaryCaptureGate
    gate=StationaryCaptureGate(1.)
    gate.update(None,10.)
    assert not gate.update(('sample','late telemetry'),10.8)
    assert not gate.update(None,11.)
    assert not gate.update(None,11.99)
    assert gate.update(None,12.)


def test_capture_gate_restarts_dwell_after_residual_motion():
    from soarm_turntable.scan_core import StationaryCaptureGate
    gate=StationaryCaptureGate(.5);gate.update(None,10.);assert gate.update(None,10.5)
    assert not gate.update(('motion','velocity raw 2'),10.6)
    assert not gate.update(None,10.7)
    assert not gate.update(None,11.19)
    assert gate.update(None,11.2)


def test_capture_gate_rejects_invalid_delay():
    import pytest
    from soarm_turntable.scan_core import StationaryCaptureGate
    for value in (0.,-.1,float('nan'),float('inf'),11.):
        with pytest.raises(ValueError):StationaryCaptureGate(value)


def test_capture_rejects_old_exposures_even_if_delivered_after_stop():
    from collections import deque
    from types import SimpleNamespace
    from soarm_turntable.scan_core import select_bundle
    def msg(ns):return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=ns//10**9,nanosec=ns%10**9)))
    keys=['cam1_color','cam1_depth','cam1_info','cam2_color','cam2_depth','cam2_info']
    buffers={k:deque([(10.1,msg(9_900_000_000))]) for k in keys}
    assert select_bundle(buffers,10_000_000_000,10.2,received_after=10.) is None
    for k in keys:buffers[k].append((10.15,msg(10_100_000_000)))
    bundle=select_bundle(buffers,10_000_000_000,10.2,received_after=10.)
    assert bundle and all(t==10.15 for t,_ in bundle.values())


def test_capture_requires_receipt_after_buffer_flush():
    from types import SimpleNamespace
    from soarm_turntable.scan_core import select_bundle
    msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=100_000_000)))
    keys=['cam1_color','cam1_depth','cam1_info','cam2_color','cam2_depth','cam2_info']
    buffers={k:[(9.99,msg)] for k in keys}
    assert select_bundle(buffers,10_000_000_000,10.2,received_after=10.) is None


def test_default_capture_settle_is_half_second():
    from soarm_turntable.scan_core import StationaryCaptureGate
    gate=StationaryCaptureGate();assert gate.seconds==.5
    assert not gate.update(None,0.0)
    assert not gate.update(None,.49)
    assert gate.update(None,.5)
