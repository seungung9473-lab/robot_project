"""Competition setup: start drivers, verify sensors, and hold the current table angle."""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from desktop_runtime import require_cuda_profile
from run_qr_expiry_cycle import control_preflight
from test_data_recorder import start_trial, atomic_json

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
DRIVERS = {'servo_bridge', 'esp32_bridge', 'xl330_turntable', 'move_group',
           'hardware_moveit_rviz', 'cam1', 'cam2'}


def stack_present():
    import rclpy
    rclpy.init()
    node = rclpy.create_node('soarm_prepare_discovery')
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            if DRIVERS.intersection(node.get_node_names()):
                return True
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    for proc in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv = proc.read_bytes().split(b'\0')
        except OSError:
            continue
        if b'desktop_ready.launch.py' in argv:
            return True
    return False


def table_and_rviz(enable_torque, *, zero_pressure=False, rviz_log=None):
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int32, String
    from std_srvs.srv import SetBool, Trigger
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node('soarm_prepare_readiness')
    feedback = {}
    joints = {'joint1', 'joint2', 'joint3', 'joint4'}

    def on_joints(msg):
        if joints.issubset(msg.name) and len(msg.name) == len(msg.position):
            feedback['joints'] = time.monotonic()

    def on_table(msg):
        feedback['table'] = time.monotonic(), json.loads(msg.data)

    def checked_table():
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if 'table' in feedback and now-feedback['table'][0] < .35 and now-feedback.get('joints', 0) < .3:
                state = feedback['table'][1]
                if state['read_only'] or state['hardware_error'] or state['moving'] or not state['zero_valid']:
                    raise RuntimeError('턴테이블 제어 모드·오류·정지 상태·저장 영점을 확인하세요')
                return state
        raise RuntimeError('최신 관절값 또는 턴테이블 상태를 받지 못했습니다')

    def call(typ, name, request):
        client = node.create_client(typ, name)
        try:
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError('서비스를 찾을 수 없습니다: '+name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done() or future.result() is None:
                raise RuntimeError('서비스 응답이 없습니다: '+name)
            reply = future.result()
            if not reply.success:
                raise RuntimeError(reply.message)
            return reply
        finally:
            node.destroy_client(client)

    try:
        node.create_subscription(JointState, '/joint_states', on_joints, qos_profile_sensor_data)
        node.create_subscription(String, '/turntable/state', on_table, qos_profile_sensor_data)
        if enable_torque and rviz_log is not None:
            probe = node.create_client(Trigger, '/soarm4/rviz_scaling')
            try:
                available = probe.wait_for_service(timeout_sec=3)
            finally:
                node.destroy_client(probe)
            if not available:
                if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
                    raise RuntimeError('RViz를 열 수 있는 데스크탑 터미널에서 실행하세요')
                print('기존 제어 유지 · RViz 다시 여는 중', flush=True)
                config = WORKSPACE/'src/soarm4_moveit_config/config/hardware_moveit.rviz'
                with rviz_log.open('w') as log:
                    subprocess.Popen(
                        ['ros2', 'run', 'rviz2', 'rviz2', '-d', str(config),
                         '--ros-args', '-r', '__node:=hardware_moveit_rviz'],
                        cwd=WORKSPACE, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        scaling = (json.loads(call(Trigger, '/soarm4/rviz_scaling', Trigger.Request()).message)
                   if enable_torque else None)
        state = checked_table()
        zero_samples = []
        if zero_pressure:
            pressure = []
            node.create_subscription(Int32, '/vacuum/diff',
                                     lambda msg: pressure.append((time.monotonic(), msg.data)),
                                     qos_profile_sensor_data)
            call(Trigger, '/vacuum/off', Trigger.Request())
            call(Trigger, '/vacuum/zero', Trigger.Request())
            # The service acknowledges the command. Confirm the resulting sensor data separately.
            after = time.monotonic() + .4
            deadline = after + 5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                samples = [value for received, value in pressure if received > after]
                if len(samples) >= 3 and all(abs(value) <= 50 for value in samples[-3:]):
                    zero_samples = samples[-3:]
                    break
            if not zero_samples:
                last = pressure[-1][1] if pressure else '수신 없음'
                raise RuntimeError(f'압력 영점 확인 실패 · 마지막 diff={last}')
        if enable_torque and not state['torque']:
            call(SetBool, '/turntable/torque', SetBool.Request(data=True))
            feedback.pop('table', None)
            state = checked_table()
        if enable_torque and not state['torque']:
            raise RuntimeError('턴테이블 토크 ON을 확인하지 못했습니다')
        if enable_torque:
            call(SetBool, '/servo/torque', SetBool.Request(data=True))
        return state, scaling, zero_samples
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    trial = start_trial('competition_demo_preparation', physical=False)
    result = {'success': False, 'motion_goals_sent': False}
    locks = []
    try:
        for name in ('tidy_demo.lock', 'scan_pick_cycle.lock'):
            lock = (HERE/name).open('a')
            locks.append(lock)
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError('이미 다른 시연 또는 스캔이 실행 중입니다') from exc
        for device in ('servo_bus', 'esp32', 'turntable_bus'):
            if not Path('/dev', device).exists():
                raise RuntimeError('장치가 연결되지 않았습니다: /dev/'+device)
        result['cuda'] = require_cuda_profile()
        if not (HERE/'checkpoints/sam2.1_hiera_tiny.pt').is_file():
            raise RuntimeError('SAM2 모델 파일을 찾을 수 없습니다')
        sys.path.insert(0, str(HERE.parent/'er2'))
        from check_connection import load_key
        if not load_key()[0]:
            raise RuntimeError('Gemini API 키 설정이 없습니다')

        reuse_stack = stack_present()
        if reuse_stack:
            print('기존 로봇 제어 시스템을 사용합니다.', flush=True)
        else:
            if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
                raise RuntimeError('RViz를 열 수 있는 데스크탑 터미널에서 실행하세요')
            with (trial.directory/'drivers.log').open('w') as log:
                process = subprocess.Popen(
                    ['ros2', 'launch', 'soarm_pipeline', 'desktop_ready.launch.py', 'allow_execution:=true'],
                    cwd=WORKSPACE, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            result['launch_pid'] = process.pid
            Path('/tmp/soarm_current_ready_record').write_text(str(trial.directory))
            print('로봇 제어·카메라·RViz 기동 중', flush=True)

        _, _, zero_samples = table_and_rviz(enable_torque=False, zero_pressure=True)
        result['vacuum_zero_samples'] = zero_samples
        print(f'압력 영점 확인 · diff={zero_samples[-1]}', flush=True)
        control_preflight()
        print('두 카메라의 정지 컬러·깊이 촬영 확인 중', flush=True)
        subprocess.run(['ros2', 'run', 'soarm_turntable', 'turntable_scan', 'capture',
                        '--output', str(trial.directory/'preflight_capture')],
                       cwd=WORKSPACE, check=True, timeout=35)
        state, scaling, _ = table_and_rviz(
            enable_torque=True, rviz_log=trial.directory/'rviz.log')
        result.update(success=True, arm_torque_enabled=True, turntable=state, rviz_scaling=scaling)
        print(f"준비 완료 · 로봇팔·턴테이블 토크 ON ({state['angle_deg']:.1f}°) · RViz 속도 {scaling['velocity']:.0%}")
        if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
            try:
                with (trial.directory/'monitor.log').open('w') as log:
                    subprocess.Popen(
                        ['/usr/bin/python3', str(HERE/'demo_monitor.py')],
                        cwd=WORKSPACE, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            except OSError as exc:
                print(f'시연 모니터 시작 실패 · {exc}', file=sys.stderr)
        print('패드 5·6을 비우고 물체를 올린 뒤 다음 명령으로 시연을 시작하세요.')
        print('./run_tidy_demo.sh "테이블 정리해" --execute')
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc) or '중단됨'
        print('준비 중단: '+result['error'], file=sys.stderr, flush=True)
        return 1
    finally:
        p = trial.directory/'preparation.json'
        atomic_json(p, result)
        trial.attach(p)
        trial.finish('complete' if result['success'] else 'stopped', error=result.get('error'))
        for lock in locks:
            lock.close()
        print('준비 기록: '+str(trial.directory), flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
