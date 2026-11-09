"""Console/file progress and bounded, non-actuating candidate memoization."""
import contextlib
import hashlib
import json
from pathlib import Path
import threading
import time


@contextlib.contextmanager
def calculating(label, emit=print, interval=5.):
    stop = threading.Event()
    started = time.monotonic()
    def report():
        while not stop.wait(interval):
            emit(f'계산중: {label} · {time.monotonic()-started:.0f}초 경과', flush=True)
    emit('계산중: ' + label, flush=True)
    thread = threading.Thread(target=report, daemon=True)
    thread.start()
    try:
        yield
    finally:
        stop.set()
        thread.join()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def geometry_key(key, option, done, placements, reservation, environment):
    """No file-name identity: different poses/cloud contents never share a key."""
    shapes = []
    for item in placements:
        item = dict(item)
        if item.get('cloud_file'):
            item['cloud_file'] = hashlib.sha256(Path(item['cloud_file']).read_bytes()).hexdigest()
        shapes.append(item)
    return digest([key, option, sorted(done), sorted(shapes, key=digest), reservation, environment])


def reservation_choices(report_entry, reservation, output):
    """Try different reserved offsets; never mutate an already reviewed plan."""
    choices = [opt for face in report_entry['matched_faces'] for opt in face['alignment_options']]
    if reservation is None:
        return [(i, opt, None) for i, opt in enumerate(choices)]
    plan = json.loads(Path(reservation).read_text())
    key = report_entry['object']
    rows = [plan['placements'][key]]
    if len(plan['placements']) == 1:
        rows += plan.get('alternatives', {}).get(key, [])
    result, seen = [], set()
    for number, row in enumerate(rows):
        signature = digest(row)
        if signature in seen:
            continue
        seen.add(signature)
        index = row.get('pickup_option_index')
        if not isinstance(index, int) or not 0 <= index < len(choices):
            raise ValueError('Invalid pickup option in placement reservation')
        if number == 0:
            path = Path(reservation)
        else:
            variant = {**plan, 'placements': {key: row}}
            path = Path(output) / f'{key}_reservation_{number}.json'
            path.write_text(json.dumps(variant, indent=2))
        result.append((index, choices[index], path))
    return result


def retry_safe(session):
    """Only a fully recorded planning rejection before execution may trigger scan."""
    plans = list(Path(session).glob('batch_*/batch_plan.json'))
    if not plans:
        marker=Path(session)/'pre_execution_safety_failure.json'
        if not marker.exists() or (Path(session)/'completed_placements.json').exists():
            return False
        report=json.loads(marker.read_text())
        return report.get('kind')=='static_placement_rejection' and report.get('execution_performed') is False and report.get('retryable',True) is True
    plan = json.loads(max(plans, key=lambda p: p.stat().st_mtime_ns).read_text())
    return (plan.get('all_stages_ready') is False and
            plan.get('execution_performed') is False and
            not (Path(session)/'completed_placements.json').exists())


class PlanningEnvironment:
    """Read live geometry, ACM, attachments, speed and start pose before memo lookup."""
    def __init__(self, session):
        import rclpy
        from sensor_msgs.msg import JointState
        self.ros = rclpy
        rclpy.init()
        self.node = rclpy.create_node('batch_cache_environment')
        self.session = Path(session)
        self.feedback = None
        def receive(msg):
            if all('joint'+str(i) in msg.name for i in range(1, 5)):
                self.feedback = ([msg.position[msg.name.index('joint'+str(i))] for i in range(1, 5)], time.monotonic())
        self.node.create_subscription(JointState, '/joint_states', receive, 10)

    def call(self, topic, typ, request):
        client = self.node.create_client(typ, topic)
        try:
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError('Cannot establish cache environment: '+topic)
            future = client.call_async(request)
            self.ros.spin_until_future_complete(self.node, future, timeout_sec=10)
            if not future.done() or future.result() is None:
                raise RuntimeError('Cache environment response timeout: '+topic)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def snapshot(self):
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents as C
        from std_srvs.srv import Trigger
        from rosidl_runtime_py.convert import message_to_ordereddict
        from batch_inputs import fingerprints
        deadline = time.monotonic()+3
        # Drain queued feedback, then require a new sample for every lookup.
        previous = self.feedback[1] if self.feedback else 0
        while time.monotonic()<deadline:
            self.ros.spin_once(self.node, timeout_sec=.05)
            if self.feedback and self.feedback[1]>previous and time.monotonic()-self.feedback[1]<.3:
                break
        else:
            raise RuntimeError('Fresh joints missing; memoization and planning stopped')
        req = GetPlanningScene.Request()
        req.components.components = C.WORLD_OBJECT_GEOMETRY|C.ALLOWED_COLLISION_MATRIX|C.ROBOT_STATE_ATTACHED_OBJECTS|C.TRANSFORMS
        scene = self.call('/get_planning_scene', GetPlanningScene, req).scene
        speed = self.call('/soarm4/rviz_scaling', Trigger, Trigger.Request())
        if not speed.success:
            raise RuntimeError('Speed unavailable; no cached result reused')
        def clean(value):
            if isinstance(value, dict):
                return {k: clean(v) for k,v in value.items() if k not in ('stamp',)}
            if isinstance(value, list):
                return [clean(v) for v in value]
            return value
        world = sorted((clean(message_to_ordereddict(o)) for o in scene.world.collision_objects), key=lambda o:o['id'])
        completed = self.session/'completed_placements.json'
        return digest({'world':world, 'acm':message_to_ordereddict(scene.allowed_collision_matrix),
                       'attached':clean(message_to_ordereddict(scene.robot_state))['attached_collision_objects'],
                       'transforms':clean([message_to_ordereddict(t) for t in scene.fixed_frame_transforms]),
                       'start':self.feedback[0], 'speed':json.loads(speed.message),
                       'inputs':fingerprints(self.session),
                       'completed':completed.read_text() if completed.exists() else None})

    def close(self):
        self.node.destroy_node()
        self.ros.try_shutdown()
