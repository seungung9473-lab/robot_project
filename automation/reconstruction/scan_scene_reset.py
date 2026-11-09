"""Clear previous scan runtime state at a new full-cycle boundary, never actuate."""
import os
import signal
import time
from pathlib import Path

SCAN_IDS = {'scanned_block', 'held_block'}
SCAN_PREFIXES = ('inclined_obstacle_object_', 'batch_virtual_', 'placed_pad_5_', 'placed_pad_6_')


def scan_object_ids(objects):
    return sorted(o.id for o in objects if o.id in SCAN_IDS or o.id.startswith(SCAN_PREFIXES))


def stop_previous_display():
    target = str(Path.home()/'robot_ws/install/soarm4_workcell/lib/soarm4_workcell/reconstructed_models')
    stopped = []
    for entry in Path('/proc').glob('[0-9]*'):
        try:
            args = (entry/'cmdline').read_bytes().split(b'\0')
            if len(args)>1 and args[1].decode()==target and entry.stat().st_uid==os.getuid():
                os.kill(int(entry.name), signal.SIGINT)
                stopped.append(int(entry.name))
        except (OSError, UnicodeError):
            continue
    end = time.monotonic()+5
    while any(Path('/proc',str(pid)).exists() for pid in stopped) and time.monotonic()<end:
        time.sleep(.05)
    if any(Path('/proc',str(pid)).exists() for pid in stopped):
        raise RuntimeError('이전 스캔 표시 종료 실패; 새 실행 중단')
    return stopped


def delete_scan_markers(node, executor):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from visualization_msgs.msg import Marker, MarkerArray
    publisher = node.create_publisher(MarkerArray, '/workcell_markers', QoSProfile(
        depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    message = MarkerArray()
    # Only this display's namespace. Keep table, camera, marker, pad and axis visuals.
    for index in range(8):
        m=Marker();m.header.frame_id='table';m.ns='reconstructed_pair_rgb';m.id=index;m.action=Marker.DELETE
        message.markers.append(m)
    executor.add_node(node)
    end=time.monotonic()+.5
    while time.monotonic()<end:
        publisher.publish(message)
        executor.spin_once(timeout_sec=.05)
    node.destroy_publisher(publisher)


def stale_attached(attached, not_holding):
    """Attached scan blocks left by an aborted pick are removed only when the pressure sensor shows the cup is NOT holding.
    Anything else attached, a held object, or no fresh pressure reading still stops the new run (nothing is guessed)."""
    found=[(a.object.id, a.link_name) for a in attached]
    if not found:return []
    if all(i in SCAN_IDS for i,_ in found) and not_holding is not None and not_holding() is True:return found
    raise RuntimeError('부착 물체가 있어 이전 스캔 초기화 중단 (흡착 해제 확인 없음: 컵 상태를 확인하세요)')


def reset_previous_scan_scene(query, apply, audit_directory, *, stop_display=stop_previous_display, clear_markers, not_holding=None):
    from moveit_msgs.msg import PlanningScene, CollisionObject
    from rosidl_runtime_py.convert import message_to_ordereddict
    from test_data_recorder import atomic_json
    folder=Path(audit_directory);folder.mkdir(parents=True,exist_ok=True)
    before=query()
    detached=stale_attached(before.robot_state.attached_collision_objects, not_holding)
    if detached:
        from moveit_msgs.msg import AttachedCollisionObject
        scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True
        for oid,link in detached:
            ao=AttachedCollisionObject();ao.link_name=link;ao.object=CollisionObject();ao.object.id=oid;ao.object.operation=CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(ao)
            world=CollisionObject();world.id=oid;world.header.frame_id='world';world.operation=CollisionObject.REMOVE;scene.world.collision_objects.append(world)
        apply(scene);before=query()
        if before.robot_state.attached_collision_objects:raise RuntimeError('이전 부착 물체 제거 확인 실패; 새 실행 중단')
        print('이전 실행에서 남은 부착 물체 제거(압력으로 비어 있음 확인): '+', '.join(i for i,_ in detached),flush=True)
    atomic_json(folder/'previous_scan_reset_before.json', message_to_ordereddict(before))
    stopped=stop_display()
    ids=scan_object_ids(before.world.collision_objects)
    if ids:
        scene=PlanningScene();scene.is_diff=True
        for key in ids:
            obj=CollisionObject();obj.id=key;obj.header.frame_id='world';obj.operation=CollisionObject.REMOVE
            scene.world.collision_objects.append(obj)
        apply(scene)
    clear_markers()
    after=query();remaining=scan_object_ids(after.world.collision_objects)
    kept={o.id for o in before.world.collision_objects}-set(ids)
    actual={o.id for o in after.world.collision_objects}
    report={'success':not remaining and kept.issubset(actual), 'removed_ids':ids,
            'remaining_scan_models':remaining, 'retained_ids':sorted(actual),
            'stopped_display_pids':stopped,'detached_stale_attached':[i for i,_ in detached], 'cleared_marker_namespace':'reconstructed_pair_rgb',
            'past_files_deleted':False, 'motor_commands_sent':False,
            'scope':'new full cycle only; old scan invalidated; fresh scan required'}
    atomic_json(folder/'previous_scan_reset.json',report)
    if not report['success']:raise RuntimeError('이전 스캔 모델 초기화 확인 실패; 새 실행 중단')
    print(f'턴테이블·패드 이전 스캔 초기화 완료 · 모델 {len(ids)}개 삭제 · 과거 파일 보존',flush=True)
    return report


def clear_previous_scan_scene(audit_directory):
    from pad_occupancy import clear_confirmed_empty_pads
    return clear_confirmed_empty_pads(audit_directory, reset_all_scans=True)
