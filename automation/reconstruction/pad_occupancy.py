"""Read actual placed collision geometry; never removes objects or sends motion."""
import hashlib
import json
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pad_packing import footprint, separated


def pose_matrix(pose):
    p = pose['position']; q = pose['orientation']
    xyz = np.array([p['x'], p['y'], p['z']], float)
    quat = np.array([q['x'], q['y'], q['z'], q['w']], float)
    if not np.isfinite(xyz).all() or not np.isfinite(quat).all():
        raise ValueError('Nonfinite placed object pose')
    matrix = np.eye(4); matrix[:3, 3] = xyz
    # An unset outer CollisionObject.pose is the legacy identity representation.
    if np.linalg.norm(quat) > 1e-12:
        matrix[:3, :3] = Rotation.from_quat(quat).as_matrix()
    return matrix


def placed_obstacles(objects, world_to_table):
    result = []
    for obj in objects:
        if not obj['id'].startswith('placed_'):
            continue
        if obj['header']['frame_id'] != 'world':
            raise ValueError('Unsupported placed object frame: '+obj['id'])
        if obj.get('planes'):
            raise ValueError('Unbounded placed object geometry: '+obj['id'])
        base = np.asarray(world_to_table, float) @ pose_matrix(obj['pose'])
        parts = []
        for mesh, pose in zip(obj.get('meshes', []), obj.get('mesh_poses', []), strict=True):
            points = np.array([[p['x'], p['y'], p['z']] for p in mesh['vertices']], float)
            T = base @ pose_matrix(pose)
            parts.append(points @ T[:3, :3].T+T[:3, 3])
        for shape, pose in zip(obj.get('primitives', []), obj.get('primitive_poses', []), strict=True):
            dims = shape['dimensions']; kind = shape['type']
            if kind == 1 and len(dims) == 3:  # BOX
                half = np.asarray(dims)/2
            elif kind == 2 and len(dims) == 1:  # SPHERE: conservative box
                half = np.array([dims[0]]*3)
            elif kind in (3, 4) and len(dims) == 2:  # CYLINDER/CONE
                half = np.array([dims[1], dims[1], dims[0]/2])
            else:
                raise ValueError('Unsupported placed primitive: '+obj['id'])
            if not np.isfinite(half).all() or np.any(half <= 0):
                raise ValueError('Invalid placed primitive: '+obj['id'])
            corners = np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*half
            T = base @ pose_matrix(pose)
            parts.append(corners @ T[:3,:3].T+T[:3,3])
        if not parts:
            raise ValueError('Missing placed geometry: '+obj['id'])
        points = np.concatenate(parts)
        if not np.isfinite(points).all():
            raise ValueError('Nonfinite placed geometry: '+obj['id'])
        result.append({'id':obj['id'], 'polygon_table_m':footprint(points).tolist(),
                       'z_min_m':float(points[:,2].min()), 'z_max_m':float(points[:,2].max())})
    return sorted(result, key=lambda r:r['id'])


def snapshot(objects, world_to_table):
    obstacles = placed_obstacles(objects, world_to_table)
    digest = hashlib.sha256(json.dumps(obstacles,sort_keys=True,separators=(',', ':')).encode()).hexdigest()
    return {'version':1, 'captured_time':time.time(), 'geometry_digest':digest,
            'obstacles':obstacles, 'scope':'live MoveIt placed geometry, table coordinates; no object removal'}


def read_occupancy(world_to_table):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from moveit_msgs.msg import PlanningSceneComponents as C
    from moveit_msgs.srv import GetPlanningScene
    from rosidl_runtime_py.convert import message_to_ordereddict
    context = Context(); rclpy.init(context=context)
    node = rclpy.create_node('pad_occupancy_reader', context=context)
    executor=SingleThreadedExecutor(context=context)
    try:
        client = node.create_client(GetPlanningScene, '/get_planning_scene')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('패드 점유 확인 실패: 장면 서비스 연결 없음')
        request = GetPlanningScene.Request()
        request.components.components = C.WORLD_OBJECT_GEOMETRY|C.ROBOT_STATE_ATTACHED_OBJECTS
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, executor=executor, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError('패드 점유 확인 실패: 장면 응답 없음')
        scene = future.result().scene
        if scene.robot_state.attached_collision_objects:
            raise RuntimeError('물체 부착 상태: 새 패드 예약·자동 재시작 금지')
        return snapshot([message_to_ordereddict(o) for o in scene.world.collision_objects], world_to_table)
    finally:
        executor.shutdown(); node.destroy_node(); context.shutdown()


def blockers(polygon, occupancy, gap=.002):
    p = np.asarray(polygon, float)
    return [row['id'] for row in occupancy['obstacles']
            if not separated(p, np.asarray(row['polygon_table_m'], float), gap)]


def free_offsets(polygon, offsets, center, radius, occupancy, edge=.002, gap=.002):
    """Batch conservative SAT filtering, then exact IK/footprint checks by caller."""
    p = np.asarray(polygon, float); offsets = np.asarray(offsets, float)
    keep = np.max(np.linalg.norm(p[None,:,:]+offsets[:,None,:]-np.asarray(center)[:2],axis=2),axis=1) <= radius-edge+1e-10
    for obstacle in occupancy['obstacles']:
        q = np.asarray(obstacle['polygon_table_m'], float)
        edges = np.concatenate((np.roll(p,-1,axis=0)-p, np.roll(q,-1,axis=0)-q))
        axes = np.column_stack((-edges[:,1],edges[:,0]))
        axes /= np.linalg.norm(axes,axis=1)[:,None]
        pa=p@axes.T; pb=q@axes.T; shift=offsets@axes.T
        apart=((pa.max(0)+shift+gap<=pb.min(0)) | (pb.max(0)+gap<=pa.min(0)+shift)).any(axis=1)
        keep &= apart
    return offsets[keep]


FOOTPRINT_MARGIN_M=.0025


def inflate_polygon(polygon, margin=FOOTPRINT_MARGIN_M, sides=16):
    """Convex polygon grown by margin (circumscribed 16-gon Minkowski sum, never smaller than the true offset)."""
    p = np.asarray(polygon, float).reshape(-1, 2)
    if margin <= 0:return p
    from scipy.spatial import ConvexHull
    a = np.linspace(0, 2*np.pi, sides, endpoint=False); ring = np.column_stack((np.cos(a), np.sin(a)))*margin/np.cos(np.pi/sides)
    pts = (p[:, None, :]+ring[None, :, :]).reshape(-1, 2)
    return pts[ConvexHull(pts).vertices]


FINE_STEP=.0005
FINE_RADIUS=.012


def min_enclosing_circle(points):
    """Exact 2D minimum enclosing circle (Welzl, iterative). Returns (center, radius)."""
    pts = np.unique(np.asarray(points, float).reshape(-1, 2), axis=0)
    rng = np.random.default_rng(0); pts = pts[rng.permutation(len(pts))]
    def circ2(a, b):
        c = (a+b)/2; return c, float(np.linalg.norm(a-c))
    def circ3(a, b, c):
        d = 2*(a[0]*(b[1]-c[1])+b[0]*(c[1]-a[1])+c[0]*(a[1]-b[1]))
        if abs(d) < 1e-15:
            cands = [circ2(a, b), circ2(a, c), circ2(b, c)]; return max(cands, key=lambda t: t[1])
        ux = ((a@a)*(b[1]-c[1])+(b@b)*(c[1]-a[1])+(c@c)*(a[1]-b[1]))/d
        uy = ((a@a)*(c[0]-b[0])+(b@b)*(a[0]-c[0])+(c@c)*(b[0]-a[0]))/d
        cen = np.array([ux, uy]); return cen, float(np.linalg.norm(a-cen))
    inside = lambda cr, p: np.linalg.norm(p-cr[0]) <= cr[1]+1e-12
    cr = (pts[0].copy(), 0.0)
    for i in range(1, len(pts)):
        if inside(cr, pts[i]):continue
        cr = (pts[i].copy(), 0.0)
        for j in range(i):
            if inside(cr, pts[j]):continue
            cr = circ2(pts[i], pts[j])
            for k in range(j):
                if inside(cr, pts[k]):continue
                cr = circ3(pts[i], pts[j], pts[k])
    return cr


def free_offsets_refined(polygon, coarse_offsets, center, radius, occupancy, edge=.002, gap=.002):
    """Same margins as free_offsets. If the 2 mm grid finds nothing, also try offsets that put the shape's
    minimum-enclosing-circle center on the pad center (plus a 0.5 mm grid near the pad center).
    Tight fits can have less slack than the coarse step. Never relaxes edge or gap margins."""
    free = free_offsets(polygon, coarse_offsets, center, radius, occupancy, edge, gap)
    if len(free):return free, False
    p = np.asarray(polygon, float); mec_center, _ = min_enclosing_circle(p)
    best = np.asarray(center, float)[:2]-mec_center
    n = int(round(FINE_RADIUS/FINE_STEP)); ticks = np.arange(-n, n+1)*FINE_STEP
    fine = np.array([(x, y) for x in ticks for y in ticks if x*x+y*y <= FINE_RADIUS**2], float)
    fine = fine[np.argsort(np.linalg.norm(fine, axis=1), kind='stable')]
    near = best+np.array([(dx, dy) for dx in (-.00025, 0., .00025) for dy in (-.00025, 0., .00025)])
    return free_offsets(p, np.vstack((near, fine)), center, radius, occupancy, edge, gap), True


def validate_reserved(plan, occupancy):
    if plan.get('occupancy_geometry_digest') not in (None, occupancy['geometry_digest']):
        raise RuntimeError('패드 점유 모델 변경: 새 예약이 필요합니다')
    for key,item in plan['placements'].items():
        ids = blockers(item['reserved_polygon_table_m'], occupancy, plan.get('object_gap_m',.002))
        if ids:
            raise RuntimeError(f'패드 {plan["pad_id"]} 예약 충돌: {key} / {ids}')


def write_snapshot(session, occupancy):
    path = Path(session)/'pad_occupancy.json'
    if path.exists() and json.loads(path.read_text())['geometry_digest'] != occupancy['geometry_digest']:
        raise RuntimeError('기존 예약 이후 패드 점유가 변경됐습니다. 새 스캔 세션이 필요합니다')
    path.write_text(json.dumps(occupancy,ensure_ascii=False,indent=2))
    return path


def stale_pad_ids(objects):
    return sorted(obj.id for obj in objects if obj.id.startswith(('placed_pad_5_', 'placed_pad_6_')))


def reset_empty_pad_scene(query, apply, audit_directory):
    """User-confirmed empty pads at a new full-run start; no actuator calls."""
    from rosidl_runtime_py.convert import message_to_ordereddict
    from moveit_msgs.msg import PlanningScene, CollisionObject
    before=query()
    if before.robot_state.attached_collision_objects:
        raise RuntimeError('부착 물체가 있어 패드 초기화 중단')
    ids=stale_pad_ids(before.world.collision_objects)
    folder=Path(audit_directory);folder.mkdir(parents=True,exist_ok=True)
    (folder/'empty_pad_reset_before.json').write_text(json.dumps(message_to_ordereddict(before),ensure_ascii=False,indent=2))
    if ids:
        scene=PlanningScene();scene.is_diff=True
        for key in ids:
            obj=CollisionObject();obj.id=key;obj.header.frame_id='world';obj.operation=CollisionObject.REMOVE
            scene.world.collision_objects.append(obj)
        apply(scene)
    after=query();remaining=stale_pad_ids(after.world.collision_objects)
    kept={o.id for o in before.world.collision_objects}-set(ids);actual={o.id for o in after.world.collision_objects}
    report={'success':not remaining and kept.issubset(actual),'removed_ids':ids,'remaining_pad_models':remaining,
            'retained_ids':sorted(actual),'pads_confirmed_empty_by_user':[5,6],
            'motor_commands_sent':False,'physical_objects_removed':False,'timestamp':time.time()}
    (folder/'empty_pad_reset.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    if not report['success']:raise RuntimeError('패드 모델 초기화 확인 실패; 실행 중단')
    print(f'패드 5·6 빈 상태 초기화 완료 · 이전 배치 모델 {len(ids)}개 삭제',flush=True)
    return report


def clear_confirmed_empty_pads(audit_directory, *, reset_all_scans=False):
    """Call under scan_pick_cycle.lock, once per new full cycle, never between picks."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from moveit_msgs.msg import PlanningSceneComponents as C
    from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
    context=Context();rclpy.init(context=context)
    node=rclpy.create_node('confirmed_empty_pad_reset',context=context)
    executor=SingleThreadedExecutor(context=context)
    def call(topic,typ,request):
        client=node.create_client(typ,topic)
        try:
            if not client.wait_for_service(timeout_sec=10):raise RuntimeError('패드 초기화 서비스 연결 실패: '+topic)
            future=client.call_async(request)
            rclpy.spin_until_future_complete(node,future,executor=executor,timeout_sec=10)
            if not future.done() or future.result() is None:raise RuntimeError('패드 초기화 응답 없음: '+topic)
            return future.result()
        finally:node.destroy_client(client)
    def query():
        request=GetPlanningScene.Request();request.components.components=C.WORLD_OBJECT_GEOMETRY|C.ROBOT_STATE_ATTACHED_OBJECTS
        return call('/get_planning_scene',GetPlanningScene,request).scene
    def apply(scene):
        request=ApplyPlanningScene.Request();request.scene=scene
        if not call('/apply_planning_scene',ApplyPlanningScene,request).success:raise RuntimeError('패드 이전 모델 삭제 거부')
    try:
        if reset_all_scans:
            from scan_scene_reset import reset_previous_scan_scene, delete_scan_markers
            def not_holding():
                # 압력이 "붙어 있지 않음"을 새 값으로 확인했을 때만 True. 값이 없거나 낡았으면 None(= 모름 → 중단).
                from std_msgs.msg import Int32
                from vacuum_thresholds import HELD_DIFF
                got={}
                sub=node.create_subscription(Int32,'/vacuum/diff',lambda m:got.update(v=m.data,t=time.monotonic()),10)
                try:
                    end=time.monotonic()+3
                    while time.monotonic()<end:
                        executor.spin_once(timeout_sec=.1)
                        if got and time.monotonic()-got['t']<.3:break
                    return None if not got or time.monotonic()-got['t']>.5 else got['v']>HELD_DIFF
                finally:node.destroy_subscription(sub)
            return reset_previous_scan_scene(query,apply,audit_directory,
                clear_markers=lambda:delete_scan_markers(node,executor),not_holding=not_holding)
        return reset_empty_pad_scene(query,apply,audit_directory)
    finally:executor.shutdown();node.destroy_node();context.shutdown()
