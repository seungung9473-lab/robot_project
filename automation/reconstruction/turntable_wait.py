"""Conservative full-rotation envelope check for a stationary, empty robot arm."""
import json,time
from vacuum_thresholds import HELD_DIFF
from pathlib import Path
import numpy as np
import yaml


def rotation_envelope(clouds,axis,top,padding=.003):
    """A solid cylinder encloses every source point at every turntable angle."""
    axis=np.asarray(axis,float)
    points=np.concatenate([np.asarray(p,float) for p in clouds])
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():
        raise ValueError('Finite source geometry required')
    radius=float(np.max(np.linalg.norm(points[:,:2]-axis[:2],axis=1)))+padding
    bottom=min(float(top),float(points[:,2].min()))-padding
    ceiling=float(points[:,2].max())+padding
    return {'center':[float(axis[0]),float(axis[1]),(bottom+ceiling)/2],
            'height':ceiling-bottom,'radius':radius}


def review_wait_pose(node,q,source,completed,require_empty=False):
    """Scene-only review; no motor/pressure/service command is sent to actuators."""
    import rclpy
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import PlanningScene,CollisionObject,RobotState,PlanningSceneComponents
    from moveit_msgs.srv import ApplyPlanningScene,GetStateValidity,GetPlanningScene
    from shape_msgs.msg import SolidPrimitive
    from scipy.spatial.transform import Rotation
    from motion.kinematics import W
    source=Path(source);report=json.loads((source/'inclined_pick_plan.json').read_text())
    clouds=[np.load(Path(row['session'])/'analysis/cloud.npz')['points'] for row in report['objects'] if row['object'] not in completed]
    if not clouds:return
    wc=yaml.safe_load((source/'workcell_snapshot.yaml').read_text())
    env=rotation_envelope(clouds,wc['turntable']['axis_point_table_m'],wc['turntable']['top_height_m'])
    def spin_until(future,timeout):
        if hasattr(node,'_scan_executor'):
            end=time.monotonic()+timeout
            while not future.done() and time.monotonic()<end:node.spin(.05)
        else:rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
    if require_empty:
        from std_msgs.msg import Int32
        from rclpy.qos import qos_profile_sensor_data
        feedback=[]
        subscription=node.create_subscription(Int32,'/vacuum/diff',lambda m:feedback.append((time.monotonic(),m.data)),qos_profile_sensor_data)
        try:
            deadline=time.monotonic()+3
            while len(feedback)<2 and time.monotonic()<deadline:
                if hasattr(node,'_scan_executor'):node.spin(.05)
                else:rclpy.spin_once(node,timeout_sec=.05)
            if len(feedback)<2 or time.monotonic()-feedback[-1][0]>.5 or any(v<=HELD_DIFF for _,v in feedback[-2:]):
                raise RuntimeError('Fresh empty-cup confirmation missing; no table rotation')
        finally:node.destroy_subscription(subscription)
    def call(path,typ,request):
        client=node.create_client(typ,path)
        try:
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Waiting pose review unavailable: '+path)
            f=client.call_async(request);spin_until(f,10)
            if not f.done() or f.result() is None:raise RuntimeError('Waiting pose review timed out')
            return f.result()
        finally:node.destroy_client(client)
    query=GetPlanningScene.Request();query.components.components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    if call('/get_planning_scene',GetPlanningScene,query).scene.robot_state.attached_collision_objects:
        raise RuntimeError('Attached payload remains; table rotation blocked')
    key='batch_pending_rotation_envelope'
    shape=CollisionObject();shape.id=key;shape.header.frame_id='world';shape.operation=CollisionObject.ADD
    shape.primitives=[SolidPrimitive(type=SolidPrimitive.CYLINDER,dimensions=[env['height'],env['radius']])]
    T=np.eye(4);T[:3,3]=env['center'];T=np.linalg.inv(W)@T
    pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,T[:3,3]);pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,Rotation.from_matrix(T[:3,:3]).as_quat());shape.primitive_poses=[pose]
    def apply(obj):
        req=ApplyPlanningScene.Request();req.scene=PlanningScene(is_diff=True);req.scene.world.collision_objects=[obj]
        if not call('/apply_planning_scene',ApplyPlanningScene,req).success:raise RuntimeError('Waiting envelope scene update failed')
    try:
        apply(shape)
        req=GetStateValidity.Request();req.group_name='arm';req.robot_state=RobotState(joint_state=JointState(name=['joint'+str(i) for i in range(1,5)],position=list(map(float,q))),is_diff=True)
        reply=call('/check_state_validity',GetStateValidity,req)
        if not reply.valid:raise RuntimeError('대기 위치 회전 안전점검 실패: '+str([(c.contact_body_1,c.contact_body_2) for c in reply.contacts]))
    finally:
        remove=CollisionObject();remove.id=key;remove.header.frame_id='world';remove.operation=CollisionObject.REMOVE;apply(remove)
    print('대기 위치 회전 안전점검 통과 · 중간 HOME 없이 다음 물체로 진행',flush=True)
