"""A single final HOME after successful, released batch stages."""
import time
from vacuum_thresholds import HELD_DIFF


def go_home_empty():
    import rclpy
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene
    from std_msgs.msg import Int32
    from std_srvs.srv import Trigger
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init();node=rclpy.create_node('batch_final_home');pressure=[]
    node.create_subscription(Int32,'/vacuum/diff',lambda m:pressure.append((time.monotonic(),m.data)),qos_profile_sensor_data)
    try:
        deadline=time.monotonic()+5
        while len(pressure)<2 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if len(pressure)<2 or time.monotonic()-pressure[-1][0]>.5 or any(v<=HELD_DIFF for _,v in pressure[-2:]):
            raise RuntimeError('Final HOME blocked: fresh empty-cup confirmation missing')
        scene=node.create_client(GetPlanningScene,'/get_planning_scene')
        if not scene.wait_for_service(timeout_sec=5):
            raise RuntimeError('HOME blocked: planning scene unavailable')
        request=GetPlanningScene.Request()
        request.components.components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        for _ in range(2):
            pending = scene.call_async(request)
            rclpy.spin_until_future_complete(node, pending, timeout_sec=5)
            response = pending.result() if pending.done() else None
            if response is not None:
                break
            pending.cancel()
        else:
            raise RuntimeError('HOME 중단 · MoveIt 부착 상태 조회 응답 없음')
        attached = response.scene.robot_state.attached_collision_objects
        if attached:
            ids = ', '.join(obj.object.id for obj in attached)
            raise RuntimeError(f'HOME 중단 · MoveIt 부착 물체 남음 ({ids})')
        client=node.create_client(Trigger,'/soarm4/go_home')
        if not client.wait_for_service(timeout_sec=8):raise RuntimeError('Final HOME service unavailable')
        checked_at = time.monotonic()
        deadline = checked_at + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            recent = pressure[-2:]
            if (len(recent) == 2 and recent[0][0] >= checked_at
                    and time.monotonic() - recent[0][0] < .5
                    and all(value > HELD_DIFF for _, value in recent)):
                break
        else:
            raise RuntimeError('HOME 중단 · 최신 빈 컵 압력 확인 실패')
        future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=120)
        if not future.done() or future.result() is None:raise RuntimeError('Final HOME response timeout; check robot state')
        reply=future.result();return {'success':reply.success,'message':reply.message}
    finally:node.destroy_node();rclpy.try_shutdown()


def finish_batch_home(plan,home=go_home_empty,save=lambda:None,emit=print):
    if not plan.get('finish_above'):return
    if not plan.get('stages') or any(s.get('execution_status')!='complete' for s in plan['stages']):
        raise RuntimeError('Final HOME requires every placement to finish successfully')
    if plan.get('final_home',{}).get('status')=='complete':return
    started=time.monotonic();plan['final_home']={'status':'running'};save()
    emit('전체 물체 분류 완료 · 마지막 HOME 복귀 중',flush=True)
    try:
        result=home()
        if not result.get('success'):raise RuntimeError('Final HOME failed: '+result.get('message','unknown'))
        plan['final_home'].update(status='complete',seconds=time.monotonic()-started,message=result.get('message',''))
        save();emit('마지막 HOME 복귀 완료',flush=True)
    except BaseException as exc:
        plan['final_home'].update(status='stopped',seconds=time.monotonic()-started,error=str(exc));save();raise
