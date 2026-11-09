"""A single final HOME after successful, released batch stages."""
import time
from vacuum_thresholds import HELD_DIFF


def go_home_empty():
    import rclpy
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
        client=node.create_client(Trigger,'/soarm4/go_home')
        if not client.wait_for_service(timeout_sec=8):raise RuntimeError('Final HOME service unavailable')
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
