import argparse, math
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Trigger,SetBool

def main():
    p=argparse.ArgumentParser();p.add_argument('command',choices=['read','zero','goto','stop','torque']);p.add_argument('value',nargs='?');p.add_argument('--timeout',type=float,default=30.0);a=p.parse_args()
    if a.command=='goto':
        try:angle=float(a.value)
        except (ValueError,TypeError):p.error('goto 각도(도)가 필요합니다')
        if not math.isfinite(angle) or not 0<a.timeout<=120:p.error('각도/timeout 범위 오류')
    if a.command=='torque' and a.value not in ('on','off'):p.error('torque on 또는 torque off')
    rclpy.init();n=rclpy.create_node('turntable_command');handle=None
    try:
        if a.command=='goto':
            client=ActionClient(n,FollowJointTrajectory,'/turntable/follow_joint_trajectory')
            if not client.wait_for_server(timeout_sec=3):raise RuntimeError('턴테이블 액션 없음')
            goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=['turntable_joint'];point=JointTrajectoryPoint(positions=[math.radians(angle)]);point.time_from_start.sec=int(a.timeout);point.time_from_start.nanosec=round((a.timeout-int(a.timeout))*1e9);goal.trajectory.points=[point]
            future=client.send_goal_async(goal);rclpy.spin_until_future_complete(n,future,timeout_sec=5)
            if not future.done():raise RuntimeError('액션 수락 응답 없음')
            handle=future.result()
            if not handle.accepted:raise RuntimeError('회전 거부: read_only/영점/중복 실행을 확인하세요')
            result=handle.get_result_async();rclpy.spin_until_future_complete(n,result,timeout_sec=a.timeout+5)
            if not result.done():handle.cancel_goal_async();raise RuntimeError('도착 응답 없음 · 정지 상태 확인 필요')
            r=result.result().result;print(r.error_string)
            if r.error_code!=0:raise RuntimeError('턴테이블 이동 실패')
        else:
            srv=SetBool if a.command=='torque' else Trigger;path='/turntable/'+('set_zero' if a.command=='zero' else a.command);c=n.create_client(srv,path)
            if not c.wait_for_service(timeout_sec=3):raise RuntimeError('서비스 없음: '+path)
            req=SetBool.Request(data=a.value=='on') if a.command=='torque' else Trigger.Request();future=c.call_async(req);rclpy.spin_until_future_complete(n,future,timeout_sec=5)
            if not future.done():raise RuntimeError('서비스 응답 없음')
            r=future.result();print(r.message)
            if not r.success:raise RuntimeError('명령 실패')
    except KeyboardInterrupt:
        if handle is not None:
            future=handle.cancel_goal_async();rclpy.spin_until_future_complete(n,future,timeout_sec=2)
    finally:n.destroy_node();rclpy.try_shutdown()
