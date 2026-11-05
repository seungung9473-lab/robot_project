#!/usr/bin/env python3
import sys
import rclpy
from std_srvs.srv import Trigger

def main():
    rclpy.init();n=rclpy.create_node('soarm4_home_command');c=n.create_client(Trigger,'/soarm4/go_home')
    try:
        if not c.wait_for_service(timeout_sec=20):print('홈 서비스가 없습니다. hardware_bringup.launch.py를 먼저 실행하세요.');return 1
        print('홈 이동 요청…');f=c.call_async(Trigger.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=600.)
        if not f.done():print('응답 대기 시간 초과. RViz 상태와 /servo/stop을 확인하세요.');return 1
        res=f.result();print(res.message);return 0 if res.success else 1
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':sys.exit(main())
