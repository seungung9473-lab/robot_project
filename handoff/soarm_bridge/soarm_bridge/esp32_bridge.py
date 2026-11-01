#!/usr/bin/env python3
"""
esp32_bridge — ESP32 진공 보드 ↔ ROS 2

  발행  /vacuum/diff     (std_msgs/Int32)  압력 변화량 (허공 ≈ 0, 잡으면 크게 음수)  10 Hz
        /vacuum/holding  (std_msgs/Bool)   diff <= hold_diff 이면 True
  서비스 /vacuum/grip     (std_srvs/Trigger) 밸브 닫고 펌프 ON → 잡히면 success=True
        /vacuum/release  (std_srvs/Trigger) 펌프 OFF + 밸브 열기 → 압력 풀리면 밸브 닫기
        /vacuum/zero     (std_srvs/Trigger) 지금 압력을 0 으로 (펌프 끈 상태에서!)
        /vacuum/off      (std_srvs/Trigger) 펌프·밸브 끄기
"""
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger

from soarm_bridge.vacuum import Vacuum


class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        p = self.declare_parameter
        port = p('port', '/dev/esp32').value
        self.grip_timeout = p('grip_timeout', 2.0).value
        self.vac = Vacuum(port, 115200, p('hold_diff', -400).value, p('release_diff', -50).value)
        cb = ReentrantCallbackGroup()
        self.pub_diff = self.create_publisher(Int32, 'vacuum/diff', 10)
        self.pub_hold = self.create_publisher(Bool, 'vacuum/holding', 10)
        self.create_service(Trigger, 'vacuum/grip', self.srv_grip, callback_group=cb)
        self.create_service(Trigger, 'vacuum/release', self.srv_release, callback_group=cb)
        self.create_service(Trigger, 'vacuum/zero', self.srv_zero, callback_group=cb)
        self.create_service(Trigger, 'vacuum/off', self.srv_off, callback_group=cb)
        self.create_timer(0.1, self.tick, callback_group=cb)
        self.warned = False
        self.get_logger().info(f'esp32_bridge 시작: {port}, hold_diff={self.vac.hold_diff}')

    def tick(self):
        if not self.vac.online:
            if not self.warned:
                self.get_logger().error('ESP32 출력이 안 들어옴 (USB / 펌웨어 / USB CDC On Boot 확인)')
                self.warned = True
            return
        self.warned = False
        self.pub_diff.publish(Int32(data=int(self.vac.diff)))
        self.pub_hold.publish(Bool(data=self.vac.holding))

    def srv_grip(self, req, res):
        ok, d = self.vac.grip(self.grip_timeout)
        res.success, res.message = ok, f'diff={d}'
        return res

    def srv_release(self, req, res):
        ok, d = self.vac.release()
        res.success, res.message = ok, f'diff={d}'
        return res

    def srv_zero(self, req, res):
        self.vac.zero()
        res.success, res.message = True, 'zero'
        return res

    def srv_off(self, req, res):
        self.vac.off()
        res.success, res.message = True, 'pump/valve off'
        return res

    def destroy_node(self):
        self.vac.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ESP32Bridge()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
