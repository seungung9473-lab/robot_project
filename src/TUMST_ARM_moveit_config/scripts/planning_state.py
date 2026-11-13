#!/usr/bin/env python3
"""Publish a fixed, valid simulation start. No motor/controller interfaces."""
import math,json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class PlanningState(Node):
    def __init__(self):
        super().__init__('planning_start_state')
        p=Path(get_package_share_directory('soarm4_moveit_config'))
        self.home=json.loads((p/'audit_model/config/model.json').read_text())['poses']['home']
        self.pub=self.create_publisher(JointState,'joint_states',10)
        self.timer=self.create_timer(.05,self.tick)
        self.get_logger().info('Simulation starts at configured HOME: '+str([math.degrees(q) for q in self.home])+'. No hardware commands.')
    def tick(self):
        msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=['joint1','joint2','joint3','joint4'];msg.position=self.home;self.pub.publish(msg)

def main():
    rclpy.init();node=PlanningState()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
