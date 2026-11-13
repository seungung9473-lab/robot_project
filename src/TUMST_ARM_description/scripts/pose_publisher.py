#!/usr/bin/env python3
import json,signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
class PosePublisher(Node):
    def __init__(self):
        super().__init__('assembly_pose_publisher');self.declare_parameter('pose','zero');m=json.loads((Path(get_package_share_directory('soarm4_description'))/'config/model.json').read_text());self.positions=m['poses'][self.get_parameter('pose').value];self.names=[j['name'] for j in m['joints']];self.pub=self.create_publisher(JointState,'joint_states',10);self.timer=self.create_timer(.1,self.publish)
    def publish(self):
        msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=self.names;msg.position=self.positions;self.pub.publish(msg)
def main():
    rclpy.init();node=PosePublisher()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:signal.signal(signal.SIGINT,signal.SIG_IGN);node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
