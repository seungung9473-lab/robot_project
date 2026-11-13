#!/usr/bin/env python3
"""Expose conditional mating-zone checks to the MoveIt response adapter."""
from pathlib import Path
import sys
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import ContactInformation

class FineCollision(Node):
    def __init__(self):
        super().__init__('fine_collision_review')
        p=Path(get_package_share_directory('soarm4_moveit_config'))
        sys.path.insert(0,str(p/'runtime_deps'));sys.path.insert(0,str(p/'audit_tools'))
        from run_scan import ReviewAuditor
        self.audit=ReviewAuditor(p/'audit_model')
        self.service=self.create_service(GetStateValidity,'fine_check_state',self.check)
        self.get_logger().info('Fine joint-coupling checks ready for RViz and service plans.')
    def check(self,req,res):
        values=dict(zip(req.robot_state.joint_state.name,req.robot_state.joint_state.position))
        if not all(n in values for n in self.audit.names):res.valid=False;return res
        q=[values[n] for n in self.audit.names]
        if not self.audit.inside_limits(q).all():res.valid=False;return res
        r=self.audit.audit(q);res.valid=not(r['outside_coupling_zones'] or r['truncated_unknown'] or r['below_table_plane'])
        for i,j in r['outside_coupling_zones']+r['truncated_unknown']:
            c=ContactInformation();c.contact_body_1=self.audit.meshes[i]['name'];c.contact_body_2=self.audit.meshes[j]['name'];res.contacts.append(c)
        return res

def main():
    rclpy.init();node=FineCollision()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
