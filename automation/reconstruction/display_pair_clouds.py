"""Show reconstructed pair using observed RGB; no motor interfaces."""
import sys
from pathlib import Path
import numpy as np,rclpy
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
rclpy.init();n=rclpy.create_node('reconstructed_pair_colors');p=n.create_publisher(MarkerArray,'/workcell_markers',QoSProfile(depth=5,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE));arr=MarkerArray()
for i,f in enumerate(sorted(Path(sys.argv[1]).glob('objects/object_*/analysis/cloud.npz'))):
 d=np.load(f);m=Marker();m.header.frame_id='table';m.ns='reconstructed_pair_rgb';m.id=i;m.type=Marker.POINTS;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=m.scale.y=.002;m.color.a=1.;step=max(1,len(d['points'])//9000)
 for xyz,rgb in zip(d['points'][::step],d['colors_rgb'][::step]/255):
  m.points.append(Point(x=float(xyz[0]),y=float(xyz[1]),z=float(xyz[2])));m.colors.append(ColorRGBA(r=float(rgb[0]),g=float(rgb[1]),b=float(rgb[2]),a=1.))
 arr.markers.append(m)
n.create_timer(1.,lambda:p.publish(arr));p.publish(arr)
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:n.destroy_node();rclpy.try_shutdown()
