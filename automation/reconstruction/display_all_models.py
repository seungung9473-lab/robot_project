"""Show all scanned objects with RGB at current measured turntable angle."""
import sys,json,time
from pathlib import Path
import numpy as np,rclpy
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA,String
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from scipy.spatial.transform import Rotation
s=Path(sys.argv[1]);context=json.loads((s/'spatial_context.json').read_text());axis=np.array(context['axis_point_table_m']);models=[]
for i,path in enumerate(sorted(s.glob('objects/object_*/analysis/cloud.npz'))):
 data=np.load(path);step=max(1,len(data['points'])//6000);models.append((i,data['points'][::step],data['colors_rgb'][::step]/255))
rclpy.init();n=rclpy.create_node('all_object_reconstruction_display');latest={};n.create_subscription(String,'/turntable/state',lambda msg:latest.update(angle=json.loads(msg.data)['angle_deg'],t=time.monotonic()),10);pub=n.create_publisher(MarkerArray,'/workcell_markers',QoSProfile(depth=5,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
def draw():
 if not latest or time.monotonic()-latest['t']>1:return
 R=Rotation.from_euler('z',latest['angle'],degrees=True).as_matrix();arr=MarkerArray()
 for i,P,C in models:
  Q=(P-axis)@R.T+axis;m=Marker();m.header.frame_id='table';m.ns='reconstructed_pair_rgb';m.id=i;m.type=Marker.POINTS;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=m.scale.y=.002;m.color.a=1.
  for p,c in zip(Q,C):m.points.append(Point(x=float(p[0]),y=float(p[1]),z=float(p[2])));m.colors.append(ColorRGBA(r=float(c[0]),g=float(c[1]),b=float(c[2]),a=1.))
  arr.markers.append(m)
 pub.publish(arr)
n.create_timer(1.,draw)
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:n.destroy_node();rclpy.try_shutdown()
