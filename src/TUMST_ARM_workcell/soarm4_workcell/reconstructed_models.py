"""Measured point-cloud display; hide only object collision proxies via colour alpha.
No geometry removal, ACM edits, motor commands, or vacuum commands.
"""
import json,time,math,copy
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String,ColorRGBA
from moveit_msgs.msg import ObjectColor, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene,ApplyPlanningScene

def resolve_session(value):
    root=Path.home()/'robot_ws/data/turntable_scans'
    candidates=sorted(root.glob('*'),reverse=True) if value=='latest' else [Path(value).expanduser()]
    for folder in candidates:
        if (folder/'spatial_context.json').exists() and list(folder.glob('objects/object_*/analysis/cloud.npz')):
            return folder.resolve()
    return None

def load_models(session,max_points=6000):
    models=[]
    for i,path in enumerate(sorted(session.glob('objects/object_*/analysis/cloud.npz'))):
        name=path.parents[1].name;raw=session/'shape_audit'/(name+'_raw_aligned.npz')
        with np.load(raw if raw.exists() else path) as d:
            P=d['points'];C=d['colors_rgb']
            if raw.exists():P=P[d['roi']];C=C[d['roi']]
            valid=np.isfinite(P).all(1)&np.isfinite(C).all(1);P=P[valid];C=C[valid]
            indices=np.linspace(0,len(P)-1,min(len(P),max_points),dtype=int) if len(P) else []
            models.append((i,name,P[indices].copy(),np.clip(C[indices]/255.,0,1).copy()))
    return models

def is_object_proxy(oid):
    return oid in ('scanned_block','held_block') or oid.startswith('inclined_obstacle_object_')

class ReconstructedModels(Node):
    def __init__(self):
        super().__init__('reconstructed_models')
        value=self.declare_parameter('session','latest').value
        self.session=resolve_session(value);self.models=[];self.latest={};self.busy=False
        self.hide_collision_proxies=bool(self.declare_parameter('hide_collision_proxies',False).value)
        self.get_scene=self.create_client(GetPlanningScene,'/get_planning_scene');self.apply_scene=self.create_client(ApplyPlanningScene,'/apply_planning_scene')
        self.pub=self.create_publisher(MarkerArray,'/workcell_markers',QoSProfile(depth=5,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(String,'/turntable/state',self.state,10)
        if self.session:
            self.models=load_models(self.session);context=json.loads((self.session/'spatial_context.json').read_text());self.axis=np.array(context['axis_point_table_m'])
            self.get_logger().info('Stored scan surface display: '+str(self.session)+'; collision geometry retained')
        else:self.get_logger().info('No complete reconstructed object models; keeping collision proxy visibility')
        self.create_timer(1.,self.draw);self.create_timer(5.,self.hide_proxies)
    def state(self,msg):
        try:
            state=json.loads(msg.data);angle=float(state['angle_deg'])
            if math.isfinite(angle):self.latest={'angle':angle,'t':time.monotonic()}
        except (ValueError,KeyError,TypeError):pass
    def draw(self):
        if not self.models or not self.latest or time.monotonic()-self.latest['t']>1:return
        R=Rotation.from_euler('z',self.latest['angle'],degrees=True).as_matrix();arr=MarkerArray()
        completed={}
        done=self.session/'completed_placements.json'
        if done.exists():
            try:completed=json.loads(done.read_text())
            except (OSError,ValueError):return
        for i,name,P,C in self.models:
            if name in completed:continue
            Q=(P-self.axis)@R.T+self.axis;m=Marker();m.header.frame_id='table';m.ns='reconstructed_pair_rgb';m.id=i;m.type=Marker.POINTS;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=m.scale.y=.0012;m.color.a=1.;m.lifetime.sec=3
            for point,color in zip(Q,C):
                m.points.append(Point(x=float(point[0]),y=float(point[1]),z=float(point[2])));m.colors.append(ColorRGBA(r=float(color[0]),g=float(color[1]),b=float(color[2]),a=1.))
            arr.markers.append(m)
        self.pub.publish(arr)
    def hide_proxies(self):
        if not self.hide_collision_proxies or not self.models or not self.latest or time.monotonic()-self.latest['t']>1 or self.busy or not self.get_scene.service_is_ready() or not self.apply_scene.service_is_ready():return
        self.busy=True;req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY|PlanningSceneComponents.OBJECT_COLORS
        self.get_scene.call_async(req).add_done_callback(self.scene_received)
    def scene_received(self,future):
        try:
            scene=future.result().scene;colors={c.id:c for c in scene.object_colors};updates=[];objects=[]
            for obj in scene.world.collision_objects:
                if not is_object_proxy(obj.id):continue
                if obj.id in colors and colors[obj.id].color.a<=.0011:continue
                c=ObjectColor();c.id=obj.id
                if obj.id in colors:c.color=colors[obj.id].color
                c.color.a=.001;updates.append(c);objects.append(copy.deepcopy(obj))
            if not updates:self.busy=False;return
            req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.object_colors=updates;req.scene.world.collision_objects=objects
            self.apply_scene.call_async(req).add_done_callback(self.applied)
        except Exception as exc:self.busy=False;self.get_logger().warning('Display colour update skipped: '+str(exc))
    def applied(self,future):
        self.busy=False
        try:
            if not future.result().success:self.get_logger().warning('Object visibility update rejected')
        except Exception as exc:self.get_logger().warning('Object visibility update failed: '+str(exc))

def main():
    rclpy.init();node=ReconstructedModels()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
