"""Register measured workcell TF, visual markers and MoveIt world objects.
No controller, trajectory, serial, vacuum or torque interface is used.
"""
import json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import TransformStamped,Pose,Point
from moveit_msgs.msg import CollisionObject,ObjectColor,PlanningScene,PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray
from tf2_ros import StaticTransformBroadcaster,Buffer,TransformListener
from .core import load,matrix,quaternion,rotation,translated,collision_geometry

def pose(T):
    T=matrix(T);m=Pose();m.position.x,m.position.y,m.position.z=map(float,T[:3,3]);m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w=map(float,quaternion(T[:3,:3]));return m

def tf(parent,child,T,stamp):
    p=pose(T);m=TransformStamped();m.header.frame_id=parent;m.header.stamp=stamp;m.child_frame_id=child;m.transform.translation.x=p.position.x;m.transform.translation.y=p.position.y;m.transform.translation.z=p.position.z;m.transform.rotation=p.orientation;return m

class WorkcellScene(Node):
    def __init__(self):
        super().__init__('workcell_scene');self.config=self.declare_parameter('config_file','').value;self.attach=self.declare_parameter('attach_camera_roots',False).value;self.d=load(self.config);self.geometry=collision_geometry(self.d)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.pub=self.create_publisher(MarkerArray,'workcell_markers',qos);self.status_pub=self.create_publisher(String,'workcell_status',qos);self.broadcaster=StaticTransformBroadcaster(self);self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.attach_done=set();self.static=[];self.scene_ready=False;self.future=None;self.phase=None;self.future_started=0.;self.last_verify=0.;self.stamp=self.get_clock().now().to_msg()
        self.static.append(tf('table','world',self.d['T_table_world'],self.stamp))
        for c,v in self.d['cameras'].items():self.static.append(tf('table',c+'_calibrated_optical',v['T_table_optical'],self.stamp))
        self.static.append(tf('table','turntable_axis',translated(self.d['turntable']['axis_point_table_m']),self.stamp))
        for key,v in self.d['pads'].items():self.static.append(tf('table','soarm4_'+key,translated(v['center_table_m']),self.stamp))
        self.broadcaster.sendTransform(self.static)
        self.apply=self.create_client(ApplyPlanningScene,'apply_planning_scene');self.get=self.create_client(GetPlanningScene,'get_planning_scene');self.timer=self.create_timer(1.,self.tick);self.marker_timer=self.create_timer(2.,self.markers)
        self.markers();self.status('Waiting for MoveIt scene; no motor commands')
    def status(self,text):
        self.status_pub.publish(String(data=text));self.get_logger().info(text)
    def request_scene(self):
        scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True
        for v in self.geometry:
            obj=CollisionObject();obj.id=v['id'];obj.header.frame_id='world';obj.operation=CollisionObject.ADD;p=SolidPrimitive();p.type=p.BOX if v['kind']=='box' else p.CYLINDER;p.dimensions=list(map(float,v['dimensions']));obj.primitives=[p];obj.primitive_poses=[pose(v['T_world_object'])];scene.world.collision_objects.append(obj)
            color=ObjectColor(id=obj.id);color.color.r,color.color.g,color.color.b,color.color.a=(.55,.58,.63,.7) if v['kind']=='box' else ((.9,.55,.12,.85) if v['id']=='workcell_turntable' else (.18,.65,.45,.85));scene.object_colors.append(color)
        return scene
    def tick(self):
        if self.attach:self.attach_cameras()
        if self.future is not None:
            if not self.future.done():
                if time.monotonic()-self.future_started>10:
                    self.future.cancel();self.future=None;self.phase=None;self.scene_ready=False;self.get_logger().warning('Scene service timed out; waiting and retrying')
                return
            try:
                result=self.future.result()
                if self.phase=='apply':
                    if not result.success:raise RuntimeError('MoveIt rejected workcell')
                    self.scene_ready=True;self.status('Applied 4 workcell collision objects; checking scene')
                else:
                    objects={o.id:o for o in result.scene.world.collision_objects};good=True
                    for v in self.geometry:
                        obj=objects.get(v['id'])
                        if obj is None or obj.header.frame_id!='world' or len(obj.primitives)!=1 or len(obj.primitive_poses)!=1 or not np.allclose(obj.primitives[0].dimensions,v['dimensions']):good=False;break
                        B=np.eye(4)
                        for P in [obj.pose,obj.primitive_poses[0]]:
                            q=P.orientation;part=translated([P.position.x,P.position.y,P.position.z]);part[:3,:3]=rotation([q.x,q.y,q.z,q.w]);B=B@part
                        if not np.allclose(B,v['T_world_object'],atol=1e-7):good=False;break
                    self.scene_ready=good
                    if good and not getattr(self,'confirmed',False):self.confirmed=True;self.status('Verified 4 workcell objects in MoveIt; placement provisional; execution not enabled by this node')
                    if not good:self.confirmed=False;self.status('Workcell objects missing or changed; reapplying')
            except Exception as e:self.scene_ready=False;self.get_logger().error(str(e))
            self.future=None;self.phase=None
        if not self.scene_ready and self.apply.service_is_ready():self.phase='apply';self.future=self.apply.call_async(ApplyPlanningScene.Request(scene=self.request_scene()));self.future_started=time.monotonic();return
        if self.scene_ready and self.get.service_is_ready() and time.monotonic()-self.last_verify>5:
            self.last_verify=time.monotonic();req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY;self.phase='verify';self.future=self.get.call_async(req);self.future_started=time.monotonic()
    def attach_cameras(self):
        for c,v in self.d['cameras'].items():
            if c in self.attach_done:continue
            try:
                # Use sensor-published extrinsics; never assign a second parent to optical frames.
                frames=__import__('yaml').safe_load(self.buffer.all_frames_as_yaml()) or {};parent=frames.get(v['camera_root_frame'],{}).get('parent')
                if parent and parent!='table':
                    self.get_logger().error(c+' root already has parent '+parent);continue
                msg=self.buffer.lookup_transform(v['optical_frame'],v['camera_root_frame'],rclpy.time.Time());q=msg.transform.rotation;t=msg.transform.translation;B=translated([t.x,t.y,t.z]);B[:3,:3]=rotation([q.x,q.y,q.z,q.w]);T=matrix(v['T_table_optical'])@B;self.static.append(tf('table',v['camera_root_frame'],T,self.get_clock().now().to_msg()));self.broadcaster.sendTransform(self.static);self.attach_done.add(c);self.status(c+' sensor root attached via published optical extrinsics')
            except Exception:pass
    def markers(self):
        arr=MarkerArray();seq=0
        def marker(name,typ,T,color=(.3,.65,.9,.9)):
            nonlocal seq
            m=Marker();m.header.frame_id='table';m.ns=name;m.id=seq;seq+=1;m.type=typ;m.action=Marker.ADD;m.pose=pose(T);m.color.r,m.color.g,m.color.b,m.color.a=color;m.frame_locked=True;arr.markers.append(m);return m
        def text(label,p,color=(.1,.1,.13,1.)):
            m=marker('labels',Marker.TEXT_VIEW_FACING,translated(p),color);m.text=label;m.scale.z=.018
        def line(points,name,color):
            m=marker(name,Marker.LINE_LIST,np.eye(4),color);m.scale.x=.0015;m.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in points];return m
        L,W=self.d['table']['size_xy_m'];grid=[]
        for x in np.arange(0,L+.001,.1):grid.extend([[x,0,0],[x,W,0]])
        for y in np.arange(0,W+.001,.1):grid.extend([[0,y,0],[L,y,0]])
        line(grid,'table_grid',(.45,.48,.5,.35));text('table origin / meters',[0,0,.025])
        for p,label,color in [([.2,0,0],'+X',(.85,.15,.12,1.)),([0,.2,0],'+Y',(.12,.65,.2,1.)),([0,0,.2],'+Z',(.15,.35,.9,1.))]:
            m=marker('axes',Marker.ARROW,np.eye(4),color);m.points=[Point(),Point(x=float(p[0]),y=float(p[1]),z=float(p[2]))];m.scale.x=.004;m.scale.y=.01;m.scale.z=.018;text(label,p)
        for id,p in self.d['fixed_markers']['centers_table_m'].items():
            m=marker('fixed_markers',Marker.CUBE,translated([p[0],p[1],.001]),(.18,.2,.24,.85));m.scale.x=m.scale.y=self.d['fixed_markers']['black_edge_m'];m.scale.z=.001;text('ID '+id,[p[0],p[1],.018])
        for name,key in [('turntable','turntable'),('pad1','pads'),('pad2','pads')]:
            v=self.d['turntable'] if key=='turntable' else self.d['pads'][name];p=v['axis_point_table_m'] if key=='turntable' else v['center_table_m'];text(name+' / z='+str(round(p[2],3)),[p[0],p[1],p[2]+.03]);m=marker('centers',Marker.SPHERE,translated(p),(.9,.2,.15,1.));m.scale.x=m.scale.y=m.scale.z=.007
        for c,v in self.d['cameras'].items():
            T=matrix(v['T_table_optical']);K=np.array(v['camera_info']['K']);w=v['camera_info']['width'];h=v['camera_info']['height'];length=.12;corners=[]
            for u,z in [(0,0),(w,0),(w,h),(0,h)]:corners.append(T[:3,:3]@np.array([(u-K[0,2])*length/K[0,0],(z-K[1,2])*length/K[1,1],length])+T[:3,3])
            pts=[]
            for i in range(4):pts.extend([T[:3,3],corners[i],corners[i],corners[(i+1)%4]])
            line(pts,c+'_frustum',(.7,.2,.65,1.) if c=='cam1' else (.35,.3,.8,1.));text(c+' / calibrated',T[:3,3]+[0,0,.025]);m=marker('camera_icons',Marker.CUBE,T,(.3,.35,.6,1.));m.scale.x=.045;m.scale.y=.02;m.scale.z=.02
        self.pub.publish(arr)

def main():
    rclpy.init();n=WorkcellScene()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
