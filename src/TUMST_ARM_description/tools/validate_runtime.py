from pathlib import Path
import os,time,signal,subprocess,json,math,xml.etree.ElementTree as E
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
P=Path(__file__).resolve().parents[1];model=json.loads((P/'config/model.json').read_text())
def check(mode):
    node=rclpy.create_node('soarm4_validator_'+mode);state={};dynamic={};fixed={};description={}
    def recvstate(m):state['msg']=m
    def recvtf(m):
        for t in m.transforms:dynamic[t.child_frame_id]=t
    def recvstatic(m):
        for t in m.transforms:fixed[t.child_frame_id]=t
    def recvdescription(m):description['msg']=m
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    subs=[node.create_subscription(JointState,'/soarm4/joint_states',recvstate,10),node.create_subscription(TFMessage,'/soarm4/tf',recvtf,10),node.create_subscription(TFMessage,'/soarm4/tf_static',recvstatic,qos),node.create_subscription(String,'/soarm4/robot_description',recvdescription,qos)]
    log=open('/tmp/soarm4_v5_runtime_'+mode+'.log','w');launch=subprocess.Popen(['ros2','launch','soarm4_description','display.launch.py','mode:='+mode,'rviz:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        end=time.monotonic()+15
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            if state and description and all(j['child'] in dynamic for j in model['joints']) and 'robot_mount_fixed' not in fixed and 'base_link' in fixed and 'electronics_base_link' in fixed:break
            if launch.poll() is not None:raise RuntimeError('Launch failed: '+mode)
        assert state and description and dynamic and fixed
        expected=(model['poses']['home'] if mode in ('gui','home') else model['poses'][mode]);m=state['msg'];assert m.name==[j['name'] for j in model['joints']];error=max(abs(a-b) for a,b in zip(m.position,expected));assert error<(0.001 if mode=='gui' else 1e-9),(mode,m.position,expected)
        for j,q in zip(model['joints'],m.position):
            t=dynamic[j['child']];assert t.header.frame_id==j['parent'];pos=t.transform.translation;assert max(abs(a-b) for a,b in zip([pos.x,pos.y,pos.z],j['origin_xyz']))<1e-10
            z=t.transform.rotation;quat=[a*math.sin(q/2) for a in j['axis']]+[math.cos(q/2)];actual=[z.x,z.y,z.z,z.w];assert min(max(abs(a-b) for a,b in zip(actual,quat)),max(abs(a+b) for a,b in zip(actual,quat)))<1e-9
        robot=E.fromstring(description['msg'].data);assert robot.get('name')=='soarm4_checked_v5_7';assert all(len(l.findall('visual'))<=1 for l in robot.findall('link'))
        box=fixed['electronics_base_link'].transform.rotation;assert abs(box.w-1)<1e-10
        mount=fixed['base_link'].transform.rotation;assert abs(mount.z-math.sin(math.pi/4))<1e-10 and abs(mount.w-math.cos(math.pi/4))<1e-10
        assert len([t for t in fixed if t.endswith('_visual_link')])==13
        return {'launch':'PASS','joint_states_and_axes_tf':'PASS','electronics_fixed_and_robot_yaw90':'PASS','all_13_visual_fixed_frames':'PASS','correct_canonical_robot_description':'PASS','initial_angle_error_rad':error}
    finally:
        node.destroy_node()
        if launch.poll() is None:
            os.killpg(launch.pid,signal.SIGINT)
            try:launch.wait(timeout=7)
            except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=3)
        log.close()
if __name__=='__main__':
    rclpy.init()
    try:result={mode:check(mode) for mode in ['home','gui']}
    finally:rclpy.shutdown()
    (P/'validation_runtime.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
