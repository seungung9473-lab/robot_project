#!/usr/bin/env python3
"""Read STS positions only: no torque, position, mode or EEPROM write commands."""
import math,time,json,signal,sys
from pathlib import Path
import serial,yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_share_directory

class ReadOnlyBus:
    def __init__(self,port,baud):
        self.ser=serial.Serial(port,baudrate=baud,timeout=.002,write_timeout=.05,exclusive=True)
    def read(self,sid,addr,n):
        # Only INST_READ (0x02) is constructed here. This class has no actuator methods.
        body=bytes([sid,4,2,addr,n]);packet=b'\xff\xff'+body+bytes([(~sum(body))&255])
        self.ser.reset_input_buffer();self.ser.write(packet);self.ser.flush()
        buf=bytearray();deadline=time.monotonic()+.025
        while time.monotonic()<deadline:
            buf.extend(self.ser.read(self.ser.in_waiting or 1));k=buf.find(b'\xff\xff')
            if k<0:continue
            if len(buf)<k+4:continue
            length=buf[k+3]
            if length<2 or length>80:del buf[:k+2];continue
            if len(buf)<k+4+length:continue
            frame=bytes(buf[k:k+4+length]);del buf[:k+4+length]
            if frame[2]!=sid or ((~sum(frame[2:-1]))&255)!=frame[-1]:continue
            if len(frame[5:-1])!=n:continue
            if frame[4]:raise RuntimeError(f'ID {sid} status error 0x{frame[4]:02x}')
            return frame[5:-1]
        raise RuntimeError(f'ID {sid} response timeout')
    def close(self):self.ser.close()

class LiveEncoders(Node):
    def __init__(self):
        super().__init__('live_encoders')
        port=self.declare_parameter('port','/dev/servo_bus').value
        baud=self.declare_parameter('baud',1000000).value
        rate=self.declare_parameter('rate_hz',20.).value
        d=Path(get_package_share_directory('soarm4_description'))
        self.model=json.loads((d/'config/model.json').read_text())
        self.mapping=yaml.safe_load((d/'config/servo_map.yaml').read_text())['joints']
        self.names=[j['name'] for j in self.model['joints']]
        self.limits={j['name']:(float(j['limit']['lower']),float(j['limit']['upper'])) for j in self.model['joints']}
        self.bus=ReadOnlyBus(port,baud);self.pub=self.create_publisher(JointState,'joint_states',10)
        self.status_pub=self.create_publisher(Marker,'live_status',10)
        self.timer=self.create_timer(1./rate,self.poll);self.last_state=None;self.last_log=0.;self.last_error=None;self.counts={}
        self.get_logger().info('READ-ONLY live feedback on '+port+'; no torque/goal changes. Stop other serial programs first.')
    def status(self,text,error=False):
        m=Marker();m.header.frame_id='world';m.ns='live_status';m.id=0;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD;m.pose.orientation.w=1.;m.pose.position.x=-.02;m.pose.position.y=.06;m.pose.position.z=.32;m.scale.z=.0095;m.color.a=1.;m.color.r=1. if error else .05;m.color.g=.1 if error else .45;m.color.b=.1 if error else .1;m.text=text;self.status_pub.publish(m)
    def poll(self):
        positions=[];velocities=[];counts=[]
        try:
            for name in self.names:
                cfg=self.mapping[name];data=self.bus.read(cfg['id'],56,8);count=int.from_bytes(data[:2],'little')
                if count>4095:raise RuntimeError(f'{name}: unexpected position {count}; check motor mode/calibration')
                speed=int.from_bytes(data[2:4],'little');speed=-(speed&0x7fff) if speed&0x8000 else speed
                positions.append(float(cfg['sign']*(count-cfg['center'])*2*math.pi/4096));velocities.append(float(cfg['sign']*speed*2*math.pi/4096));counts.append(count)
        except (RuntimeError,serial.SerialException,OSError) as exc:
            msg='LIVE FEEDBACK LOST\n'+str(exc)+'\nLast robot display is stale'
            self.status(msg,True)
            if str(exc)!=self.last_error:self.get_logger().error(msg);self.last_error=str(exc)
            return
        msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=self.names;msg.position=positions;msg.velocity=velocities;self.pub.publish(msg)
        outside=[name for name,q in zip(self.names,positions) if not self.limits[name][0]-1e-9<=q<=self.limits[name][1]+1e-9]
        # Do not clamp measured positions: the display must reflect the real encoders.
        self.status('LIVE encoders\n'+' | '.join(str(c) for c in counts)+ ('\nOutside plan limits: '+', '.join(outside) if outside else ''),bool(outside))
        if self.last_state is None or self.last_error is not None or time.monotonic()-self.last_log>5:
            self.get_logger().info(json.dumps({'counts':counts,'angles_deg':[round(math.degrees(q),3) for q in positions],'outside_planning_limits':outside}));self.last_log=time.monotonic()
        self.last_state=msg;self.last_error=None;self.counts=dict(zip(self.names,counts))
    def destroy_node(self):
        self.bus.close();super().destroy_node()

def main():
    rclpy.init();node=None
    try:
        node=LiveEncoders();rclpy.spin(node)
    except KeyboardInterrupt:pass
    except (serial.SerialException,OSError) as exc:print('Cannot open servo bus: '+str(exc),flush=True)
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node:node.destroy_node()
        rclpy.try_shutdown()
if __name__=='__main__':main()
