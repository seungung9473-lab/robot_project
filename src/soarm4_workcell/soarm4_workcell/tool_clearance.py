"""Conservative tool AABB clearance against registered obstacle AABBs.
Includes all cup parts, nominal TCP, and supplied calibration pin. Not a
continuous collision proof, nor a substitute for calibrated robot geometry.
"""
from pathlib import Path
import struct
import numpy as np
from .core import collision_geometry

def stl_bounds(path,scale):
    raw=Path(path).read_bytes();count=struct.unpack_from('<I',raw,80)[0]
    if len(raw)!=84+50*count:raise ValueError('Binary STL required: '+str(path))
    dt=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')]);v=np.frombuffer(raw,offset=84,dtype=dt)['vertices'].reshape(-1,3)*scale
    return v.min(0),v.max(0)

def corners(lo,hi):return np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
def aabb_distance(a,b):
    return float(np.linalg.norm(np.maximum(np.maximum(b[0]-a[1],a[0]-b[1]),0)))

class ToolClearance:
    def __init__(self,arm,wc,W,tcp,margin=.02):
        self.arm=arm;self.W=W;self.margin=margin;bounds=[]
        root=Path.home()/'robot_ws/src/soarm4_description/meshes'
        self.sources=[]
        for name in ['tool_link_print.stl','tool_link_alu.stl','tool_link_cup.stl']:
            path=root/name;bounds.append(stl_bounds(path,1.));self.sources.append(str(path))
        path=Path.home()/'바탕화면/TUMST_ARM_cal_pin_v3_1/TUMST_ARM_cal_pin_v3.stl'
        bounds.append(stl_bounds(path,.001));self.sources.append(str(path))
        bounds.append((tcp[:3],tcp[:3]));lo=np.min([b[0] for b in bounds],0);hi=np.max([b[1] for b in bounds],0);self.tool_corners=corners(lo,hi)
        self.obstacles=[]
        for o in collision_geometry(wc):
            if o['kind']=='box':half=np.array(o['dimensions'])/2
            else:
                height,radius=o['dimensions'];half=np.array([radius,radius,height/2])
            C=corners(-half,half);T=W@np.array(o['T_world_object']);P=C@T[:3,:3].T+T[:3,3];self.obstacles.append((o['id'],(P.min(0),P.max(0))))
    def check(self,q):
        T=self.W@self.arm.fk(q);p=self.tool_corners@T[:3,:3].T+T[:3,3];box=(p.min(0),p.max(0));d,name=min((aabb_distance(box,b),name) for name,b in self.obstacles)
        return d>=self.margin,d,name
