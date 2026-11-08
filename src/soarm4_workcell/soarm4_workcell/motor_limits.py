"""Exact servo_bridge conversion and non-clamping limits. No hardware writes."""
from pathlib import Path
import hashlib
import numpy as np
import yaml

class MotorLimits:
    def __init__(self):
        self.path=Path.home()/'robot_ws/src/soarm_bridge/config/soarm.yaml';raw=self.path.read_bytes();self.sha256=hashlib.sha256(raw).hexdigest();p=yaml.safe_load(raw)['servo_bridge']['ros__parameters']
        self.centers=np.array(p['centers'],float);self.signs=np.array(p['signs'],float);self.gain=np.array(p.get('positive_gain',[0]*4),float)
        self.low=np.array(p['min_counts'],float)+np.array(p.get('margin_lo',[0]*4));self.high=np.array(p['max_counts'],float)-np.array(p.get('margin_hi',[0]*4))
        ends=[]
        for c in [self.low,self.high]:
            q=self.signs*(c-self.centers)*2*np.pi/4096;ends.append(np.where(q>0,q/(1+self.gain),q))
        self.lo=np.minimum(*ends);self.hi=np.maximum(*ends)
    def counts(self,q):
        q=np.asarray(q,float);return self.centers+self.signs*np.where(q>0,q*(1+self.gain),q)*4096/(2*np.pi)
    def valid(self,q):
        c=self.counts(q);return bool(np.all(np.isfinite(c)) and np.all(c>=self.low-1e-7) and np.all(c<=self.high+1e-7))
    def intersect(self,arm):
        arm.lo=np.maximum(arm.lo,self.lo);arm.hi=np.minimum(arm.hi,self.hi)
        if np.any(arm.lo>=arm.hi):raise ValueError('Empty URDF/motor limit intersection')
