"""One-shot encoder-distance trigger; never uses pressure or stops motion."""
import numpy as np
class PreSuctionTrigger:
    def __init__(self,target,normal,send,threshold=.005):
        self.target=np.asarray(target,float);self.normal=np.asarray(normal,float)
        self.normal/=np.linalg.norm(self.normal)
        self.send=send;self.threshold=float(threshold);self.future=None;self.distance=None
    def update(self,tcp):
        distance=float((np.asarray(tcp)-self.target)@self.normal)
        if self.future is None and distance<=self.threshold:
            self.distance=distance;self.future=self.send()
