"""Pure translational preview. No ROS, robot transport, or motor output."""
import numpy as np
from touch_clutch import Clutch

class Preview:
    def __init__(self, scale=0.1, radius=0.02, speed=0.005):
        if not all(np.isfinite(x) and x>0 for x in (scale,radius,speed)) or scale>0.2 or radius>0.02 or speed>0.005:
            raise ValueError('Preview limits: scale<=0.2, radius<=20 mm, speed<=5 mm/s')
        self.clutch=Clutch(scale=scale,rot_scale=0)
        self.radius,self.speed=radius,speed
        self.target=None
        self.origin=None
        self.last=None

    def update(self, now, xyz, buttons, calibrated, epoch, measured_tcp, fresh=True):
        tcp=np.asarray(measured_tcp,dtype=float)
        if tcp.shape!=(3,) or not np.all(np.isfinite(tcp)):
            self.clutch.reset();self.last=None;return None
        dt=now-self.last if self.last is not None else 0
        self.last=now
        if not fresh or dt<0 or dt>0.1:
            self.clutch.reset();self.target=None;self.origin=None;return None
        was_active=self.clutch.active
        target=self.clutch.update(xyz,buttons,calibrated,epoch,tcp)
        if target is None:
            self.target=None;self.origin=None;return None
        if not was_active:
            self.origin=tcp.copy();self.target=tcp.copy()
        delta=target-self.origin
        norm=np.linalg.norm(delta)
        if norm>self.radius:delta*=self.radius/norm
        step=self.origin+delta-self.target
        length=np.linalg.norm(step)
        if length>self.speed*dt and length>0:step*=self.speed*dt/length
        self.target+=step
        return self.target.copy()
