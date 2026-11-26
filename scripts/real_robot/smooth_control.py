"""ROS-independent hold gesture and finite, smooth stopping trajectories."""
import numpy as np


class HoldGesture:
    """A hold moves; three short taps REQUEST RCM while released, never enable it."""
    def __init__(self):
        self.down=False
        self.pressed_at=0.
        self.last_release=None
        self.taps=0
        self.released=False

    def update(self,down,now,valid=True):
        if not valid:
            self.__init__()
            return False,False
        request=False
        if down and not self.down:
            if self.last_release is None or now-self.last_release>.35:self.taps=0
            self.pressed_at=now
        if not down:
            if self.down:
                if now-self.pressed_at<=.20:
                    self.taps+=1
                    if self.taps==3:
                        request=True
                        self.taps=0
                else:self.taps=0
                self.last_release=now
            self.released=True
        self.down=bool(down)
        held=self.down and self.released and now-self.pressed_at>=.25
        if held:self.taps=0
        return held,request


class JointDynamicsLimit(ValueError):
    """A candidate violates the unchanged joint velocity/acceleration bounds."""


class Quintic:
    """Preserve incoming q/v/a, end at target with zero velocity/acceleration."""
    def __init__(self,q,v,a,end,duration=.30):
        q,v,a,end=[np.asarray(x,float) for x in (q,v,a,end)]
        if any(x.shape!=(6,) or not np.all(np.isfinite(x)) for x in (q,v,a,end)):
            raise ValueError('Invalid trajectory state')
        if not .15<=duration<=.5:raise ValueError('Invalid stopping horizon')
        self.duration=duration
        t=duration
        self.c=np.zeros((6,6))
        self.c[:3]=[q,v,a/2]
        self.c[3:]=np.linalg.solve(np.array([[t**3,t**4,t**5],
                    [3*t*t,4*t**3,5*t**4],[6*t,12*t*t,20*t**3]]),
                    np.array([end-q-v*t-a*t*t/2,-v-a*t,-a]))
        self.turning_times=[]
        for j in range(6):
            roots=np.polynomial.polynomial.polyroots(np.arange(1,6)*self.c[1:,j])
            self.turning_times.append([float(r.real) for r in roots
                                       if abs(r.imag)<1e-9 and 0<r.real<duration])

    def sample(self,t):
        t=float(np.clip(t,0,self.duration))
        return (np.array([1,t,t*t,t**3,t**4,t**5])@self.c,
                np.array([0,1,2*t,3*t*t,4*t**3,5*t**4])@self.c,
                np.array([0,0,2,6*t,12*t*t,20*t**3])@self.c)

    def joint_travel(self,start,end):
        """Total absolute joint travel, including reversals between samples."""
        travel=0.
        for j in range(6):
            times=sorted([start,end]+[t for t in self.turning_times[j] if start<t<end])
            positions=np.polynomial.polynomial.polyval(times,self.c[:,j])
            travel+=np.abs(np.diff(positions)).sum()
        return travel

    def validate(self,model,origin,orientation,workspace_m,*,max_cartesian_speed=.012):
        from scipy.spatial.transform import Rotation
        if not np.isfinite(max_cartesian_speed) or not .001<=max_cartesian_speed<=.050:
            raise ValueError('Physical speed must be 1–50 mm/s')
        times=np.linspace(0,self.duration,31)
        states=[self.sample(t) for t in times]
        if any(np.max(np.abs(v))>.08 or np.max(np.abs(a))>.8 for _,v,a in states):
            raise JointDynamicsLimit('Joint rate/acceleration limit')
        xyz=[]
        for q,v,a in states:
            if np.any(q<model.lower+.01) or np.any(q>model.upper-.01):raise ValueError('Joint limit')
            pose=model.fk(q);xyz.append(pose[:3,3])
            if np.linalg.norm(pose[:3,3]-origin)>workspace_m+1e-6:raise ValueError('Workspace boundary')
            if np.linalg.norm(Rotation.from_matrix(pose[:3,:3]@orientation.T).as_rotvec())>.005:
                raise ValueError('Orientation bound')
        xyz=np.array(xyz)
        if np.max(np.linalg.norm(xyz-xyz[0],axis=1))>.002:raise ValueError('Outstanding motion exceeds 2 mm')
        if np.max(np.linalg.norm(np.diff(xyz,axis=0)/(times[1]-times[0]),axis=1))>max_cartesian_speed:
            raise ValueError('Cartesian speed bound')
        return states


class ActionWindow:
    """One acceptance in flight; retain all accepted handles until terminal."""
    def __init__(self):
        self.sequence=0
        self.pending=None
        self.handles={}
        self.stopping=False

    def begin(self):
        if self.pending is not None or self.stopping:raise RuntimeError('Acceptance pending or stopping')
        self.sequence+=1;self.pending=self.sequence
        return self.sequence

    def accept(self,seq,handle):
        if seq!=self.pending:raise RuntimeError('Unexpected acceptance')
        self.pending=None
        self.handles[seq]=handle
        return self.stopping

    def terminal(self,seq):
        self.handles.pop(seq,None)

    def drained(self):return self.pending is None and not self.handles
