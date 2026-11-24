"""Session-wide limits for small-segment physical teleoperation."""
import numpy as np
from scipy.spatial.transform import Rotation

class TravelBudget:
    def __init__(self, q, pose, total_budget_m=0.05, max_step_m=0.0001):
        self.q0_session=np.asarray(q,float).copy()  # Session start joints (never reset).
        self.q0=np.asarray(q,float).copy()          # Checkpoint origin (reset at each checkpoint).
        self.pose0=np.asarray(pose,float).copy()     # Checkpoint origin pose.
        self.total_budget_m=float(total_budget_m)
        self.max_step_m=float(max_step_m)
        self.commanded=0.
        self.measured=0.
        self.last_measured=self.pose0[:3,3].copy()
        self.checkpoints=0  # Number of re-arm events.

    def reset_origin(self, q, pose):
        """Re-arm from a new spatial origin.  Session-wide joint-displacement
        envelope is preserved (anchored to q0_session, never reset)."""
        q=np.asarray(q,float)
        pose=np.asarray(pose,float)
        if q.shape!=(6,) or pose.shape!=(4,4):
            raise ValueError('Invalid reset state')
        self.q0=q.copy()
        self.pose0=pose.copy()
        self.commanded=0.
        self.measured=0.
        self.last_measured=pose[:3,3].copy()
        self.checkpoints+=1

    def check(self,q,pose):
        q,pose=np.asarray(q,float),np.asarray(pose,float)
        if q.shape!=(6,) or pose.shape!=(4,4) or not np.all(np.isfinite(q)) or not np.all(np.isfinite(pose)):
            raise ValueError('Invalid budget state')
        # Joint-displacement envelope relative to session start — survives re-arms.
        if np.sum(np.abs(q-self.q0_session))>1.0:
            raise ValueError('Session joint-displacement envelope reached (1.0 rad)')
        # Checkpoint sphere — reset on each re-arm.
        if np.linalg.norm(pose[:3,3]-self.pose0[:3,3])>self.total_budget_m+1e-4:
            raise ValueError(f'{int(self.total_budget_m*1000)} mm checkpoint workspace reached')
        if np.linalg.norm(Rotation.from_matrix(pose[:3,:3]@self.pose0[:3,:3].T).as_rotvec())>.005:
            raise ValueError('Session orientation bound exceeded')

    def reserve(self,q,pose,length=None):
        self.check(q,pose)
        step_len = self.max_step_m if length is None else float(length)
        if not np.isfinite(step_len) or not 0<step_len<=self.max_step_m+1e-7 or self.commanded+step_len>self.total_budget_m+1e-7:
            raise ValueError(f'{int(self.total_budget_m*1000)} mm cumulative command budget exhausted')
        self.commanded+=step_len  # Never refunded on cancellation or clutch release.

    def observe(self,q,pose):
        self.check(q,pose)
        position=pose[:3,3]
        # Count observed travel above a 0.1 mm anchor spacing to avoid integrating
        # each encoder jitter sample. The command budget is the strict primary cap.
        distance=np.linalg.norm(position-self.last_measured)
        if distance>=.0001:
            self.measured+=distance
            self.last_measured=position.copy()
        if self.measured>self.total_budget_m:
            raise ValueError(f'{int(self.total_budget_m*1000)} mm measured travel budget exhausted')
