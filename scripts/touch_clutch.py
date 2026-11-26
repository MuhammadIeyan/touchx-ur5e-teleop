"""Pure clutch/deadman logic with 6-DOF support; no ROS or hardware dependencies."""
import time
import numpy as np


def _quat_to_mat(qxyzw):
    """Convert [qx, qy, qz, qw] quaternion to 3x3 rotation matrix."""
    if np.shape(qxyzw)!=(4,) or not np.all(np.isfinite(qxyzw)) or abs(np.linalg.norm(qxyzw)-1)>0.02:
        raise ValueError("Invalid quaternion")
    x, y, z, w = qxyzw
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm > 1e-12:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def _scale_rotation(R, factor):
    """Scale a rotation by *factor* via axis-angle interpolation toward identity."""
    cos_a = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_a)
    if angle < 1e-10:
        return np.eye(3)
    if np.pi-angle < 1e-6:
        # Eigenvector for eigenvalue one remains well conditioned at pi.
        values, vectors = np.linalg.eigh((R+R.T)/2)
        axis = vectors[:, np.argmax(values)]
    else:
        axis = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])/(2*np.sin(angle))
    scaled = angle * factor
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(scaled) * K + (1 - np.cos(scaled)) * (K @ K)


class Clutch:
    """Incremental clutch supporting 3-DOF translation and full 6-DOF teleop,
    with support for position-tracking and continuous rate (velocity) teleoperation."""

    def __init__(self, scale=0.2, rot_scale=1.0, mode='position',
                 deadband=0.003, max_linear_speed=0.08, rate_gain=2.5):
        if not np.isfinite(scale) or not 0<scale<=20 or not np.isfinite(rot_scale) or not 0<=rot_scale<=1:
            raise ValueError("Scales must be finite; translation in (0,5], rotation in [0,1]")
        if mode not in ('position', 'rate'):
            raise ValueError("Mode must be 'position' or 'rate'")
        self.scale = scale
        self.rot_scale = rot_scale
        self.mode = mode
        self.deadband = float(deadband)
        self.max_linear_speed = float(max_linear_speed)
        self.rate_gain = float(rate_gain)
        self.mapping = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
        self.active = False
        self.released = False
        self.anchor_touch = None
        self.anchor_tip = None
        self.anchor_device_rot = None
        self.anchor_tip_rot = None
        self.current_tip = None
        self.current_velocity = np.zeros(3)
        self.last_update_time = None
        self.previous = None
        self.epoch = None

    def reset(self):
        self.active = False
        self.released = False
        self.previous = None
        self.current_tip = None
        self.current_velocity = np.zeros(3)
        self.last_update_time = None

    def update(self, xyz, buttons, calibrated, epoch, tip,
               device_quat=None, tip_rot=None, valid=True):
        """Update clutch state and compute targets.

        If device_quat and tip_rot are omitted (None), returns pos_target for
        3-DOF callers. If provided, returns (pos_target, rot_target).
        Returns None when disengaged or on fault.
        """
        xyz, tip = np.asarray(xyz, dtype=float), np.asarray(tip, dtype=float)
        if xyz.shape!=(3,) or tip.shape!=(3,) or len(buttons)!=2:
            self.reset()
            return None
        if device_quat is not None or tip_rot is not None:
            try:
                _quat_to_mat(device_quat)
                r = np.asarray(tip_rot, dtype=float)
                if r.shape!=(3,3) or not np.all(np.isfinite(r)) or not np.allclose(r.T@r,np.eye(3),atol=0.02) or abs(np.linalg.det(r)-1)>0.02:
                    raise ValueError('Invalid rotation')
            except (ValueError, TypeError):
                self.reset()
                return None
        jump = self.previous is not None and np.linalg.norm(xyz - self.previous) > 0.025
        epoch_changed = self.epoch is not None and epoch != self.epoch
        self.epoch = epoch
        if (not valid or not calibrated or xyz.shape != (3,) or tip.shape != (3,)
                or not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(tip))
                or jump or epoch_changed):
            self.reset()
            return None
        self.previous = xyz.copy()
        deadman, clutch = bool(buttons[0]), bool(buttons[1])
        if not deadman:
            self.active = False
            self.released = not clutch
            self.current_velocity = np.zeros(3)
            return None
        if clutch:
            self.reset()
            return None
        if not self.active:
            if not self.released:
                return None
            self.anchor_touch, self.anchor_tip = xyz.copy(), tip.copy()
            self.current_tip = tip.copy()
            self.current_velocity = np.zeros(3)
            self.last_update_time = time.monotonic()
            if device_quat is not None and tip_rot is not None:
                dq = np.asarray(device_quat, dtype=float)
                self.anchor_device_rot = _quat_to_mat(dq)
                self.anchor_tip_rot = np.asarray(tip_rot, dtype=float).copy()
            else:
                self.anchor_device_rot = None
                self.anchor_tip_rot = None
            self.active, self.released = True, False

        now = time.monotonic()
        dt = 0.05
        if self.last_update_time is not None:
            dt = max(0.001, min(0.1, now - self.last_update_time))
        self.last_update_time = now

        if self.mode == 'rate':
            delta_touch = self.mapping @ (xyz - self.anchor_touch)
            dist = np.linalg.norm(delta_touch)
            if dist <= self.deadband:
                target_vel = np.zeros(3)
            else:
                unit = delta_touch / dist
                speed = min(self.max_linear_speed, self.rate_gain * (dist - self.deadband))
                target_vel = unit * speed
            alpha = min(1.0, dt / 0.04)
            self.current_velocity = (1.0 - alpha) * self.current_velocity + alpha * target_vel
            if self.current_tip is None:
                self.current_tip = tip.copy()
            self.current_tip = self.current_tip + self.current_velocity * dt
            pos_target = self.current_tip.copy()
        else:
            pos_target = self.anchor_tip + self.scale * (self.mapping @ (xyz - self.anchor_touch))

        # If caller did not provide orientation, return pos_target (3-DOF backward compatibility)
        if device_quat is None or tip_rot is None:
            return pos_target

        # 6-DOF orientation target
        rot_target = self.anchor_tip_rot
        dq = np.asarray(device_quat, dtype=float)
        if dq.shape == (4,) and np.all(np.isfinite(dq)) and self.anchor_device_rot is not None:
            device_rot = _quat_to_mat(dq)
            # Relative rotation in device frame: R_rel = R_cur @ R_anchor^T
            R_rel = device_rot @ self.anchor_device_rot.T
            # Transform rotation to robot base frame: M @ R_rel @ M^T
            R_rel_robot = self.mapping @ R_rel @ self.mapping.T
            if abs(self.rot_scale - 1.0) > 1e-6:
                R_rel_robot = _scale_rotation(R_rel_robot, self.rot_scale)
            rot_target = R_rel_robot @ self.anchor_tip_rot

        return pos_target, rot_target
