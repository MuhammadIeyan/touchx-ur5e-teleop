"""Pure, bounded single-step planning for physical bench commissioning.

URDF serial-chain parsing/FK and local least-squares IK adapted from the user's
UR5e_Teleop archive, surgery_control/kinematics/chain.py (2026-11-18).
No ROS, activation, robot connection or force output in this module.
"""
import json
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
STEP_M = 0.0001
DURATION_S = 0.4
JOINT_STEP_RAD = 0.0005


def validate_payload(telemetry, manifest):
    mass = manifest['reported_payload_mass_kg']
    cog = np.asarray(manifest['reported_payload_cog_mm'], dtype=float) / 1000.
    actual_cog = np.asarray(telemetry['payload_cog'], dtype=float)
    if (manifest.get('payload_verified') is not True or not np.isfinite(mass) or mass <= 0
            or cog.shape != (3,) or actual_cog.shape != (3,)
            or not np.all(np.isfinite(cog)) or not np.all(np.isfinite(actual_cog))
            or not np.isfinite(telemetry['payload'])
            or abs(telemetry['payload'] - mass) > .005
            or not np.allclose(actual_cog, cog, atol=.002, rtol=0)):
        raise RuntimeError('Active payload differs from the verified commissioned assembly')


def validate_session(path, manifest, now=None):
    """Require a recent, recorded operator setup check; never infer clearance."""
    session = json.loads(path.read_text())
    now = time.time() if now is None else now
    age = now - session['confirmed_at_unix_s']
    if not 0 <= age <= 7200:
        raise ValueError('Bench session confirmation missing/expired (2 hours)')
    for key in ('operator_at_pendant', 'workspace_empty', 'external_control_host_confirmed'):
        if session.get(key) is not True:
            raise ValueError(f'Operator check required: {key}')
    clearance = session.get('minimum_clearance_mm', 0)
    if not np.isfinite(clearance) or clearance < 50:
        raise ValueError('First-step test requires measured clearance >=50 mm for tool and links')
    validate_commissioning(manifest)
    return session


def validate_commissioning(manifest):
    """Persistent assembly checks; this is not a renewed operator attestation."""
    for key in ('payload_verified', 'mounting_configuration_verified', 'calibration_match_verified'):
        if manifest.get(key) is not True:
            raise ValueError(f'Commissioning check required: {key}')
    if manifest['robot_ipv4'] != '192.168.1.103' or manifest['ros_domain_id'] != 43:
        raise ValueError('Unexpected robot identity/domain')


class Model:
    """Calibrated URDF model expressed in the controller's base frame."""
    def __init__(self, xml, expected_hash, expected_ip=None):
        tree = ET.fromstring(xml)
        hashes = [p.text for p in tree.findall('.//hardware/param')
                  if p.get('name') == 'kinematics/hash']
        if hashes != [expected_hash]:
            raise ValueError('Robot description calibration hash mismatch')
        if expected_ip is not None:
            addresses = [p.text for p in tree.findall('.//hardware/param') if p.get('name') == 'robot_ip']
            if addresses != [expected_ip]:
                raise ValueError('Robot description IP mismatch')
        self.by_child = {j.find('child').get('link'): j for j in tree.findall('joint')}
        self.names = JOINTS.copy()
        self.lower, self.upper = [], []
        by_name = {j.get('name'): j for j in self.by_child.values()}
        for name in self.names:
            j = by_name[name]
            if j.get('type') != 'revolute':
                raise ValueError('Expected six bounded UR revolute joints')
            self.lower.append(float(j.find('limit').get('lower')))
            self.upper.append(float(j.find('limit').get('upper')))
        self.lower, self.upper = np.array(self.lower), np.array(self.upper)
        self.tool_chain = self._chain('tool0')
        self.base_chain = self._chain('base')
        if any(axis is not None for _, axis, _ in self.base_chain):
            raise ValueError('Base frame must be fixed')
        self.to_base = np.linalg.inv(self._fk(self.base_chain, np.zeros(6)))

    def _chain(self, tip):
        result, seen = [], set()
        while tip in self.by_child:
            if tip in seen:
                raise ValueError('Cyclic URDF')
            seen.add(tip)
            j = self.by_child[tip]
            transform = np.eye(4)
            origin = j.find('origin')
            if origin is not None:
                transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                transform[:3, :3] = Rotation.from_euler(
                    'xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            axis, index = None, None
            if j.get('type') != 'fixed':
                index = self.names.index(j.get('name'))
                axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
                axis /= np.linalg.norm(axis)
            result.append((transform, axis, index))
            tip = j.find('parent').get('link')
        return list(reversed(result))

    @staticmethod
    def _fk(chain, q):
        result = np.eye(4)
        for origin, axis, index in chain:
            result = result @ origin
            if axis is not None:
                turn = np.eye(4)
                turn[:3, :3] = Rotation.from_rotvec(axis * q[index]).as_matrix()
                result = result @ turn
        return result

    def fk(self, q):
        return self.to_base @ self._fk(self.tool_chain, q)

    def plan(self, q, direction, orientation=None, step_m=None, joint_step_rad=None):
        step_m = STEP_M if step_m is None else float(step_m)
        joint_step_rad = (max(JOINT_STEP_RAD, 0.006 * (step_m / 0.001))
                          if joint_step_rad is None else float(joint_step_rad))
        q, direction = np.asarray(q, float), np.asarray(direction, float)
        if q.shape != (6,) or direction.shape != (3,) or not np.all(np.isfinite(np.r_[q, direction])):
            raise ValueError('Invalid step inputs')
        if np.linalg.norm(direction) < 1e-9:
            raise ValueError('Step direction is zero')
        if np.any(q <= self.lower + .01) or np.any(q >= self.upper - .01):
            raise ValueError('Too close to joint limit')
        start = self.fk(q)
        target = start.copy()
        orientation_limit = max(1e-5, 0.00025 * (step_m / 0.001))
        if orientation is not None:
            target[:3, :3] = orientation
            correction = np.linalg.norm(Rotation.from_matrix(target[:3,:3] @ start[:3,:3].T).as_rotvec())
            if correction > .005:
                raise ValueError('Fixed-orientation error exceeds 0.005 rad')
            orientation_limit += correction
        target[:3, 3] += step_m * direction / np.linalg.norm(direction)

        def residual(candidate):
            pose = self.fk(candidate)
            return np.r_[pose[:3, 3] - target[:3, 3],
                         Rotation.from_matrix(target[:3, :3] @ pose[:3, :3].T).as_rotvec()]

        solved = least_squares(residual, q, bounds=(q - joint_step_rad, q + joint_step_rad),
                               ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=60)
        end = solved.x
        if not solved.success or np.linalg.norm(residual(end)) > 1e-7:
            raise ValueError('Step IK unavailable within local joint bounds')
        # The controller interpolates quintically with zero velocity/acceleration
        # at both ends. Check the resulting joint path, not just its endpoint.
        for u in np.linspace(0, 1, 41):
            s = 10*u**3 - 15*u**4 + 6*u**5
            pose = self.fk(q + s*(end-q))
            if np.linalg.norm(pose[:3, 3] - start[:3, 3]) > step_m * 1.01:
                raise ValueError('Interpolated translation exceeds step envelope')
            angle = np.linalg.norm(Rotation.from_matrix(pose[:3, :3] @ start[:3, :3].T).as_rotvec())
            if angle > orientation_limit:
                raise ValueError('Interpolated orientation drift exceeds bound')
        return end, target


class GoalLease:
    """One goal per process, including late acceptance after a stop request."""
    def __init__(self):
        self.sent = False
        self.stop_requested = False
        self.handle = None
        self.terminal = False

    def reserve(self):
        if self.sent or self.stop_requested:
            raise RuntimeError('Single-step goal already used or stopped')
        self.sent = True

    def stop(self):
        self.stop_requested = True
        return self.handle is not None and not self.terminal

    def accepted(self, handle):
        self.handle = handle
        return self.stop_requested
