"""Process-isolated planning. This module cannot publish or activate a robot."""
import time
import numpy as np
from real_robot.bench_motion import Model
from real_robot.path_geometry import PathGeometry
from real_robot.smooth_control import Quintic

_model = _geometry = None


def initialize(xml, calibration_hash, robot_ip, initial_q):
    global _model, _geometry
    _model = Model(xml, calibration_hash, robot_ip)
    _geometry = PathGeometry(_model, xml, initial_q)
    _geometry.check(initial_q)
    return True


def build_plan(model, geometry, request):
    started = time.monotonic()
    q, v, a = request['reference']
    delta = request['target'] - model.fk(q)[:3, 3]
    distance = np.linalg.norm(delta)
    end = q.copy() if distance < 1e-9 else model.plan(
        q, delta, orientation=request['origin'][:3, :3],
        step_m=min(distance, .0015))[0]
    curve = Quintic(q, v, a, end)
    curve.validate(model, request['origin'][:3, 3], request['origin'][:3, :3], request['workspace_m'])
    geometry.curve(curve)
    return dict(end=end, planning_s=time.monotonic()-started)


def plan(request):
    if _model is None or _geometry is None:
        raise RuntimeError('Planner has not initialized')
    return build_plan(_model, _geometry, request)


def rejection(request, generation, reference_at, target, actual_q, now):
    """Recheck authorization and the ORIGINAL snapshot, not just fresh callbacks."""
    if request['generation'] != generation:
        return 'engagement changed'
    if now-request['reference_at'] >= .1 or now-reference_at >= .1:
        return 'plan expired'
    if np.max(np.abs(request['reference'][0]-actual_q)) > .01:
        return 'robot moved beyond planned reference'
    if np.linalg.norm(request['target']-target) > .005:
        return 'hand target changed'
    return None
