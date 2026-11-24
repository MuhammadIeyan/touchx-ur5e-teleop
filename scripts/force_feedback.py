"""Translation force mapping: power-conjugate at gain=1; SI units, no torque rendering."""
import numpy as np

def validate_tuning(force_gain=1.0, force_limit=0.5, max_linear_speed=0.015, max_joint_speed=0.1):
    for name, value, low, high in (
        ('force_gain', force_gain, 0.1, 3.0),
        ('force_limit', force_limit, 0.05, 1.0),
        ('max_linear_speed', max_linear_speed, 0.001, 0.20),
        ('max_joint_speed', max_joint_speed, 0.01, 1.50),
    ):
        if not np.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{name} must be finite and within [{low}, {high}]')


def reflected_force(world_force, scale, mapping, gain=1, limit=0.5):
    validate_tuning(force_gain=gain, force_limit=limit)
    f = np.asarray(world_force, dtype=float)
    if f.shape != (3,) or not np.all(np.isfinite(f)) or not np.isfinite(scale) or not 0 < scale <= 1:
        raise ValueError('Invalid force or motion scale')
    result = gain * scale * (mapping.T @ f)
    magnitude = np.linalg.norm(result)
    if magnitude > limit:
        result *= limit/magnitude
    return result
