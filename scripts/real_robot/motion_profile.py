"""Physical trajectory timing, independent of hand gain and workspace radius."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MotionProfile:
    speed_mm_s: float = 12.0

    def __post_init__(self):
        if not math.isfinite(self.speed_mm_s) or not 1.0 <= self.speed_mm_s <= 50.0:
            raise ValueError('Physical speed must be 1–50 mm/s')

    @property
    def speed_m_s(self):
        return self.speed_mm_s / 1000.0

    @property
    def duration_s(self):
        return min(.5, max(.15, .30 * 12.0 / self.speed_mm_s))

    @property
    def endpoint_step_m(self):
        # A rest-to-rest quintic peaks at 1.875 * distance / duration.
        # Existing nonzero derivatives still require full curve validation.
        return min(.0015, self.speed_m_s * self.duration_s / 1.875)


def duration_nanoseconds(seconds):
    if not math.isfinite(seconds) or not .15 <= seconds <= .5:
        raise ValueError('Invalid planned trajectory duration')
    return round(seconds * 1_000_000_000)
