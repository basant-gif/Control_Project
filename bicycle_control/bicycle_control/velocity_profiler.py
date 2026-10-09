"""
Target Velocity Profiler based on track curvature.
Calculates maximum safe cornering speeds subject to lateral acceleration limits.
"""

import math


class VelocityProfiler:
    """Generates target speed profiles based on track curvature or precomputed data."""

    def __init__(self, default_speed=4.0, max_speed=8.0, max_lat_accel=5.0):
        self.default_speed = default_speed
        self.max_speed = max_speed
        self.max_lat_accel = max_lat_accel

    def compute_target_speed(self, kappa, fallback_speed=None):
        """Calculates curvature-limited velocity."""

        # Near-zero curvature means a straight section.
        # Use the maximum configured speed.
        if abs(kappa) < 1e-6:
            return float(self.max_speed)

        # Curvature-limited speed:
        # v_max = sqrt(a_lat_max / |kappa|)
        target_speed = math.sqrt(
            self.max_lat_accel / abs(kappa)
        )

        # Clamp to the allowed speed range.
        return float(
            max(0.0, min(target_speed, self.max_speed))
        )
