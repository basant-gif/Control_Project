"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math  # noqa: F401
import numpy as np  # noqa: F401


class PurePursuitController:
    """Adaptive Pure Pursuit lateral controller."""

    def __init__(self, wheelbase=1.25, kv=0.25, l_min=0.8, l_max=2.5,
                 max_steer_rad=math.radians(35.0)):
        self.L = wheelbase
        self.kv = kv
        self.l_min = l_min
        self.l_max = l_max
        self.max_steer_rad = max_steer_rad

    def compute_lookahead(self, v):
        """Adaptive lookahead distance: Ld = clip(kv * v + l_min, l_min, l_max)."""
        # TODO: Milestone 5.3 Step 1 — Adaptive Lookahead Horizon
        # The car looks further ahead at higher speeds to plan smoother turns.
        # Implement the speed-scaled lookahead formula and clamp it to the allowed range.
        # Speed-dependent lookahead.
        lookahead = self.kv * max(float(v), 0.0) + self.l_min

        # Keep lookahead inside the configured bounds.
        lookahead = float(
            np.clip(
                lookahead,
                self.l_min,
                self.l_max
            )
        )

        return lookahead

    def find_target_waypoint(self, x, y, path_points, lookahead):
        """Searches along path for the target waypoint at lookahead distance."""
        # TODO: Milestone 5.3 Step 2 — Target Waypoint Selection
        # This selects the goal point the car will steer toward.
        # Find the nearest waypoint on the path, then walk forward until
        # you reach one that is at least 'lookahead' meters away.
        if len(path_points) == 0:
            return 0, (x, y)

        # Find the nearest waypoint to the vehicle.
        nearest_idx = 0
        min_dist_sq = float('inf')

        for i, pt in enumerate(path_points):
            dx = pt[0] - x
            dy = pt[1] - y
            dist_sq = dx * dx + dy * dy

            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                nearest_idx = i

        # Walk forward around the track until the waypoint is
        # at least the requested lookahead distance away.
        n = len(path_points)

        for offset in range(n):
            idx = (nearest_idx + offset) % n

            dx = path_points[idx][0] - x
            dy = path_points[idx][1] - y
            distance = math.hypot(dx, dy)

            if distance >= lookahead:
                return idx, path_points[idx]

        # Fallback if the whole path is shorter than the lookahead.
        return nearest_idx, path_points[nearest_idx]

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        """Computes steering angle in radians using Pure Pursuit geometry."""
        # TODO: Milestone 5.3 Steps 3 & 4 — Coordinate Transformation & Arc Law
        # This is the core of Pure Pursuit: transform the target into the vehicle's
        # local frame, then use the arc geometry formula to compute the steering angle.
        if target_pt is None:
            return 0.0

        # Target point in world coordinates.
        tx, ty = target_pt[0], target_pt[1]

        # Relative position from vehicle to target.
        dx = tx - x
        dy = ty - y

        # Transform target point into the vehicle local frame.
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        local_y = -sin_yaw * dx + cos_yaw * dy

        # Protect against an invalid/zero lookahead.
        lookahead = max(float(lookahead), 1e-6)

        # Pure Pursuit curvature.
        curvature = 2.0 * local_y / (lookahead * lookahead)

        # Convert curvature to front-wheel steering.
        delta = math.atan(self.L * curvature)

        # Enforce steering limits.
        delta = float(
            np.clip(
                delta,
                -self.max_steer_rad,
                self.max_steer_rad
            )
        )

        return delta
