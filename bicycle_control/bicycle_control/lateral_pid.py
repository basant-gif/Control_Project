"""
High-Level Lateral Steering Controller: Reactive Lateral PID.
Steers based on instantaneous Cross-Track Error (CTE) and Heading Error.
"""

import math
import numpy as np  # noqa: F401


class LateralPIDController:
    """Lateral PID steering controller based on Cross-Track Error (CTE) and Heading Error.

    Commands front wheel steering based on instantaneous lateral offset (cross-track error)
    and orientation error relative to the nearest path waypoint.
    """

    def __init__(self, kp=0.8, ki=0.02, kd=0.15, k_yaw=0.5, dt=0.1,
                 max_steer_rad=math.radians(35.0), integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.k_yaw = k_yaw
        self.dt = dt
        self.max_steer_rad = max_steer_rad
        self.integral_limit = integral_limit

        self.integral_cte = 0.0
        self.prev_cte = 0.0

    def compute_steering(self, cte, heading_err, v=0.0):
        """Computes front wheel steering angle delta in radians.

        Args:
            cte: Signed cross-track error in meters (positive = vehicle is left of path).
            heading_err: Heading error in radians (psi_vehicle - psi_path).
            v: Longitudinal speed in m/s, used for the analytic CTE derivative.

        Returns:
            delta_rad: Commanded front steering angle in radians [-max_steer_rad, max_steer_rad].
        """
        # TODO: Milestone 5.2 — Reactive Lateral PID Controller
        # This is the lateral steering controller. It corrects for how far the car
        # is off the path (CTE) and how misaligned its heading is.
        # Implement PID on the CTE with anti-windup, add a heading correction term,
        # and clamp the output to the steering limits.
        # Signed CTE error.
        error = cte

        # Analytic derivative of the cross-track error. For a kinematic
        # bicycle referenced at the rear axle, d(cte)/dt = v * sin(e_psi)
        # exactly. A finite difference of the measured CTE is used instead
        # in many implementations, but at 10 Hz it amplifies CTE estimation
        # noise by 1/dt = 10 and dominates the steering command.
        derivative_cte = v * math.sin(heading_err)

        # Candidate integral update.
        p_term = self.kp * error
        d_term = self.kd * derivative_cte
        candidate_integral = self.integral_cte + error * self.dt

        # Two-stage anti-windup: only accumulate integral when the
        # unsaturated effort is not pushing further into saturation.
        effort_unsaturated = p_term + self.ki * candidate_integral + d_term
        pushing_into_upper = (
            effort_unsaturated > self.max_steer_rad and error > 0.0
        )
        pushing_into_lower = (
            effort_unsaturated < -self.max_steer_rad and error < 0.0
        )
        if not (pushing_into_upper or pushing_into_lower):
            self.integral_cte = float(
                np.clip(
                    candidate_integral,
                    -self.integral_limit,
                    self.integral_limit
                )
            )

        # PID contribution from cross-track error.
        pid_term = (
            p_term
            + self.ki * self.integral_cte
            + d_term
        )

        # Heading correction.
        heading_term = self.k_yaw * heading_err

        # Positive CTE/heading error means the vehicle is to the
        # left of the desired path, so steer right (negative delta).
        delta = -(pid_term + heading_term)

        # Store current error for next derivative calculation.
        self.prev_cte = error

        # Enforce steering limits.
        delta = float(
            np.clip(
                delta,
                -self.max_steer_rad,
                self.max_steer_rad
            )
        )

        return delta

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral_cte = 0.0
        self.prev_cte = 0.0
