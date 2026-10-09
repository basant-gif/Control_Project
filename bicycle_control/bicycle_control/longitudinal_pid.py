"""
Low-Level Powertrain Cruise Controller (Longitudinal PID).
Regulates vehicle speed via normalized throttle/braking effort.
"""

import numpy as np  # noqa: F401


class PIDLongitudinalController:
    """Low-Level Powertrain Cruise Controller / Electronic Speed Control (ESC).

    Translates high-level velocity requests into normalized throttle/brake effort.
    Because physical vehicles experience friction and speed-squared aerodynamic drag,
    a closed-loop speed regulator is required to maintain target velocity.
    """

    def __init__(self, kp=1.0, ki=0.2, kd=0.05, dt=0.1,
                 max_throttle=1.0, max_brake=1.0, integral_limit=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target_vel, current_vel):
        """Computes normalized throttle/braking effort in [-1.0, 1.0]."""
        # TODO: Milestone 4.1 — Longitudinal PID Speed Control & Anti-Windup
        # This is the speed regulator. Because the car has drag, simply setting
        # a target speed isn't enough — it needs closed-loop control.
        # Implement a PID controller on the velocity error with anti-windup on the integrator.
        # Velocity tracking error
        error = target_vel - current_vel

        # Proportional term
        p_term = self.kp * error

        # Derivative term
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative

        # Anti-windup, stage 1: conditional integration.
        # Provisionally integrate, then test the resulting effort. If the
        # actuator is already saturated and the error would push it further
        # into saturation, discard the update and hold the integrator.
        candidate_integral = self.integral + error * self.dt
        effort_unsaturated = (
            p_term
            + self.ki * candidate_integral
            + d_term
        )

        pushing_into_upper = (
            effort_unsaturated > self.max_throttle and error > 0.0
        )
        pushing_into_lower = (
            effort_unsaturated < -self.max_brake and error < 0.0
        )

        if not (pushing_into_upper or pushing_into_lower):
            # Anti-windup, stage 2: hard clamp on the accumulator.
            self.integral = float(
                np.clip(
                    candidate_integral,
                    -self.integral_limit,
                    self.integral_limit,
                )
            )

        i_term = self.ki * self.integral

        # Total PID effort
        effort = p_term + i_term + d_term

        # Store error for the next iteration
        self.prev_error = error

        # Output saturation:
        # positive = throttle, negative = braking
        effort = float(
            np.clip(
                effort,
                -self.max_brake,
                self.max_throttle,
            )
        )

        return effort

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral = 0.0
        self.prev_error = 0.0
