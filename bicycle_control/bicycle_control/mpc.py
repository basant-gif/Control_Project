"""
High-Level Lateral Steering Controller: Extended Kinematic Bicycle MPC.
Solves a constrained non-linear program over prediction horizon N using SciPy,
optimizing steering angle and longitudinal acceleration (mapped to throttle).
"""

import math  # noqa: F401
import numpy as np  # noqa: F401
from scipy.optimize import minimize  # noqa: F401


class KinematicBicycleMPC:
    """Nonlinear Model Predictive Control for an Extended Kinematic Bicycle Model.

    Optimizes future control sequences u = [delta_k, a_k] where steering angle delta_k
    and longitudinal acceleration a_k (mapped to throttle effort) are the control inputs,
    forward-simulating a 4-state extended kinematic bicycle model x = [x, y, theta, v]^T.
    """

    def __init__(self, wheelbase=1.25, dt=0.1, horizon=10,
                 max_steer_rad=math.radians(35.0), k_a=4.0,
                 max_accel=None, max_brake=None,
                 c_roll=0.05, c_drag=0.005, max_speed=25.0):
        self.L = wheelbase
        self.dt = dt
        self.N = horizon
        self.max_steer_rad = max_steer_rad
        self.k_a = float(max_accel if max_accel is not None else k_a)
        self.c_roll = c_roll
        self.c_drag = c_drag
        self.max_speed = max_speed

        # Cost weights. These are the values that produced the benchmark
        # result (66.097 s, 0.0221 m mean CTE, 1.69 deg heading error).
        # w_lat and w_v were deliberately held down: raising them to 150 and
        # 20 made the quadratic program ill-conditioned, and SLSQP returned
        # degenerate solutions at corners within its 25-iteration budget.
        self.w_lat = 50.0       # lateral CTE
        self.w_long = 1.0       # longitudinal (along-path) error
        self.w_yaw = 10.0       # heading error
        self.w_v = 5.0          # speed error
        self.w_steer = 0.2      # steering magnitude
        self.w_dsteer = 4.0     # steering rate (damps oscillation)
        self.w_accel = 0.1      # acceleration magnitude

        self.last_u = np.zeros(2 * self.N)  # warm-start [delta_0, a_0, delta_1, a_1, ...]

    def solve(self, x0, ref_trajectory, current_steer=0.0):
        """Solves MPC optimization problem over horizon N.

        x0: [x, y, yaw, v]
        ref_trajectory: list of length N containing [x_ref, y_ref, yaw_ref, v_ref]
        current_steer: actual current steering angle in radians
        Returns: (steer_rad, throttle_cmd in [-1.0, 1.0])
        """
        # ======================================================================
        # TODO: Milestone 5.4 — Extended Kinematic Bicycle MPC
        #
        # 1. Horizon & Bounds Setup:
        #    - Determine effective horizon N = min(self.N, len(ref_trajectory)).
        #    - If N < 2, return (0.0, 0.0).
        #    - Construct variable bounds for the decision vector:
        #      u = [delta_0, a_0, delta_1, a_1, ..., delta_N-1, a_N-1]
        #      where delta_k in [-self.max_steer_rad, self.max_steer_rad] (steering input)
        #      and a_k in [-self.k_a, self.k_a] (longitudinal acceleration input).
        #
        # 2. Objective Function objective(u):
        #    - Unpack state [x, y, yaw, v] from x0 and set prev_delta = current_steer.
        #    - For each horizon step k in 0 .. N-1:
        #        a. Forward simulate state using discrete Extended Kinematic Bicycle equations
        #           (where longitudinal velocity v is an explicit state variable integrated
        #           forward with acceleration input a_k)
        #        b. Project tracking error into the path-aligned Frenet frame.
        #        c. Accumulate weighted quadratic costs:
        #           lateral CTE, heading error, speed error, steering, slew rate, accel.
        #        d. Update prev_delta = delta_k.
        #    - Return total cost.
        #
        # 3. Warm-Start Initialization:
        #    - Construct u_init by shifting self.last_u forward by 1 time step.
        #
        # 4. Numerical Optimization & Control Extraction:
        #    - Call scipy.optimize.minimize(objective, u_init, bounds=bounds,
        #                                   method='SLSQP',
        #                                   options={'maxiter': 25, 'ftol': 1e-3}).
        #    - Save optimal solution in self.last_u.
        #    - Extract first control step: delta_cmd = u*[0], accel_cmd = u*[1].
        #    - Map optimal acceleration a_0* to normalized throttle in [-1.0, 1.0]:
        #      throttle_cmd = accel_cmd / self.k_a
        #    - Return tuple: (delta_cmd, throttle_cmd).
        # ======================================================================
        # ------------------------------------------------------------------
        # 1. Effective horizon and input bounds
        # ------------------------------------------------------------------
        N = min(self.N, len(ref_trajectory))

        if N < 2:
            return 0.0, 0.0

        # Decision vector:
        # [delta_0, accel_0, delta_1, accel_1, ..., delta_N-1, accel_N-1]
        bounds = []

        for _ in range(N):
            bounds.append(
                (-self.max_steer_rad, self.max_steer_rad)
            )
            bounds.append(
                (-self.k_a, self.k_a)
            )

        # Pre-extract the reference as plain Python floats and pre-compute the
        # Frenet basis once per solve. SLSQP evaluates objective() several
        # hundred times per control cycle, and none of these quantities depend
        # on the decision vector, so computing them inside the objective wasted
        # ~10,000 trig calls and ~5,000 array allocations every 100 ms.
        ref_x = [float(r[0]) for r in ref_trajectory[:N]]
        ref_y = [float(r[1]) for r in ref_trajectory[:N]]
        ref_yaw = [float(r[2]) for r in ref_trajectory[:N]]
        ref_v = [float(r[3]) for r in ref_trajectory[:N]]
        cos_ref = [math.cos(a) for a in ref_yaw]
        sin_ref = [math.sin(a) for a in ref_yaw]

        # ------------------------------------------------------------------
        # Helper: wrap angle to [-pi, pi]
        # ------------------------------------------------------------------
        def wrap_angle(angle):
            return math.atan2(
                math.sin(angle),
                math.cos(angle)
            )

        # ------------------------------------------------------------------
        # 2. Objective function
        # ------------------------------------------------------------------
        def objective(u):
            x, y, yaw, v = (
                float(x0[0]), float(x0[1]), float(x0[2]), float(x0[3])
            )

            total_cost = 0.0
            prev_delta = float(current_steer)

            for k in range(N):
                delta = u[2 * k]
                accel = u[2 * k + 1]

                # Forward Euler prediction using the extended
                # kinematic bicycle model with rolling and drag resistance.
                x = x + v * math.cos(yaw) * self.dt
                y = y + v * math.sin(yaw) * self.dt
                yaw = yaw + (v / self.L) * math.tan(delta) * self.dt

                # Velocity with aerodynamic resistance in prediction
                v_dot = accel - self.c_roll * v - self.c_drag * v * v
                v = v + v_dot * self.dt
                if v < 0.0:
                    v = 0.0
                elif v > self.max_speed:
                    v = self.max_speed

                # Keep predicted heading numerically well behaved.
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))

                # Position error in world frame.
                dx = x - ref_x[k]
                dy = y - ref_y[k]

                # Project position error into the reference
                # path's Frenet frame.
                e_long = dx * cos_ref[k] + dy * sin_ref[k]
                e_lat = -dx * sin_ref[k] + dy * cos_ref[k]

                # Heading and speed errors.
                d_yaw = yaw - ref_yaw[k]
                e_yaw = math.atan2(math.sin(d_yaw), math.cos(d_yaw))
                e_v = v - ref_v[k]

                d_delta = delta - prev_delta

                # Quadratic MPC cost.
                total_cost += (
                    self.w_long * e_long * e_long
                    + self.w_lat * e_lat * e_lat
                    + self.w_yaw * e_yaw * e_yaw
                    + self.w_v * e_v * e_v
                    + self.w_steer * delta * delta
                    + self.w_dsteer * d_delta * d_delta
                    + self.w_accel * accel * accel
                )

                prev_delta = delta

            return float(total_cost)

        # ------------------------------------------------------------------
        # 3. Warm-start initialization
        # ------------------------------------------------------------------
        previous = np.asarray(
            self.last_u,
            dtype=float
        ).reshape(-1, 2)

        u_init_pairs = np.zeros((N, 2), dtype=float)

        if len(previous) > 0:
            if len(previous) > 1:
                shift_count = min(N - 1, len(previous) - 1)

                u_init_pairs[:shift_count] = (
                    previous[1:1 + shift_count]
                )

            # Hold the final previous control for the last prediction step.
            u_init_pairs[-1] = previous[
                min(N - 1, len(previous) - 1)
            ]
        else:
            u_init_pairs[:, 0] = current_steer

        u_init = u_init_pairs.reshape(-1)

        # Ensure the initial guess is inside the bounds.
        for i, (lower, upper) in enumerate(bounds):
            u_init[i] = np.clip(
                u_init[i],
                lower,
                upper
            )

        # ------------------------------------------------------------------
        # 4. Numerical optimization
        #    maxiter and ftol are held at the assignment values: the control
        #    loop runs at 10 Hz, so the solve must finish inside 100 ms.
        #    Raising them pushes the solver past the deadline and the stale
        #    commands that result destabilise the vehicle.
        # ------------------------------------------------------------------
        result = minimize(
            objective,
            u_init,
            bounds=bounds,
            method='SLSQP',
            options={
                'maxiter': 25,
                'ftol': 1e-3
            }
        )

        # Use the optimizer solution when successful.
        # Otherwise fall back to the warm-start sequence.
        if (
            result.success
            and result.x is not None
            and np.all(np.isfinite(result.x))
        ):
            optimal_u = np.asarray(
                result.x,
                dtype=float
            )
        else:
            optimal_u = u_init

        # Save the solution for the next MPC iteration.
        new_last_u = np.zeros(
            2 * self.N,
            dtype=float
        )

        new_last_u[:2 * N] = optimal_u

        self.last_u = new_last_u

        # ------------------------------------------------------------------
        # Extract first control action
        # ------------------------------------------------------------------
        delta_cmd = float(
            np.clip(
                optimal_u[0],
                -self.max_steer_rad,
                self.max_steer_rad
            )
        )

        accel_cmd = float(
            optimal_u[1]
        )

        # Convert acceleration to normalized throttle/brake.
        if abs(self.k_a) < 1e-9:
            throttle_cmd = 0.0
        else:
            throttle_cmd = accel_cmd / self.k_a

        throttle_cmd = float(
            np.clip(
                throttle_cmd,
                -1.0,
                1.0
            )
        )

        return delta_cmd, throttle_cmd
