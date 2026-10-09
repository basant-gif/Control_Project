"""
Integrated Two-Tier Autonomous Vehicle Controller Node.
Coordinates target speed profiling, longitudinal ESC regulation, and lateral path tracking.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32

from bicycle_control.longitudinal_pid import PIDLongitudinalController
from bicycle_control.velocity_profiler import VelocityProfiler
from bicycle_control.lateral_pid import LateralPIDController
from bicycle_control.pure_pursuit import PurePursuitController
from bicycle_control.mpc import KinematicBicycleMPC


class ControllerNode(Node):
    """ROS 2 Node coordinating Two-Tier Autonomous Vehicle Control."""

    def __init__(self):
        super().__init__('controller')
        self.get_logger().info('Initializing Two-Tier Autonomous Vehicle Controller...')

        # Parameters
        # Available control modes: 'lateral_pid', 'pure_pursuit', 'mpc'
        self.declare_parameter('control_mode', 'pure_pursuit')
        self.declare_parameter('target_speed', 4.0)             # m/s base speed
        self.declare_parameter('velocity_mode', 'curvature')    # 'curvature', 'constant'
        self.declare_parameter('wheelbase', 1.25)
        self.declare_parameter('max_speed', 7.5)                # m/s cap
        self.declare_parameter('max_lat_accel', 5.0)            # m/s^2 lateral limit

        self.control_mode = str(self.get_parameter('control_mode').value).lower()
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.velocity_mode = str(self.get_parameter('velocity_mode').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_lat_accel = float(self.get_parameter('max_lat_accel').value)

        # Controllers - tuned parameters
        self.pid_longitudinal = PIDLongitudinalController(kp=1.2, ki=0.3, kd=0.05, dt=0.1)
        self.profiler = VelocityProfiler(
            default_speed=self.target_speed,
            max_speed=self.max_speed,
            max_lat_accel=self.max_lat_accel
        )
        # Lateral PID - gains set for zeta ~= 0.80 at cruise speed.
        # zeta = k_yaw / (2*sqrt(kp*L)); wn = v*sqrt(kp/L).
        self.lateral_pid = LateralPIDController(
            kp=0.45, ki=0.4, kd=0.1, k_yaw=0.55, dt=0.1
        )
        # Pure Pursuit - assignment values; raising l_max measured no better
        self.pure_pursuit = PurePursuitController(
            wheelbase=self.wheelbase, kv=0.25, l_min=0.8, l_max=2.5
        )
        # MPC - horizon 10, tuned weights in KinematicBicycleMPC constructor
        self.mpc = KinematicBicycleMPC(
            wheelbase=self.wheelbase, dt=0.1, horizon=10,
            c_roll=0.05, c_drag=0.005, max_speed=self.max_speed
        )

        # Publishers (10 Hz rate per assignment specification)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.steer_pub = self.create_publisher(Float32, '/steer', 10)

        # Subscribers
        self.state_sub = self.create_subscription(Odometry, '/state', self.state_callback, 10)
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)

        # State storage
        self.current_state = None  # (x, y, yaw, v)
        self.path_points = []     # [(x, y, yaw)]
        self.current_steer = 0.0

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Vehicle Controller Active in mode: {self.control_mode.upper()}')

        # Diagnostic CSV log at 10 Hz, lateral PID only.
        self.log_file = None
        if self.control_mode == 'lateral_pid':
            self.log_file = open('/tmp/pid_log.csv', 'w')
            self.log_file.write(
                't_ros,x,y,v,cte,heading_err,kappa,steer,target_v\n'
            )

    def state_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        v = msg.twist.twist.linear.x
        self.current_state = (x, y, yaw, v)

    def path_callback(self, msg: Path):
        pts = []
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            qz = p.pose.orientation.z
            qw = p.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            pts.append((x, y, yaw))
        self.path_points = pts

    # ------------------------------------------------------------------
    # Arc-length windowed helpers - jitter-rejecting tangent and curvature
    # ------------------------------------------------------------------

    def path_window(self, idx, window):
        """Returns (back_idx, fwd_idx) that bracket +/-window metres around idx."""
        pts = self.path_points
        n = len(pts)
        # Walk backward
        back_idx = idx
        d_acc = 0.0
        for _ in range(n):
            prev_i = (back_idx - 1) % n
            seg = math.hypot(
                pts[back_idx][0] - pts[prev_i][0],
                pts[back_idx][1] - pts[prev_i][1]
            )
            if d_acc + seg > window:
                break
            d_acc += seg
            back_idx = prev_i
        # Walk forward
        fwd_idx = idx
        d_acc = 0.0
        for _ in range(n):
            nxt_i = (fwd_idx + 1) % n
            seg = math.hypot(
                pts[nxt_i][0] - pts[fwd_idx][0],
                pts[nxt_i][1] - pts[fwd_idx][1]
            )
            if d_acc + seg > window:
                break
            d_acc += seg
            fwd_idx = nxt_i
        return back_idx, fwd_idx

    def path_tangent(self, idx, window=1.5):
        """Returns path yaw at idx using arc-length-windowed chord (jitter-rejecting)."""
        pts = self.path_points
        back_idx, fwd_idx = self.path_window(idx, window)
        dx = pts[fwd_idx][0] - pts[back_idx][0]
        dy = pts[fwd_idx][1] - pts[back_idx][1]
        if dx * dx + dy * dy < 1e-6:
            return pts[idx][2]
        return math.atan2(dy, dx)

    def path_curvature(self, idx, window=2.0):
        """Returns curvature at idx using Menger circle through three arc-spaced points."""
        pts = self.path_points
        back_idx, fwd_idx = self.path_window(idx, window)
        ax, ay = pts[back_idx][0], pts[back_idx][1]
        bx, by = pts[idx][0], pts[idx][1]
        cx, cy = pts[fwd_idx][0], pts[fwd_idx][1]
        # Menger formula: kappa = 2 * |cross| / (|ab| * |bc| * |ac|)
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        d_ab = math.hypot(bx - ax, by - ay)
        d_bc = math.hypot(cx - bx, cy - by)
        d_ac = math.hypot(cx - ax, cy - ay)
        denom = d_ab * d_bc * d_ac
        if denom < 1e-6:
            return 0.0
        return 2.0 * abs(cross) / denom

    # ------------------------------------------------------------------
    # Track error computation using windowed tangent
    # ------------------------------------------------------------------

    def segment_foot(self, x, y, i0, i1):
        """Closest point on the segment pts[i0]->pts[i1] to (x, y).

        Returns (distance, foot_x, foot_y) or None for a degenerate segment.
        The sign of the cross-track error is NOT taken from here.
        """
        pts = self.path_points
        p1 = pts[i0]
        p2 = pts[i1]
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-9:
            return None
        t = max(0.0, min(1.0, ((x - p1[0]) * dx + (y - p1[1]) * dy) / seg_len_sq))
        px = p1[0] + t * dx
        py = p1[1] + t * dy
        return math.hypot(x - px, y - py), px, py

    def compute_track_errors(self, x, y, yaw):
        """Computes orthogonal CTE, heading error, and local curvature."""
        pts = self.path_points
        n = len(pts)
        if n < 2:
            return 0.0, 0.0, 0.0

        min_d_sq = float('inf')
        nearest_idx = 0
        for i in range(n):
            dx = pts[i][0] - x
            dy = pts[i][1] - y
            d_sq = dx * dx + dy * dy
            if d_sq < min_d_sq:
                min_d_sq = d_sq
                nearest_idx = i

        # Jitter-rejecting tangent, used for BOTH the heading error and the
        # sign of the cross-track error.
        path_yaw = self.path_tangent(nearest_idx)

        # Foot point: nearest point on either segment adjacent to nearest_idx.
        # Only the LOCATION comes from the segment. Taking the sign from an
        # individual segment makes the CTE flip sign at the sub-metre
        # backtracking joints in the centreline CSV, because the two candidate
        # segments point in different directions there.
        prev_idx = (nearest_idx - 1) % n
        next_idx = (nearest_idx + 1) % n

        px, py = pts[nearest_idx][0], pts[nearest_idx][1]
        best_dist = float('inf')
        for i0, i1 in ((prev_idx, nearest_idx), (nearest_idx, next_idx)):
            result = self.segment_foot(x, y, i0, i1)
            if result is None:
                continue
            dist, fx, fy = result
            if dist < best_dist:
                best_dist = dist
                px, py = fx, fy

        # Signed lateral offset in the path frame: positive = left of path.
        # Because path_yaw is averaged over a +/-1.5 m arc, this sign is
        # continuous from one control cycle to the next.
        cte = (
            -(x - px) * math.sin(path_yaw)
            + (y - py) * math.cos(path_yaw)
        )
        heading_err = math.atan2(math.sin(yaw - path_yaw), math.cos(yaw - path_yaw))

        # Menger curvature
        kappa = self.path_curvature(nearest_idx)

        return cte, heading_err, kappa

    def control_loop(self):
        """Executes selected controller at 10 Hz."""
        if self.current_state is None or len(self.path_points) < 2:
            return

        x, y, yaw, v = self.current_state

        if self.control_mode == 'lateral_pid':
            # Mode A: Lateral PID Benchmark
            cte, heading_err, kappa = self.compute_track_errors(x, y, yaw)
            steer_rad = self.lateral_pid.compute_steering(cte, heading_err, v)

            target_v = self.target_speed
            if self.velocity_mode == 'curvature':
                target_v = self.profiler.compute_target_speed(
                    kappa, fallback_speed=self.target_speed
                )

            throttle_cmd = self.pid_longitudinal.compute(target_v, v)

            if self.log_file is not None:
                now_s = self.get_clock().now().nanoseconds * 1e-9
                self.log_file.write(
                    f'{now_s:.4f},{x:.4f},{y:.4f},{v:.4f},'
                    f'{cte:.5f},{heading_err:.5f},{kappa:.5f},'
                    f'{steer_rad:.5f},{target_v:.4f}\n'
                )
                self.log_file.flush()

        elif self.control_mode == 'mpc':
            # Mode C: Kinematic Bicycle MPC Benchmark
            ref_traj = self.build_mpc_reference(x, y, v, horizon=10)
            steer_rad, throttle_cmd = self.mpc.solve(
                [x, y, yaw, v], ref_traj, current_steer=self.current_steer
            )

        else:
            # Mode B: Geometric Pure Pursuit Benchmark (Default)
            lookahead = self.pure_pursuit.compute_lookahead(v)
            tgt_idx, tgt_pt = self.pure_pursuit.find_target_waypoint(
                x, y, self.path_points, lookahead
            )
            steer_rad = self.pure_pursuit.compute_steering(x, y, yaw, tgt_pt, lookahead)

            target_v = self.target_speed
            if self.velocity_mode == 'curvature':
                kappa = self.path_curvature(tgt_idx)
                target_v = self.profiler.compute_target_speed(
                    kappa, fallback_speed=self.target_speed
                )

            throttle_cmd = self.pid_longitudinal.compute(target_v, v)

        self.current_steer = float(steer_rad)

        # Publish commands: steering in radians, powertrain throttle in [-1.0, 1.0]
        s_msg = Float32()
        s_msg.data = float(steer_rad)
        self.steer_pub.publish(s_msg)

        t_msg = Float32()
        t_msg.data = float(throttle_cmd)
        self.throttle_pub.publish(t_msg)

    def get_waypoint_at_distance(self, start_idx, distance_ahead):
        """Walks forward along path by distance_ahead and interpolates reference pose.

        The reference heading uses the arc-length-windowed tangent rather than
        the raw waypoint yaw. The raw yaw carries the sub-metre backtracking
        noise present in the centreline CSV, and the MPC builds its entire
        Frenet frame from this angle.
        """
        pts = self.path_points
        n = len(pts)
        if n < 2:
            return pts[0] if pts else (0.0, 0.0, 0.0, start_idx)

        cur_idx = start_idx
        d_acc = 0.0
        while d_acc < distance_ahead:
            next_idx = (cur_idx + 1) % n
            seg = math.hypot(
                pts[next_idx][0] - pts[cur_idx][0],
                pts[next_idx][1] - pts[cur_idx][1]
            )
            if d_acc + seg >= distance_ahead:
                frac = (distance_ahead - d_acc) / max(seg, 1e-4)
                px = pts[cur_idx][0] + frac * (pts[next_idx][0] - pts[cur_idx][0])
                py = pts[cur_idx][1] + frac * (pts[next_idx][1] - pts[cur_idx][1])
                psi = self.path_tangent(cur_idx)
                return (px, py, psi, cur_idx)
            d_acc += seg
            cur_idx = next_idx
            if cur_idx == start_idx:
                break
        last = pts[cur_idx]
        return (last[0], last[1], self.path_tangent(cur_idx), cur_idx)

    def build_mpc_reference(self, x, y, v=4.0, horizon=10):
        """Generates distance-based future reference poses along path for MPC.

        Each reference point carries a curvature-limited v_ref so the MPC
        prediction is consistent with what the velocity profiler targets.
        """
        pts = self.path_points
        n = len(pts)
        if n < 2:
            return []

        min_d = float('inf')
        nearest_idx = 0
        for i in range(n):
            d = (pts[i][0] - x) ** 2 + (pts[i][1] - y) ** 2
            if d < min_d:
                min_d = d
                nearest_idx = i

        speed = max(float(v), 1.0)
        dt = 0.1
        ref = []
        for k in range(horizon):
            dist_ahead = (k + 1) * speed * dt
            px, py, pyaw, pidx = self.get_waypoint_at_distance(nearest_idx, dist_ahead)
            if self.velocity_mode == 'curvature':
                v_ref = self.profiler.compute_target_speed(
                    self.path_curvature(pidx),
                    fallback_speed=self.target_speed
                )
            else:
                v_ref = self.target_speed
            ref.append([px, py, pyaw, v_ref])
        return ref


def main(args=None):
    rclpy.init(args=args)
    controller = ControllerNode()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if controller.log_file is not None:
            controller.log_file.close()
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
