"""
Lap Analyzer Node:
Performance evaluation, real-time telemetry, lap timing, and RViz HUD visualization.
Decoupled observer monitoring /path and /state to compute cross-track error, heading error,
lap times, and dynamic metrics.
"""

import json  # noqa: F401
import math
import numpy as np  # noqa: F401
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Float32  # noqa: F401
from geometry_msgs.msg import Point  # noqa: F401
from visualization_msgs.msg import Marker, MarkerArray


class LapAnalyzer(Node):
    def __init__(self):
        super().__init__('lap_analyzer')
        self.get_logger().info('Initializing Lap Analyzer Node...')

        # Subscriptions
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)
        self.state_sub = self.create_subscription(Odometry, '/state', self.state_callback, 10)

        # Publishers
        self.metrics_pub = self.create_publisher(String, '/lap/metrics', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/lap/visualization', 10)

        # Standardized Plottable Telemetry Publishers (for rqt_plot & PlotJuggler)
        self.cte_pub = self.create_publisher(Float32, '/telemetry/cte', 10)
        self.speed_pub = self.create_publisher(Float32, '/telemetry/speed', 10)
        self.heading_err_pub = self.create_publisher(
            Float32, '/telemetry/heading_err_deg', 10
        )
        self.lap_time_pub = self.create_publisher(Float32, '/telemetry/lap_time', 10)

        # Path storage
        self.path_points = []  # [(x, y, psi)]
        self.path_cum_dist = []
        self.track_length = 0.0
        self.path_received = False

        # State & Timing
        self.start_sim_time = None
        self.last_state_time = None
        self.lap_start_time = None

        # Lap Tracking
        self.lap_count = 0
        self.last_s = 0.0
        self.total_distance = 0.0
        self.last_xy = None

        # Lap Times
        self.current_lap_time = 0.0
        self.last_lap_time = None
        self.best_lap_time = None
        self.lap_times = []

        # Error & Speed Statistics (Per Lap)
        self.lap_ctes = []
        self.lap_heading_errors = []
        self.lap_speeds = []

        # Global Statistics
        self.global_ctes = []
        self.global_max_speed = 0.0

        # Current live metrics
        self.current_cte = 0.0
        self.current_heading_err = 0.0
        self.current_speed = 0.0
        self.current_xy = (0.0, 0.0)
        self.proj_xy = (0.0, 0.0)

        # Publish periodic summary and HUD at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_telemetry)

    def path_callback(self, msg: Path):
        """Processes received path and precomputes cumulative distance."""
        if self.path_received and len(msg.poses) == len(self.path_points):
            return  # Path already loaded and unchanged

        pts = []
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            # Extract yaw from quaternion
            qz = p.pose.orientation.z
            qw = p.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            pts.append((x, y, yaw))

        if len(pts) < 2:
            return

        self.path_points = pts
        # Compute cumulative distance
        cum = [0.0]
        for i in range(1, len(pts)):
            d = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            cum.append(cum[-1] + d)

        self.path_cum_dist = cum
        self.track_length = cum[-1]
        self.path_received = True
        self.get_logger().info(
            f"Lap Analyzer: Loaded path with {len(pts)} waypoints, "
            f"perimeter: {self.track_length:.2f} m"
        )

    def state_callback(self, msg: Odometry):
        """Processes vehicle odometry and updates progress, lap timing, and errors."""
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_sim_time is None:
            self.start_sim_time = now_sec
            self.lap_start_time = now_sec

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_xy = (x, y)
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        v = msg.twist.twist.linear.x

        self.current_speed = v
        self.global_max_speed = max(self.global_max_speed, v)

        # Distance traveled
        if self.last_xy is not None:
            step_d = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            self.total_distance += step_d
        self.last_xy = (x, y)

        if not self.path_received or len(self.path_points) < 2:
            return

        # Find nearest point & projection
        proj_x, proj_y, s, cte, heading_err = self.project_to_path(x, y, yaw)
        self.proj_xy = (proj_x, proj_y)
        self.current_cte = cte
        self.current_heading_err = heading_err

        # Accumulate metrics
        abs_cte = abs(cte)
        self.lap_ctes.append(abs_cte)
        self.lap_heading_errors.append(abs(heading_err))
        self.lap_speeds.append(v)
        self.global_ctes.append(abs_cte)

        self.current_lap_time = now_sec - self.lap_start_time

        # Lap Crossing Detection (s wrapped around track_length while moving forward).
        # e.g., last_s near end (> 70% length) and current s near start (< 30% length).
        if self.track_length > 5.0 and v > 0.1:
            if self.last_s > 0.75 * self.track_length and s < 0.25 * self.track_length:
                # Sub-tick lap time interpolation
                ds_total = (self.track_length - self.last_s) + s
                dt_step = max(now_sec - (self.last_state_time or now_sec), 1e-4)
                frac = (self.track_length - self.last_s) / max(ds_total, 1e-4)
                t_crossing = (self.last_state_time or now_sec) + frac * dt_step

                lap_duration = t_crossing - self.lap_start_time
                self.record_lap_completion(lap_duration, now_sec)
                self.lap_start_time = t_crossing

        self.last_s = s
        self.last_state_time = now_sec

    def path_tangent(self, idx, window=1.5):
        """Path heading at waypoint idx, averaged over +/- window metres of arc.

        The supplied centerline contains sub-metre jitter: roughly a third of its
        waypoint joints turn by more than 20 degrees and many reverse direction
        outright. Taking the heading from a single adjacent segment therefore
        reports large, meaningless heading errors. A chord spanning a window of
        real arc length on either side rejects that jitter while still following
        genuine track curvature.
        """
        pts = self.path_points
        n = len(pts)
        if n < 2:
            return 0.0

        lo = hi = idx

        travelled = 0.0
        while travelled < window:
            nxt = (lo - 1) % n
            travelled += math.hypot(
                pts[lo][0] - pts[nxt][0],
                pts[lo][1] - pts[nxt][1]
            )
            lo = nxt
            if lo == idx:
                break

        travelled = 0.0
        while travelled < window:
            nxt = (hi + 1) % n
            travelled += math.hypot(
                pts[nxt][0] - pts[hi][0],
                pts[nxt][1] - pts[hi][1]
            )
            hi = nxt
            if hi == idx:
                break

        dx = pts[hi][0] - pts[lo][0]
        dy = pts[hi][1] - pts[lo][1]

        if math.hypot(dx, dy) < 1e-6:
            return pts[idx][2]

        return math.atan2(dy, dx)

    def project_to_path(self, x, y, yaw):
        """Finds closest segment and projects (x, y) to compute exact orthogonal CTE."""
        pts = self.path_points
        n = len(pts)

        # 1. Find nearest waypoint
        min_dist_sq = float('inf')
        nearest_idx = 0
        for i in range(n):
            dx = pts[i][0] - x
            dy = pts[i][1] - y
            d_sq = dx * dx + dy * dy
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                nearest_idx = i

        # 2. Check candidate segments: (prev, nearest) and (nearest, next)
        best_dist = float('inf')
        best_proj = (pts[nearest_idx][0], pts[nearest_idx][1])
        best_s = self.path_cum_dist[nearest_idx]
        best_signed_cte = 0.0

        candidate_segments = [
            ((nearest_idx - 1) % n, nearest_idx),
            (nearest_idx, (nearest_idx + 1) % n)
        ]
        for prev_i, next_i in candidate_segments:
            x1, y1, yaw1 = pts[prev_i]
            x2, y2, _ = pts[next_i]
            dx = x2 - x1
            dy = y2 - y1
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-6:
                continue

            t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / seg_len_sq))
            px = x1 + t * dx
            py = y1 + t * dy
            dist = math.hypot(x - px, y - py)

            if dist < best_dist:
                best_dist = dist
                best_proj = (px, py)
                best_s = self.path_cum_dist[prev_i] + t * math.sqrt(seg_len_sq)

                # Signed cross track error: positive if car is to the left of path
                cross = dx * (y - y1) - dy * (x - x1)
                best_signed_cte = math.copysign(dist, cross)

        # Heading error in [-pi, pi], measured against the jitter-rejecting
        # tangent rather than the single nearest segment.
        path_yaw = self.path_tangent(nearest_idx)
        heading_err = math.atan2(math.sin(yaw - path_yaw), math.cos(yaw - path_yaw))

        return best_proj[0], best_proj[1], best_s, best_signed_cte, heading_err

    def record_lap_completion(self, lap_duration, now_sec):
        """Records finished lap and prints summary."""
        self.lap_count += 1
        self.last_lap_time = lap_duration
        self.lap_times.append(lap_duration)

        if self.best_lap_time is None or lap_duration < self.best_lap_time:
            self.best_lap_time = lap_duration

        # ======================================================================
        # TODO: Lap Performance Analysis & Metrics Aggregation
        #
        # 1. Summary Statistics:
        #    Compute key performance indicators from self.lap_ctes and self.lap_speeds:
        #      - mean_cte: Mean absolute Cross-Track Error (m)
        #      - max_cte: Maximum Cross-Track Error (m)
        #      - rms_cte: Root-Mean-Square Cross-Track Error: sqrt(mean(cte^2)) (m)
        #      - mean_speed: Average speed across the lap (m/s)
        #      - max_speed: Peak instantaneous speed (m/s)
        #
        # 2. Console Summary:
        #    Log a clean, structured terminal banner reporting lap time, best lap,
        #    CTE metrics (Mean, RMS, Max), speed metrics, and total distance.
        #
        # 3. Buffer Reset:
        #    Clear per-lap history buffers (self.lap_ctes, self.lap_heading_errors,
        #    self.lap_speeds) so the next lap starts fresh.
        # ======================================================================
        # Compute per-lap performance statistics.
        if self.lap_ctes:
            mean_cte = float(np.mean(self.lap_ctes))
            max_cte = float(np.max(self.lap_ctes))
            rms_cte = float(
                np.sqrt(np.mean(np.square(self.lap_ctes)))
            )
        else:
            mean_cte = 0.0
            max_cte = 0.0
            rms_cte = 0.0

        if self.lap_speeds:
            mean_speed = float(np.mean(self.lap_speeds))
            max_speed = float(np.max(self.lap_speeds))
        else:
            mean_speed = 0.0
            max_speed = 0.0

        mean_heading_error = (
            float(np.mean(self.lap_heading_errors))
            if self.lap_heading_errors
            else 0.0
        )

        # Build a structured lap summary.
        metrics = {
            "event": "lap_complete",
            "lap": self.lap_count,
            "lap_time": float(lap_duration),
            "best_lap_time": (
                float(self.best_lap_time)
                if self.best_lap_time is not None
                else None
            ),
            "mean_cte": mean_cte,
            "max_cte": max_cte,
            "rms_cte": rms_cte,
            "mean_speed": mean_speed,
            "max_speed": max_speed,
            "mean_heading_error_deg": math.degrees(
                mean_heading_error
            ),
            "total_distance": float(self.total_distance),
        }

        # Publish the completed-lap summary as JSON.
        msg = String()
        msg.data = json.dumps(metrics)
        self.metrics_pub.publish(msg)

        # Structured console summary.
        self.get_logger().info(
            "\n"
            "========== LAP COMPLETED ==========\n"
            f"Lap:              {self.lap_count}\n"
            f"Lap time:         {lap_duration:.3f} s\n"
            f"Best lap:         {self.best_lap_time:.3f} s\n"
            f"Mean CTE:         {mean_cte:.4f} m\n"
            f"RMS CTE:          {rms_cte:.4f} m\n"
            f"Max CTE:          {max_cte:.4f} m\n"
            f"Mean speed:       {mean_speed:.3f} m/s\n"
            f"Max speed:        {max_speed:.3f} m/s\n"
            f"Mean heading err: {math.degrees(mean_heading_error):.3f} deg\n"
            f"Total distance:   {self.total_distance:.3f} m\n"
            "==================================="
        )

        # Reset per-lap statistics for the next lap.
        self.lap_ctes.clear()
        self.lap_heading_errors.clear()
        self.lap_speeds.clear()

    def publish_telemetry(self):
        """Periodically publishes numerical telemetry and RViz visual markers at 10 Hz."""

        # --------------------------------------------------------------
        # 1. Plottable scalar signals (rqt_plot / PlotJuggler)
        # --------------------------------------------------------------
        cte_msg = Float32()
        cte_msg.data = float(self.current_cte)
        self.cte_pub.publish(cte_msg)

        speed_msg = Float32()
        speed_msg.data = float(self.current_speed)
        self.speed_pub.publish(speed_msg)

        heading_msg = Float32()
        heading_msg.data = float(
            math.degrees(self.current_heading_err)
        )
        self.heading_err_pub.publish(heading_msg)

        lap_time_msg = Float32()
        lap_time_msg.data = float(self.current_lap_time)
        self.lap_time_pub.publish(lap_time_msg)

        # --------------------------------------------------------------
        # 2. Aggregated JSON telemetry on /lap/metrics
        # --------------------------------------------------------------
        if self.lap_ctes:
            rms_cte = float(
                np.sqrt(np.mean(np.square(self.lap_ctes)))
            )
        else:
            rms_cte = 0.0

        telemetry = {
            "event": "telemetry",
            "lap": self.lap_count,
            "current_lap_time": float(self.current_lap_time),
            "last_lap_time": (
                float(self.last_lap_time)
                if self.last_lap_time is not None
                else None
            ),
            "best_lap_time": (
                float(self.best_lap_time)
                if self.best_lap_time is not None
                else None
            ),
            "speed": float(self.current_speed),
            "current_cte": float(self.current_cte),
            "rms_cte": rms_cte,
            "heading_err_deg": float(
                math.degrees(self.current_heading_err)
            ),
        }

        telemetry_msg = String()
        telemetry_msg.data = json.dumps(telemetry)
        self.metrics_pub.publish(telemetry_msg)

        # --------------------------------------------------------------
        # 3. Visual telemetry (RViz)
        # --------------------------------------------------------------
        self.publish_rviz_markers(telemetry)

    def publish_rviz_markers(self, telemetry=None):
        """Renders start gate, error whisker, and on-screen HUD text in RViz."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # ----------------------------------------------------------------------
        # Marker 1: Start/Finish Gate Line (Provided for Track Origin Reference)
        # ----------------------------------------------------------------------
        if self.path_points:
            p0 = self.path_points[0]
            gate = Marker()
            gate.header.frame_id = 'map'
            gate.header.stamp = now
            gate.ns = 'start_gate'
            gate.id = 0
            gate.type = Marker.CYLINDER
            gate.action = Marker.ADD
            gate.pose.position.x = p0[0]
            gate.pose.position.y = p0[1]
            gate.pose.position.z = 0.5
            gate.pose.orientation.w = 1.0
            gate.scale.x = 0.1
            gate.scale.y = 1.2
            gate.scale.z = 1.0
            gate.color.r = 0.1
            gate.color.g = 0.9
            gate.color.b = 0.2
            gate.color.a = 0.7
            ma.markers.append(gate)

        # ----------------------------------------------------------------------
        # Marker 2: Cross-Track Error Whisker, colour-graded by |CTE|
        # Green at or below cte_good, red at or above cte_bad, linear between.
        # ----------------------------------------------------------------------
        abs_cte = abs(float(self.current_cte))
        cte_good = 0.25
        cte_bad = 1.00
        severity = (abs_cte - cte_good) / (cte_bad - cte_good)
        severity = max(0.0, min(1.0, severity))

        whisker = Marker()
        whisker.header.frame_id = 'map'
        whisker.header.stamp = now
        whisker.ns = 'tracking_error'
        whisker.id = 0
        whisker.type = Marker.LINE_STRIP
        whisker.action = Marker.ADD
        whisker.pose.orientation.w = 1.0
        whisker.scale.x = 0.05
        whisker.color.r = float(severity)
        whisker.color.g = float(1.0 - severity)
        whisker.color.b = 0.0
        whisker.color.a = 1.0

        p_vehicle = Point()
        p_vehicle.x = float(self.current_xy[0])
        p_vehicle.y = float(self.current_xy[1])
        p_vehicle.z = 0.05
        whisker.points.append(p_vehicle)

        p_projection = Point()
        p_projection.x = float(self.proj_xy[0])
        p_projection.y = float(self.proj_xy[1])
        p_projection.z = 0.05
        whisker.points.append(p_projection)

        ma.markers.append(whisker)

        # ----------------------------------------------------------------------
        # Marker 3: Projected reference point on the path
        # ----------------------------------------------------------------------
        projection = Marker()
        projection.header.frame_id = 'map'
        projection.header.stamp = now
        projection.ns = 'tracking_error'
        projection.id = 1
        projection.type = Marker.SPHERE
        projection.action = Marker.ADD
        projection.pose.position.x = float(self.proj_xy[0])
        projection.pose.position.y = float(self.proj_xy[1])
        projection.pose.position.z = 0.15
        projection.pose.orientation.w = 1.0
        projection.scale.x = 0.30
        projection.scale.y = 0.30
        projection.scale.z = 0.30
        projection.color.r = 0.0
        projection.color.g = 0.8
        projection.color.b = 1.0
        projection.color.a = 0.9
        ma.markers.append(projection)

        # ----------------------------------------------------------------------
        # Marker 4: 3D telemetry HUD scoreboard, trailing the vehicle
        # ----------------------------------------------------------------------
        if telemetry is None:
            telemetry = {
                "lap": self.lap_count,
                "current_lap_time": float(self.current_lap_time),
                "best_lap_time": self.best_lap_time,
                "speed": float(self.current_speed),
                "current_cte": float(self.current_cte),
                "rms_cte": 0.0,
                "heading_err_deg": float(
                    math.degrees(self.current_heading_err)
                ),
            }

        best = telemetry.get("best_lap_time")
        best_text = f"{best:.2f} s" if best is not None else "--"

        last = telemetry.get("last_lap_time")
        last_text = f"{last:.2f} s" if last is not None else "--"

        hud = Marker()
        hud.header.frame_id = 'map'
        hud.header.stamp = now
        hud.ns = 'telemetry_hud'
        hud.id = 0
        hud.type = Marker.TEXT_VIEW_FACING
        hud.action = Marker.ADD
        hud.pose.position.x = float(self.current_xy[0])
        hud.pose.position.y = float(self.current_xy[1])
        hud.pose.position.z = 1.8
        hud.pose.orientation.w = 1.0
        hud.scale.z = 0.45
        hud.color.r = 1.0
        hud.color.g = 1.0
        hud.color.b = 1.0
        hud.color.a = 1.0
        hud.text = (
            f"LAP {telemetry.get('lap', 0)}   "
            f"T {telemetry.get('current_lap_time', 0.0):.2f} s\n"
            f"Last {last_text}   Best {best_text}\n"
            f"v {telemetry.get('speed', 0.0):.2f} m/s\n"
            f"CTE {telemetry.get('current_cte', 0.0):+.3f} m   "
            f"RMS {telemetry.get('rms_cte', 0.0):.3f} m\n"
            f"Heading {telemetry.get('heading_err_deg', 0.0):+.1f} deg"
        )
        ma.markers.append(hud)

        self.viz_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    analyzer = LapAnalyzer()
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
