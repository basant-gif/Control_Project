"""
Drive-by-Wire Extended Kinematic Bicycle Simulation Node.
Simulates an Extended Kinematic Bicycle Model with Drive-by-Wire Powertrain Dynamics
(throttle/acceleration input, motor torque gain, aerodynamic drag, rolling resistance),
broadcasts TF transforms, and publishes odometry on /state and joint states on /joint_states.
"""

import math
import numpy as np
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class Car(Node):
    """Extended Kinematic Bicycle Model simulator with Drive-by-Wire Powertrain Dynamics.

    Unlike a basic 3-state kinematic bicycle model where velocity is directly commanded,
    this extended formulation tracks longitudinal velocity as a dynamic state variable in R^4
    and uses normalized throttle/braking effort and front steering angle as control inputs in R^2.

    State Vector x in R^4:
        x[0]: x position (m) [rear axle center]
        x[1]: y position (m) [rear axle center]
        x[2]: heading theta / yaw (rad)
        x[3]: longitudinal velocity v (m/s)

    Control Inputs u in R^2:
        u[0]: normalized throttle/brake command u_throttle in [-1.0, 1.0] via /throttle
              [0.0, 1.0]  -> Forward motor propulsion effort (scaled by k_a m/s^2)
              [-1.0, 0.0) -> Mechanical braking effort (does NOT move the car in reverse)
        u[1]: front steering angle delta in radians via /steer (positive = turn left)
    """

    def __init__(self, xInitial=None, dt=0.1, wheelbase_length=1.25):
        super().__init__('KinematicBicycle')
        self.get_logger().info('Drive-by-Wire Extended Kinematic Bicycle Model Initialized')

        # Parameters
        self.declare_parameter('wheelbase_length', float(wheelbase_length))
        self.declare_parameter('dt', float(dt))
        self.declare_parameter('car_name', 'ego_racecar')
        self.declare_parameter('k_a', 4.0)                # m/s² powertrain acceleration gain
        self.declare_parameter('c_drag', 0.005)           # aerodynamic drag coefficient (v^2)
        self.declare_parameter('c_roll', 0.05)            # rolling resistance coefficient (v)
        self.declare_parameter('max_steer_rad', float(math.radians(35.0)))
        self.declare_parameter('max_speed', 25.0)
        self.declare_parameter('wheel_radius', 0.5)

        self.wheelbase_length = float(self.get_parameter('wheelbase_length').value)
        self.dt = float(self.get_parameter('dt').value)
        self.car_name = str(self.get_parameter('car_name').value)
        self.k_a = float(self.get_parameter('k_a').value)
        self.c_drag = float(self.get_parameter('c_drag').value)
        self.c_roll = float(self.get_parameter('c_roll').value)
        self.max_steer_rad = float(self.get_parameter('max_steer_rad').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)

        # State initialization [x, y, theta, v]
        default_x = [0.0, 0.0, 0.0, 0.0]
        self.x = np.array(xInitial if xInitial is not None else default_x, dtype=np.float64)

        self.u = np.array([0.0, 0.0], dtype=np.float64)  # [u_throttle, delta (rad)]
        self.x_dot = np.zeros(4, dtype=np.float64)
        self.wheel_rotation = 0.0

        # Dedicated TF broadcaster (created once)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publishers
        self.state_pub = self.create_publisher(Odometry, '/state', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribers: High-level steering (rad) and low-level powertrain throttle [-1, 1]
        self.steering_sub = self.create_subscription(
            Float32, '/steer', self.steering_callback, 10)
        self.throttle_sub = self.create_subscription(
            Float32, '/throttle', self.throttle_callback, 10)

        # Simulation timer
        self.timer = self.create_timer(self.dt, self.update_simulation)

    def steering_callback(self, msg: Float32):
        """Processes steering angle commands in radians."""
        steer_input = float(msg.data)
        if abs(steer_input) > (self.max_steer_rad + 1e-4):
            deg_lim = math.degrees(self.max_steer_rad)
            self.get_logger().warn(
                f"Steering {steer_input:.3f} rad ({math.degrees(steer_input):.1f}°) "
                f"exceeds limit [±{self.max_steer_rad:.3f} rad ({deg_lim:.1f}°)]. Clamping.",
                throttle_duration_sec=1.0
            )
            steer_input = float(np.clip(steer_input, -self.max_steer_rad, self.max_steer_rad))

        self.u[1] = steer_input

    def throttle_callback(self, msg: Float32):
        """Processes low-level powertrain throttle/braking effort command in [-1.0, 1.0]."""
        throttle_input = float(msg.data)
        if throttle_input > 1.0 or throttle_input < -1.0:
            self.get_logger().warn(
                f"Throttle {throttle_input:.2f} exceeds range [-1.0, 1.0]. Clamping.",
                throttle_duration_sec=1.0
            )
            throttle_input = float(np.clip(throttle_input, -1.0, 1.0))

        self.u[0] = throttle_input

    def update_x_dot(self):
        """Computes the state derivative vector x_dot = f(x, u).

        State Vector self.x (R^4):
            self.x[0]: x position (m) [rear axle center]
            self.x[1]: y position (m) [rear axle center]
            self.x[2]: heading theta / yaw angle (rad, 0 = +x axis)
            self.x[3]: longitudinal velocity v (m/s)

        Control Input Vector self.u (R^2):
            self.u[0]: normalized throttle/brake command u_throttle in [-1.0, 1.0]
            self.u[1]: front steering angle delta in radians (positive = left)

        Vehicle Physical Parameters:
            self.wheelbase_length (L): 1.25 m
            self.k_a: 4.0 m/s^2 (powertrain acceleration scaling gain)
            self.c_drag: 0.005 (aerodynamic drag coefficient)
            self.c_roll: 0.05 (rolling resistance coefficient)
        """
        # Current state
        theta = self.x[2]
        v = self.x[3]

        # Current control inputs
        u_throttle = self.u[0]
        delta = self.u[1]

        # Rear axle center position dynamics
        self.x_dot[0] = v * math.cos(theta)
        self.x_dot[1] = v * math.sin(theta)

        # Heading/yaw dynamics
        self.x_dot[2] = (
            v / self.wheelbase_length
        ) * math.tan(delta)

        # Longitudinal dynamics:
        # powertrain acceleration - rolling resistance - aerodynamic drag
        self.x_dot[3] = (
            self.k_a * u_throttle
            - self.c_roll * v
            - self.c_drag * v * v
        )

    def update_x(self):
        """Integrates state forward using discrete Forward Euler numerical integration."""
        # Forward Euler integration
        self.x = self.x + self.x_dot * self.dt

        # Keep vehicle speed within the physical limits
        self.x[3] = float(np.clip(self.x[3], 0.0, self.max_speed))

        # Wrap heading to [-pi, pi]
        self.x[2] = math.atan2(
            math.sin(self.x[2]),
            math.cos(self.x[2])
        )

    def update_simulation(self):
        """Timer callback coordinating physics update and telemetry broadcast."""
        self.update_x_dot()
        self.update_x()

        # Update wheel spin rotation based on actual speed
        if self.wheel_radius > 0:
            d_rot = self.x[3] * self.dt / self.wheel_radius
            self.wheel_rotation = (self.wheel_rotation + d_rot) % (2 * math.pi)

        # Publish TF, JointState, and Odometry
        now = self.get_clock().now().to_msg()
        self.publish_tf(now)
        self.publish_joint_states(now)
        self.publish_state(now, self.x_dot[2])

    def publish_tf(self, timestamp):
        """Broadcasts map -> ego_racecar/base_link transform."""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'map'
        t.child_frame_id = f'{self.car_name}/base_link'

        t.transform.translation.x = float(self.x[0])
        t.transform.translation.y = float(self.x[1])
        t.transform.translation.z = 0.0

        half_yaw = self.x[2] / 2.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half_yaw)
        t.transform.rotation.w = math.cos(half_yaw)

        self.tf_broadcaster.sendTransform(t)

    def publish_joint_states(self, timestamp):
        """Publishes joint states for steering hinges and rotating wheels."""
        msg = JointState()
        msg.header.stamp = timestamp
        msg.name = [
            'base_to_front_left_hinge',
            'base_to_front_right_hinge',
            'front_left_hinge_to_wheel',
            'front_right_hinge_to_wheel',
            'base_to_back_left_wheel',
            'base_to_back_right_wheel'
        ]
        delta = float(self.u[1])
        rot = float(self.wheel_rotation)
        msg.position = [delta, delta, rot, rot, rot, rot]
        self.joint_pub.publish(msg)

    def publish_state(self, timestamp, yaw_rate):
        """Publishes vehicle odometry state on /state topic."""
        state = Odometry()
        state.header.stamp = timestamp
        state.header.frame_id = 'map'
        state.child_frame_id = f'{self.car_name}/base_link'

        state.pose.pose.position.x = float(self.x[0])
        state.pose.pose.position.y = float(self.x[1])
        state.pose.pose.position.z = 0.0

        half_yaw = self.x[2] / 2.0
        state.pose.pose.orientation.x = 0.0
        state.pose.pose.orientation.y = 0.0
        state.pose.pose.orientation.z = math.sin(half_yaw)
        state.pose.pose.orientation.w = math.cos(half_yaw)

        state.twist.twist.linear.x = float(self.x[3])
        state.twist.twist.linear.y = 0.0
        state.twist.twist.linear.z = 0.0
        state.twist.twist.angular.z = float(yaw_rate)

        self.state_pub.publish(state)
