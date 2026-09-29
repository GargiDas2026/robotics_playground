import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import tf2_ros


class JacobianValidationNode(Node):

    def __init__(self):
        super().__init__('jacobian_validation')

        # ---------------------------------------------------------
        # TF2
        # ---------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ---------------------------------------------------------
        # Robot geometry
        # ---------------------------------------------------------
        self.L1 = None
        self.L2 = None
        self.robot_geometry_ready = False

        self.geometry_timer = self.create_timer(
            0.5,
            self.initialize_robot_geometry
        )

        # ---------------------------------------------------------
        # Previous joint sample
        # ---------------------------------------------------------
        self.previous_q1 = None
        self.previous_q2 = None
        self.previous_time = None

        # ---------------------------------------------------------
        # Validation statistics
        # ---------------------------------------------------------
        self.sample_count = 0
        self.sum_error = 0.0
        self.max_error = 0.0

        # ---------------------------------------------------------
        # Subscriber
        # ---------------------------------------------------------
        self.joint_solution_subscriber = self.create_subscription(
            JointState,
            '/ik_joint_solution',
            self.joint_solution_callback,
            10
        )

        self.get_logger().info(
            'Jacobian velocity validation node started.'
        )

    # =============================================================
    # Get robot geometry from TF
    # =============================================================
    def initialize_robot_geometry(self):

        if self.robot_geometry_ready:
            return

        try:

            transform_1 = self.tf_buffer.lookup_transform(
                'link1',
                'link2',
                rclpy.time.Time()
            )

            transform_2 = self.tf_buffer.lookup_transform(
                'link2',
                'ee_link',
                rclpy.time.Time()
            )

            self.L1 = math.sqrt(
                transform_1.transform.translation.x ** 2 +
                transform_1.transform.translation.y ** 2 +
                transform_1.transform.translation.z ** 2
            )

            self.L2 = math.sqrt(
                transform_2.transform.translation.x ** 2 +
                transform_2.transform.translation.y ** 2 +
                transform_2.transform.translation.z ** 2
            )

            self.robot_geometry_ready = True

            self.get_logger().info(
                f'Robot geometry obtained from TF: '
                f'L1 = {self.L1:.4f} m, '
                f'L2 = {self.L2:.4f} m'
            )

        except Exception:
            pass

    # =============================================================
    # Forward kinematics
    # =============================================================
    def forward_kinematics(self, q1, q2):

        x = (
            self.L1 * math.cos(q1)
            + self.L2 * math.cos(q1 + q2)
        )

        y = (
            self.L1 * math.sin(q1)
            + self.L2 * math.sin(q1 + q2)
        )

        return x, y

    # =============================================================
    # Jacobian
    # =============================================================
    def calculate_jacobian(self, q1, q2):

        q12 = q1 + q2

        J11 = (
            -self.L1 * math.sin(q1)
            - self.L2 * math.sin(q12)
        )

        J12 = (
            -self.L2 * math.sin(q12)
        )

        J21 = (
            self.L1 * math.cos(q1)
            + self.L2 * math.cos(q12)
        )

        J22 = (
            self.L2 * math.cos(q12)
        )

        return J11, J12, J21, J22

    # =============================================================
    # Joint solution callback
    # =============================================================
    def joint_solution_callback(self, msg):

        if not self.robot_geometry_ready:
            return

        if 'joint1' not in msg.name or 'joint2' not in msg.name:
            return

        q1 = msg.position[msg.name.index('joint1')]
        q2 = msg.position[msg.name.index('joint2')]

        current_time = self.get_clock().now()

        # ---------------------------------------------------------
        # First sample
        # ---------------------------------------------------------
        if self.previous_time is None:

            self.previous_q1 = q1
            self.previous_q2 = q2
            self.previous_time = current_time

            return

        # ---------------------------------------------------------
        # Time difference
        # ---------------------------------------------------------
        dt = (
            current_time - self.previous_time
        ).nanoseconds * 1e-9

        if dt <= 0.0:
            return

        # ---------------------------------------------------------
        # Joint velocity
        # ---------------------------------------------------------
        q1_dot = (
            q1 - self.previous_q1
        ) / dt

        q2_dot = (
            q2 - self.previous_q2
        ) / dt

        # ---------------------------------------------------------
        # Current and previous Cartesian positions
        # ---------------------------------------------------------
        x_current, y_current = self.forward_kinematics(
            q1,
            q2
        )

        x_previous, y_previous = self.forward_kinematics(
            self.previous_q1,
            self.previous_q2
        )

        # ---------------------------------------------------------
        # Cartesian velocity from FK
        # ---------------------------------------------------------
        x_dot_fk = (
            x_current - x_previous
        ) / dt

        y_dot_fk = (
            y_current - y_previous
        ) / dt

        # ---------------------------------------------------------
        # Jacobian
        # ---------------------------------------------------------
        J11, J12, J21, J22 = self.calculate_jacobian(
            q1,
            q2
        )

        # ---------------------------------------------------------
        # Cartesian velocity predicted by Jacobian
        # ---------------------------------------------------------
        x_dot_jacobian = (
            J11 * q1_dot
            + J12 * q2_dot
        )

        y_dot_jacobian = (
            J21 * q1_dot
            + J22 * q2_dot
        )

        # ---------------------------------------------------------
        # Velocity error
        # ---------------------------------------------------------
        error_x = (
            x_dot_jacobian - x_dot_fk
        )

        error_y = (
            y_dot_jacobian - y_dot_fk
        )

        velocity_error = math.sqrt(
            error_x ** 2 +
            error_y ** 2
        )

        # ---------------------------------------------------------
        # Statistics
        # ---------------------------------------------------------
        self.sample_count += 1

        self.sum_error += velocity_error

        self.max_error = max(
            self.max_error,
            velocity_error
        )

        mean_error = (
            self.sum_error /
            self.sample_count
        )

        # ---------------------------------------------------------
        # Print validation result
        # ---------------------------------------------------------
        self.get_logger().info(
            f'Sample {self.sample_count:03d} | '
            f'Jq_dot = '
            f'({x_dot_jacobian:.6f}, '
            f'{y_dot_jacobian:.6f}) | '
            f'FK velocity = '
            f'({x_dot_fk:.6f}, '
            f'{y_dot_fk:.6f}) | '
            f'Error = '
            f'{velocity_error:.3e} m/s'
        )

        # Print statistics every 20 samples
        if self.sample_count % 20 == 0:

            self.get_logger().info(
                f'Validation statistics | '
                f'Samples = {self.sample_count} | '
                f'Mean error = {mean_error:.3e} m/s | '
                f'Max error = {self.max_error:.3e} m/s'
            )

        # ---------------------------------------------------------
        # Update previous sample
        # ---------------------------------------------------------
        self.previous_q1 = q1
        self.previous_q2 = q2
        self.previous_time = current_time


def main(args=None):

    rclpy.init(args=args)

    node = JacobianValidationNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
