import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

import tf2_ros


class TrajectoryValidationNode(Node):

    def __init__(self):
        super().__init__('trajectory_validation')

        # ---------------------------------------------------------
        # TF2 setup
        # ---------------------------------------------------------

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ---------------------------------------------------------
        # Robot geometry
        #
        # Obtained from TF.
        # Nothing is hard-coded.
        # ---------------------------------------------------------

        self.L1 = None
        self.L2 = None

        self.robot_geometry_ready = False

        # ---------------------------------------------------------
        # Store latest desired Cartesian position
        # ---------------------------------------------------------

        self.desired_position = None

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

        self.desired_position_subscriber = self.create_subscription(
            Point,
            '/ee_desired_position',
            self.desired_position_callback,
            10
        )

        self.joint_solution_subscriber = self.create_subscription(
            JointState,
            '/ik_joint_solution',
            self.joint_solution_callback,
            10
        )

        # ---------------------------------------------------------
        # Timer for TF geometry initialization
        # ---------------------------------------------------------

        self.tf_timer = self.create_timer(
            0.5,
            self.initialize_robot_geometry
        )

        # ---------------------------------------------------------
        # Validation statistics
        # ---------------------------------------------------------

        self.sample_count = 0

        self.max_position_error = 0.0

        self.total_position_error = 0.0

        # ---------------------------------------------------------
        # Startup messages
        # ---------------------------------------------------------

        self.get_logger().info(
            'Trajectory validation node started.'
        )

        self.get_logger().info(
            'Waiting for robot geometry from TF...'
        )

    # =============================================================
    # Get robot geometry from TF
    # =============================================================

    def initialize_robot_geometry(self):

        if self.robot_geometry_ready:
            return

        try:

            # -----------------------------------------------------
            # TF: link1 -> link2
            # -----------------------------------------------------

            tf_link1_link2 = self.tf_buffer.lookup_transform(
                'link1',
                'link2',
                rclpy.time.Time()
            )

            x1 = tf_link1_link2.transform.translation.x
            y1 = tf_link1_link2.transform.translation.y
            z1 = tf_link1_link2.transform.translation.z

            self.L1 = math.sqrt(
                x1**2 +
                y1**2 +
                z1**2
            )

            # -----------------------------------------------------
            # TF: link2 -> ee_link
            # -----------------------------------------------------

            tf_link2_ee = self.tf_buffer.lookup_transform(
                'link2',
                'ee_link',
                rclpy.time.Time()
            )

            x2 = tf_link2_ee.transform.translation.x
            y2 = tf_link2_ee.transform.translation.y
            z2 = tf_link2_ee.transform.translation.z

            self.L2 = math.sqrt(
                x2**2 +
                y2**2 +
                z2**2
            )

            self.robot_geometry_ready = True

            self.get_logger().info(
                f'Robot geometry obtained from TF: '
                f'L1 = {self.L1:.4f} m, '
                f'L2 = {self.L2:.4f} m'
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):

            # TF may not be available immediately.
            # The timer will try again.
            pass

    # =============================================================
    # Desired Cartesian position callback
    # =============================================================

    def desired_position_callback(self, msg):

        self.desired_position = msg

    # =============================================================
    # IK joint solution callback
    # =============================================================

    def joint_solution_callback(self, msg):

        # ---------------------------------------------------------
        # Make sure geometry is available
        # ---------------------------------------------------------

        if not self.robot_geometry_ready:
            return

        # ---------------------------------------------------------
        # Make sure we have a desired Cartesian position
        # ---------------------------------------------------------

        if self.desired_position is None:
            return

        # ---------------------------------------------------------
        # Make sure the JointState contains two joints
        # ---------------------------------------------------------

        if len(msg.position) < 2:
            self.get_logger().warn(
                'Received IK solution with fewer than 2 joint positions.'
            )
            return

        # ---------------------------------------------------------
        # Extract desired Cartesian position
        # ---------------------------------------------------------

        x_desired = self.desired_position.x
        y_desired = self.desired_position.y

        # ---------------------------------------------------------
        # Extract IK joint solution
        # ---------------------------------------------------------

        q1 = msg.position[0]
        q2 = msg.position[1]

        # ---------------------------------------------------------
        # Reconstruct Cartesian position using FK
        #
        # x = L1*cos(q1) + L2*cos(q1 + q2)
        #
        # y = L1*sin(q1) + L2*sin(q1 + q2)
        # ---------------------------------------------------------

        x_fk = (
            self.L1 * math.cos(q1)
            + self.L2 * math.cos(q1 + q2)
        )

        y_fk = (
            self.L1 * math.sin(q1)
            + self.L2 * math.sin(q1 + q2)
        )

        # ---------------------------------------------------------
        # Calculate Cartesian error
        # ---------------------------------------------------------

        error_x = x_fk - x_desired

        error_y = y_fk - y_desired

        position_error = math.sqrt(
            error_x**2 +
            error_y**2
        )

        # ---------------------------------------------------------
        # Update statistics
        # ---------------------------------------------------------

        self.sample_count += 1

        self.total_position_error += position_error

        self.max_position_error = max(
            self.max_position_error,
            position_error
        )

        mean_error = (
            self.total_position_error /
            self.sample_count
        )

        # ---------------------------------------------------------
        # Print validation result
        # ---------------------------------------------------------

        self.get_logger().info(
            f'Sample {self.sample_count:03d} | '
            f'Desired = '
            f'({x_desired:.4f}, {y_desired:.4f}) | '
            f'FK = '
            f'({x_fk:.4f}, {y_fk:.4f}) | '
            f'Error = '
            f'{position_error:.3e} m'
        )

        # ---------------------------------------------------------
        # Print statistics periodically
        # ---------------------------------------------------------

        if self.sample_count % 20 == 0:

            self.get_logger().info(
                f'Validation statistics | '
                f'Samples = {self.sample_count} | '
                f'Mean error = {mean_error:.3e} m | '
                f'Max error = {self.max_position_error:.3e} m'
            )

    # =============================================================
    # Shutdown
    # =============================================================

    def destroy_node(self):

        if self.sample_count > 0:

            mean_error = (
                self.total_position_error /
                self.sample_count
            )

            self.get_logger().info(
                '================================================'
            )

            self.get_logger().info(
                'Trajectory validation summary'
            )

            self.get_logger().info(
                f'Total samples: {self.sample_count}'
            )

            self.get_logger().info(
                f'Mean position error: '
                f'{mean_error:.6e} m'
            )

            self.get_logger().info(
                f'Maximum position error: '
                f'{self.max_position_error:.6e} m'
            )

            self.get_logger().info(
                '================================================'
            )

        self.get_logger().info(
            'Shutting down trajectory validation node.'
        )

        super().destroy_node()


# ================================================================
# Main
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryValidationNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
