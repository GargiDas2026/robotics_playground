import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class CartesianTrajectoryNode(Node):

    def __init__(self):
        super().__init__('cartesian_trajectory')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------
        self.declare_parameter('trajectory_duration', 5.0)
        self.declare_parameter('publish_rate', 20.0)

        self.trajectory_duration = (
            self.get_parameter('trajectory_duration').value
        )

        self.publish_rate = (
            self.get_parameter('publish_rate').value
        )

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

        # Current end-effector position from FK
        self.current_position_subscriber = self.create_subscription(
            Point,
            '/ee_position',
            self.current_position_callback,
            10
        )

        # Desired final target position
        self.target_position_subscriber = self.create_subscription(
            Point,
            '/ee_target_position',
            self.target_position_callback,
            10
        )

        # ---------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------

        # Desired point along the Cartesian trajectory
        self.desired_position_publisher = self.create_publisher(
            Point,
            '/ee_desired_position',
            10
        )

        # ---------------------------------------------------------
        # State variables
        # ---------------------------------------------------------

        # Latest measured/current EE position
        self.current_position = None

        # Latest requested target
        self.target_position = None

        # Starting point of the current trajectory
        self.trajectory_start_position = None

        # Final point of the current trajectory
        self.trajectory_target_position = None

        # Time at which the current trajectory started
        self.trajectory_start_time = None

        # Whether a trajectory is currently being executed
        self.trajectory_active = False

        # ---------------------------------------------------------
        # Timer
        # ---------------------------------------------------------

        self.trajectory_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.generate_trajectory
        )

        # ---------------------------------------------------------
        # Startup messages
        # ---------------------------------------------------------

        self.get_logger().info(
            'Cartesian trajectory node started.'
        )

        self.get_logger().info(
            f'Trajectory duration: '
            f'{self.trajectory_duration:.2f} s'
        )

        self.get_logger().info(
            f'Publishing rate: '
            f'{self.publish_rate:.1f} Hz'
        )

    # =============================================================
    # Current EE position callback
    # =============================================================

    def current_position_callback(self, msg):

        self.current_position = msg

    # =============================================================
    # Target position callback
    # =============================================================

    def target_position_callback(self, msg):

        self.target_position = msg

        self.get_logger().info(
            f'Received target: '
            f'x = {msg.x:.4f} m, '
            f'y = {msg.y:.4f} m'
        )

        # We need the current EE position to generate
        # a trajectory from the current position to the target.
        if self.current_position is None:

            self.get_logger().warn(
                'Current EE position is not available yet.'
            )

            return

        # ---------------------------------------------------------
        # Store trajectory start and target
        # ---------------------------------------------------------

        self.trajectory_start_position = self.current_position

        self.trajectory_target_position = msg

        # Record the start time
        self.trajectory_start_time = self.get_clock().now()

        # Activate trajectory generation
        self.trajectory_active = True

        self.get_logger().info(
            f'Starting trajectory: '
            f'({self.current_position.x:.4f}, '
            f'{self.current_position.y:.4f}) '
            f'-> '
            f'({msg.x:.4f}, {msg.y:.4f})'
        )

    # =============================================================
    # Generate Cartesian trajectory
    # =============================================================

    def generate_trajectory(self):

        # ---------------------------------------------------------
        # Do nothing if there is no active trajectory
        # ---------------------------------------------------------

        if not self.trajectory_active:

            return

        # Safety check
        if self.trajectory_start_position is None:

            return

        if self.trajectory_target_position is None:

            return

        if self.trajectory_start_time is None:

            return

        # ---------------------------------------------------------
        # Extract start and target positions
        # ---------------------------------------------------------

        x0 = self.trajectory_start_position.x
        y0 = self.trajectory_start_position.y

        xf = self.trajectory_target_position.x
        yf = self.trajectory_target_position.y

        # ---------------------------------------------------------
        # Calculate elapsed time
        # ---------------------------------------------------------

        current_time = self.get_clock().now()

        elapsed_time = (
            current_time - self.trajectory_start_time
        ).nanoseconds * 1e-9

        # ---------------------------------------------------------
        # Normalize time
        #
        # tau = 0 -> trajectory start
        # tau = 1 -> trajectory end
        # ---------------------------------------------------------

        tau = elapsed_time / self.trajectory_duration

        # Make sure tau stays within [0, 1]
        tau = max(0.0, min(1.0, tau))

        # ---------------------------------------------------------
        # Cubic time-scaling
        #
        # s(tau) = 3*tau^2 - 2*tau^3
        #
        # This gives:
        #
        # s(0) = 0
        # s(1) = 1
        # ds/dtau(0) = 0
        # ds/dtau(1) = 0
        #
        # Therefore the trajectory starts and ends
        # with zero velocity.
        # ---------------------------------------------------------

        s = (
            3.0 * tau ** 2
            - 2.0 * tau ** 3
        )

        # ---------------------------------------------------------
        # Cartesian interpolation
        # ---------------------------------------------------------

        x_desired = (
            x0
            + s * (xf - x0)
        )

        y_desired = (
            y0
            + s * (yf - y0)
        )

        # This is a planar robot
        z_desired = 0.0

        # ---------------------------------------------------------
        # Create desired position message
        # ---------------------------------------------------------

        desired_position = Point()

        desired_position.x = x_desired
        desired_position.y = y_desired
        desired_position.z = z_desired

        # ---------------------------------------------------------
        # Publish desired Cartesian position
        # ---------------------------------------------------------

        self.desired_position_publisher.publish(
            desired_position
        )

        # ---------------------------------------------------------
        # Check whether trajectory is complete
        # ---------------------------------------------------------

        if tau >= 1.0:

            self.get_logger().info(
                f'Trajectory completed at '
                f'({x_desired:.4f}, '
                f'{y_desired:.4f})'
            )

            # Stop the trajectory
            self.trajectory_active = False

            self.trajectory_start_time = None


# ================================================================
# Main
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    node = CartesianTrajectoryNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()