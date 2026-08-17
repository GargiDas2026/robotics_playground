import math

import rclpy
from rclpy.node import Node

import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point


class ForwardKinematicsNode(Node):

    def __init__(self):
        super().__init__('forward_kinematics')

        # =====================================================
        # TF
        # =====================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # =====================================================
        # Robot geometry parameters
        # =====================================================

        self.declare_parameter(
            'link1_length',
            0.0
        )

        self.declare_parameter(
            'link2_length',
            0.0
        )

        self.L1 = self.get_parameter(
            'link1_length'
        ).value

        self.L2 = self.get_parameter(
            'link2_length'
        ).value

        # Geometry will be obtained from TF once.
        self.geometry_initialized = False

        # =====================================================
        # Joint states
        # =====================================================

        self.q1 = None
        self.q2 = None

        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # =====================================================
        # EE Position Publisher
        # =====================================================

        self.ee_position_publisher = self.create_publisher(
            Point,
            '/ee_position',
            10
        )

        # =====================================================
        # Timer
        # =====================================================

        self.timer = self.create_timer(
            0.1,
            self.calculate_fk
        )

        self.get_logger().info(
            'Forward Kinematics Node Started'
        )

    # =========================================================
    # Joint State Callback
    # =========================================================

    def joint_state_callback(self, msg):

        try:

            q1_index = msg.name.index('joint1')
            q2_index = msg.name.index('joint2')

        except ValueError:

            self.get_logger().warn(
                'joint1 or joint2 not found in /joint_states'
            )

            return

        # Current joint positions
        self.q1 = msg.position[q1_index]
        self.q2 = msg.position[q2_index]

    # =========================================================
    # Initialize Robot Geometry
    # =========================================================

    def initialize_geometry(self):

        try:

            # -------------------------------------------------
            # link1 -> link2
            # -------------------------------------------------

            T_link1_link2 = self.tf_buffer.lookup_transform(
                'link1',
                'link2',
                rclpy.time.Time()
            )

            # -------------------------------------------------
            # link2 -> ee_link
            # -------------------------------------------------

            T_link2_ee = self.tf_buffer.lookup_transform(
                'link2',
                'ee_link',
                rclpy.time.Time()
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):

            return False

        # -----------------------------------------------------
        # Extract translations
        # -----------------------------------------------------

        p_link1_link2 = T_link1_link2.transform.translation
        p_link2_ee = T_link2_ee.transform.translation

        # -----------------------------------------------------
        # Calculate physical link lengths
        # -----------------------------------------------------

        L1 = math.sqrt(
            p_link1_link2.x ** 2
            + p_link1_link2.y ** 2
            + p_link1_link2.z ** 2
        )

        L2 = math.sqrt(
            p_link2_ee.x ** 2
            + p_link2_ee.y ** 2
            + p_link2_ee.z ** 2
        )

        # -----------------------------------------------------
        # Store the geometry
        # -----------------------------------------------------

        self.L1 = L1
        self.L2 = L2

        # -----------------------------------------------------
        # Expose geometry as ROS parameters
        # -----------------------------------------------------

        self.set_parameters([
            rclpy.parameter.Parameter(
                'link1_length',
                rclpy.Parameter.Type.DOUBLE,
                self.L1
            ),

            rclpy.parameter.Parameter(
                'link2_length',
                rclpy.Parameter.Type.DOUBLE,
                self.L2
            )
        ])

        self.geometry_initialized = True

        self.get_logger().info(
            f'Robot geometry initialized: '
            f'L1 = {self.L1:.4f} m, '
            f'L2 = {self.L2:.4f} m'
        )

        return True

    # =========================================================
    # Forward Kinematics
    # =========================================================

    def calculate_fk(self):

        # -----------------------------------------------------
        # Initialize geometry only once
        # -----------------------------------------------------

        if not self.geometry_initialized:

            if not self.initialize_geometry():

                self.get_logger().warn(
                    'Waiting for TF to initialize '
                    'robot geometry...'
                )

                return

        # -----------------------------------------------------
        # Wait for joint states
        # -----------------------------------------------------

        if self.q1 is None or self.q2 is None:

            return

        # =====================================================
        # OUR FORWARD KINEMATICS CALCULATION
        # =====================================================

        x_fk = (
            self.L1 * math.cos(self.q1)
            + self.L2 * math.cos(self.q1 + self.q2)
        )

        y_fk = (
            self.L1 * math.sin(self.q1)
            + self.L2 * math.sin(self.q1 + self.q2)
        )

        z_fk = 0.0

        # =====================================================
        # PUBLISH EE POSITION
        # =====================================================

        ee_position = Point()

        ee_position.x = x_fk
        ee_position.y = y_fk
        ee_position.z = z_fk

        self.ee_position_publisher.publish(
            ee_position
        )

        # =====================================================
        # TF REFERENCE
        # =====================================================

        try:

            T_base_ee = self.tf_buffer.lookup_transform(
                'base_link',
                'ee_link',
                rclpy.time.Time()
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):

            self.get_logger().warn(
                'Waiting for base_link -> ee_link transform'
            )

            return

        # -----------------------------------------------------
        # Extract TF EE position
        # -----------------------------------------------------

        x_tf = T_base_ee.transform.translation.x
        y_tf = T_base_ee.transform.translation.y
        z_tf = T_base_ee.transform.translation.z

        # =====================================================
        # FK VS TF ERROR
        # =====================================================

        error_x = x_fk - x_tf
        error_y = y_fk - y_tf
        error_z = z_fk - z_tf

        position_error = math.sqrt(
            error_x ** 2
            + error_y ** 2
            + error_z ** 2
        )

        # =====================================================
        # DISPLAY
        # =====================================================

        self.get_logger().info(
            f'q1 = {self.q1:.3f}, '
            f'q2 = {self.q2:.3f} | '
            f'FK = ({x_fk:.4f}, {y_fk:.4f}) | '
            f'TF = ({x_tf:.4f}, {y_tf:.4f}) | '
            f'error = {position_error:.2e} m'
        )


# =============================================================
# Main
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = ForwardKinematicsNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()