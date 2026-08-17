import rclpy
from rclpy.node import Node
import math

import tf2_ros
from sensor_msgs.msg import JointState


class ForwardKinematicsNode(Node):

    def __init__(self):
        super().__init__('forward_kinematics')

        # -----------------------------------------------------
        # TF Buffer
        # -----------------------------------------------------

        self.tf_buffer = tf2_ros.Buffer()

        # -----------------------------------------------------
        # TF Listener
        # -----------------------------------------------------

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )
        # -----------------------------------------------------
        # Joint States
        # -----------------------------------------------------
        self.q1 = None
        self.q2 = None
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # ------------------------------------------------------
        # Link lebgths
        # ------------------------------------------------------
        self.L1 = None
        self.L2 = None

        # -----------------------------------------------------
        # Timer
        # -----------------------------------------------------

        self.timer = self.create_timer(
            0.1,
            self.calculate_fk
        )

        self.get_logger().info(
            'Forward Kinematics Node Started'
        )
    # ===========================================================
    # Joint State Callback
    # ===========================================================
    def joint_state_callback(self, msg):

        # -----------------------------------------------------
        # Find joint indices by name
        # -----------------------------------------------------

        try:
            q1_index = msg.name.index('joint1')
            q2_index = msg.name.index('joint2')

        except ValueError:

            self.get_logger().warn(
            'joint1 or joint2 not found in /joint_states'
            )

            return

        # -----------------------------------------------------
        # Store current joint positions
        # -----------------------------------------------------

        self.q1 = msg.position[q1_index]
        self.q2 = msg.position[q2_index]

    # ==========================================================
    # FK / TF Processing
    # ==========================================================

    def calculate_fk(self):

        # -----------------------------------------------------
        # Get link1 -> link2 transform
        # -----------------------------------------------------

        try:

            T_link1_link2 = self.tf_buffer.lookup_transform(
                'link1',
                'link2',
                rclpy.time.Time()
            )

            # -------------------------------------------------
            # Get link2 -> ee_link transform
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

            self.get_logger().warn(
                'Waiting for TF transforms...'
            )

            return

        # -----------------------------------------------------
        # Extract translations
        # -----------------------------------------------------

        p_link1_link2 = T_link1_link2.transform.translation
        p_link2_ee = T_link2_ee.transform.translation

        # -----------------------------------------------------
        # Calculate link lengths from TF
        # -----------------------------------------------------

        self.L1 = math.sqrt(
            p_link1_link2.x ** 2
            + p_link1_link2.y ** 2
            + p_link1_link2.z ** 2
        )

        self.L2 = math.sqrt(
            p_link2_ee.x ** 2
            + p_link2_ee.y ** 2
            + p_link2_ee.z ** 2
        )

        # -----------------------------------------------------
        # Forward Kinematics
        # -----------------------------------------------------

        
        if self.q1 is None or self.q2 is None:
            return

        x_fk = (
            self.L1 * math.cos(self.q1)
            + self.L2 * math.cos(self.q1 + self.q2)
        )

        y_fk = (
            self.L1 * math.sin(self.q1)
            + self.L2 * math.sin(self.q1 + self.q2)
        )

        z_fk = 0.0

        # -----------------------------------------------------
        # TF reference: base_link -> ee_link
        # -----------------------------------------------------

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

        x_tf = T_base_ee.transform.translation.x
        y_tf = T_base_ee.transform.translation.y
        z_tf = T_base_ee.transform.translation.z

        # -----------------------------------------------------
        # FK vs TF error
        # -----------------------------------------------------

        error_x = x_fk - x_tf
        error_y = y_fk - y_tf
        error_z = z_fk - z_tf

        position_error = math.sqrt(
            error_x ** 2
            + error_y ** 2
            + error_z ** 2
        )

        # -----------------------------------------------------
        # Display comparison
        # -----------------------------------------------------

        self.get_logger().info(
            f'q1 = {self.q1:.3f}, '
            f'q2 = {self.q2:.3f} | '
            f'FK = ({x_fk:.4f}, {y_fk:.4f}) | '
            f'TF = ({x_tf:.4f}, {y_tf:.4f}) | '
            f'error = {position_error:.2e} m'
        )
    

def main(args=None):

    rclpy.init(args=args)

    node = ForwardKinematicsNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()