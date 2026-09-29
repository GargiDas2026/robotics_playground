import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import tf2_ros


class JacobianNode(Node):

    def __init__(self):
        super().__init__('jacobian')

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

        # Try to obtain robot geometry periodically from TF
        self.geometry_timer = self.create_timer(
            0.5,
            self.initialize_robot_geometry
        )

        # ---------------------------------------------------------
        # Subscriber
        # ---------------------------------------------------------
        self.joint_solution_subscriber = self.create_subscription(
            JointState,
            '/ik_joint_solution',
            self.joint_solution_callback,
            10
        )

        # ---------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------
        self.jacobian_publisher = self.create_publisher(
            Float64MultiArray,
            '/jacobian',
            10
        )

        self.get_logger().info(
            'Jacobian node started.'
        )

    # =============================================================
    # Get robot geometry from TF
    # =============================================================
    def initialize_robot_geometry(self):

        if self.robot_geometry_ready:
            return

        try:
            # link2 origin expressed in link1 frame
            transform_1 = self.tf_buffer.lookup_transform(
                'link1',
                'link2',
                rclpy.time.Time()
            )

            # ee_link origin expressed in link2 frame
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
    # Joint-state callback
    # =============================================================
    def joint_solution_callback(self, msg):

        if not self.robot_geometry_ready:
            return

        # ---------------------------------------------------------
        # Find q1 and q2 using joint names
        # ---------------------------------------------------------
        if 'joint1' not in msg.name or 'joint2' not in msg.name:
            self.get_logger().warn(
                'joint1 or joint2 not found in JointState message.'
            )
            return

        q1 = msg.position[msg.name.index('joint1')]
        q2 = msg.position[msg.name.index('joint2')]

        # ---------------------------------------------------------
        # Calculate Jacobian
        # ---------------------------------------------------------
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

        # ---------------------------------------------------------
        # Publish Jacobian
        # ---------------------------------------------------------
        jacobian_msg = Float64MultiArray()

        jacobian_msg.data = [
            J11,
            J12,
            J21,
            J22
        ]

        self.jacobian_publisher.publish(jacobian_msg)

        # ---------------------------------------------------------
        # Debug information
        # ---------------------------------------------------------
        self.get_logger().debug(
            f'q1 = {q1:.4f}, '
            f'q2 = {q2:.4f} | '
            f'J = [[{J11:.4f}, {J12:.4f}], '
            f'[{J21:.4f}, {J22:.4f}]]'
        )


def main(args=None):

    rclpy.init(args=args)

    node = JacobianNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
