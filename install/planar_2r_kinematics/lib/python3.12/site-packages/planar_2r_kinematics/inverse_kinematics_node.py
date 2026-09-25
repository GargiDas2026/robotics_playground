import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

import tf2_ros


class InverseKinematicsNode(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')

        # ---------------------------------------------------------
        # TF2 setup
        # ---------------------------------------------------------

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ---------------------------------------------------------
        # Robot dimensions
        #
        # These are NOT hard-coded.
        # They will be obtained from TF.
        # ---------------------------------------------------------

        self.L1 = None
        self.L2 = None

        self.robot_geometry_ready = False

        # ---------------------------------------------------------
        # IK branch selection
        # ---------------------------------------------------------

        self.declare_parameter(
            'ik_branch',
            'elbow_up'
        )

        self.ik_branch = self.get_parameter(
            'ik_branch'
        ).value

        # Check TF periodically until the required transforms
        # become available.
        self.tf_timer = self.create_timer(
            0.5,
            self.initialize_robot_geometry
        )

        # ---------------------------------------------------------
        # Subscriber
        # ---------------------------------------------------------

        self.target_subscriber = self.create_subscription(
            Point,
            '/ee_target_position',
            self.target_callback,
            10
        )

        # ---------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------

        self.elbow_up_publisher = self.create_publisher(
            JointState,
            '/ik_elbow_up',
            10
        )

        self.elbow_down_publisher = self.create_publisher(
            JointState,
            '/ik_elbow_down',
            10
        )

        self.joint_solution_publisher = self.create_publisher(
            JointState,
            '/ik_joint_solution',
            10
        )

        self.get_logger().info(
            'Inverse kinematics node started.'
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
            #
            # The translation magnitude gives L1.
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
            #
            # The translation magnitude gives L2.
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

            # TF may not be available immediately after startup.
            # The timer will try again.
            pass

    # =============================================================
    # Inverse Kinematics
    # =============================================================

    def target_callback(self, target):

        # ---------------------------------------------------------
        # Make sure robot geometry has been obtained from TF
        # ---------------------------------------------------------

        if not self.robot_geometry_ready:

            self.get_logger().warn(
                'Robot geometry is not available from TF yet.'
            )

            return

        # ---------------------------------------------------------
        # Desired end-effector position
        # ---------------------------------------------------------

        x = target.x
        y = target.y

        self.get_logger().info(
            f'Received target: '
            f'x = {x:.4f} m, '
            f'y = {y:.4f} m'
        )

        # ---------------------------------------------------------
        # Step 1:
        # Calculate cos(q2)
        # ---------------------------------------------------------

        cos_q2 = (
            x**2
            + y**2
            - self.L1**2
            - self.L2**2
        ) / (
            2.0 * self.L1 * self.L2
        )

        # ---------------------------------------------------------
        # Step 2:
        # Check reachability
        # ---------------------------------------------------------

        if cos_q2 < -1.0 or cos_q2 > 1.0:

            self.get_logger().warn(
                f"Target ({x:.3f}, {y:.3f}) is outside the workspace."
            )

            return

        # Protect against very small numerical errors
        # such as 1.00000000001.
        cos_q2 = max(-1.0, min(1.0, cos_q2))

        # ---------------------------------------------------------
        # Step 3:
        # Calculate q2 for both elbow-up and elbow-down
        #
        # ---------------------------------------------------------

        q2_elbow_up = math.acos(cos_q2)
        q2_elbow_down = -math.acos(cos_q2)

        # ---------------------------------------------------------
        # Step 4:
        # Calculate corresponding q1
        # ---------------------------------------------------------

        k1_up = (
            self.L1
            + self.L2 * math.cos(q2_elbow_up)
        )

        k2_up = (
            self.L2
            * math.sin(q2_elbow_up)
        )

        q1_elbow_up = (
            math.atan2(y, x)
            - math.atan2(k2_up, k1_up)
        )

        k1_down = (
                    self.L1
                    + self.L2 * math.cos(q2_elbow_down)
                )
        
        k2_down = (
                    self.L2
                    * math.sin(q2_elbow_down)
                )
        
        q1_elbow_down = (
                    math.atan2(y, x)
                    - math.atan2(k2_down, k1_down)
                )

        # ---------------------------------------------------------
        # Step 5:
        # Publish joint solution
        # ---------------------------------------------------------

        # ---------------------------------------------------------
        # Create elbow-up solution message    
        # ---------------------------------------------------------

        elbow_up_solution = JointState()

        elbow_up_solution.header.stamp = (
            self.get_clock().now().to_msg()
        )

        elbow_up_solution.name = [
            'joint1',
            'joint2'
        ]

        elbow_up_solution.position = [
            q1_elbow_up,
            q2_elbow_up
        ]

        # ---------------------------------------------------------
        # Create elbow-down solution message    
        # ---------------------------------------------------------
        
        elbow_down_solution = JointState()
        
        elbow_down_solution.header.stamp = (
                self.get_clock().now().to_msg()
        )
        
        elbow_down_solution.name = [
            'joint1',
            'joint2'
        ]
        
        elbow_down_solution.position = [
            q1_elbow_down,
            q2_elbow_down
        ]

        self.elbow_up_publisher.publish(
            elbow_up_solution
        )

        self.elbow_down_publisher.publish(
            elbow_down_solution
        )

        # ---------------------------------------------------------
        # Select IK branch
        # ---------------------------------------------------------

        if self.ik_branch == 'elbow_up':

            q1 = q1_elbow_up
            q2 = q2_elbow_up

        elif self.ik_branch == 'elbow_down':

            q1 = q1_elbow_down
            q2 = q2_elbow_down

        else:

            self.get_logger().error(
                f'Invalid IK branch: {self.ik_branch}. '
                f'Use "elbow_up" or "elbow_down".'
            )

            return

        joint_solution = JointState()

        joint_solution.header.stamp = (
            self.get_clock().now().to_msg()
        )

        joint_solution.name = [
            'joint1',
            'joint2'
        ]

        joint_solution.position = [
            q1,
            q2
        ]

        self.joint_solution_publisher.publish(
            joint_solution
        )

        # ---------------------------------------------------------
        # Display result
        # ---------------------------------------------------------

        self.get_logger().info(
            f'Target = ({x:.4f}, {y:.4f}) | '
            f'Elbow Up = '
            f'({q1_elbow_up:.4f}, {q2_elbow_up:.4f}) | '
            f'Elbow Down = '
            f'({q1_elbow_down:.4f}, {q2_elbow_down:.4f}) | '
            f'Selected = {self.ik_branch}'
        )

    # =============================================================
    # Shutdown
    # =============================================================

    def destroy_node(self):

        self.get_logger().info(
            'Shutting down inverse kinematics node.'
        )

        super().destroy_node()


# ================================================================
# Main
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    node = InverseKinematicsNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()