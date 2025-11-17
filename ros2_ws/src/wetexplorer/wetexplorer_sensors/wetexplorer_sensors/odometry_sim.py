#!/usr/bin/env python3

# JUST MODIFY THE SECTION MARKED "ALGORITHM BLOCK"

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


def yaw_to_quaternion(yaw: float):
    """
    Convert yaw (rotation around Z) to a quaternion (x, y, z, w).
    Roll = pitch = 0.
    """
    half = yaw * 0.5
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    return qx, qy, qz, qw


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__("forward_kinematics_node")

        # Robot parameters
        self.tracks_separation = 0.6108
        self.gear_ratio = 1.0
        self.radius_sprocket = 0.075

        # Time
        self.last_time = self.get_clock().now()

        # Topics
        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/odometry/forward_kinematics")

        input_topic = (
            self.get_parameter("input_topic")
            .get_parameter_value()
            .string_value
        )
        output_topic = (
            self.get_parameter("output_topic")
            .get_parameter_value()
            .string_value
        )

        # Subscriber and publisher
        self.sub = self.create_subscription(
            JointState,
            input_topic,
            self.joint_state_callback,
            10,
        )
        self.pub = self.create_publisher(Odometry, output_topic, 10)

        self.get_logger().info("ForwardKinematicsNode (from joint_states) initialized")

    def joint_state_callback(self, msg: JointState):
        # Find left/right wheel angular velocities (rad/s) from joint_states
        omega_L = 0.0
        omega_R = 0.0
        left_found = False
        right_found = False

        for i, name in enumerate(msg.name):
            if i >= len(msg.velocity):
                continue

            if name == "left_wheel_joint":
                omega_L = msg.velocity[i]
                left_found = True
            elif name == "right_wheel_joint":
                omega_R = msg.velocity[i]
                right_found = True

        if not (left_found and right_found):
            # Missing data, do not publish
            return

        # Time step
        now = self.get_clock().now()        
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.get_logger().info(f"New measurement received at {now.nanoseconds* 1e-9:.6f} s")

        if dt <= 0.0:
            dt = 1e-6  # avoid zero dt

        
        # Velocities in world/robot frame (initial values)
        vx = 0.0
        vy = 0.0
        omega = 0.0

        # Position in world coordinates (initial values)
        x = 0.0
        y = 0.0
        theta = 0.0

        # Covariances (initially zero)
        cov_x = 0.0        # variance in x [m^2]
        cov_y = 0.0        # variance in y [m^2]
        cov_theta = 0.0    # variance in theta [rad^2]

        cov_vx = 0.0       # variance of vx [ (m/s)^2 ]
        cov_vy = 0.0       # variance of vy [ (m/s)^2 ]
        cov_omega = 0.0    # variance of omega [ (rad/s)^2 ]

        # ----------------- ALGORITHM BLOCK -----------------
        # You have access to:
        #   - Wheel angular velocities (rad/s): omega_L, omega_R
        #   - Robot parameters:
        #         self.tracks_separation, self.radius_sprocket, self.gear_ratio
        #   - Time step: dt
        #
        # Define:
        #   - Body twist in robot frame: vx, vy, omega
        #   - Pose in world frame:       x, y, theta
        #
        # And the uncertainties:
        #   Pose:  cov_x, cov_y, cov_theta
        #   Twist: cov_vx, cov_vy, cov_omega
        #
        # By default everything is      
        # ----------------------------------------------------

        # Build Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        # Use "forward_kinematics" as reference frame
        odom.header.frame_id = "forward_kinematics"
        odom.child_frame_id = "base_link"

        # Pose
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(theta)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Twist (in the child frame)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        # Pose covariance: x, y, theta only; rest = 0
        pose_cov = [0.0] * 36
        pose_cov[0] = cov_x       # var(x)
        pose_cov[7] = cov_y       # var(y)
        pose_cov[35] = cov_theta  # var(theta around z)
        odom.pose.covariance = pose_cov

        # Twist covariance: vx, vy, omega only; rest = 0
        twist_cov = [0.0] * 36
        twist_cov[0] = cov_vx          # var(vx)
        twist_cov[7] = cov_vy          # var(vy)
        twist_cov[35] = cov_omega      # var(omega)
        odom.twist.covariance = twist_cov

        # Publish
        self.pub.publish(odom)
        self.last_time = now


def main():
    rclpy.init()
    node = ForwardKinematicsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
