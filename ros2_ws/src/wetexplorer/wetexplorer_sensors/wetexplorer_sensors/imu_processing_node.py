#!/usr/bin/env python3

# JUST MODIFY THE FUNCTION imu_callback()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
import math
from .quaternions import quaternion_to_rpy, rpy_to_quaternion


class IMUProcessingNode(Node):
    def __init__(self):
        super().__init__('imu_processing_node')

        # Parameters
        self.declare_parameter('input_topic', 'imu/data')
        self.declare_parameter('output_topic', 'imu/data_processed')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=50,
        )

        self.sub_imu = self.create_subscription(Imu, input_topic, self.imu_callback, qos)
        self.pub_imu = self.create_publisher(Imu, output_topic, qos)

        self.get_logger().info('IMU Signal Processing Node initialized')

    def imu_callback(self, msg: Imu):      
        
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # In ROS we use logger.info() to print insteat of the usual print() in python
        self.get_logger().info(f"New measurement received at {time:.6f} s")
        
        # Extract the readings from the message
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z


        roll, pitch, yaw = quaternion_to_rpy(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)


        
        # -------- algorithm goes here --------
        # roll, pitch, yaw, gx, gy, gz, ax, ay, az can be modified
        # -------------------------------------


        # Now we publish the processed data, so we update the following message with your results:

        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)

        out = Imu()
        out.header = msg.header
        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw
        out.orientation_covariance = msg.orientation_covariance

        out.angular_velocity.x = gx
        out.angular_velocity.y = gy
        out.angular_velocity.z = gz
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = az
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub_imu.publish(out)
        


def main():
    rclpy.init()
    node = IMUProcessingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
