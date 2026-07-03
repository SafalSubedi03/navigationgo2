#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


class RestampNode(Node):

    def __init__(self):
        super().__init__('restamp_node')

        # PointCloud2 restamper
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/utlidar/cloud_deskewed_restamped', 10)
        self.create_subscription(
            PointCloud2, '/utlidar/cloud_deskewed',
            self.cloud_cb, 10)

        # Odometry restamper
        self.odom_pub = self.create_publisher(
            Odometry, '/utlidar/robot_odom_restamped', 10)
        self.create_subscription(
            Odometry, '/utlidar/robot_odom',
            self.odom_cb, 10)

        self.get_logger().info(
            'RestampNode started -- fixing Unitree ~126s clock offset'
        )

    def cloud_cb(self, msg: PointCloud2):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cloud_pub.publish(msg)

    def odom_cb(self, msg: Odometry):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = RestampNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
