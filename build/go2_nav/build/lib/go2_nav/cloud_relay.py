# cloud_relay.py
# Fixes two issues with /utlidar/cloud_deskewed:
# 1. Restamps to current time (fixes 104 second timestamp delay)
# 2. Changes frame_id from 'odom' to 'base_link' (fixes sensor origin at 0,0,0)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class CloudRelay(Node):
    def __init__(self):
        super().__init__('cloud_relay')
        self.sub = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_deskewed',
            self.callback,
            10)
        self.pub = self.create_publisher(
            PointCloud2,
            '/cloud_restamped',
            10)
        self.get_logger().info('Cloud relay started')
        self.get_logger().info('Subscribing: /utlidar/cloud_deskewed')
        self.get_logger().info('Publishing:  /cloud_restamped (frame_id=base_link, current timestamp)')

    def callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()  # fix timestamp
        msg.header.frame_id = 'base_link'                   # fix frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(CloudRelay())
    rclpy.shutdown()