#!/usr/bin/env python3
import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from ament_index_python.packages import get_package_share_directory


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')

        default_calib = os.path.join(
            get_package_share_directory('go2_nav'),
            'config', 'go2_front_calib.json'
        )

        self.declare_parameter('calib_path', default_calib)
        self.declare_parameter('topic', '/front_camera/camera_info')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('publish_rate_hz', 30.0)

        calib_path = self.get_parameter('calib_path').value
        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate_hz').value

        with open(calib_path) as f:
            c = json.load(f)
        self.width = c['width']
        self.height = c['height']
        self.K = c['K']
        self.D = c['D']
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        self.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        self.pub = self.create_publisher(CameraInfo, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_info)

        self.get_logger().info(
            f"Publishing CameraInfo on '{topic}' at {rate} Hz "
            f"(loaded from {calib_path})"
        )

    def publish_info(self):
        info = CameraInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.d = self.D
        info.k = self.K
        info.p = self.P
        info.r = self.R
        self.pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
