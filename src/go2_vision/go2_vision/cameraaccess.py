import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import os, sys, time
import cv2
from cv_bridge import CvBridge
import numpy as np

# Dynamically inject local Unitree SDK into sys.path by climbing up to the workspace root
current_dir = os.path.dirname(os.path.abspath(__file__))
SDK_PATH = None
while current_dir != os.path.dirname(current_dir):  # Stop if we hit the filesystem root '/'
    possible_sdk = os.path.join(current_dir, "sdk", "unitree_sdk2_python")
    if os.path.exists(possible_sdk):
        SDK_PATH = possible_sdk
        break
    current_dir = os.path.dirname(current_dir)

if SDK_PATH and SDK_PATH not in sys.path:
    sys.path.append(SDK_PATH)
else:
    print(f"Warning: Could not find Unitree SDK path dynamically!", file=sys.stderr)

# Unitree SDK Imports
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

NETWORK_INTERFACE = "eth0"


class cameraimg(Node):
    def __init__(self):
        super().__init__('cameraimg')

        if NETWORK_INTERFACE:
            ChannelFactoryInitialize(0, NETWORK_INTERFACE)
        else:
            ChannelFactoryInitialize(0)

        # 2. Setup Unitree Video Client
        self.client = VideoClient()
        self.client.SetTimeout(3.0)  # matches Unitree's own example; 30s was unnecessarily long
        self.client.Init()

        self.publisher = self.create_publisher(Image, 'go2/camera', 10)
        self.bridge = CvBridge()

        self.timer_period = 0.08  # ~12.5 Hz, easier on the video RPC channel than 25 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Track consecutive failures so we can back off and avoid log spam
        self.consecutive_failures = 0
        self.max_consecutive_failures = 10

        self.get_logger().info("Go2 Camera Node has started.")

    def timer_callback(self):
        try:
            code, data = self.client.GetImageSample()
        except Exception as e:
            self.get_logger().error(f"GetImageSample raised an exception: {str(e)}",
                                     throttle_duration_sec=5.0)
            return

        if code == 0:
            self.consecutive_failures = 0
            try:
                image_data = np.frombuffer(bytes(data), dtype=np.uint8)
                cv_image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

                if cv_image is not None:
                    ros_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                    ros_image_msg.header.stamp = self.get_clock().now().to_msg()
                    ros_image_msg.header.frame_id = "base_link"
                    self.publisher.publish(ros_image_msg)
                else:
                    self.get_logger().warn("Decoded image is empty.", throttle_duration_sec=5.0)

            except Exception as e:
                self.get_logger().error(f"Failed to convert or publish image: {str(e)}",
                                         throttle_duration_sec=5.0)

        else:
            # Error 3104 is a known intermittent Unitree video-service hiccup.
            # Don't spam the logs for every dropped frame; just track and throttle.
            self.consecutive_failures += 1

            if self.consecutive_failures <= self.max_consecutive_failures:
                self.get_logger().warn(
                    f"GetImageSample failed with error code: {code} "
                    f"(consecutive failures: {self.consecutive_failures})",
                    throttle_duration_sec=2.0
                )
            else:
                # Sustained failure — this is worth surfacing as a real error,
                # but still throttled so it doesn't flood the log.
                self.get_logger().error(
                    f"GetImageSample has failed {self.consecutive_failures} times in a row "
                    f"(code: {code}). Check robot connection / network interface.",
                    throttle_duration_sec=10.0
                )


def main(args=None):
    rclpy.init(args=args)
    node = cameraimg()
    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Go2 Camera Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()