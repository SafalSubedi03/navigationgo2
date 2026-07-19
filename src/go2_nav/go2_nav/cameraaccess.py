import os, sys, time
# os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

# Unitree SDK Imports
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

NETWORK_INTERFACE = "eth0"

# JPEG quality: 0-100. 80 is a good balance of size vs visual quality.
JPEG_QUALITY = 80


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

        # Publishing CompressedImage instead of raw Image -- a raw 1920x1080
        # bgr8 frame is ~6.22 MB uncompressed. At 12.5 Hz that needs ~77 MB/s
        # of sustained throughput, which no ordinary ethernet/WiFi link can
        # sustain -- that bandwidth ceiling was the actual cause of the
        # "latency" (frames queuing up faster than the network could drain
        # them). JPEG at quality 80 typically shrinks each frame to
        # ~100-300 KB, a 20-60x reduction.
        self.publisher = self.create_publisher(CompressedImage, 'go2/camera/compressed', 10)

        self.timer_period = 0.08  # ~12.5 Hz, easier on the video RPC channel than 25 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Track consecutive failures so we can back off and avoid log spam
        self.consecutive_failures = 0
        self.max_consecutive_failures = 10

        self.get_logger().info("Go2 Camera Node has started (publishing CompressedImage).")

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
                    success, encoded = cv2.imencode(
                        '.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
                    )
                    if success:
                        msg = CompressedImage()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "base_link"
                        msg.format = "jpeg"
                        msg.data = encoded.tobytes()
                        self.publisher.publish(msg)
                    else:
                        self.get_logger().warn("JPEG encoding failed.", throttle_duration_sec=5.0)
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