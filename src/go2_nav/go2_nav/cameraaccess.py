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
from std_msgs.msg import Bool

NETWORK_INTERFACE = "eth0"
JPEG_QUALITY = 80

# Where captured detection images are saved.
# /workspace is the mount point for ~/navigationgo2 on the Jetson host,
# so files land in ~/navigationgo2/detections/ outside the container too.
SAVE_DIR = "/workspace/detections"


class cameraimg(Node):
    def __init__(self):
        super().__init__('cameraimg')

        if NETWORK_INTERFACE:
            ChannelFactoryInitialize(0, NETWORK_INTERFACE)
        else:
            ChannelFactoryInitialize(0)

        # Setup Unitree Video Client
        self.client = VideoClient()
        self.client.SetTimeout(3.0)
        self.client.Init()

        self.create_subscription(Bool, 'go2/objectDetected', self.clickimg, 10)
        self.imgsave = self.create_publisher(Bool,'go2/objectDetected',10)
        self.publisher = self.create_publisher(CompressedImage, 'go2/camera/compressed', 10)

        self.timer_period = 0.08  # ~12.5 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Track consecutive failures so we can back off and avoid log spam
        self.consecutive_failures = 0
        self.max_consecutive_failures = 10

        # Cache of the most recently decoded frame, used for on-detection capture
        self.latest_frame = None

        # Debounce so a sustained "True" doesn't flood the disk with saves
        self.last_capture_time = 0.0
        self.capture_cooldown = 1.0  # seconds between saves

        # Make sure the save directory exists
        os.makedirs(SAVE_DIR, exist_ok=True)

        self.get_logger().info("Go2 Camera Node has started (publishing CompressedImage).")
        self.get_logger().info(f"Detection captures will be saved to {SAVE_DIR}")

    def clickimg(self, msg: Bool):
        self.imgsave.publish(Bool(data=False))
        if not msg.data:
            return

        if self.latest_frame is None:
            self.get_logger().warn("Object detected but no frame available yet to save.",
                                    throttle_duration_sec=5.0)
            return

        now = time.time()
        if (now - self.last_capture_time) < self.capture_cooldown:
            return  # too soon since last save, skip

        self.last_capture_time = now
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
        ms = int((now % 1) * 1000)
        filename = os.path.join(SAVE_DIR, f"detection_{timestamp}_{ms:03d}.jpg")

        try:
            success = cv2.imwrite(filename, self.latest_frame)
            if success:
                self.get_logger().info(f"Saved detection image: {filename}")
            else:
                self.get_logger().warn(f"cv2.imwrite failed for {filename}",
                                        throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f"Failed to save detection image: {str(e)}",
                                     throttle_duration_sec=5.0)

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
                    # Cache the latest frame for on-detection capture
                    self.latest_frame = cv_image

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
            self.consecutive_failures += 1

            if self.consecutive_failures <= self.max_consecutive_failures:
                self.get_logger().warn(
                    f"GetImageSample failed with error code: {code} "
                    f"(consecutive failures: {self.consecutive_failures})",
                    throttle_duration_sec=2.0
                )
            else:
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