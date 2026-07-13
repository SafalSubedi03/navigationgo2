import os
import sys

# We keep this because your camera node forces ROS 2 to use FastRTPS.
# Both nodes must match to see each other's topics.
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO

ENGINE_PATH = "/workspace/yolo11n.engine"
PROCESS_EVERY_N_FRAMES = 2  
CONFIDENCE_THRESHOLD = 0.5
JPEG_QUALITY = 80


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.get_logger().info(f"Loading TensorRT engine from {ENGINE_PATH} ...")
        self.model = YOLO(ENGINE_PATH, task='detect')
        self.get_logger().info("Model loaded.")

        self.frame_counter = 0

        # Subscribing using the relative topic name matching your camera node
        self.subscription = self.create_subscription(
            CompressedImage,
            '/go2/camera/compressed',
            self.image_callback,
            10
        )

        # Publishing the output topic
        self.annotated_pub = self.create_publisher(
            CompressedImage, 'go2/camera/detections', 10
        )

        self.get_logger().info(
            f"YOLO detector started. Subscribed to go2/camera/compressed, "
            f"publishing annotated frames to go2/camera/detections."
        )

    def image_callback(self, msg: CompressedImage):
        self.get_logger().info("--> RECEIVED A FRAME! Running YOLO inference...")
        self.frame_counter += 1
        if self.frame_counter % PROCESS_EVERY_N_FRAMES != 0:
            return

        # Decode the incoming compressed JPEG frame
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn("Failed to decode incoming frame.", throttle_duration_sec=5.0)
            return

        # Run YOLO Inference
        try:
            results = self.model.predict(img, conf=CONFIDENCE_THRESHOLD, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Inference failed: {str(e)}", throttle_duration_sec=5.0)
            return

        # Draw bounding boxes onto the frame
        annotated = results[0].plot()

        # Re-compress the annotated image back to JPEG to save network bandwidth
        success, encoded = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not success:
            self.get_logger().warn("Failed to encode annotated frame.", throttle_duration_sec=5.0)
            return

        # Publish the annotated frame
        out_msg = CompressedImage()
        out_msg.header = msg.header
        out_msg.format = "jpeg"
        out_msg.data = encoded.tobytes()
        self.annotated_pub.publish(out_msg)

        num_detections = len(results[0].boxes)
        if num_detections > 0:
            self.get_logger().info(f"Detected {num_detections} object(s)", throttle_duration_sec=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLO detector...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()