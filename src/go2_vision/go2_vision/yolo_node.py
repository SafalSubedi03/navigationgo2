import os
import time
import cv2
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from ultralytics import YOLO
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool
MODEL_PATH = 'yolo26n.engine'

CAMERA_TOPIC = "/go2/camera/compressed"  
CONF_THRESHOLD = 0.5
DEFAULT_CLASS = ("person")
DEVICE = "cuda:0" if torch.cuda.is_available() else "cpu"
JPEG_QUALITY = 80


PROCESS_EVERY_N_FRAMES = 2


class YoloCameraNode(Node):
    def __init__(self):
        super().__init__("yolo_camera_node")

        # Resolve path to custom tracker configuration (custombotsort.yaml)
        try:
            package_share_dir = get_package_share_directory('go2_vision')
            default_tracker_path = os.path.join(package_share_dir, 'config', 'custombotsort.yaml')
        except Exception:
            default_tracker_path = os.path.abspath(
                os.path.join(os.path.dirname(__file__), '..', 'config', 'custombotsort.yaml')
            )

        self.declare_parameter("tracker_config", default_tracker_path)
        self.tracker_config = self.get_parameter("tracker_config").get_parameter_value().string_value

        self.get_logger().info(f"Loading {MODEL_PATH} onto device = {DEVICE}")
        self.get_logger().info(f"Using tracker configuration: {self.tracker_config}")
        self.model = YOLO(MODEL_PATH, task='detect')

        self.frame_counter = 0
        self._device_confirmed = False
        self.locked_track_id = None
        self.target_class = DEFAULT_CLASS
        self.seen_track_ids = set() 
        self.last_seen_time = time.time()
        self.newObject = True 

        self.detection = self.create_publisher(Bool,"go2/objectDetected",10)
        self.sub = self.create_subscription(
            CompressedImage, CAMERA_TOPIC, self.image_callback, 10
        )
        self.annotated_pub = self.create_publisher(
            CompressedImage, "/yolo/annotated_image/compressed", 10
        )
        self.detections_pub = self.create_publisher(
            Detection2DArray, "/yolo/detections", 10
        )

        self.get_logger().info(
            f"YOLO tracker started. Subscribed to {CAMERA_TOPIC}, "
            f"publishing to /yolo/annotated_image/compressed and /yolo/detections "
            f"(processing every {PROCESS_EVERY_N_FRAMES} frames)."
            "Saving images when new objects are detected"
        )
        self.create_subscription(String,'/go2/select_target_class',self.on_target_change,10)

    def on_target_change(self, msg: String):
        new_class = msg.data.strip()
        if not new_class:
            return
        self.target_class = new_class
        self.locked_track_id = None
        self.seen_track_ids.clear()  # switching class means "new" objects should recapture
        self.get_logger().info(f"*** Target class switched to: '{self.target_class}' ***")


    def image_callback(self, msg: CompressedImage):
        self.frame_counter += 1
        if self.frame_counter % PROCESS_EVERY_N_FRAMES != 0:
            return

        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("Failed to decode incoming frame.", throttle_duration_sec=5.0)
            return

        t0 = time.time()
        try:
            results = self.model.track(
                frame,
                conf=CONF_THRESHOLD,
                persist=True,
                verbose=False,
                device=DEVICE,
                tracker=self.tracker_config
            )[0]
        except Exception as e:
            self.get_logger().error(f"Inference failed: {str(e)}", throttle_duration_sec=5.0)
            return
        inference_ms = (time.time() - t0) * 1000.0

        if not self._device_confirmed:
            if hasattr(self.model, 'model') and not isinstance(self.model.model, str):
                actual_device = next(self.model.model.parameters()).device
            else:
                actual_device = "cuda:0"
            self.get_logger().info(f"Model is running on: {actual_device} (requested: {DEVICE})")
            if str(actual_device) != DEVICE and DEVICE != "cpu":
                self.get_logger().warn("Requested GPU but model is NOT on GPU — check CUDA/torch install.")
            self._device_confirmed = True

        # Staleness check for the followed/locked target — runs every frame
        # regardless of whether any matching-class box exists this frame.
        if self.locked_track_id is not None and (time.time() - self.last_seen_time > 5.0):
            self.get_logger().warn("Locked target lost. Resetting tracker!")
            self.locked_track_id = None

        det_array = Detection2DArray()
        det_array.header = msg.header
        found_locked_target = False

        if results.boxes is not None and results.boxes.id is not None:
            for box in results.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                track_id = int(box.id[0]) if box.id is not None else -1
                self.get_logger().debug(
                    f"raw detection: class={cls_name}, track_id={track_id}, conf={float(box.conf[0]):.2f}"
                )

                if cls_name != self.target_class:
                    continue

                # --- Capture trigger: fires for ANY never-before-seen track_id
                #     of the target class, independent of the lock ---
                if track_id not in self.seen_track_ids:
                    self.seen_track_ids.add(track_id)
                    self.get_logger().info(
                        f"*** NEW {cls_name} detected (ID: {track_id}) — requesting capture ***"
                    )
                    self.detection.publish(Bool(data=True))

                # --- Lock acquisition (following logic, unchanged) ---
                if self.locked_track_id is None:
                    self.locked_track_id = track_id
                    self.last_seen_time = time.time()
                    self.get_logger().info(f"*** LOCKED ONTO {cls_name} (ID: {track_id}) ***")

                if track_id != self.locked_track_id:
                    continue  # only the locked target gets published to /yolo/detections

                found_locked_target = True
                self.last_seen_time = time.time()

                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(float, box.xyxy[0])

                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = (x1 + x2) / 2.0
                det.bbox.center.position.y = (y1 + y2) / 2.0
                det.bbox.size_x = x2 - x1
                det.bbox.size_y = y2 - y1

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls_name
                hyp.hypothesis.score = conf
                det.results.append(hyp)

                det_array.detections.append(det)

                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)
                label_y = min(int(y2) + 18, frame.shape[0] - 5)
                cv2.putText(
                    frame, f"LOCKED: {cls_name} ID:{track_id} {conf:.2f}", (int(x1), label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
                )

                break  # only draw/publish the locked target's box

        self.detections_pub.publish(det_array)

        success, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if success:
            out_msg = CompressedImage()
            out_msg.header = msg.header
            out_msg.format = "jpeg"
            out_msg.data = encoded.tobytes()
            self.annotated_pub.publish(out_msg)
        else:
            self.get_logger().warn("Failed to encode annotated frame.", throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = YoloCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()