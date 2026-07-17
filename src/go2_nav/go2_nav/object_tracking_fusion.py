#!/usr/bin/env python3
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped

from message_filters import Subscriber, ApproximateTimeSynchronizer
import sensor_msgs_py.point_cloud2 as pc2
from image_geometry import PinholeCameraModel

import tf2_ros
import tf2_geometry_msgs  # Registers PoseStamped support in tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class ObjectTrackingFusionNode(Node):
    def __init__(self):
        super().__init__('object_tracking_fusion')

        # Parameters
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('detection_topic', '/yolo/detections')
        self.declare_parameter('cloud_topic', '/utlidar/cloud_deskewed_restamped')
        self.declare_parameter('camera_info_topic', '/front_camera/camera_info')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('min_points_in_box', 5)

        self.target_class = self.get_parameter('target_class').value
        detection_topic = self.get_parameter('detection_topic').value
        cloud_topic = self.get_parameter('cloud_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        goal_pose_topic = self.get_parameter('goal_pose_topic').value
        sync_queue_size = self.get_parameter('sync_queue_size').value
        sync_slop = self.get_parameter('sync_slop').value
        self.min_points_in_box = self.get_parameter('min_points_in_box').value

        self.get_logger().info(
            f"Initializing Object Tracking Fusion Node.\n"
            f"Target class: '{self.target_class}'\n"
            f"Subscribing to: Detections: '{detection_topic}', Cloud: '{cloud_topic}', CameraInfo: '{camera_info_topic}'\n"
            f"Publishing to: '{goal_pose_topic}'"
        )

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera Model Setup
        self.cam_model = None
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )

        # Tracking variables
        self.consecutive_hits = 0
        self.consecutive_failures = 0
        self.last_seen_time = self.get_clock().now()

        # Synchronized Subscriptions
        det_sub = Subscriber(self, Detection2DArray, detection_topic)
        cloud_sub = Subscriber(self, PointCloud2, cloud_topic)

        self.sync = ApproximateTimeSynchronizer(
            [det_sub, cloud_sub],
            queue_size=sync_queue_size,
            slop=sync_slop
        )
        self.sync.registerCallback(self.synced_callback)

        # Publisher for goal_pose
        self.goal_pose_pub = self.create_publisher(PoseStamped, goal_pose_topic, 10)

    def camera_info_callback(self, msg: CameraInfo):
        if self.cam_model is None:
            self.cam_model = PinholeCameraModel()
            self.cam_model.fromCameraInfo(msg)
            self.get_logger().info("Camera model initialized successfully.")

    def synced_callback(self, det_array: Detection2DArray, cloud_msg: PointCloud2):
        if self.cam_model is None:
            self.get_logger().warn("Waiting for CameraInfo initialization...", throttle_duration_sec=5.0)
            return

        # Look for the target detection
        target_det = None
        for det in det_array.detections:
            for result in det.results:
                if result.hypothesis.class_id == self.target_class:
                    target_det = det
                    break
            if target_det is not None:
                break

        if target_det is None:
            self.consecutive_failures += 1
            if self.consecutive_failures % 30 == 0:
                self.get_logger().info(f"[SEARCHING] Target class '{self.target_class}' not found in detections.")
            return

        # Lookup transform from lidar point cloud frame to camera link
        try:
            transform = self.tf_buffer.lookup_transform(
                "camera_link",
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().debug(f"TF Lookup LiDAR -> Camera failed: {e}")
            return

        # Transform Point Cloud to Camera optical Frame
        try:
            cloud_in_cam = do_transform_cloud(cloud_msg, transform)
        except Exception as e:
            self.get_logger().warn(f"Cloud transform to camera_link failed: {e}", throttle_duration_sec=2.0)
            return

        # Read x, y, z points
        points_struct = pc2.read_points(cloud_in_cam, field_names=("x", "y", "z"), skip_nans=True)
        points = np.stack([points_struct["x"], points_struct["y"], points_struct["z"]], axis=-1).astype(np.float32)

        # Convert robot mechanical frame coordinates to camera optical frame coordinates:
        # LiDAR Forward (X) -> Optical Z (Forward)
        # LiDAR Left (Y) -> Optical -X (Right)
        # LiDAR Up (Z) -> Optical -Y (Down)
        optical_x = -points[:, 1]
        optical_y = -points[:, 2]
        optical_z = points[:, 0]
        points = np.stack([optical_x, optical_y, optical_z], axis=-1)

        # Filter out points that are behind or extremely close to the camera
        points = points[points[:, 2] > 0.05]

        if points.shape[0] == 0:
            return

        # Project 3D points onto 2D image coordinates
        fx = self.cam_model.fx()
        fy = self.cam_model.fy()
        cx = self.cam_model.cx()
        cy = self.cam_model.cy()

        us = (points[:, 0] * fx / points[:, 2]) + cx
        vs = (points[:, 1] * fy / points[:, 2]) + cy

        # Calculate bounding box coordinates
        cx_box = target_det.bbox.center.position.x
        cy_box = target_det.bbox.center.position.y
        half_w = target_det.bbox.size_x / 2.0
        half_h = target_det.bbox.size_y / 2.0
        x1, x2 = cx_box - half_w, cx_box + half_w
        y1, y2 = cy_box - half_h, cy_box + half_h

        # Filter points lying inside bounding box
        in_box = (us >= x1) & (us <= x2) & (vs >= y1) & (vs <= y2)
        box_points = points[in_box]

        if box_points.shape[0] < self.min_points_in_box:
            self.consecutive_failures += 1
            return

        # Filter outliers by depth (10th to 90th percentile)
        z_vals = box_points[:, 2]
        lo, hi = np.percentile(z_vals, (10, 90))
        filtered = box_points[(z_vals >= lo) & (z_vals <= hi)]
        if filtered.shape[0] == 0:
            filtered = box_points

        # Compute centroid (median) in camera optical coordinates
        centroid_cam = np.median(filtered, axis=0)

        # Convert back to camera mechanical coordinates:
        mech_x = centroid_cam[2]      # forward = optical z
        mech_y = -centroid_cam[0]     # left = -optical x
        mech_z = -centroid_cam[1]     # up = -optical y

        # Create Camera-frame Pose
        pose_cam = PoseStamped()
        pose_cam.header.frame_id = "camera_link"
        pose_cam.header.stamp = det_array.header.stamp
        pose_cam.pose.position.x = float(mech_x)
        pose_cam.pose.position.y = float(mech_y)
        pose_cam.pose.position.z = float(mech_z)

        # Transform to Map-frame Pose
        try:
            # Set header stamp to current time for stability in real-time lookup
            pose_cam.header.stamp = self.get_clock().now().to_msg()
            pose_map = self.tf_buffer.transform(pose_cam, "map", timeout=Duration(seconds=0.1))

            # Publish goal_pose
            self.goal_pose_pub.publish(pose_map)

            self.consecutive_failures = 0
            self.consecutive_hits += 1
            self.last_seen_time = self.get_clock().now()

            status = "LOCKED" if self.consecutive_hits >= 3 else "SEARCHING"
            self.get_logger().info(
                f"[{status}] Hits: {self.consecutive_hits} | Target Location: "
                f"X={pose_map.pose.position.x:.2f}, Y={pose_map.pose.position.y:.2f}"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to transform object pose to map frame: {e}", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackingFusionNode()
    try:
        # Using MultiThreadedExecutor to handle TF2 listeners and Synchronized callbacks concurrently
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
