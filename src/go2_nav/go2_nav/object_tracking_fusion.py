#!/usr/bin/env python3
import threading
import numpy as np
import math 
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import PointCloud2, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from message_filters import Subscriber, ApproximateTimeSynchronizer
import sensor_msgs_py.point_cloud2 as pc2
from image_geometry import PinholeCameraModel

import tf2_ros
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 -- REQUIRED: registers PoseStamped support in tf2, even though nothing in this file calls it directly


TIMEOUT_SECONDS = 2.0
LOCK_ON_HITS = 3
STANDOFF_DISTANCE = 0.5  # meters -- stop this far short of the object, facing it
ARRIVAL_TOLERANCE = 0.6 #Make sure this is greater then the offset set in the navigation parameter



class ObjectPursuitNode(Node):
    def __init__(self):
        super().__init__('object_pursuit_node')

        # Configurable via launch file / ros2 param, defaults to "person"
        self.declare_parameter('target_class', 'person')
        self.target_class = self.get_parameter('target_class').value

        self.declare_parameter('search_yaw_rate', 0.6)
        self.search_yaw_rate = float(self.get_parameter('search_yaw_rate').value)

        self.declare_parameter('sync_slop', 0.3)
        self.sync_slop = float(self.get_parameter('sync_slop').value)

        self.create_subscription(String, '/go2/select_target_class', self.targetInfo, 10)
        
        self.declare_parameter('cloud_topic', '/utlidar/cloud_deskewed_restamped')
        self.cloud_topic = self.get_parameter('cloud_topic').value

        # --- Goal Pose Optimization Parameters (now tunable via launch/ros2 param) ---
        self.declare_parameter('cluster_threshold', 0.25)  # meters, depth gap that splits clusters
        self.cluster_threshold = float(self.get_parameter('cluster_threshold').value)

        self.declare_parameter('min_cluster_points', 10)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)

        self.declare_parameter('bbox_shrink_factor', 0.8)  # fraction of full bbox width/height kept, symmetric on both axes
        self.bbox_shrink_factor = float(self.get_parameter('bbox_shrink_factor').value)

        self.sensor_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.state_lock = threading.Lock()
        self.latest_pose = None
        self.last_seen_time = self.get_clock().now()
        self.consecutive_hits = 0
        
        self.last_target_depth = None  # camera-frame depth of last locked-on target, protected by state_lock

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cam_model = None
        self.create_subscription(
            CameraInfo, "/front_camera/camera_info",
            self.camera_info_callback, 10,
            callback_group=self.sensor_cb_group
        )

        det_sub = Subscriber(self, Detection2DArray, "/yolo/detections")
        cloud_sub = Subscriber(self, PointCloud2, self.cloud_topic)
        self.sync = ApproximateTimeSynchronizer([det_sub, cloud_sub], queue_size=10, slop=self.sync_slop)
        self.sync.registerCallback(self.synced_callback)

        self.goal_pose_pub = self.create_publisher(PoseStamped, '/folow_object/goal_pose_raw', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)

        # Simple periodic status/timeout check and search spin timer
        self.status_timer = self.create_timer(
            0.5, self.status_check, callback_group=self.timer_cb_group
        )

        self.get_logger().info(
            f"Object Pursuit Node started. Target class: '{self.target_class}', Search Yaw Rate: {self.search_yaw_rate} rad/s"
        )


    def targetInfo(self, msg: String):
        self.target_class = msg.data.strip()
        self.get_logger().info(f"Target class updated to: '{self.target_class}'")

    def camera_info_callback(self, msg: CameraInfo):
        if self.cam_model is None:
            self.cam_model = PinholeCameraModel()
            self.cam_model.fromCameraInfo(msg)
            self.get_logger().info("Camera model initialized.")

    def synced_callback(self, det_array, cloud_msg):

        if self.cam_model is None:
            self.get_logger().warn("Waiting for camera_info...", throttle_duration_sec=5.0)
            return

        if len(det_array.detections) == 0:
            with self.state_lock:
                self.consecutive_hits = 0
                self.last_target_depth = None  # lost track -- don't compare future clusters to a stale depth
            return

        target_det = det_array.detections[0]

        try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "camera_link",
                    cloud_msg.header.frame_id,
                    cloud_msg.header.stamp,
                    timeout=Duration(seconds=0.1))
            except Exception:
                transform = self.tf_buffer.lookup_transform(
                    "camera_link",
                    cloud_msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1))
                self.get_logger().warn(f"[TF] Using latest-available transform instead of exact timestamp",throttle_duration_sec=5.0)
        
        except Exception as e:
            self.get_logger().warn(f"TF Lidar->Camera failed ({cloud_msg.header.frame_id} -> camera_link): {e}", throttle_duration_sec=5.0)
            return

        # Read x/y/z from the ORIGINAL cloud (no transform yet)
        points_struct = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        points_raw = np.stack([points_struct["x"], points_struct["y"], points_struct["z"]], axis=-1).astype(np.float64)

        # Apply the LiDAR->camera transform manually via rotation matrix + translation
        t = transform.transform.translation
        q = transform.transform.rotation
        translation = np.array([t.x, t.y, t.z])

        def quat_to_rot_matrix(q):
            x, y, z, w = q.x, q.y, q.z, q.w
            return np.array([
                [1 - 2*(y*y + z*z),     2*(x*y - z*w),         2*(x*z + y*w)],
                [2*(x*y + z*w),         1 - 2*(x*x + z*z),     2*(y*z - x*w)],
                [2*(x*z - y*w),         2*(y*z + x*w),         1 - 2*(x*x + y*y)],
            ])

        R = quat_to_rot_matrix(q)
        points = (R @ points_raw.T).T + translation
        points = points.astype(np.float32)
        optical_x = -points[:, 1]
        optical_y = -points[:, 2]
        optical_z = points[:, 0]
        points = np.stack([optical_x, optical_y, optical_z], axis=-1)
        points = points[points[:, 2] > 0.05]

        fx, fy = self.cam_model.fx(), self.cam_model.fy()
        cx, cy = self.cam_model.cx(), self.cam_model.cy()
        us = (points[:, 0] * fx / points[:, 2]) + cx
        vs = (points[:, 1] * fy / points[:, 2]) + cy

        cx_box = target_det.bbox.center.position.x
        cy_box = target_det.bbox.center.position.y
        # Symmetric shrink on both axes, tunable via bbox_shrink_factor
        # (previously width was hardcoded to a different shrink than height)
        half_w = (target_det.bbox.size_x * self.bbox_shrink_factor*0.5) / 2.0
        half_h = (target_det.bbox.size_y * self.bbox_shrink_factor) / 2.0
        x1, x2 = cx_box - half_w, cx_box + half_w
        y1, y2 = cy_box - half_h, cy_box + half_h

        in_box = (us >= x1) & (us <= x2) & (vs >= y1) & (vs <= y2)
        box_points = points[in_box]
        if box_points.shape[0] < 25:
            self.get_logger().warn(f"Insufficient LiDAR points ({box_points.shape[0]}) projected into detection box.", throttle_duration_sec=5.0)
            with self.state_lock:
                self.consecutive_hits = 0
                self.last_target_depth = None
            return

        #Cluster Separation filtration and Selection

        # Sort points by depth (camera-frame z) first -- clustering only makes
        # sense on a depth-sorted array, otherwise "adjacent" points aren't
        # actually neighbors in depth.
        sort_idx = np.argsort(box_points[:, 2])
        box_points = box_points[sort_idx]

        clusters = []          # list of (start_index, end_index) boundaries into box_points
        cluster_start = 0

        for i in range(1, len(box_points)):
            depth_gap = abs(box_points[i, 2] - box_points[i - 1, 2])

            if depth_gap > self.cluster_threshold:
                # Cluster boundary found -- close out the current cluster
                cluster_len = i - cluster_start
                if cluster_len >= self.min_cluster_points:
                    clusters.append((cluster_start, i))
                cluster_start = i

        # Don't forget the final cluster after the loop ends
        final_len = len(box_points) - cluster_start
        if final_len >= self.min_cluster_points:
            clusters.append((cluster_start, len(box_points)))

        # If no valid cluster is found, return no lidar match
        if len(clusters) == 0:
            self.get_logger().warn(
                "Too diverse lidar points to form valid clusters.",
                throttle_duration_sec=5.0
            )
            with self.state_lock:
                self.consecutive_hits = 0
                self.last_target_depth = None
            return

        # Selection: use "currently locked on" (consecutive_hits) as the signal,
        # not "has a pose ever been published" -- latest_pose never resets to
        # None on its own, so it can't tell us whether the track is still live.
        with self.state_lock:
            is_locked = self.consecutive_hits >= LOCK_ON_HITS
            last_depth = self.last_target_depth

        if not is_locked or last_depth is None:
            # Unlocked case: no trustworthy prior depth, fall back to nearest
            # cluster (clusters[0], since box_points is sorted ascending by depth)
            selected_start, selected_end = clusters[0]
        else:
            # Locked case: pick the cluster whose mean depth is closest to
            # the last known target depth, not just nearest to the camera
            best_diff = None
            selected_start, selected_end = clusters[0]  # fallback default

            for start, end in clusters:
                cluster_depth = np.mean(box_points[start:end, 2])
                diff = abs(cluster_depth - last_depth)

                if best_diff is None or diff < best_diff:
                    best_diff = diff
                    selected_start, selected_end = start, end

        box_points = box_points[selected_start:selected_end, :]
        self.get_logger().info(f"Selected cluster: x {box_points[0,0]}, y {box_points[0,1]} ")

        z_vals = box_points[:, 2]
        lo, hi = np.percentile(z_vals, (10, 90))
        filtered = box_points[(z_vals >= lo) & (z_vals <= hi)]
        if filtered.shape[0] == 0:
            filtered = box_points

        centroid_cam = np.median(filtered, axis=0)
        mech_x = centroid_cam[2]
        mech_y = -centroid_cam[0]
        mech_z = -centroid_cam[1]

        pose_cam = PoseStamped()
        pose_cam.header.frame_id = "camera_link"
        pose_cam.header.stamp = det_array.header.stamp
        pose_cam.pose.position.x = float(mech_x)
        pose_cam.pose.position.y = float(mech_y)
        pose_cam.pose.position.z = float(mech_z)

        try:
            pose_cam.header.stamp = rclpy.time.Time().to_msg()
            pose_map = self.tf_buffer.transform(pose_cam, "map", timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f"TF camera->map failed: {e}", throttle_duration_sec=5.0)
            return


        # Get the robot's current position in the map frame
        try:
            robot_tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"Could not get robot pose for goal orientation: {e}", throttle_duration_sec=5.0)
            robot_tf = None
      
        if robot_tf is not None:
            rx = robot_tf.transform.translation.x
            ry = robot_tf.transform.translation.y

            dx = pose_map.pose.position.x - rx
            dy = pose_map.pose.position.y - ry
            distance = math.hypot(dx, dy)
            yaw = math.atan2(dy, dx)

            if distance > STANDOFF_DISTANCE:
                pose_map.pose.position.x = rx + (distance - STANDOFF_DISTANCE) * math.cos(yaw)
                pose_map.pose.position.y = ry + (distance - STANDOFF_DISTANCE) * math.sin(yaw)

            # Set orientation to face the object (yaw only)
            pose_map.pose.orientation.z = math.sin(yaw / 2.0)
            pose_map.pose.orientation.w = math.cos(yaw / 2.0)
            pose_map.pose.orientation.x = 0.0
            pose_map.pose.orientation.y = 0.0

        with self.state_lock:
            self.latest_pose = pose_map
            self.last_seen_time = self.get_clock().now()
            self.consecutive_hits += 1
            self.last_target_depth = float(centroid_cam[2])  # update only on a trusted, published detection

        self.get_logger().info("[SYNC] Publishing Goal Message")
        self.goal_pose_pub.publish(pose_map)
        




    def status_check(self):
        with self.state_lock:
            last_seen = self.last_seen_time
            hits = self.consecutive_hits
            pose = self.latest_pose

        time_since_last_seen = self.get_clock().now() - last_seen
        locked_on = hits >= LOCK_ON_HITS           
        
        #State-1, Idle / Robot can see the object before timeout
        if time_since_last_seen < Duration(seconds=TIMEOUT_SECONDS):
            if locked_on:
                self.get_logger().info("Robot is locked on the object")               
            else:
                self.get_logger().info("Robot is actively searching for the object")  
            return         
               

        #State-2 when object timeout has occured
        
        if pose is None:
            self.get_logger().warn("Pose Not Received")
            return            
        try:
            robot_tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"Could not get robot pose for goal orientation: {e}", throttle_duration_sec=5.0)
            return 

        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y    

        distance = math.hypot(pose.pose.position.x - rx, pose.pose.position.y - ry)
        self.get_logger().info(f"[Status] \n Distance =  {distance} \n Arrival_Tolerance = {ARRIVAL_TOLERANCE} ")

        if distance <= ARRIVAL_TOLERANCE:
            cmd_msg = Twist()
            cmd_msg.angular.z = self.search_yaw_rate
            self.cmd_vel_pub.publish(cmd_msg)
            self.get_logger().warn("Rotation Cmd Sent")

       
        
        status = "LOCKED" if locked_on else "SEARCHING"
        self.get_logger().info(
            f"[{status}] Hits: {hits} | "
            f"X={pose.pose.position.x:.2f}, Y={pose.pose.position.y:.2f}",
            throttle_duration_sec=1.0
    )
    
    


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPursuitNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info("Starting MultiThreadedExecutor...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C pressed. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()