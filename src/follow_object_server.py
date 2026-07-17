
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import PointCloud2, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped


from message_filters import Subscriber, ApproximateTimeSynchronizer
import sensor_msgs_py.point_cloud2 as pc2
from image_geometry import PinholeCameraModel

import tf2_ros
import tf2_geometry_msgs  # Note: Just importing this registers PoseStamped support in tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

# Make sure your package actually compiles this interface before running!
from go2_vision_msgs.action import FollowObject


#Global offsets


class FollowObjectActionNode(Node):
    def __init__(self):
        super().__init__('follow_object_server')

        self.sensor_cb_group = MutuallyExclusiveCallbackGroup()
        self.action_cb_group = MutuallyExclusiveCallbackGroup()

        self.state_lock = threading.Lock()

        self.is_tracking_active = False
        self.target_class = ""
        self.latest_pose = None 
        self.last_seen_time = None 
        self.consecutive_hits = 0 

        self.get_logger().info("Follow Object Action Server: Phase 1 Initialized.")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer,self)

        self.cam_model = None 
        
        self.create_subscription(CameraInfo,"/front_camera/camera_info",self.camera_info_callback,10,callback_group=self.sensor_cb_group)

        det_sub = Subscriber(self,Detection2DArray, "/yolo/detections")
        cloud_sub = Subscriber(self,PointCloud2,"/utlidar/robot_odom_restamped")

        self.sync = ApproximateTimeSynchronizer([det_sub,cloud_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.synced_callback)

        #Action Server 
        self._action_server = ActionServer(
            self,
            FollowObject,
            'follow_object',
            self.execute_callback,
            callback_group=self.action_cb_group
        )
        self.get_logger().info("Follow Object Action Server ready.")
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    
    def camera_info_callback(self, msg: CameraInfo):
        if self.cam_model is None:
            self.cam_model = PinholeCameraModel()
            self.cam_model.fromCameraInfo(msg)
            self.get_logger().info("Camera model initialized.")

    def synced_callback(self, det_array, cloud_msg):
        if not self.is_tracking_active:
            return
        
        if len(det_array.detections) == 0:
            with self.state_lock:
                self.consecutive_hits = 0
            return 
        self.get_logger().info(f"synced_callback fired, {len(det_array.detections)} detections")
        target_det = det_array.detections[0]


        #Transform lidar tf -> camera tf, whenever we recive a camera_link msg

        try:
            # Ask TF2: "Where was the Lidar relative to the Camera at this exact moment?"
            transform = self.tf_buffer.lookup_transform(
                "camera_link",
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
                timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().debug(f"TF Lidar->Camera failed: {e}")
            return


        #calculation of the lidar points as seen by the camera's view.
        #for example if the lidar reading says object is at 2m,1m,3m then the below section will calcualte that exact same point form the camera's view
        cloud_in_cam = do_transform_cloud(cloud_msg,transform)
        points_struct = pc2.read_points(cloud_in_cam,field_names=("x" , "y", 'z'), skip_nans=True)
        points = np.stack([points_struct["x"],points_struct["y"],points_struct["z"]], axis=-1).astype(np.float32)


        #Conversion of the robot points xyz conrdinates to the camera y-z-z cordinates system 
        optical_x = -points[:, 1]
        optical_y = -points[:, 2]
        optical_z = points[:, 0]
        points = np.stack([optical_x, optical_y, optical_z], axis=-1)
        points = points[points[:, 2] > 0.05]

        #calculation of the projection of 3d points on a image frame.
        fx, fy = self.cam_model.fx(), self.cam_model.fy() #focal points
        cx, cy = self.cam_model.cx(), self.cam_model.cy()
        us = (points[:, 0] * fx / points[:, 2]) + cx
        vs = (points[:, 1] * fy / points[:, 2]) + cy

        #us and vs are the 2D pixel cordinates.

        #filter pixel values, i.e keep only the lidar reaadings which matches with the bouding box form the detection node
        cx_box = target_det.bbox.center.position.x
        cy_box = target_det.bbox.center.position.y
        half_w, half_h = target_det.bbox.size_x / 2.0, target_det.bbox.size_y / 2.0
        x1, x2 = cx_box - half_w, cx_box + half_w
        y1, y2 = cy_box - half_h, cy_box + half_h


        in_box = (us >= x1) & (us <= x2) & (vs >= y1) & (vs <= y2)
        box_points = points[in_box]
        #look for at least 5 2d pixels lying inside the detection frame else discard
        if box_points.shape[0] < 5:
            with self.state_lock:
                self.consecutive_hits = 0 
            return
        

        #calculation of the position of the object relative to the camera.
        z_vals = box_points[:, 2]
        lo, hi = np.percentile(z_vals, (10, 90))
        filtered = box_points[(z_vals >= lo) & (z_vals <= hi)]
        if filtered.shape[0] == 0: filtered = box_points

        # Calculate the center of mass, a single point representing the object.
        centroid_cam = np.median(filtered, axis=0) 
        mech_x = centroid_cam[2]      # forward = optical z
        mech_y = -centroid_cam[0]     # left = -optical x (right)
        mech_z = -centroid_cam[1]     # up = -optical y (down)


        #Encode the position as a goal_pose 
        pose_cam = PoseStamped()
        pose_cam.header.frame_id = "camera_link"
        pose_cam.header.stamp = det_array.header.stamp
        pose_cam.pose.position.x = float(mech_x)
        pose_cam.pose.position.y = float(mech_y)
        pose_cam.pose.position.z = float(mech_z)


        try:
            pose_cam.header.stamp = rclpy.time.Time().to_msg()
            pose_map = self.tf_buffer.transform(pose_cam, "map", timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            return

       
        with self.state_lock:
            self.latest_pose = pose_map
            self.last_seen_time = self.get_clock().now()
            self.consecutive_hits += 1
        self.goal_pose_pub.publish(pose_map)


    def execute_callback(self, goal_handle):
        req_class = goal_handle.request.target_class
        self.get_logger().info(f"Mission started: Following '{req_class}'")

        with self.state_lock:
            self.target_class = req_class
            self.is_tracking_active = True
            self.last_seen_time = self.get_clock().now()
            self.latest_pose = None
            self.consecutive_hits = 0

        feedback_msg = FollowObject.Feedback()
        result_msg = FollowObject.Result()

        rate = self.create_rate(10)
        timeout_duration = Duration(seconds=2.0)

        last_valid_pose = None

        #Main Controller Loop 
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Mission cancelled by supervisor.")
                goal_handle.cancled()

                with self.state_lock:
                    self.is_tracking_active = False #put the cb 1 back to sleep 
                
                result_msg.target_lost = False 
                result_msg.message = "Cancelled successfully."

                # This inline if/else statement says: "If we have a saved coordinate, send it.
                #  But if last_valid_pose is still None, create a brand new, empty, default PoseStamped() message 
                # so ROS 2 doesn't crash when sending the result."
                result_msg.final_pose = last_valid_pose if last_valid_pose else PoseStamped()
                return result_msg

            with self.state_lock:
                current_pose = self.latest_pose
                last_seen = self.last_seen_time
                hits = self.consecutive_hits 

            if current_pose is not None:
                last_valid_pose = current_pose
            
            time_since_last_seen = self.get_clock().now() - last_seen
            
            if time_since_last_seen > timeout_duration:
                self.get_logger().warn(f"Lost sight of target for > 2s. Aborting.")
                goal_handle.abort()

                with self.state_lock:
                    self.is_tracking_active = False # Put Chef 1 back to sleep

                result_msg.target_lost = True
                result_msg.message = "Target tracking timeout."
                # We hand the Boss the exact last place we saw the person!
                result_msg.final_pose = last_valid_pose if last_valid_pose else PoseStamped()
                return result_msg
            
            if current_pose is not None:
                feedback_msg.current_pose = current_pose
                feedback_msg.consecutive_hits = hits
                # We consider it "locked on" if we've seen it for 3 frames in a row
                feedback_msg.locked_on = (hits >= 3)
                
                goal_handle.publish_feedback(feedback_msg)

            rate.sleep()

def main(args=None):
   
    rclpy.init(args=args)    
    node = FollowObjectActionNode()
    
    # =========================================================
    # 3. THE MULTI-THREADED ENGINE
    # =========================================================
    # By default, ROS 2 uses a SingleThreadedExecutor (One Chef).
    # Because we created two MutuallyExclusiveCallbackGroups in our __init__,
    # we MUST use the MultiThreadedExecutor here so ROS 2 knows it is 
    # allowed to spin up multiple CPU threads on your Jetson Orin.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info("Starting MultiThreadedExecutor...")
        # spin() keeps the node alive forever, listening for incoming data
        executor.spin()
    except KeyboardInterrupt:
        # Gracefully handle when you press Ctrl+C in the terminal
        node.get_logger().info("Ctrl+C pressed. Shutting down...")
    finally:
        # Clean up the node and shut down ROS 2
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()






