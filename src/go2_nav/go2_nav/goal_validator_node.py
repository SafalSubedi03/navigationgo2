import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped

from action_msgs.msg import GoalInfo, GoalStatus

class check_goal_pose(Node):
    def __init__(self):
        super().__init__('check_goal_pose')

        #Path planner nav2 server client
        self.planner_client = ActionClient(self,ComputePathToPose,'compute_path_to_pose')

        self.raw_goal_pose = self.create_subscription(PoseStamped,'/folow_object/goal_pose_raw',self.validity_check,10)
        self.valid_goal_pose = self.create_publisher(PoseStamped,'/goal_pose',10)
        
        self.validity_check_flag = 0 # set to 1 if any validation process is currently running

        #current goal postion 
        self.x = 0
        self.y = 0
        self.z = 0

        #per-request handles, set when a check starts, cleared when it ends
        self.current_goal_handle = None
        self.timeout_timer = None
        self.get_logger().info('Running Pose Validation')

    def validity_check(self, msg : PoseStamped):
        self.get_logger().info('Running Pose Validation..')

        #skip cb if there is a validity check going on. 
        if self.validity_check_flag:
            self.get_logger().warn("Another pose validation is currently running")
            return
        
        self.validity_check_flag = 1 #Since a validation check is currently being processed
        self.x = msg.pose.position.x 
        self.y = msg.pose.position.y 
        self.z = msg.pose.position.z 

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()

        goal_msg.goal.pose.position.x = self.x
        goal_msg.goal.pose.position.y = self.y 
        # goal_msg.goal.pose.position.z = self.z 

        goal_msg.planner_id = 'GridBased'

        if not self.planner_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('[VN] ComputePathToPose action server is not available')
            self.validity_check_flag = 0
            return

        #start a fresh one-shot timeout for THIS request
        self.current_goal_handle = None
        self.timeout_timer = self.create_timer(4.0, self.timeout_callback)

        self._send_goal_future = self.planner_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def timeout_callback(self):
        self.get_logger().warn('[VN] Validation request timed out')

        #cancel the in-flight goal if we have a handle for it
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()

        self._cleanup_request()

    def goal_response_callback(self,future):
        goal_handle = future.result()

        #store the handle so the timeout callback can cancel it if needed
        self.current_goal_handle = goal_handle

        if not goal_handle.accepted:
            self.get_logger().info('[VN] Goal rejected by planner server')
            self._cleanup_request()
            return

        self.get_logger().info('[VN] Goal accepted, waiting for path')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        #request already timed out and was cleaned up; ignore late result
        if self.timeout_timer is None:
            return

        wrapped_result = future.result()
        status = wrapped_result.status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'[VN] Planning action failed with status code: {status}')
            self._cleanup_request()
            return 

        result = wrapped_result.result
        path = result.path

        if len(path.poses) == 0:
            self.get_logger().warn(f'[VN] Could not Generate a path. Error Code = {result.error_code}')
            #point is unreacble in future add a mechanism to search available nearest path.
            self._cleanup_request()
            return

        self.get_logger().info('[VN] Valid pose was created.')

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.position.x = self.x
        goal_msg.pose.position.y = self.y
        goal_msg.pose.position.z = self.z

        goal_msg.pose.orientation.w = 1.0

        self.valid_goal_pose.publish(goal_msg)
        self._cleanup_request()

    def _cleanup_request(self):
        #cancel/destroy the per-request timeout timer and clear handle state
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.destroy_timer(self.timeout_timer)
            self.timeout_timer = None

        self.current_goal_handle = None
        self.validity_check_flag = 0

def main(args=None):
    rclpy.init(args=args)
    node = check_goal_pose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()