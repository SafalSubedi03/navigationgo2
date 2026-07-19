import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import the custom action interface
from go2_vision_msgs.action import FollowObject


class MissionSupervisor(Node):
    def __init__(self):
        super().__init__("mission_supervisor_node")

        self._action_client = ActionClient(self, FollowObject, 'follow_object')
        self.get_logger().info("Mission Supervisor booted up.")

        # Remembers which target to keep re-requesting across retries
        self.target_name = None

        # How long to wait before automatically retrying after a mission ends
        self.retry_delay_sec = 1.0

        # Handle to the current one-shot retry timer (so we can cancel it)
        self._retry_timer = None

        # Optional: stop retrying after too many consecutive failures.
        # Set to None to retry forever.
        self.max_consecutive_failures = None  # e.g. set to 10 to give up eventually
        self.consecutive_failures = 0

    def start_mission(self, target_name):
        self.target_name = target_name  # remembered so retries target the same class

        self.get_logger().info("Waiting for Action Server to connect...")
        self._action_client.wait_for_server()
        self.get_logger().info("Connection established! Preparing goal...")

    
        goal_msg = FollowObject.Goal()
        goal_msg.target_class = target_name

        self.get_logger().info(f"Sending goal to follow: '{target_name}'")

        # send_goal_async returns immediately! It gives us a "pager" (a Future)
        # that will trigger a callback when the server replies.
        # We also hand it a dedicated function to process the continuous Feedback.
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        # We attach a sticky note to the "pager": "When the server accepts or
        # rejects the goal, run the 'goal_response_callback' function."
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Mission rejected by the Action Server! Is it busy?")
            self._on_mission_failed()
            return

        self.get_logger().info("Mission accepted! The robot is actively tracking.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        status = "LOCKED" if feedback.locked_on else "SEARCHING"
        hits = feedback.consecutive_hits

        x = feedback.current_pose.pose.position.x
        y = feedback.current_pose.pose.position.y
        self.get_logger().info(f"[{status}] Hits: {hits} | Target Location: X={x:.2f}, Y={y:.2f}")

    def get_result_callback(self, future):
        final_result = future.result().result

        if final_result.target_lost:
            self.get_logger().warn(f"MISSION FAILED: {final_result.message}")
            self.get_logger().info(
                f"Last known position to search: "
                f"X={final_result.final_pose.pose.position.x:.2f}, "
                f"Y={final_result.final_pose.pose.position.y:.2f}"
            )
            self._on_mission_failed()
        else:
            self.get_logger().info(f"MISSION COMPLETED: {final_result.message}")
            self._on_mission_succeeded()

    def _on_mission_succeeded(self):
        self.consecutive_failures = 0
        self._schedule_retry()

    def _on_mission_failed(self):
        self.consecutive_failures += 1

        if (self.max_consecutive_failures is not None
                and self.consecutive_failures >= self.max_consecutive_failures):
            self.get_logger().error(
                f"Giving up after {self.consecutive_failures} consecutive failures."
            )
            return

        self._schedule_retry()

    def _schedule_retry(self):
        self.get_logger().info(f"Retrying in {self.retry_delay_sec}s...")
        # create_timer fires repeatedly by default; we cancel it inside the
        # callback itself so it effectively behaves as a one-shot timer.
        self._retry_timer = self.create_timer(self.retry_delay_sec, self._retry_once)

    def _retry_once(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None
        self.start_mission(self.target_name)


def main(args=None):
    rclpy.init(args=args)
    boss_node = MissionSupervisor()
    boss_node.start_mission("person")
    
    try:
        rclpy.spin(boss_node)
    except KeyboardInterrupt:
        boss_node.get_logger().info("Mission Supervisor shutting down (Ctrl+C)...")
    finally:
        boss_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()