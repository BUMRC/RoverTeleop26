#!/usr/bin/env python3

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class GoalBridge(Node):
    def __init__(self) -> None:
        super().__init__("goal_bridge")

        self._goal_topic = self.declare_parameter("goal_topic", "/goal_pose").value
        self._nav_action = self.declare_parameter("nav_action", "/navigate_to_pose").value
        self._expected_frame = self.declare_parameter("expected_goal_frame", "map").value
        self._wait_server_sec = float(self.declare_parameter("wait_server_timeout_sec", 3.0).value)

        self._action = ActionClient(self, NavigateToPose, self._nav_action)
        self._sub = self.create_subscription(PoseStamped, self._goal_topic, self._on_goal_msg, 10)
        self._status_pub = self.create_publisher(String, "/goal_bridge/status", 10)

        self._goal_handle = None

        self.get_logger().info(
            f"Listening on '{self._goal_topic}' and forwarding to '{self._nav_action}'"
        )

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)

    def _on_goal_msg(self, pose: PoseStamped) -> None:
        frame_id = pose.header.frame_id
        if self._expected_frame and frame_id != self._expected_frame:
            self._publish_status(
                f"Rejected goal in frame '{frame_id}'. Expected '{self._expected_frame}'."
            )
            return

        if not self._action.wait_for_server(timeout_sec=self._wait_server_sec):
            self._publish_status("NavigateToPose action server not available.")
            return

        if self._goal_handle is not None:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _: self._send_goal(pose))
            return

        self._send_goal(pose)

    def _send_goal(self, pose: PoseStamped) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._publish_status(
            "Sending goal x=%.2f y=%.2f yaw(z,w)=%.3f,%.3f"
            % (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
        )

        send_future = self._action.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._goal_handle = None
            self._publish_status("Goal rejected by Nav2.")
            return

        self._goal_handle = goal_handle
        self._publish_status("Goal accepted by Nav2.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self._publish_status(
            "Feedback: distance_remaining=%.2f m" % feedback.distance_remaining
        )

    def _on_result(self, future) -> None:
        self._goal_handle = None
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_status("Goal succeeded.")
            return

        if status == GoalStatus.STATUS_ABORTED:
            self._publish_status("Goal aborted.")
            return

        if status == GoalStatus.STATUS_CANCELED:
            self._publish_status("Goal canceled.")
            return

        self._publish_status(f"Goal finished with status code {status}.")


def main() -> None:
    rclpy.init()
    node = GoalBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
