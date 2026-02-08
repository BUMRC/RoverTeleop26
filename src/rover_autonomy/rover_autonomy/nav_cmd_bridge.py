#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class NavCmdBridge(Node):
    def __init__(self) -> None:
        super().__init__("nav_cmd_bridge")

        in_topic = self.declare_parameter("input_topic", "/cmd_vel").value
        out_topic = self.declare_parameter("output_topic", "/cmd_vel_nav").value

        self._pub = self.create_publisher(Twist, out_topic, 20)
        self._sub = self.create_subscription(Twist, in_topic, self._on_cmd, 20)

        self.get_logger().info(f"Bridging Nav2 cmd_vel: '{in_topic}' -> '{out_topic}'")

    def _on_cmd(self, msg: Twist) -> None:
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = NavCmdBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
