#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("odom_relay")

        self._input_topic = self.declare_parameter("input_topic", "/zed/zed_node/odom").value
        self._output_topic = self.declare_parameter("output_topic", "/zed_odom").value

        self._frame_id = self.declare_parameter("frame_id", "odom").value
        self._child_frame_id = self.declare_parameter("child_frame_id", "base_link").value
        self._publish_tf = bool(self.declare_parameter("publish_tf", True).value)

        self._pub = self.create_publisher(Odometry, self._output_topic, 20)
        self._sub = self.create_subscription(Odometry, self._input_topic, self._on_odom, 20)
        self._tf_pub = TransformBroadcaster(self)

        self.get_logger().info(
            "Relaying odometry from '%s' to '%s' (publish_tf=%s frame=%s child=%s)"
            % (
                self._input_topic,
                self._output_topic,
                str(self._publish_tf),
                self._frame_id,
                self._child_frame_id,
            )
        )

    def _on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        if self._frame_id:
            out.header.frame_id = self._frame_id
        if self._child_frame_id:
            out.child_frame_id = self._child_frame_id

        self._pub.publish(out)

        if self._publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header = out.header
            tf_msg.child_frame_id = out.child_frame_id
            tf_msg.transform.translation.x = out.pose.pose.position.x
            tf_msg.transform.translation.y = out.pose.pose.position.y
            tf_msg.transform.translation.z = out.pose.pose.position.z
            tf_msg.transform.rotation = out.pose.pose.orientation
            self._tf_pub.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = OdomRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
