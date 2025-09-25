import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import Dict, Optional


class AlertManagerNode(Node):
    """Node that manages and logs robot health alerts with throttling."""

    def __init__(self) -> None:
        super().__init__('alert_manager_node')

        # ---------------- Communication ----------------
        # Subscriber to /health_status
        self.subscriber_ = self.create_subscription(
            String, 'health_status', self.callback_health_status, 10
        )

        # ---------------- Internal state ----------------
        self.current_health_status_: Optional[str] = None

        # Track last log times per status type (in seconds, using ROS time)
        self.last_log_time_: Dict[str, float] = {
            "HEALTHY": 0.0,
            "WARNING": 0.0,
            "CRITICAL": 0.0,
            "UNKNOWN": 0.0,
        }

        # Frequency limit (seconds)
        self.alert_interval_sec_: float = 10.0

    def callback_health_status(self, msg: String) -> None:
        """Handle incoming health status messages with throttled logging."""
        try:
            msg_type = msg.data.split(':')[0].strip()
            now = self.get_clock().now().nanoseconds / 1e9

            # Log immediately if status changed
            if msg.data != self.current_health_status_:
                self.log_by_type(msg_type, msg.data)
                self.current_health_status_ = msg.data
                self.last_log_time_[msg_type] = now
                return

            # If status is the same, log only once per alert_interval_sec_
            if now - self.last_log_time_.get(msg_type, 0.0) >= self.alert_interval_sec_:
                self.log_by_type(msg_type, msg.data)
                self.last_log_time_[msg_type] = now

        except Exception as e:
            self.get_logger().error(f"Error in callback_health_status: {e}")

    def log_by_type(self, msg_type: str, message: str) -> None:
        """Log a message with the appropriate severity based on type."""
        try:
            if msg_type == 'HEALTHY':
                self.get_logger().info(message)
            elif msg_type == 'WARNING':
                self.get_logger().warn(message)
            elif msg_type == 'CRITICAL':
                # ROS logger includes timestamp automatically
                self.get_logger().error(message)
                # Extra emphasis for critical alerts
                self.get_logger().error(f"CRITICAL ALERT at node time: {message}")
            elif msg_type == 'UNKNOWN':
                self.get_logger().warn(f"Unexpected health state: {message}")
            else:
                self.get_logger().warn(f"Unrecognized message type: {message}")
        except Exception as e:
            self.get_logger().error(f"Error in log_by_type: {e}")


def main(args=None) -> None:
    """Main entry point for the Alert Manager Node."""
    rclpy.init(args=args)
    node = AlertManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AlertManagerNode (KeyboardInterrupt)")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
