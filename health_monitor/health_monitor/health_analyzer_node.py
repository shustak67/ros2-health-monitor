import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_interfaces.msg import RobotStatus
from my_interfaces.srv import GetStatusHistory
from collections import deque
from typing import Optional


class HealthAnalyzerNode(Node):
    """Node that analyzes robot status and publishes health classification."""

    def __init__(self) -> None:
        super().__init__("health_analyzer_node")

        # Parameters 
        # Thresholds for battery percentage and motor RPMs
        self.declare_parameter("max_rpm_threshold", 2000)
        self.declare_parameter("battery_warning_level", 25.0)
        self.declare_parameter("battery_critical_level", 15.0)

        self.max_rpm_threshold_: int = self.get_parameter("max_rpm_threshold").value
        self.battery_warning_level_: float = self.get_parameter("battery_warning_level").value
        self.battery_critical_level_: float = self.get_parameter("battery_critical_level").value

        # Communication 
        # Publishes health status ("HEALTHY", "WARNING", "CRITICAL")
        self.health_publisher_ = self.create_publisher(String, "health_status", 10)

        # Subscribes to raw robot status messages (battery, RPM, bit errors)
        self.status_subscriber_ = self.create_subscription(
            RobotStatus, "robot_status", self.callback_robot_status, 10
        )

        # Timer: re-publishes the latest health status every 0.2s
        self.publish_timer_ = self.create_timer(0.2, self.publish_health_status)

        # Service: allows clients to query recent RobotStatus history
        self.history_service_ = self.create_service(
            GetStatusHistory, "get_status_history", self.handle_status_history
        )

        # Internal state 
        self.zero_rpm_start_time_: Optional[float] = None       # time when both RPMs first hit zero
        self.unresponsive_timeout_sec_: float = 5.0             # threshold to classify as "unresponsive"
        self.status_history_window_: deque = deque(maxlen=10)

        # Default message (before any RobotStatus is received)
        self.latest_health_msg_: String = String()
        self.latest_health_msg_.data = "No status received yet"

    def callback_robot_status(self, msg: RobotStatus) -> None:
        """Handle incoming RobotStatus messages."""
        try:
            self.status_history_window_.append(msg)
            self.latest_health_msg_ = self.health_logic(msg)
        except Exception as e:
            self.get_logger().error(f"Error processing RobotStatus: {e}")

    def health_logic(self, msg: RobotStatus) -> String:
        """Decide whether the robot is HEALTHY, WARNING, or CRITICAL."""
        health_msg = String()
        issues = []

        try:
            # Check if motors are unresponsive (stopped for too long)
            rpm_unresponsive = self.check_rpm(msg)

            # ---------------- CRITICAL ----------------
            if msg.battery_percentage <= self.battery_critical_level_ or rpm_unresponsive:
                if msg.battery_percentage <= self.battery_critical_level_:
                    issues.append(f"Battery critical: {msg.battery_percentage}%")
                if rpm_unresponsive:
                    issues.append("Motors unresponsive (>5s at 0 RPM)")
                health_msg.data = "CRITICAL: " + ", ".join(issues)
                return health_msg

            #  WARNING 
            battery_low_warning = (
                self.battery_critical_level_ < msg.battery_percentage < self.battery_warning_level_
            )
            rpm_high_warning = (
                msg.left_rpm > self.max_rpm_threshold_ or msg.right_rpm > self.max_rpm_threshold_
            )
            bit_error_warning = bool(msg.bit_errors)

            if battery_low_warning or rpm_high_warning or bit_error_warning:
                if battery_low_warning:
                    issues.append(f"Low battery: {msg.battery_percentage}%")
                if msg.right_rpm > self.max_rpm_threshold_:
                    issues.append(f"Right RPM too high: {msg.right_rpm}")
                if msg.left_rpm > self.max_rpm_threshold_:
                    issues.append(f"Left RPM too high: {msg.left_rpm}")
                if bit_error_warning:
                    issues.append("BIT error present")
                health_msg.data = "WARNING: " + ", ".join(issues)
                return health_msg

            # HEALTHY 
            if (
                msg.battery_percentage >= self.battery_warning_level_
                and msg.left_rpm <= self.max_rpm_threshold_
                and msg.right_rpm <= self.max_rpm_threshold_
                and not msg.bit_errors
            ):
                health_msg.data = (
                    f"HEALTHY: Battery: {msg.battery_percentage}%, "
                    f"Left RPM: {msg.left_rpm}, Right RPM: {msg.right_rpm}, BIT: {msg.bit_errors}"
                )
                return health_msg

            # Fallback 
            health_msg.data = "UNKNOWN: Unclassified state (check thresholds & inputs)"
            return health_msg

        except Exception as e:
            self.get_logger().error(f"Error in health_logic: {e}")
            health_msg.data = "ERROR: Exception during health check"
            return health_msg

    def check_rpm(self, msg: RobotStatus) -> bool:
        """Detect if both motors have been unresponsive (0 RPM) for too long."""
        try:
            now = self.get_clock().now().nanoseconds / 1e9
            motors_stopped = msg.left_rpm == 0 and msg.right_rpm == 0

            if motors_stopped:
                if self.zero_rpm_start_time_ is None:
                    self.zero_rpm_start_time_ = now
                stopped_duration = now - self.zero_rpm_start_time_
                return stopped_duration >= self.unresponsive_timeout_sec_
            else:
                self.zero_rpm_start_time_ = None
                return False
        except Exception as e:
            self.get_logger().error(f"Error in check_rpm: {e}")
            return False

    def publish_health_status(self) -> None:
        """Publish the latest health status message."""
        try:
            self.health_publisher_.publish(self.latest_health_msg_)
            self.get_logger().info(self.latest_health_msg_.data)
        except Exception as e:
            self.get_logger().error(f"Error publishing health status: {e}")

    def handle_status_history(self, request: GetStatusHistory.Request,
                              response: GetStatusHistory.Response) -> GetStatusHistory.Response:
        """Return the last N RobotStatus messages."""
        try:
            n = min(request.count, len(self.status_history_window_))
            response.statuses = list(self.status_history_window_)[-n:]
            self.get_logger().info(f"Returning last {n} status readings")
            return response
        except Exception as e:
            self.get_logger().error(f"Error handling status history service: {e}")
            return response


def main(args=None) -> None:
    """Main entry point for the Health Analyzer Node."""
    rclpy.init(args=args)
    node = HealthAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down HealthAnalyzerNode (KeyboardInterrupt)")
    finally:        
        if rclpy.ok():
            rclpy.shutdown()   
            node.destroy_node()  


if __name__ == "__main__":
    main()
