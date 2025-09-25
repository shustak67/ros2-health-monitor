import rclpy
from rclpy.node import Node
from my_interfaces.msg import RobotStatus
import random
from typing import Optional


class CanRos2StatusNode(Node):
    """Node that simulates CAN data and publishes RobotStatus messages."""

    def __init__(self) -> None:
        super().__init__("can_ros2_status_node")

        # ---------------- Communication ----------------
        self.publisher_ = self.create_publisher(RobotStatus, "robot_status", 10)

        # Timer for periodic publishing (1 Hz)
        self.timer_ = self.create_timer(1.0, self.publish_status)

        # ---------------- Internal state ----------------
        self.last_status_: Optional[RobotStatus] = None

    def publish_status(self) -> None:
        """Publish a simulated RobotStatus message."""
        try:
            msg = RobotStatus()
            msg.left_rpm = random.randint(0, 3000)
            msg.right_rpm = random.randint(0, 3000)
            msg.battery_percentage = random.uniform(0.0, 100.0)
            msg.bit_errors = random.choice([True, False])

            self.publisher_.publish(msg)
            self.last_status_ = msg

            self.get_logger().info(
                f"Publishing RobotStatus: "
                f"L={msg.left_rpm}, R={msg.right_rpm}, "
                f"Battery={msg.battery_percentage:.2f}%, BIT={msg.bit_errors}"
            )

        except Exception as e:
            self.get_logger().error(f"Error publishing RobotStatus: {e}")


def main(args=None) -> None:
    """Main entry point for the CAN-to-ROS2 status simulator node."""
    rclpy.init(args=args)
    node = CanRos2StatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CanRos2StatusNode (KeyboardInterrupt)")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
