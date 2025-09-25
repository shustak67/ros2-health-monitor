from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    can_ros2_status = Node(
        package="health_monitor",
        executable= "can_ros2_status_node",
        name= "can_ros2_status",
        output= "screen"
    )

    health_analyzer = Node(
        package="health_monitor",
        executable= "health_analyzer_node",
        name= "health_analyzer",
        output= "screen"
    )

    alert_manager = Node(
        package="health_monitor",
        executable= "alert_manager_node",
        name= "alert_manager",
        output= "screen"     
    )  

    ld.add_action(health_analyzer)
    ld.add_action(alert_manager)
    ld.add_action(can_ros2_status)

    return ld