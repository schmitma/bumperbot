from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    trajectory_publisher = Node(
                package="bumperbot_utils",
                executable="trajectory_publisher",
                arguments=[],
            )

    return LaunchDescription([
        trajectory_publisher       
    ])