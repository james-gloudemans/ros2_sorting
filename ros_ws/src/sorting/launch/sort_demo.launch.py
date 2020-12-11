from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package='sorting', executable='sorter'),
            Node(package='sorting', executable='object_selector'),
            Node(package='sorting', executable='picker'),
            Node(package='sorting', executable='placer'),
        ]
    )
