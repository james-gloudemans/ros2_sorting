from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bt_file = '/opt/ros/foxy/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml'
    with open(bt_file, 'r') as f:
        behavior_tree = f.read()
    return LaunchDescription(
        [
            Node(
                package='sorting',
                executable='sorter',
                parameters=[{'behavior_tree': behavior_tree}],
            ),
            Node(package='sorting', executable='object_selector'),
            Node(package='sorting', executable='picker'),
            Node(package='sorting', executable='placer'),
        ]
    )
