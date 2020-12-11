import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    param_file = LaunchConfiguration(
        "params",
        default=os.path.join(
            get_package_share_directory("jackal_navigation2"),
            "param",
            "jackal_nav2.yaml",
        ),
    )

    rviz_config = os.path.join(
        get_package_share_directory("jackal_navigation2"), "rviz", "sorting.rviz",
    )

    default_bt_file = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_w_replanning_and_recovery.xml',
    )

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'recoveries_server',
        'bt_navigator',
    ]
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    slam_launch_dir = os.path.join(
        get_package_share_directory("slam_toolbox"), "launch"
    )

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local,
    }

    configured_params = RewrittenYaml(
        source_file=param_file, param_rewrites=param_substitutions, convert_types=True
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value=param_file,
                description="Full path to param file to load",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                'default_bt_xml_filename',
                default_value=default_bt_file,
                description='Full path to the behavior tree xml file to use',
            ),
            DeclareLaunchArgument(
                'autostart',
                default_value='true',
                description='Automatically startup the nav2 stack',
            ),
            DeclareLaunchArgument(
                'map_subscribe_transient_local',
                default_value='false',
                description='Whether to set the map subscriber QoS to transient local',
            ),
            # Nodes
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                output="own_log",
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                name='recoveries_server',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                ],
            ),
            Node(
                parameters=[
                    get_package_share_directory("slam_toolbox")
                    + '/config/mapper_params_online_async.yaml',
                    {'use_sim_time': use_sim_time},
                ],
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
            ),
        ]
    )
