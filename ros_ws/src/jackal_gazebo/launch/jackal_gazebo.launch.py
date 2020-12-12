import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'jackal_gazebo'
    sdf_file = os.path.join(
        get_package_share_directory(package_name), 'models', 'jackal', 'model.sdf'
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world_file_name = "sorting.world"
    world = os.path.join(
        get_package_share_directory(package_name), "worlds", world_file_name
    )
    launch_file_dir = os.path.join(get_package_share_directory(package_name), "launch")

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "gzserver",
                    "--verbose",
                    world,
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ros2", "param", "set", "/gazebo", "use_sim_time", use_sim_time],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [launch_file_dir, "/robot_state_publisher.launch.py"]
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            ExecuteProcess(cmd=['ros2', 'run', package_name, 'spawn_jackal', sdf_file]),
        ]
    )

