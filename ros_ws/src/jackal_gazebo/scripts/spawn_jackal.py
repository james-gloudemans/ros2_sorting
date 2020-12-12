#!/usr/bin/env python3
# WARNING: colcon build --symlink-instaall does not install this with a symlink!
import argparse
import sys

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Point, Pose, Quaternion


def get_sdf() -> str:
    """Get sdf model file from command line arg."""
    parser = argparse.ArgumentParser(description="Jackal spawner")
    parser.add_argument(
        'sdf', type=argparse.FileType(), help='The sdf of the robot to spawn.'
    )
    return parser.parse_args(sys.argv[1:]).sdf.read()


def main(args=None):
    rclpy.init(args=args)
    sdf: str = get_sdf()
    node = rclpy.create_node('robot_spawner')
    initial_pose = Pose(position=Point(x=0.0, y=8.0, z=0.0))
    request = SpawnEntity.Request(
        name='jackal', xml=sdf, robot_namespace='', initial_pose=initial_pose
    )
    spawn_client = node.create_client(SpawnEntity, 'spawn_entity')
    spawn_client.wait_for_service()
    node.get_logger().info(f'Spawning robot at {initial_pose}.')
    future = spawn_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is None:
        raise RuntimeError(f'Exception while spawning robot: {future.exception()}.')
    node.get_logger().info('Finished spawning robot.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
