#!/usr/bin/env python3
import argparse
import sys

import rclpy


def get_args():
    """Get command line arguments."""
    parser = argparse.ArgumentParser(description="Jackal spawner")
    parser.add_argument(
        'urdf', type=argparse.FileType(), help='The urdf of the robot to spawn.'
    )
    return parser.parse_args(sys.argv[1:])


def main(args=None):
    rclpy.init(args=args)
    urdf: str = get_args()
    node = rclpy.create_node('test_node')
    node.get_logger().info(f'{urdf}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
