#!/usr/bin/env python3
import rclpy


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('test_node')
    node.get_logger().info('Ready!')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
