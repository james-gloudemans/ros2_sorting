import rclpy
from rclpy.node import Node


class Picker(Node):
    """
    A node for picking up an object.
    """

    def __init__(self) -> None:
        super().__init__('picker')


def main(args=None):
    rclpy.init(args=args)
    picker = Picker()
    rclpy.spin(picker)
    manipulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
