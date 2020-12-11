import rclpy
from rclpy.node import Node


class ObjectSelector(Node):
    """
    A node for selecting which object to sort next.
    """

    def __init__(self) -> None:
        super().__init__('object_selector')


def main(args=None):
    rclpy.init(args=args)
    selector = Selector()
    rclpy.spin(selector)
    manipulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
