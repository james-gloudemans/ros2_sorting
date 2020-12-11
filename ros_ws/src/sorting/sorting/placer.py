import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sorting_msgs.action import PlaceObject


class Placer(Node):
    """
    A node for picking up an object.
    """

    def __init__(self) -> None:
        super().__init__('placer')
        self._place_server: ActionServer = ActionServer(
            self, PlaceObject, 'PlaceObject', self._place_cb
        )

    def _place_cb(self, goal_handle) -> PlaceObject.Result:
        raise NotImplementedError


def main(args=None):
    rclpy.init(args=args)
    placer = Placer()
    rclpy.spin(placer)
    manipulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
