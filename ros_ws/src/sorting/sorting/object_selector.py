import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sorting_msgs.msg import Object
from sorting_msgs.action import SelectObject


class ObjectSelector(Node):
    """
    A node for selecting which object to sort next.
    """

    def __init__(self) -> None:
        super().__init__('object_selector')
        self._select_server: ActionServer = ActionServer(
            self, SelectObject, 'SelectObject', self._select_cb
        )

    def _select_cb(self, goal_handle) -> SelectObject.Result:
        raise NotImplementedError


def main(args=None):
    rclpy.init(args=args)
    selector = ObjectSelector()
    rclpy.spin(selector)
    manipulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
