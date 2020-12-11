import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sorting_msgs.msg import Object
from sorting_msgs.action import PickObject


class Picker(Node):
    """
    A node for picking up an object.
    """

    def __init__(self) -> None:
        super().__init__('picker')
        self._pick_server: ActionServer = ActionServer(
            self, PickObject, 'PickObject', self._pick_cb
        )

    def _pick_cb(self, goal_handle) -> PickObject.Result:
        raise NotImplementedError


def main(args=None):
    rclpy.init(args=args)
    picker = Picker()
    rclpy.spin(picker)
    manipulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
