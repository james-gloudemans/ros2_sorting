import asyncio
from typing import Tuple

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from msg_printer.geometry_yaml import YamlPoint
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sorting_msgs.msg import Object
from sorting_msgs.action import PickObject, PlaceObject, SelectObject


class Sorter(Node):
    """
    A node for sorting objects.
    """

    def __init__(self) -> None:
        super().__init__('sorter')

        # Initialize clients and wait for servers
        self._nav_client: ActionClient = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self._picker_client: ActionClient = ActionClient(self, PickObject, 'PickObject')
        self._placer_client: ActionClient = ActionClient(
            self, PlaceObject, 'PlaceObject'
        )
        self._selector_client: ActionClient = ActionClient(
            self, SelectObject, 'SelectObject'
        )
        self._picker_client.wait_for_server()
        self._placer_client.wait_for_server()
        self._selector_client.wait_for_server()
        self._nav_client.wait_for_server()
        self.get_logger().info('Sorter ready.')

        # Used to check goal status against e.g. self.goal_result.index('succeeded')
        self.goal_result: Tuple[str, ...] = (
            "unknown",
            "accepted",
            "executing",
            "canceling",
            "succeeded",
            "canceled",
            "aborted",
        )  # There should be some way to translate goal status codes to english in rclpy

        self.declare_parameter('behavior_tree')
        self.bt: str = self.get_parameter(
            'behavior_tree'
        ).get_parameter_value().string_value

    async def sort(self) -> None:
        self.get_logger().info(f'{self.bt}')
        await asyncio.sleep(10)
        self.get_logger().info('Sorting complete.')
        self._nav_client.destroy()
        self._picker_client.destroy()
        self._placer_client.destroy()
        self._selector_client.destroy()


def main(args=None):
    rclpy.init(args=args)
    sorter = Sorter()
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(sorter.sort())
    finally:
        loop.close()
    sorter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
