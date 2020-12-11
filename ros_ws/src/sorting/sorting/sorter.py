import asyncio

import rclpy
from rclpy.node import Node


class Sorter(Node):
    """
    A node for sorting objects.
    """

    def __init__(self) -> None:
        super().__init__('sorter')

    async def sort(self) -> None:
        await asyncio.sleep(100)


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
