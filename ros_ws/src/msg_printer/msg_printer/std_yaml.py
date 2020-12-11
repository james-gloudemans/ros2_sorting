import yaml

import rclpy
import std_msgs.msg as std

from msg_printer.builtin_yaml import YamlTime


class YamlHeader(yaml.YAMLObject):
    yaml_tag = u"Header"

    def __init__(self, h: std.Header):
        self._dict = {"stamp": YamlTime(h.stamp).dict, "frame_id": h.frame_id}

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict,)


def main(args=None):
    test_msg = std.Header(frame_id="A")
    rclpy.init(args=args)
    node = rclpy.create_node("std_msg")
    pub = node.create_publisher(std.Header, "std_msg", 10)
    pub.publish(test_msg)
    node.get_logger().info(yaml.dump(YamlHeader(test_msg), canonical=False))


if __name__ == "__main__":
    main()
