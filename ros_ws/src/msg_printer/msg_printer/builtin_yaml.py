import yaml

import rclpy
import builtin_interfaces.msg as builtin


class YamlTime(yaml.YAMLObject):
    yaml_tag = u"!Time"

    def __init__(self, t: builtin.Time):
        self._dict = {"sec": t.sec, "nanosec": t.nanosec}

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlDuration(yaml.YAMLObject):
    yaml_tag = u"!Duration"

    def __init__(self, t: builtin.Duration):
        self._dict = {"sec": t.sec, "nanosec": t.nanosec}

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


def main(args=None):
    test_msg = builtin.Duration(sec=1, nanosec=100)
    rclpy.init(args=args)
    node = rclpy.create_node("std_msg")
    pub = node.create_publisher(builtin.Duration, "std_msg", 10)
    pub.publish(test_msg)
    node.get_logger().info(yaml.dump(YamlDuration(test_msg)))


if __name__ == "__main__":
    main()
