import yaml

import rclpy
import std_msgs.msg as std
import geometry_msgs.msg as geom

from msg_printer.std_yaml import YamlHeader


class YamlAccel(yaml.YAMLObject):
    yaml_tag = u"!Accel"

    def __init__(self, a: geom.Accel):
        self._dict = {
            "linear": YamlVector3(a.linear).dict,
            "angular": YamlVector3(a.angular).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlAccelStamped(yaml.YAMLObject):
    yaml_tag = u"!AccelStamped"

    def __init__(self, a: geom.AccelStamped):
        self._dict = {
            "header": YamlHeader(a.header).dict,
            "accel": YamlAccel(a.accel).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlAccelWithCovariance(yaml.YAMLObject):
    yaml_tag = u"!AccelWithCovariance"

    def __init__(self, a: geom.AccelWithCovariance):
        self._dict = {
            "accel": YamlAccel(a.accel).dict,
            "covariance": [float(elem) for elem in a.covariance],
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlAccelWithCovarianceStamped(yaml.YAMLObject):
    yaml_tag = u"!AccelWithCovarianceStamped"

    def __init__(self, a: geom.AccelWithCovarianceStamped):
        self._dict = {
            "header": YamlHeader(a.header).dict,
            "accel": YamlAccelWithCovariance(a.accel).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlInertia(yaml.YAMLObject):
    yaml_tag = u"!Inertia"

    def __init__(self, i: geom.Inertia):
        self._dict = {
            "m": i.m,
            "com": YamlVector3(i.com).dict,
            "ixx": i.ixx,
            "ixy": i.ixy,
            "ixz": i.ixz,
            "iyy": i.iyy,
            "iyz": i.iyz,
            "izz": i.izz,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlInertiaStamped(yaml.YAMLObject):
    yaml_tag = u"!InertiaStamped"

    def __init__(self, i: geom.InertiaStamped):
        self._dict = {
            "header": YamlHeader(i.header).dict,
            "inertia": YamlInertia(i.inertia).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlPoint(yaml.YAMLObject):
    yaml_tag = u"!Point"

    def __init__(self, p: geom.Point):
        self._dict = {"x": p.x, "y": p.y, "z": p.z}

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlPointStamped(yaml.YAMLObject):
    yaml_tag = u"!PointStamped"

    def __init__(self, p: geom.PointStamped):
        self._dict = {
            "header": YamlHeader(p.header).dict,
            "point": YamlPoint(p.point).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlPoint32(yaml.YAMLObject):
    yaml_tag = u"!Point32"

    def __init__(self, p: geom.Point32):
        self._dict = {"x": p.x, "y": p.y, "z": p.z}

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlPolygon(yaml.YAMLObject):
    yaml_tag = u"!Polygon"

    def __init__(self, p: geom.Polygon):
        self._dict = {"points": [YamlPoint32(point).dict for point in p.points]}

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlPolygonStamped(yaml.YAMLObject):
    yaml_tag = u"!PolygonStamped"

    def __init__(self, p: geom.PolygonStamped):
        self._dict = {
            "header": YamlHeader(p.header).dict,
            "polygon": YamlPolygon(p.polygon).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlPose(yaml.YAMLObject):
    yaml_tag = u"!Pose"

    def __init__(self, p: geom.Pose):
        self._dict = {
            "position": YamlPoint(p.position).dict,
            "orientation": YamlQuaternion(p.orientation).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlPose2D(yaml.YAMLObject):
    yaml_tag = u"!Pose2D"

    def __init__(self, p: geom.Pose2D):
        self._dict = {"x": p.x, "y": p.y, "theta": p.theta}

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlPoseArray(yaml.YAMLObject):
    yaml_tag = u"!PoseArray"

    def __init__(self, p: geom.PoseArray):
        self._dict = {
            "header": YamlHeader(p.header).dict,
            "poses": [YamlPose(pose).dict for pose in p.poses],
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlPoseStamped(yaml.YAMLObject):
    yaml_tag = u"!PoseStamped"

    def __init__(self, p: geom.PoseStamped):
        self._dict = {
            "header": YamlHeader(p.header).dict,
            "pose": YamlPose(p.pose).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlPoseWithCovariance(yaml.YAMLObject):
    yaml_tag = u"!PoseWithCovariance"

    def __init__(self, p: geom.PoseWithCovariance):
        self._dict = {
            "pose": YamlPose(p.pose).dict,
            "covariance": [float(elem) for elem in p.covariance],
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlPoseWithCovarianceStamped(yaml.YAMLObject):
    yaml_tag = u"!PoseWithCovarianceStamped"

    def __init__(self, p: geom.PoseWithCovarianceStamped):
        self._dict = {
            "header": YamlHeader(p.header).dict,
            "pose": YamlPoseWithCovariance(p.pose).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlQuaternion(yaml.YAMLObject):
    yaml_tag = u"!Quaternion"

    def __init__(self, q: geom.Quaternion):
        self._dict = {"x": q.x, "y": q.y, "z": q.z, "w": q.w}

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlQuaternionStamped(yaml.YAMLObject):
    yaml_tag = u"!QuaternionStamped"

    def __init__(self, q: geom.QuaternionStamped):
        self._dict = {
            "header": YamlHeader(q.header).dict,
            "quaternion": YamlQuaternion(q.quaternion).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlTransform(yaml.YAMLObject):
    yaml_tag = u"!Transform"

    def __init__(self, t: geom.Transform):
        self._dict = {
            "translation": YamlVector3(t.translation).dict,
            "rotation": YamlQuaternion(t.rotation).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlTransformStamped(yaml.YAMLObject):
    yaml_tag = u"!TransformStamped"

    def __init__(self, t: geom.TransformStamped):
        self._dict = {
            "header": YamlHeader(t.header).dict,
            "child_frame_id": t.child_frame_id,
            "transform": YamlTransform(t.transform).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlTwist(yaml.YAMLObject):
    yaml_tag = u"!Twist"

    def __init__(self, t: geom.Twist):
        self._dict = {
            "linear": YamlVector3(t.linear).dict,
            "angular": YamlVector3(t.angular).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlTwistStamped(yaml.YAMLObject):
    yaml_tag = u"!TwistStamped"

    def __init__(self, t: geom.TwistStamped):
        self._dict = {
            "header": YamlHeader(t.header).dict,
            "twist": YamlTwist(t.twist).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlTwistWithCovariance(yaml.YAMLObject):
    yaml_tag = u"!TwistWithCovariace"

    def __init__(self, t: geom.TwistWithCovariance):
        self._dict = {
            "twist": YamlTwist(t.twist).dict,
            "covariance": [float(elem) for elem in t.covariance],
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlTwistWithCovarianceStamped(yaml.YAMLObject):
    yaml_tag = u"!TwistWithCovarianceStamped"

    def __init__(self, t: geom.TwistWithCovarianceStamped):
        self._dict = {
            "header": YamlHeader(t.header).dict,
            "twist": YamlTwistWithCovariance(t.twist).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlVector3(yaml.YAMLObject):
    yaml_tag = u"!Vector3"

    def __init__(self, v: geom.Vector3):
        self._dict = {"x": v.x, "y": v.y, "z": v.z}

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlVector3Stamped(yaml.YAMLObject):
    yaml_tag = u"!Vector3Stamped"

    def __init__(self, v: geom.Vector3Stamped):
        self._dict = {
            "header": YamlHeader(v.header).dict,
            "vector": YamlVector3(v.vector).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlWrench(yaml.YAMLObject):
    yaml_tag = u"!Wrench"

    def __init__(self, w: geom.Wrench):
        self._dict = {
            "force": YamlVector3(w.force).dict,
            "torque": YamlVector3(w.torque).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


class YamlWrenchStamped(yaml.YAMLObject):
    yaml_tag = u"!WrenchStamped"

    def __init__(self, w: geom.WrenchStamped):
        self._dict = {
            "header": YamlHeader(w.header).dict,
            "wrench": YamlWrench(w.wrench).dict,
        }

    @property
    def dict(self):
        return self._dict

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_mapping(cls.yaml_tag, data.dict)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("test")
    test_msg = geom.WrenchStamped()
    node.get_logger().info(yaml.dump(YamlWrenchStamped(test_msg), sort_keys=False))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
