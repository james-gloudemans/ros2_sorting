from setuptools import setup

package_name = "msg_printer"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="james",
    maintainer_email="james.gloudemans@gmail.com",
    description="Yaml encodings to print ROS messages with rclpy",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "geometry = msg_printer.geometry_yaml:main",
            "builtin = msg_printer.builtin_yaml:main",
            "std = msg_printer.std_yaml:main",
        ],
    },
)
