from setuptools import setup

package_name = "jackal_teleop"

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
    description="Scripts for controlling the Clearpath Jackal with a PS4 Controller",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["jackal_teleop = jackal_teleop.jackal_teleop:main"],
    },
)
