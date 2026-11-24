from glob import glob
import os
from setuptools import find_packages, setup

package = "t701_ht_ros2"
setup(
    name=package,
    version="2.0.0",
    packages=find_packages(exclude=("tests",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package]),
        ("share/" + package, ["package.xml"]),
        (os.path.join("share", package, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="T701-HT SDK maintainers",
    maintainer_email="maintainer@example.com",
    description="T701-HT ROS 2 Humble SDK with mock mode and collision checks",
    license="MIT",
    entry_points={"console_scripts": [
        "driver = t701_ht_ros2.driver_node:main",
        "demo = t701_ht_ros2.demo_node:main",
    ]},
)
