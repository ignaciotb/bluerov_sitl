import os
from glob import glob

from setuptools import find_packages, setup

package_name = "bluerov_sitl"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description=(
        "SITL stand-in for a BlueROV2 Heavy flight controller: ArduSub "
        "MANUAL and STABILIZE modes driving the real AP_Motors6DOF "
        "thruster mixing matrix, from RC override to per-thruster PWM."
    ),
    license="GPL-3.0-or-later",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sitl_node = bluerov_sitl.sitl_node:main",
        ],
    },
)
