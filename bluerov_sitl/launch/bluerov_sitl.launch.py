import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("bluerov_sitl")
    default_config = os.path.join(pkg_share, "config", "bluerov2_heavy.yaml")

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="YAML file with bluerov_sitl node parameters",
    )

    node = Node(
        package="bluerov_sitl",
        executable="sitl_node",
        name="bluerov_sitl",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([config_arg, node])
