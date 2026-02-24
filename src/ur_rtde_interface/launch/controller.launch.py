#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="172.25.0.2",
        description="Robot IP address"
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="left",
        description="Namespace for all robot nodes"
    )

    # LaunchConfigurations
    robot_ip = LaunchConfiguration("robot_ip")
    namespace = LaunchConfiguration("namespace")

    # Robot controller node under same namespace
    robot_controller_node = Node(
        package="ur_rtde_interface",
        executable="robot_controller",
        name="robot_controller",
        output="screen",
        parameters=[
            {"robot_ip": robot_ip},
            {"gripper_active": False},
            {"namespace": namespace}
        ],
    )


    return LaunchDescription([
        robot_ip_arg,
        namespace_arg,
        robot_controller_node,
    ])
