from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_manager = LaunchConfiguration("controller_manager")
    controller_name = LaunchConfiguration("controller_name")
    timeout = LaunchConfiguration("controller_spawner_timeout")
    enabled = LaunchConfiguration("enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_manager",
                default_value="/controller_manager",
                description="Controller manager service namespace.",
            ),
            DeclareLaunchArgument(
                "controller_name",
                default_value="gripper_action_controller",
                description="Controller name to spawn/activate.",
            ),
            DeclareLaunchArgument(
                "controller_spawner_timeout",
                default_value="30",
                description="Timeout in seconds for spawner service calls.",
            ),
            DeclareLaunchArgument(
                "enabled",
                default_value="true",
                description="Whether to spawn the gripper controller.",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller_name,
                    "--controller-manager",
                    controller_manager,
                    "--controller-manager-timeout",
                    timeout,
                ],
                condition=IfCondition(enabled),
            ),
        ]
    )
