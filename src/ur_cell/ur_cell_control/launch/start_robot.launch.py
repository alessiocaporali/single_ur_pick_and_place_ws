from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.actions import TimerAction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    controllers_file = LaunchConfiguration("controllers_file")
    launch_gripper_controller = LaunchConfiguration("launch_gripper_controller")
    gripper_spawn = LaunchConfiguration("gripper_spawn")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim_gripper = LaunchConfiguration("use_sim_gripper")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tty_port = LaunchConfiguration("tty_port")
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="172.25.0.2",  # put your robot's IP address here
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_cell_control"),
                    "config",
                    "ros2_controllers.yaml",
                ]
            ),
            description="YAML file with controller_manager configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_gripper_controller",
            default_value="true",
            description="Spawn gripper_action_controller for Hand-E action interface.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_spawn",
            default_value=launch_gripper_controller,
            description="Alias for launch_gripper_controller. Set false to start robot without gripper controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_gripper",
            default_value="false",
            description="Launch gripper with simulated hardware.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use mock hardware for UR driver (disables controller_stopper).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock sensor commands when use_mock_hardware is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="true",
            description="Enable headless mode.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client",
            default_value="false",
            description="Disable dashboard client in mock mode.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Enable UR tool communication bridge.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="Pseudo-tty created by UR tool communication node.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="TCP port used by UR tool communication bridge.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tty_port",
            default_value="/tmp/ttyUR",
            description="Serial port for real gripper RTU communication.",
        )
    )


    return LaunchDescription(
        declared_arguments
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ur_robot_driver"),
                                "launch",
                                "ur_control.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "tf_prefix": [LaunchConfiguration("ur_type"), "_"],
                    "rviz_config_file": PathJoinSubstitution(
                        [
                            FindPackageShare("ur_cell_description"),
                            "rviz",
                            "urdf.rviz",
                        ]
                    ),
                    "description_launchfile": PathJoinSubstitution(
                        [
                            FindPackageShare("ur_cell_control"),
                            "launch",
                            "rsp.launch.py",
                        ]
                    ),
                    "controllers_file": controllers_file,
                    "use_mock_hardware": use_mock_hardware,
                    "use_mock_gripper_hardware": use_sim_gripper,
                    "mock_sensor_commands": mock_sensor_commands,
                    "headless_mode": headless_mode,
                    "launch_dashboard_client": launch_dashboard_client,
                    "use_tool_communication": use_tool_communication,
                    "tool_device_name": tool_device_name,
                    "tool_tcp_port": tool_tcp_port,
                    "tty_port": tty_port,
                }.items(),
            ),
            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare("ur_cell_control"),
                                        "launch",
                                        "gripper_controller.launch.py",
                                    ]
                                )
                            ]
                        ),
                        launch_arguments={
                            "enabled": gripper_spawn,
                            "controller_name": "gripper_action_controller",
                            "controller_manager": "/controller_manager",
                            "controller_spawner_timeout": "30",
                        }.items(),
                    ),
                ],
            ),
        ]
    )
