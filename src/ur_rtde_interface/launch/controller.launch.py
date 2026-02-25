#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- Launch arguments ---
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="172.25.0.2",
        description="Robot IP address",
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="robot",
        description="Namespace for all robot nodes",
    )

    gripper_active_arg = DeclareLaunchArgument(
        "gripper_active",
        default_value="false",
        description="Enable gripper control over UR robot socket.",
    )

    gripper_mode_arg = DeclareLaunchArgument(
        "gripper_mode",
        default_value="sim",
        description="Gripper control mode: 'rtde' for direct RTDE commands, 'modbus' for USB control, 'sim' for simulation (no real gripper control).",
    )

    gripper_tty_port_arg = DeclareLaunchArgument(
        "gripper_tty_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for gripper control in 'modbus' mode (e.g. /dev/ttyUSB0).",
    )

    controller_manager_arg = DeclareLaunchArgument(
        "controller_manager",
        default_value="/controller_manager",
        description="Controller manager service namespace.",
    )

    controller_name_arg = DeclareLaunchArgument(
        "controller_name",
        default_value="gripper_action_controller",
        description="Controller name to spawn/activate.",
    )

    controller_spawner_timeout_arg = DeclareLaunchArgument(
        "controller_spawner_timeout",
        default_value="30",
        description="Timeout in seconds for spawner service calls.",
    )

    # --- LaunchConfigurations (substitutions used at runtime) ---
    robot_ip = LaunchConfiguration("robot_ip")
    namespace = LaunchConfiguration("namespace")
    gripper_active = LaunchConfiguration("gripper_active")
    gripper_mode = LaunchConfiguration("gripper_mode")
    gripper_tty_port = LaunchConfiguration("gripper_tty_port")
    controller_manager = LaunchConfiguration("controller_manager")
    controller_name = LaunchConfiguration("controller_name")
    timeout = LaunchConfiguration("controller_spawner_timeout")

    # --- Robot controller node placed under the configured namespace ---
    robot_controller_node = Node(
        package="ur_rtde_interface",
        executable="robot_controller",
        name="robot_controller",
        namespace=namespace,  # place the node under the given namespace
        output="screen",
        parameters=[
            {"robot_ip": robot_ip},
            {"gripper_active": gripper_active},
            {"gripper_mode": gripper_mode},
            {"namespace": namespace},
        ],
    )

    # Condition: gripper_mode == 'sim' and gripper_active == 'true'
    spawn_controller_condition = IfCondition(
        PythonExpression(
            [
                "'",
                gripper_mode,
                "' == 'sim' and '",
                gripper_active,
                "' == 'true'"
            ]
        )
    )

    # Condition: gripper_mode == 'modbus' and gripper_active == 'true'
    modbus_gripper_condition = IfCondition(
        PythonExpression(
            [
                "'",
                gripper_mode,
                "' == 'modbus' and '",
                gripper_active,
                "' == 'true'"
            ]
        )
    )

    # --- Spawner for the gripper controller (only active if gripper_active is true AND in 'sim' mode) ---
    gripper_controller_activation = Node(
        package="controller_manager",
        executable="spawner",
        name="gripper_controller_spawner",
        output="screen",
        arguments=[
            controller_name,  # controller to spawn (LaunchConfiguration)
            "--controller-manager",
            controller_manager,
            "--controller-manager-timeout",
            timeout,
        ],
        condition=spawn_controller_condition,
    )

    # --- Include the real gripper launch file (only when gripper_active is true AND mode is 'modbus') ---
    full_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robotiq_hande_driver"), "bringup", "launch", "gripper.launch.py"]
            )
        ),
        launch_arguments={  
            "tty_port": gripper_tty_port,
            "namespace": namespace,
            "use_fake_hardware": "false",
            "launch_rviz": "false",
        }.items(),
        condition=modbus_gripper_condition,
    )

    return LaunchDescription(
        [
            # Declare args
            robot_ip_arg,
            namespace_arg,
            gripper_active_arg,
            gripper_mode_arg,
            gripper_tty_port_arg,
            controller_manager_arg,
            controller_name_arg,
            controller_spawner_timeout_arg,
            # Nodes / includes
            robot_controller_node,
            gripper_controller_activation,
            full_gripper_launch,
        ]
    )