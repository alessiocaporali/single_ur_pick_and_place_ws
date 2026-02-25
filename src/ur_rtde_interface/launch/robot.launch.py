from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    namespace = LaunchConfiguration("namespace")
    ur_type = LaunchConfiguration("ur_type")
    gripper_mode = LaunchConfiguration("gripper_mode")
    tty_port = LaunchConfiguration("tty_port")

    is_mode_modbus = PythonExpression(["'", gripper_mode, "' == 'modbus'"])
    is_mode_rtde = PythonExpression(["'", gripper_mode, "' == 'rtde'"])
    is_mode_sim = PythonExpression(["'", gripper_mode, "' == 'simulation'"])

    gripper_spawn = PythonExpression(["(", is_mode_modbus, ") or (", is_mode_rtde, ") or (", is_mode_sim, ")"])
    launch_gripper_controller = PythonExpression(["(", is_mode_modbus, ") or (", is_mode_sim, ")"])
    use_sim_gripper = PythonExpression(["(", is_mode_rtde, ") or (", is_mode_sim, ")"])
    gripper_active = PythonExpression(["(", is_mode_rtde, ")"])

    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="172.25.0.2",
            description="IP address for the robot.",
        ),
    
        DeclareLaunchArgument(
            "namespace",
            default_value="robot",
            description="Namespace for the RTDE controller.",
        ),
       
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of the UR robot.",
        ),
        DeclareLaunchArgument(
            "gripper_mode",
            default_value="false",
            choices=["false", "modbus", "rtde", "simulation"],
            description="Gripper mode: false (no gripper), modbus (USB), rtde (UR socket), simulation (fake hardware).",
        ),
        DeclareLaunchArgument(
            "tty_port",
            default_value="/dev/ttyUSB0",
            description="Serial port used for Modbus gripper mode.",
        ),
    ]

    debug_argument_logs = [
        LogInfo(msg=["[robot.launch] robot_ip: ", robot_ip]),
        LogInfo(msg=["[robot.launch] namespace: ", namespace]),
        LogInfo(msg=["[robot.launch] ur_type: ", ur_type]),
        LogInfo(msg=["[robot.launch] gripper_mode: ", gripper_mode]),
        LogInfo(msg=["[robot.launch] tty_port: ", tty_port]),
        LogInfo(msg=["[robot.launch] gripper_active: ", gripper_active]),
    ]

    start_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_cell_control"),
                    "launch",
                    "start_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "namespace": namespace,
            "ur_type": ur_type,
            "gripper_spawn": gripper_spawn,
            "launch_gripper_controller": launch_gripper_controller,
            "use_sim_gripper": use_sim_gripper,
            "tty_port": tty_port,
        }.items(),
    )

    rtde_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_rtde_interface"),
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "namespace": namespace,
            "gripper_active": gripper_active,
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + debug_argument_logs
        + [start_robots_launch, rtde_controllers_launch]
    )
