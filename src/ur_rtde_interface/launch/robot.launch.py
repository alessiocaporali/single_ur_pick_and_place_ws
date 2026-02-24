from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    namespace = LaunchConfiguration("namespace")
    ur_type = LaunchConfiguration("ur_type")
    gripper_spawn = LaunchConfiguration("gripper_spawn")

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
            "gripper_spawn",
            default_value="true",
            description="Spawn/include Hand-E gripper in the robot launch.",
        ),
    ]

    debug_argument_logs = [
        LogInfo(msg=["[robot.launch] robot_ip: ", robot_ip]),
        LogInfo(msg=["[robot.launch] namespace: ", namespace]),
        LogInfo(msg=["[robot.launch] ur_type: ", ur_type]),
        LogInfo(msg=["[robot.launch] gripper_spawn: ", gripper_spawn]),
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
        }.items(),
    )

    mapping_node = Node(
        package="ur_joint_states_mapping_cpp",
        executable="mapping",
        name="ur_joint_states_mapping_node",
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + debug_argument_logs
        + [start_robots_launch, rtde_controllers_launch, mapping_node]
    )
