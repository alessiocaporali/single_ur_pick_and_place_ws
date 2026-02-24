from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_gripper = LaunchConfiguration("use_sim_gripper")
    tty_port = LaunchConfiguration("tty_port")

    declared_arguments = [
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with gripper visualization.",
        ),
        DeclareLaunchArgument(
            "use_sim_gripper",
            default_value="true",
            description="Use use_sim_gripper bringup.",
        ),
        DeclareLaunchArgument(
            "tty_port",
            default_value="/tmp/ttyUR",
            description="Serial port for real gripper RTU connection.",
        ),
    ]

    gripper_preview_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robotiq_hande_driver"),
                    "bringup",
                    "launch",
                    "gripper_controller_preview.launch.py",
                ]
            )
        ),
        launch_arguments={
            "launch_rviz": launch_rviz,
            "use_fake_hardware": use_sim_gripper,
            "tty_port": tty_port,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [gripper_preview_launch])
