from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_left = LaunchConfiguration("can_left")
    can_right = LaunchConfiguration("can_right")
    use_foxglove = LaunchConfiguration("use_foxglove")
    foxglove_layout = LaunchConfiguration("foxglove_layout")

    joy_config = PathJoinSubstitution([
        FindPackageShare("rover_bringup"),
        "config",
        "joy_teleop.yaml",
    ])

    twist_mux_config = PathJoinSubstitution([
        FindPackageShare("rover_autonomy"),
        "config",
        "twist_mux.yaml",
    ])

    default_foxglove_layout = PathJoinSubstitution([
        FindPackageShare("rover_autonomy"),
        "config",
        "foxglove_layout_nav_teleop_virtual_joystick.json",
    ])

    four_wheel = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rover_bringup"),
                "launch",
                "four_wheel.launch.py",
            ])
        ),
        launch_arguments={
            "can_left": can_left,
            "can_right": can_right,
        }.items(),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_config],
        output="screen",
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[joy_config],
        remappings=[("/cmd_vel", "/cmd_vel_teleop")],
        output="screen",
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_config],
        remappings=[
            ("/cmd_vel_out", "/diff_drive_controller/cmd_vel"),
            ("/cmd_vel", "/diff_drive_controller/cmd_vel"),
        ],
        output="screen",
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="screen",
        condition=IfCondition(use_foxglove),
    )

    foxglove_layout_hint = LogInfo(
        msg=["Foxglove layout file: ", foxglove_layout],
        condition=IfCondition(use_foxglove),
    )

    return LaunchDescription([
        DeclareLaunchArgument("can_left", default_value="can0"),
        DeclareLaunchArgument("can_right", default_value="can1"),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_layout", default_value=default_foxglove_layout),
        four_wheel,
        joy_node,
        teleop_node,
        twist_mux_node,
        foxglove_layout_hint,
        foxglove_node,
    ])
