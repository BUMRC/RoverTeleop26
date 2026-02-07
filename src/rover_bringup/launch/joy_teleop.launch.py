from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_config = PathJoinSubstitution([
        FindPackageShare("rover_bringup"),
        "config",
        "joy_teleop.yaml",
    ])

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
        remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel")],
        output="screen",
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])
