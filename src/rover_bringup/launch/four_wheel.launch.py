from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    can_left = LaunchConfiguration("can_left")
    can_right = LaunchConfiguration("can_right")
    controllers_file = LaunchConfiguration("controllers_file")

    xacro_file = PathJoinSubstitution([
        FindPackageShare("rover_description"),
        "urdf",
        "simple_4wd_rover_sameid_2bus.urdf.xacro",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file,
                     " can_left:=", can_left,
                     " can_right:=", can_right]),
            value_type=str,
        )
    }

    default_controllers_yaml = PathJoinSubstitution([
        FindPackageShare("rover_bringup"),
        "config",
        "four_wheel_sameid_diff_controllers.yaml",
    ])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("can_left", default_value="can0"),
        DeclareLaunchArgument("can_right", default_value="can1"),
        DeclareLaunchArgument("controllers_file", default_value=default_controllers_yaml),
        rsp,
        control_node,
        jsb_spawner,
        diff_spawner,
    ])
