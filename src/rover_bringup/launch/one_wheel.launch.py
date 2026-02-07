from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")

    xacro_file = PathJoinSubstitution([
        FindPackageShare("rover_description"),
        "urdf",
        "one_wheel.urdf.xacro",
    ])

    robot_description = {
        "robot_description": Command(["xacro ", xacro_file, " can_interface:=", can_interface])
    }

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("rover_bringup"),
        "config",
        "one_wheel_controllers.yaml",
    ])

    # Publishes TF from URDF
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Runs controller manager + loads hardware plugin
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    # Spawners load & activate controllers
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    vel_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can0"),
        rsp,
        control_node,
        jsb_spawner,
        vel_spawner,
    ])
