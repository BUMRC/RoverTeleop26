from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_left = LaunchConfiguration("can_left")
    can_right = LaunchConfiguration("can_right")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_zed_odom = LaunchConfiguration("use_zed_odom")
    use_teleop_override = LaunchConfiguration("use_teleop_override")
    zed_odom_topic = LaunchConfiguration("zed_odom_topic")

    navigation_bringup_launch = PathJoinSubstitution([
        FindPackageShare("rover_autonomy"),
        "launch",
        "navigation_bringup.launch.py",
    ])

    nav2_voxel_params = PathJoinSubstitution([
        FindPackageShare("rover_autonomy"),
        "config",
        "nav2_with_voxel_obstacles.yaml",
    ])

    nav2_voxel_params_zed = PathJoinSubstitution([
        FindPackageShare("rover_autonomy"),
        "config",
        "nav2_with_voxel_obstacles_zed.yaml",
    ])

    nav_with_obstacles = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_bringup_launch),
        launch_arguments={
            "can_left": can_left,
            "can_right": can_right,
            "use_foxglove": use_foxglove,
            "use_sim_time": use_sim_time,
            "use_zed_odom": use_zed_odom,
            "use_teleop_override": use_teleop_override,
            "zed_odom_topic": zed_odom_topic,
            "nav2_params_file": nav2_voxel_params,
            "nav2_params_file_zed": nav2_voxel_params_zed,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("can_left", default_value="can0"),
        DeclareLaunchArgument("can_right", default_value="can1"),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_zed_odom", default_value="true"),
        DeclareLaunchArgument("use_teleop_override", default_value="false"),
        DeclareLaunchArgument("zed_odom_topic", default_value="/zed/zed_node/odom"),
        nav_with_obstacles,
    ])
