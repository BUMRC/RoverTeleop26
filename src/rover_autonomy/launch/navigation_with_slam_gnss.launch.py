from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_left = LaunchConfiguration("can_left")
    can_right = LaunchConfiguration("can_right")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_zed_odom = LaunchConfiguration("use_zed_odom")
    use_teleop_override = LaunchConfiguration("use_teleop_override")
    zed_odom_topic = LaunchConfiguration("zed_odom_topic")

    enable_slam = LaunchConfiguration("enable_slam")
    enable_gnss_fusion = LaunchConfiguration("enable_gnss_fusion")

    nav_with_obstacles_launch = PathJoinSubstitution([
        FindPackageShare("rover_autonomy"),
        "launch",
        "navigation_with_obstacles.launch.py",
    ])

    base_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_with_obstacles_launch),
        launch_arguments={
            "can_left": can_left,
            "can_right": can_right,
            "use_foxglove": use_foxglove,
            "use_sim_time": use_sim_time,
            "use_zed_odom": use_zed_odom,
            "use_teleop_override": use_teleop_override,
            "zed_odom_topic": zed_odom_topic,
        }.items(),
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(enable_slam),
    )

    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("rover_autonomy"),
                "config",
                "ekf_local.yaml",
            ])
        ],
        remappings=[("/odometry/filtered", "/odometry/filtered/local")],
        output="screen",
        condition=IfCondition(enable_gnss_fusion),
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("rover_autonomy"),
                "config",
                "navsat_transform.yaml",
            ])
        ],
        remappings=[
            ("/imu", "/zed/zed_node/imu/data"),
            ("/gps/fix", "/gps/fix"),
            ("/odometry/filtered", "/odometry/filtered/local"),
        ],
        output="screen",
        condition=IfCondition(enable_gnss_fusion),
    )

    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("rover_autonomy"),
                "config",
                "ekf_global.yaml",
            ])
        ],
        remappings=[("/odometry/filtered", "/odometry/filtered/global")],
        output="screen",
        condition=IfCondition(enable_gnss_fusion),
    )

    return LaunchDescription([
        DeclareLaunchArgument("can_left", default_value="can0"),
        DeclareLaunchArgument("can_right", default_value="can1"),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_zed_odom", default_value="true"),
        DeclareLaunchArgument("use_teleop_override", default_value="false"),
        DeclareLaunchArgument("zed_odom_topic", default_value="/zed/zed_node/odom"),
        DeclareLaunchArgument("enable_slam", default_value="false"),
        DeclareLaunchArgument("enable_gnss_fusion", default_value="false"),
        base_nav,
        slam_node,
        ekf_local_node,
        navsat_transform_node,
        ekf_global_node,
    ])
