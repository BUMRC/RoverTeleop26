from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
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

    controllers_file = LaunchConfiguration("controllers_file")
    controllers_file_no_tf = LaunchConfiguration("controllers_file_no_tf")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav2_params_file_zed = LaunchConfiguration("nav2_params_file_zed")

    joy_config = PathJoinSubstitution([
        FindPackageShare("rover_bringup"),
        "config",
        "joy_teleop.yaml",
    ])

    four_wheel_launch = PathJoinSubstitution([
        FindPackageShare("rover_bringup"),
        "launch",
        "four_wheel.launch.py",
    ])

    nav2_navigation_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    ])

    default_controllers_file = PathJoinSubstitution([
        FindPackageShare("rover_bringup"),
        "config",
        "four_wheel_sameid_diff_controllers.yaml",
    ])

    default_controllers_file_no_tf = PathJoinSubstitution([
        FindPackageShare("rover_autonomy"),
        "config",
        "four_wheel_sameid_diff_controllers_no_tf.yaml",
    ])

    default_nav2_params = PathJoinSubstitution([
        FindPackageShare("rover_autonomy"),
        "config",
        "nav2_no_obstacles.yaml",
    ])

    default_nav2_params_zed = PathJoinSubstitution([
        FindPackageShare("rover_autonomy"),
        "config",
        "nav2_no_obstacles_zed.yaml",
    ])

    control_launch_no_zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(four_wheel_launch),
        launch_arguments={
            "can_left": can_left,
            "can_right": can_right,
            "controllers_file": controllers_file,
        }.items(),
        condition=UnlessCondition(use_zed_odom),
    )

    control_launch_zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(four_wheel_launch),
        launch_arguments={
            "can_left": can_left,
            "can_right": can_right,
            "controllers_file": controllers_file_no_tf,
        }.items(),
        condition=IfCondition(use_zed_odom),
    )

    nav2_no_zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "params_file": nav2_params_file,
        }.items(),
        condition=UnlessCondition(use_zed_odom),
    )

    nav2_zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "params_file": nav2_params_file_zed,
        }.items(),
        condition=IfCondition(use_zed_odom),
    )

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    zed_odom_relay = Node(
        package="rover_autonomy",
        executable="odom_relay",
        parameters=[
            {
                "input_topic": zed_odom_topic,
                "output_topic": "/zed_odom",
                "frame_id": "odom",
                "child_frame_id": "base_link",
                "publish_tf": True,
            }
        ],
        condition=IfCondition(use_zed_odom),
        output="screen",
    )

    nav_cmd_bridge = Node(
        package="rover_autonomy",
        executable="nav_cmd_bridge",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/cmd_vel_nav",
            }
        ],
        output="screen",
    )

    goal_bridge = Node(
        package="rover_autonomy",
        executable="goal_bridge",
        parameters=[
            {
                "goal_topic": "/goal_pose",
                "nav_action": "/navigate_to_pose",
                "expected_goal_frame": "map",
            }
        ],
        output="screen",
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("rover_autonomy"),
                "config",
                "twist_mux.yaml",
            ])
        ],
        remappings=[
            ("/cmd_vel_out", "/diff_drive_controller/cmd_vel"),
            ("/cmd_vel", "/diff_drive_controller/cmd_vel"),
        ],
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_config],
        output="screen",
        condition=IfCondition(use_teleop_override),
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[joy_config],
        remappings=[("/cmd_vel", "/cmd_vel_teleop")],
        output="screen",
        condition=IfCondition(use_teleop_override),
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="screen",
        condition=IfCondition(use_foxglove),
    )

    return LaunchDescription([
        DeclareLaunchArgument("can_left", default_value="can0"),
        DeclareLaunchArgument("can_right", default_value="can1"),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_zed_odom", default_value="false"),
        DeclareLaunchArgument("use_teleop_override", default_value="false"),
        DeclareLaunchArgument("zed_odom_topic", default_value="/zed/zed_node/odom"),
        DeclareLaunchArgument("controllers_file", default_value=default_controllers_file),
        DeclareLaunchArgument("controllers_file_no_tf", default_value=default_controllers_file_no_tf),
        DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
        DeclareLaunchArgument("nav2_params_file_zed", default_value=default_nav2_params_zed),
        control_launch_no_zed,
        control_launch_zed,
        map_to_odom,
        zed_odom_relay,
        nav_cmd_bridge,
        goal_bridge,
        twist_mux_node,
        nav2_no_zed,
        nav2_zed,
        joy_node,
        teleop_node,
        foxglove_node,
    ])
