from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_map_path = DeclareLaunchArgument(
        "map",
        default_value="",
        description="absolute path to map yaml",
    )

    declare_heatmap_type = DeclareLaunchArgument(
        "heatmap_type",
        default_value="3",
        description="heatmap metric type: 0 temperature, 1 humidity, 2 tvoc, 3 eco2",
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        choices=["true", "false"],
        description="whether to launch rviz",
    )

    declare_settle_time = DeclareLaunchArgument("settle_time", default_value="2.5")
    declare_sample_period_sec = DeclareLaunchArgument("sample_period_sec", default_value="1.0")
    declare_min_samples = DeclareLaunchArgument("min_samples", default_value="5")
    declare_max_samples = DeclareLaunchArgument("max_samples", default_value="10")
    declare_max_collect_sec = DeclareLaunchArgument("max_collect_sec", default_value="15.0")
    declare_stable_window = DeclareLaunchArgument("stable_window", default_value="4")
    declare_metric_sigma_threshold = DeclareLaunchArgument("metric_sigma_threshold", default_value="-1.0")
    declare_metric_ci_threshold = DeclareLaunchArgument("metric_ci_threshold", default_value="-1.0")
    declare_metric_slope_threshold = DeclareLaunchArgument("metric_slope_threshold", default_value="-1.0")

    declare_initial_x = DeclareLaunchArgument("initial_x", default_value="0.0",
                                               description="robot initial x in map frame")
    declare_initial_y = DeclareLaunchArgument("initial_y", default_value="0.0",
                                               description="robot initial y in map frame")
    declare_initial_yaw = DeclareLaunchArgument("initial_yaw", default_value="0.0",
                                                 description="robot initial yaw in radians")

    declare_adaptive_stride = DeclareLaunchArgument("adaptive_stride", default_value="0.8")
    declare_adaptive_top_k = DeclareLaunchArgument("adaptive_top_k", default_value="15")
    declare_adaptive_min_distance = DeclareLaunchArgument("adaptive_min_distance", default_value="0.8")
    declare_adaptive_travel_weight = DeclareLaunchArgument("adaptive_travel_weight", default_value="0.25")
    declare_adaptive_grad_weight = DeclareLaunchArgument("adaptive_grad_weight", default_value="0.6",
                                                          description="gradient vs coverage weight in adaptive scoring (0-1)")
    declare_idw_power = DeclareLaunchArgument("idw_power", default_value="2.0")
    declare_auto_tune_idw_power = DeclareLaunchArgument("auto_tune_idw_power", default_value="true")
    declare_idw_power_candidates = DeclareLaunchArgument("idw_power_candidates", default_value="0.5,1.0,1.5,2.0,2.5,3.0,4.0")

    stretch_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("stretch_nav2"), "/launch/navigation.launch.py"]
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    wacc_sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("stretch_wacc_sensor"), "/launch/wacc_sensor.launch.py"]
        ),
        launch_arguments={}.items(),
    )

    monitor_node = Node(
        package="air_quality_monitor",
        executable="monitor",
        name="air_quality_monitor",
        output="screen",
        parameters=[
            {
                "map": LaunchConfiguration("map"),
                "heatmap_type": LaunchConfiguration("heatmap_type"),
                "settle_time": LaunchConfiguration("settle_time"),
                "sample_period_sec": LaunchConfiguration("sample_period_sec"),
                "min_samples": LaunchConfiguration("min_samples"),
                "max_samples": LaunchConfiguration("max_samples"),
                "max_collect_sec": LaunchConfiguration("max_collect_sec"),
                "stable_window": LaunchConfiguration("stable_window"),
                "metric_sigma_threshold": LaunchConfiguration("metric_sigma_threshold"),
                "metric_ci_threshold": LaunchConfiguration("metric_ci_threshold"),
                "metric_slope_threshold": LaunchConfiguration("metric_slope_threshold"),
                "initial_x": LaunchConfiguration("initial_x"),
                "initial_y": LaunchConfiguration("initial_y"),
                "initial_yaw": LaunchConfiguration("initial_yaw"),
                "adaptive_stride": LaunchConfiguration("adaptive_stride"),
                "adaptive_top_k": LaunchConfiguration("adaptive_top_k"),
                "adaptive_min_distance": LaunchConfiguration("adaptive_min_distance"),
                "adaptive_travel_weight": LaunchConfiguration("adaptive_travel_weight"),
                "adaptive_grad_weight": LaunchConfiguration("adaptive_grad_weight"),
                "idw_power": LaunchConfiguration("idw_power"),
                "auto_tune_idw_power": LaunchConfiguration("auto_tune_idw_power"),
                "idw_power_candidates": LaunchConfiguration("idw_power_candidates"),
            }
        ],
    )

    return LaunchDescription(
        [
            declare_map_path,
            declare_heatmap_type,
            declare_use_rviz,
            declare_settle_time,
            declare_sample_period_sec,
            declare_min_samples,
            declare_max_samples,
            declare_max_collect_sec,
            declare_stable_window,
            declare_metric_sigma_threshold,
            declare_metric_ci_threshold,
            declare_metric_slope_threshold,
            declare_initial_x,
            declare_initial_y,
            declare_initial_yaw,
            declare_adaptive_stride,
            declare_adaptive_top_k,
            declare_adaptive_min_distance,
            declare_adaptive_travel_weight,
            declare_adaptive_grad_weight,
            declare_idw_power,
            declare_auto_tune_idw_power,
            declare_idw_power_candidates,
            stretch_nav_launch,
            wacc_sensor_launch,
            monitor_node,
        ]
    )
