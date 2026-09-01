from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("agt_rtabmap_backend"))
    sensor_share = Path(get_package_share_directory("agt_sensor_adapters"))
    return LaunchDescription(
        [
            # ---------------- robot/sensor parameters (never hard-coded) ----
            DeclareLaunchArgument(
                "lidar_topic",
                default_value="/agt/sensors/lidar/points",
                description="Standard PointCloud2 LiDAR topic consumed by RTAB-Map",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/rtabmap/no_imu",
                description=(
                    "Optional sensor_msgs/Imu topic remapped to RTAB-Map's `imu` "
                    "subscription; leave at the dummy default to disable IMU"
                ),
            ),
            DeclareLaunchArgument(
                "backend_odom_topic",
                default_value="/rtabmap/odom",
                description="Internal topic carrying the RTAB-Map ICP odometry",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="lidar_link",
                description="RTAB-Map backend body frame (the LiDAR frame)",
            ),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("map_frame", default_value="odom"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_lidar_self_filter", default_value="true"),
            DeclareLaunchArgument("lidar_custom_topic", default_value="/agt/sensors/lidar/custom"),
            DeclareLaunchArgument(
                "lidar_filtered_custom_topic",
                default_value="/agt/sensors/lidar/custom_filtered",
            ),
            DeclareLaunchArgument(
                "platform_profile",
                default_value=str(share.parents[3] / "profiles" / "platforms" / "bunker.yaml"),
            ),
            DeclareLaunchArgument("lidar_self_filter_geometry_source", default_value="urdf"),
            DeclareLaunchArgument(
                "lidar_self_filter_params_file",
                default_value=str(sensor_share / "config" / "livox_self_filter.yaml"),
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="runtime/maps/rtabmap/rtabmap.db",
                description="RTAB-Map database file for this mapping session",
            ),
            DeclareLaunchArgument(
                "odom_params_file",
                default_value=str(share / "config" / "rtabmap_odometry.yaml"),
            ),
            DeclareLaunchArgument(
                "mapping_params_file",
                default_value=str(share / "config" / "rtabmap_mapping.yaml"),
            ),
            DeclareLaunchArgument(
                "adapter_params_file",
                default_value=str(share / "config" / "rtabmap_adapter.yaml"),
            ),
            DeclareLaunchArgument(
                "publish_backend_tf",
                default_value="false",
                description=(
                    "Must stay false: the AGT adapter is the sole odom->base TF "
                    "publisher and GlobalCorrectionManager the sole map->odom one"
                ),
            ),
            SetLaunchConfiguration(
                "rtabmap_livox_input_topic",
                LaunchConfiguration("lidar_filtered_custom_topic"),
                condition=IfCondition(LaunchConfiguration("start_lidar_self_filter")),
            ),
            SetLaunchConfiguration(
                "rtabmap_livox_input_topic",
                LaunchConfiguration("lidar_custom_topic"),
                condition=UnlessCondition(LaunchConfiguration("start_lidar_self_filter")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(sensor_share / "launch" / "lidar_self_filter.launch.py")
                ),
                launch_arguments={
                    "filter_params_file": LaunchConfiguration("lidar_self_filter_params_file"),
                    "input_topic": LaunchConfiguration("lidar_custom_topic"),
                    "output_topic": LaunchConfiguration("lidar_filtered_custom_topic"),
                    "platform_profile": LaunchConfiguration("platform_profile"),
                    "geometry_source": LaunchConfiguration("lidar_self_filter_geometry_source"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_lidar_self_filter")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(sensor_share / "launch" / "livox_points.launch.py")
                ),
                launch_arguments={
                    "input_topic": LaunchConfiguration("rtabmap_livox_input_topic"),
                    "output_topic": LaunchConfiguration("lidar_topic"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
            # ---------------- RTAB-Map ICP odometry backend ----------------
            Node(
                package="rtabmap_odom",
                executable="icp_odometry",
                name="rtabmap_odometry_backend",
                output="screen",
                sigterm_timeout="15",
                sigkill_timeout="5",
                parameters=[
                    LaunchConfiguration("odom_params_file"),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "odom_frame_id": LaunchConfiguration("odom_frame"),
                        "publish_tf": ParameterValue(
                            LaunchConfiguration("publish_backend_tf"), value_type=bool
                        ),
                    },
                ],
                remappings=[
                    ("scan_cloud", LaunchConfiguration("lidar_topic")),
                    # No 2D scan input: point the unused /scan subscription at a
                    # dummy topic so it can never accidentally pick up a /scan.
                    ("scan", "/rtabmap/no_scan"),
                    ("odom", LaunchConfiguration("backend_odom_topic")),
                ],
            ),
            # ---------------- RTAB-Map mapping (loop closure) ---------------
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap_mapping",
                output="screen",
                sigterm_timeout="30",
                sigkill_timeout="10",
                parameters=[
                    LaunchConfiguration("mapping_params_file"),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "odom_frame_id": LaunchConfiguration("odom_frame"),
                        "map_frame_id": LaunchConfiguration("map_frame"),
                        "publish_tf": ParameterValue(
                            LaunchConfiguration("publish_backend_tf"), value_type=bool
                        ),
                        "database_path": LaunchConfiguration("database_path"),
                    },
                ],
                remappings=[
                    ("scan_cloud", LaunchConfiguration("lidar_topic")),
                    ("odom", LaunchConfiguration("backend_odom_topic")),
                    ("imu", LaunchConfiguration("imu_topic")),
                ],
            ),
            # ---------------- AGT contract adapter --------------------------
            Node(
                package="agt_rtabmap_backend",
                executable="rtabmap_adapter.py",
                name="agt_mapping_rtabmap_adapter",
                output="screen",
                sigterm_timeout="10",
                sigkill_timeout="5",
                parameters=[
                    LaunchConfiguration("adapter_params_file"),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "input_odometry": LaunchConfiguration("backend_odom_topic"),
                        "input_scan_cloud": LaunchConfiguration("lidar_topic"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "backend_body_frame": LaunchConfiguration("frame_id"),
                        "base_frame": LaunchConfiguration("base_frame"),
                    },
                ],
            ),
        ]
    )
