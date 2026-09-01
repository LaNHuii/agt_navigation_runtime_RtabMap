from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("agt_rtabmap_backend"))
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
