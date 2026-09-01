from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Publish the standard PointCloud2 LiDAR topic for RTAB-Map.

    Converts the self-filtered Livox CustomMsg stream into
    `/agt/sensors/lidar/points` (sensor_msgs/PointCloud2, frame of the input
    CustomMsg, i.e. `livox_frame` by default). FAST-LIVO2 keeps consuming
    `/agt/sensors/lidar/custom_filtered`; RTAB-Map consumes the standard cloud,
    so the two backends do not interfere.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic",
                default_value="/agt/sensors/lidar/custom_filtered",
                description=(
                    "Livox CustomMsg source. Defaults to the self-filtered "
                    "stream; use /agt/sensors/lidar/custom only for an explicit "
                    "unfiltered baseline"
                ),
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/agt/sensors/lidar/points",
                description="Standard PointCloud2 output topic",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="",
                description=(
                    "Output frame id; empty keeps the input CustomMsg frame "
                    "(livox_frame)"
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="agt_sensor_adapters",
                executable="livox_custom_to_pointcloud2",
                name="agt_livox_custom_to_pointcloud2",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
        ]
    )
