from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def validate_backend(context):
    backend = LaunchConfiguration("backend").perform(context).strip().lower()
    if backend not in {"fast_livo2", "rtabmap"}:
        raise RuntimeError(
            "backend must be 'fast_livo2' or 'rtabmap' "
            f"(got {backend!r})"
        )
    return []


def backend_is(name):
    return IfCondition(
        PythonExpression(["'", LaunchConfiguration("backend"), "' == '", name, "'"])
    )


def generate_launch_description():
    mapping_share = Path(get_package_share_directory("agt_mapping"))
    rtabmap_share = Path(get_package_share_directory("agt_rtabmap_backend"))
    fast_livo_launch = mapping_share / "launch" / "fast_livo2_mapping.launch.py"
    rtabmap_launch = rtabmap_share / "launch" / "rtabmap_mapping.launch.py"

    shared_arguments = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "platform_profile": LaunchConfiguration("platform_profile"),
        "start_lidar_self_filter": LaunchConfiguration("start_lidar_self_filter"),
        "lidar_self_filter_geometry_source": LaunchConfiguration(
            "lidar_self_filter_geometry_source"
        ),
        "lidar_self_filter_params_file": LaunchConfiguration(
            "lidar_self_filter_params_file"
        ),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="fast_livo2",
                description="Mapping backend selected before launch: fast_livo2 or rtabmap",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "platform_profile",
                default_value=str(
                    mapping_share.parents[3] / "profiles" / "platforms" / "bunker.yaml"
                ),
            ),
            DeclareLaunchArgument("start_lidar_self_filter", default_value="true"),
            DeclareLaunchArgument("lidar_self_filter_geometry_source", default_value="urdf"),
            DeclareLaunchArgument(
                "lidar_self_filter_params_file",
                default_value=str(
                    Path(get_package_share_directory("agt_sensor_adapters"))
                    / "config"
                    / "livox_self_filter.yaml"
                ),
            ),
            OpaqueFunction(function=validate_backend),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(fast_livo_launch)),
                launch_arguments=shared_arguments.items(),
                condition=backend_is("fast_livo2"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(rtabmap_launch)),
                launch_arguments=shared_arguments.items(),
                condition=backend_is("rtabmap"),
            ),
        ]
    )
