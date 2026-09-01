from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_gazebo(context):
    if LaunchConfiguration("start_gazebo").perform(context).lower() not in {"1", "true", "yes", "on"}:
        return []
    try:
        gazebo_share = Path(get_package_share_directory("gazebo_ros"))
    except PackageNotFoundError:
        return [LogInfo(msg="gazebo_ros not found; running the RViz/terminal demo without Gazebo")]
    mapping_share = Path(get_package_share_directory("agt_mapping"))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_share / "launch" / "gazebo.launch.py")),
            launch_arguments={
                "world": str(mapping_share / "worlds" / "operator_demo.world"),
                "verbose": "false",
            }.items(),
        )
    ]


def generate_launch_description():
    mapping_share = Path(get_package_share_directory("agt_mapping"))
    description_share = Path(get_package_share_directory("agt_description"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="fast_livo2"),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=start_gazebo),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(description_share / "launch" / "description.launch.py")
                ),
                launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
            ),
            Node(
                package="agt_mapping",
                executable="demo_mapping_backend_sim.py",
                name="agt_demo_mapping_backend",
                output="screen",
                parameters=[
                    {
                        "backend": LaunchConfiguration("backend"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "rviz2",
                    "rviz2",
                    "-d",
                    str(mapping_share / "rviz" / "mapping_operator_sim.rviz"),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_rviz")),
            ),
        ]
    )
