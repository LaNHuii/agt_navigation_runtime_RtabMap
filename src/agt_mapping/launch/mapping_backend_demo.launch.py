from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="fast_livo2"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
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
        ]
    )
