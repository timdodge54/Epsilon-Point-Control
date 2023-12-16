from launch_ros.actions import Node
import os

from launch import LaunchDescription  # type: ignore
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    controller = Node(
        package="epsilon_control",
        executable="point_control",
        name="controller",
        output="screen",
    )
    # launch the empty world launch file
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    "turtlebot3_gazebo"), "launch", "empty_world.launch.py"
                )

            )))

    ld.add_action(controller)
    return ld
