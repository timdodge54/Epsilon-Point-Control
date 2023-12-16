from launch_ros.actions import Node

from launch import LaunchDescription  # type: ignore


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    controller = Node(
        package="epsilon_control",
        executable="point_control",
        name="controller",
        output="screen",
    )
    grapher = Node(
        package="epsilon_control",
        executable="state_plotter",
        name="grapher",
        output="screen",
    )
    ld.add_action(controller)
    ld.add_action(grapher)
    return ld
