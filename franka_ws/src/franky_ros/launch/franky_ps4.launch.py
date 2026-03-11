from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # TODO: add config files for control as arguments

    # Define the node
    franky_ps4_control = Node(
        package="franky_ros",  # Ensure this matches your package name in setup.py
        executable="franky_ps4",
        name="franky_ps4",
        output="screen",
    )

    joy = Node(
        package="joy",  # Ensure this matches your package name in setup.py
        executable="joy_node",
        name="joy_node",
    )

    return LaunchDescription(
        [
            franky_ps4_control,
            joy,
        ]
    )
