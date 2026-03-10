from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    doing_corrections_arg = DeclareLaunchArgument(
        "doing_corrections",
        default_value="true",
        description="Whether to perform corrections during execution",
    )

    conditioning_type_arg = DeclareLaunchArgument(
        "conditioning_type",
        default_value="cartesian",
        description="Type of conditioning for inference (cartesian or joint)",
    )

    takeover_type_arg = DeclareLaunchArgument(
        "takeover_type",
        default_value="multiple",
        description="Type of takeover strategy (single or multiple)",
    )

    # Define the node
    franky_ros = Node(
        package="franka_flow",  # Ensure this matches your package name in setup.py
        executable="flow_inference_node",
        name="flow_inference_node",
        parameters=[
            {
                "doing_corrections": LaunchConfiguration("doing_corrections"),
                "takeover_type": LaunchConfiguration("takeover_type"),
                "conditioning_type": LaunchConfiguration("conditioning_type"),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            doing_corrections_arg,
            conditioning_type_arg,
            takeover_type_arg,
            franky_ros,
        ]
    )
