from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.1.105",
        description="IP address of the Franka Emika Panda",
    )

    realtime_arg = DeclareLaunchArgument(
        "realtime",
        default_value="false",
        description="Whether to use the Real-time kernel configuration",
    )

    vel_limit_arg = DeclareLaunchArgument(
        "velocity_limit",
        default_value="0.05",
        description="Velocity limit for the robot (0.0 - 1.0)",
    )
    accel_limit_arg = DeclareLaunchArgument(
        "acceleration_limit",
        default_value="0.1",
        description="Acceleration limit for the robot (0.0 - 1.0)",
    )
    jerk_limit_arg = DeclareLaunchArgument(
        "jerk_limit",
        default_value="0.15",
        description="Jerk limit for the robot (0.0 - 1.0)",
    )

    # Define the node
    franky_ros = Node(
        package="franky_ros",  # Ensure this matches your package name in setup.py
        executable="franky_bridge",
        name="franky_ros_bridge",
        parameters=[
            {
                "robot_ip": LaunchConfiguration("robot_ip"),
                "realtime": LaunchConfiguration("realtime"),
                "max_velocity": LaunchConfiguration("velocity_limit"),
                "max_acceleration": LaunchConfiguration("acceleration_limit"),
                "max_jerk": LaunchConfiguration("jerk_limit"),
            }
        ],
        output="screen",
    )

    franky_home = Node(
        package="franky_ros",  # Ensure this matches your package name in setup.py
        executable="home",
        name="franky_home_service",
        parameters=[],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_ip_arg,
            realtime_arg,
            vel_limit_arg,
            accel_limit_arg,
            jerk_limit_arg,
            franky_ros,
            franky_home
        ]
    )
