from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "camera_namespace": "cameras",
            "camera_name": "realsense",
            "enable_color": "true",
            "enable_depth": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            "align_depth.enable": "false",
            "pointcloud.enable": "false",
        }.items(),
    )

    logitech_camera = Node(
        package="multi_camera_bringup",
        executable="usb_camera_node",
        name="logitech_camera",
        output="screen",
        parameters=[
            {
                "camera_index": 0,
                "camera_name": "logitech",
                "frame_id": "logitech_frame",
                "fps": 30.0,
                "width": 640,
                "height": 480,
            }
        ],
    )

    # Add another USB camera later by copying this block:
    #
    # second_usb_camera = Node(
    #     package="multi_camera_bringup",
    #     executable="usb_camera_node",
    #     name="usb_camera_2",
    #     output="screen",
    #     parameters=[
    #         {
    #             "camera_index": 2,
    #             "camera_name": "usb_camera_2",
    #             "frame_id": "usb_camera_2_frame",
    #             "fps": 30.0,
    #             "width": 640,
    #             "height": 480,
    #         }
    #     ],
    # )

    sync_node = Node(
    package="multi_camera_bringup",
    executable="multi_camera_sync_node",
    name="multi_camera_sync_node",
    output="screen",
    parameters=[
        {
            "use_third_camera": False,
        }
    ],
    )
    
    return LaunchDescription([
        realsense_launch,
        logitech_camera,
        sync_node
    ])