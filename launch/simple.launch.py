import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    # config = os.path.join(
    #     get_package_share_directory('trajcontrol'),
    #     'config',
    #     'aurora_smart_template_params.yaml'
    #     )

    # aurora = Node(
    #     package="ros2_igtl_bridge",
    #     executable="igtl_node",
    #     parameters=[
    #         {"RIB_server_ip":"localhost"},
    #         {"RIB_port": 18944},
    #         {"RIB_type": "client"}
    #     ]
    # )

    sensor = Node(
        package = "trajcontrol",
        executable = "sensor_processing"
    )

    controller = Node(
        package="trajcontrol",
        executable="controller_sequence",
    )   

    save_file = Node(
        package="trajcontrol",
        executable="save_file",
        parameters=[{"filename":LaunchConfiguration('filename')}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "filename",
            default_value="my_data",
            description="File name to save .csv file with experimental data"
        ),
        actions.LogInfo(msg=["filename: ", LaunchConfiguration('filename')]),
        # aurora,
        sensor,
        controller,
        save_file,
    ])
