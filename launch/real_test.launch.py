import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    aurora = Node(
        package="ros2_igtl_bridge",
        executable="igtl_node",
        parameters=[
            {"RIB_server_ip":"localhost"},
            {"RIB_port": 18944},
            {"RIB_type": "client"}
        ]
    )

    sensor = Node(
        package = "trajcontrol",
        executable = "sensor_processing",
        parameters=[{"registration":LaunchConfiguration('registration')}]
    )

    controller = Node(
        package="trajcontrol",
        executable="controller_node"
    )   

    return LaunchDescription([
        DeclareLaunchArgument(
            "registration",
            default_value="0",
            description="0=load previous / 1=new registration"
        ),
        actions.LogInfo(msg=["registration: ", LaunchConfiguration('registration')]),
        aurora,
        sensor,
        controller,
    ])
