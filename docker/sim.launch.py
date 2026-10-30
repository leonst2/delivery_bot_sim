"""Container launch (delivery_bot_sim#5): the Unity TCP endpoint plus the delivery_bot_core stack in sim mode.

The stack comes from delivery_bot_bringup (delivery_bot_core#4). Until the pinned submodule provides it, only the
endpoint starts. stack:=false starts the endpoint alone on purpose.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _stack(context):
    if LaunchConfiguration('stack').perform(context) != 'true':
        return [LogInfo(msg='stack:=false, starting the endpoint only')]
    try:
        bringup_share = get_package_share_directory('delivery_bot_bringup')
    except PackageNotFoundError:
        return [LogInfo(msg='delivery_bot_bringup not installed (delivery_bot_core#4), starting the endpoint only')]
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'stack.launch.py')),
            launch_arguments={'sim': 'true'}.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stack', default_value='true', choices=['true', 'false'],
                              description='also start stack.launch.py sim:=true from delivery_bot_bringup'),
        # Unity connects to this endpoint over TCP (ROS-TCP-Connector, delivery_bot_sim#4).
        Node(package='ros_tcp_endpoint', executable='default_server_endpoint', name='UnityEndpoint',
             output='screen', parameters=[{'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': 10000}]),
        OpaqueFunction(function=_stack),
    ])
