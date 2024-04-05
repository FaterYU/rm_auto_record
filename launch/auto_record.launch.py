import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rm_auto_record_node = Node(
        package='rm_auto_record',
        executable='rm_auto_record_node',
        output='both',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level',
                   'armor_detector:=INFO'],
    )

    return LaunchDescription([rm_auto_record_node])
