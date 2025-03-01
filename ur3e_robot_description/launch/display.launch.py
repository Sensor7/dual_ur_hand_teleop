#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import xacro


def generate_launch_description():

    package_name = 'ur3e_robot_description'
    rviz_file_name = 'view_robot.rviz'
    rviz_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_file_name
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file_path],
        output='screen')


    # Launch Description
    launch_description = LaunchDescription()
    launch_description.add_action(rviz)
    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    