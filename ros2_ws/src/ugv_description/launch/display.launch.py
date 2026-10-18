"""Display the Patrol Ackermann UGV with movable joints in RViz2."""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description_dir = get_package_share_directory('ugv_description')
    xacro_path = os.path.join(description_dir, 'urdf', 'patrol.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                description_dir, 'rviz', 'display.rviz')],
        ),
    ])
