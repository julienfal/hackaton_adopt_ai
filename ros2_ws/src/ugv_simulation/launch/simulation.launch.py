import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================
    # 1. Locate ROS 2 packages
    # =========================================================

    simulation_dir = get_package_share_directory(
        "ugv_simulation"
    )

    description_dir = get_package_share_directory(
        "ugv_description"
    )

    gazebo_dir = get_package_share_directory(
        "ros_gz_sim"
    )

    # =========================================================
    # 2. Declare launch arguments
    # =========================================================

    default_world = os.path.join(
        simulation_dir,
        "worlds",
        "ugv_world.sdf"
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Path to the Gazebo world"
    )

    world = LaunchConfiguration("world")

    # =========================================================
    # 3. Generate robot URDF from Xacro
    # =========================================================

    xacro_path = os.path.join(
        description_dir,
        "urdf",
        "patrol.urdf.xacro"
    )

    robot_description = xacro.process_file(
        xacro_path
    ).toxml()

    # =========================================================
    # 4. Start Gazebo
    # =========================================================

    gazebo_launch_path = os.path.join(
        gazebo_dir,
        "launch",
        "gz_sim.launch.py"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_launch_path
        ),
        launch_arguments={
            "gz_args": [world, " -r"]
        }.items()
    )

    # =========================================================
    # 5. Publish robot description and transforms
    # =========================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    # =========================================================
    # 6. Spawn UGV in Gazebo
    # =========================================================

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "ugv_world",
            "-name", "patrol",
            "-topic", "/robot_description",
            "-z", "0.3"
        ],
        output="screen"
    )

    # =========================================================
    # 7. Return the complete launch description
    # =========================================================

    return LaunchDescription([
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])