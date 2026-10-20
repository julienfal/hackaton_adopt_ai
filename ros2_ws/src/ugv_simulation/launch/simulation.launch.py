"""Launch the patrol Ackermann UGV in Gazebo Harmonic (ROS 2 Jazzy)."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Find installed package assets (not paths inside src/).
    simulation_dir = get_package_share_directory("ugv_simulation")
    description_dir = get_package_share_directory("ugv_description")
    gazebo_dir = get_package_share_directory("ros_gz_sim")

    # 2. Let callers choose the SDF world file with world:=/path/to/world.sdf.
    default_world = os.path.join(simulation_dir, "worlds", "ugv_world.sdf")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Absolute path to the Gazebo world SDF",
    )
    world = LaunchConfiguration("world")  # Resolved at launch time.

    # 3. Expand Xacro into URDF; enable Gazebo's ros2_control plugin.
    xacro_path = os.path.join(description_dir, "urdf", "patrol.urdf.xacro")
    controllers_file = os.path.join(simulation_dir, "config", "controllers.yaml")
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "use_gazebo": "true",
            "controllers_file": controllers_file,
        },
    ).toxml()

    # 4. Reuse ros_gz_sim's Gazebo launcher; -r starts physics unpaused.
    gazebo_launch_path = os.path.join(gazebo_dir, "launch", "gz_sim.launch.py")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={"gz_args": ["-v 4 -r ", world]}.items(),
    )

    # 5. Publish the URDF and link transforms for the ROS ecosystem.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # 6. Spawn the URDF into the already-running Gazebo world.
    # NOTE: 'ugv_world' must match <world name="ugv_world"> in your SDF.
    # The 'world' launch argument changes the SDF PATH, not this world NAME.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "ugv_world",
            "-name", "patrol",
            "-topic", "/robot_description",
            "-z", "0.3",
        ],
        output="screen",
    )

    # 7. Spawners ask controller_manager (created by Gazebo's plugin)
    # to load and activate the joint-state and Ackermann controllers.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
    ackermann_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_file,
        ],
        output="screen",
    )

    # 8. Sequence controller activation after spawning the robot.
    # OnProcessExit fires on ANY exit, including failure: inspect the logs.
    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    start_ackermann_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermann_controller_spawner],
        )
    )

    # 9. Bridge Gazebo's /clock to ROS 2; required for use_sim_time=True.
    # '[' means one-way Gazebo -> ROS in the ros_gz_bridge CLI syntax.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # 10. Describe all actions. Event handlers must be registered before
    # the spawn process can exit; do not also launch the spawners here.
    return LaunchDescription(
        [
            world_arg,
            gazebo,
            clock_bridge,
            robot_state_publisher,
            start_joint_state_broadcaster,
            start_ackermann_controller,
            spawn_robot,
        ]
    )
