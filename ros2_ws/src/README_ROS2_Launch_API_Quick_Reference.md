# ROS 2 Jazzy Launch API — Quick Reference

A practical reference for the **Adopt AI Hackathon** UGV project (**Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic**).

This document covers the functions and classes used to launch the `patrol` Ackermann UGV from a Xacro description into a Gazebo world.

## 1. How a ROS 2 Python launch file works

A Python launch file is a **description of actions** for the ROS 2 launch system to execute. The `generate_launch_description()` function builds and returns a `LaunchDescription`; constructing an action does **not** execute it immediately.

```text
simulation.launch.py
  ├── DeclareLaunchArgument("world")
  ├── IncludeLaunchDescription(gz_sim.launch.py)  → Gazebo world
  ├── Node(robot_state_publisher)                 → /robot_description, /tf
  └── Node(ros_gz_sim/create)                     → Spawn "patrol"
```

## 2. API quick-reference table

| Function / class | Import / origin | What it does | Example |
|---|---|---|---|
| `generate_launch_description()` | Function you define | ROS 2 Python launch entry point; returns a `LaunchDescription`. | `def generate_launch_description(): ...` |
| `LaunchDescription()` | `from launch import LaunchDescription` | Collects launch actions to execute. | `LaunchDescription([world_arg, gazebo, rsp, spawn])` |
| `DeclareLaunchArgument()` | `from launch.actions import DeclareLaunchArgument` | Declares a user-configurable launch argument with an optional default. | `DeclareLaunchArgument('world', default_value=world_path)` |
| `LaunchConfiguration()` | `from launch.substitutions import LaunchConfiguration` | **References** a launch argument; its value is resolved at launch runtime. | `world = LaunchConfiguration('world')` |
| `IncludeLaunchDescription()` | `from launch.actions import IncludeLaunchDescription` | Includes another launch description (e.g., Gazebo's launch file). | `IncludeLaunchDescription(source, launch_arguments=...)` |
| `PythonLaunchDescriptionSource()` | `from launch.launch_description_sources import PythonLaunchDescriptionSource` | Identifies a Python launch file to include. It does not launch it by itself. | `PythonLaunchDescriptionSource(gazebo_launch_path)` |
| `Node()` | `from launch_ros.actions import Node` | Starts a ROS 2 executable with parameters, arguments, remappings, etc. | `Node(package='ros_gz_sim', executable='create', ...)` |
| `get_package_share_directory()` | `from ament_index_python.packages import get_package_share_directory` | Finds an **installed** ROS 2 package's `share/` directory. | `get_package_share_directory('ugv_description')` |
| `xacro.process_file()` | `import xacro` | Expands Xacro includes/macros/expressions into an XML document. | `xacro.process_file(xacro_path).toxml()` |
| `os.path.join()` | `import os` (standard Python) | Joins filesystem path components safely. | `os.path.join(pkg_dir, 'urdf', 'patrol.urdf.xacro')` |

Other useful expressions:

- `.toxml()` serializes the processed XML document into the **URDF string** passed to `robot_state_publisher`.
- `dict.items()` converts `{ 'gz_args': ... }` into pairs expected by `launch_arguments=`. This is **standard Python**, not a ROS API.
- `LaunchConfiguration('world')` is **not a Python string**. Avoid using it with regular `os.path` functions or trying to read its value before launch runtime.

## 3. Actions vs. substitutions: the essential distinction

**Actions** tell the launch system what to do:

```python
world_arg = DeclareLaunchArgument(
    'world',
    default_value='/absolute/path/to/ugv_world.sdf',
)
```

**Substitutions** refer to values that are resolved later:

```python
world = LaunchConfiguration('world')
```

Run-time override:

```bash
ros2 launch ugv_simulation simulation.launch.py \
  world:=/absolute/path/to/another_world.sdf
```

The declaration defines the argument/default; the `LaunchConfiguration` resolves the supplied value during launch execution.

## 4. `Node()`: parameters vs. arguments

```python
robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'robot_description': robot_description,
        'use_sim_time': True,
    }],
    output='screen',
)
```

| `Node` keyword | Meaning |
|---|---|
| `package` | Package containing the executable. |
| `executable` | Program to run. |
| `name` | Optional ROS node name. |
| `parameters` | ROS parameters (e.g., `robot_description`, `use_sim_time`). |
| `arguments` | CLI options passed to the executable (e.g., `-name patrol`). |
| `remappings` | Remap topic/service names. |
| `output='screen'` | Print subprocess output to the terminal. |

Example of **executable arguments**, rather than ROS parameters:

```python
spawn_robot = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-world', 'ugv_world',
        '-name', 'patrol',
        '-topic', '/robot_description',
        '-z', '0.3',
    ],
    output='screen',
)
```

**Important:** `-world` must match `<world name="ugv_world">` **inside** the SDF, which might differ from the SDF filename. The example below assumes that world name.

## 5. Minimal complete `simulation.launch.py`

```python
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1) Locate installed package assets.
    simulation_dir = get_package_share_directory('ugv_simulation')
    description_dir = get_package_share_directory('ugv_description')
    gazebo_dir = get_package_share_directory('ros_gz_sim')

    # 2) Define a runtime-configurable world file path.
    default_world = os.path.join(simulation_dir, 'worlds', 'ugv_world.sdf')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to the Gazebo world SDF',
    )
    world = LaunchConfiguration('world')

    # 3) Expand Xacro into the URDF string.
    xacro_path = os.path.join(description_dir, 'urdf', 'patrol.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    # 4) Reuse Gazebo's existing launch file.
    gazebo_launch_path = os.path.join(gazebo_dir, 'launch', 'gz_sim.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'gz_args': [world, ' -r']}.items(),
    )

    # 5) Publish the URDF and TF tree.
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 6) Spawn the UGV into the named Gazebo world.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'ugv_world',
            '-name', 'patrol',
            '-topic', '/robot_description',
            '-z', '0.3',
        ],
        output='screen',
    )

    return LaunchDescription([world_arg, gazebo, rsp, spawn])
```

**Assumptions:** `ugv_world.sdf` exists in the installed `ugv_simulation/worlds/` directory; its internal world name is `ugv_world`; `ugv_description` installs `urdf/`; `ugv_simulation` installs `launch/` and `worlds/`. The robot is spawned dynamically, so avoid also embedding a duplicate `patrol` model in the world SDF.

**Timing:** Actions are started asynchronously. Their order in the list is not a guarantee that Gazebo is fully ready before `create` starts; for more complex bringups, add explicit readiness handling. **Simulation clock:** using `use_sim_time=True` also requires a `/clock` source/bridge when ROS nodes need to advance in simulation time.

## 6. Test and debug

```bash
# Build packages and source your workspace.
cd ~/hackaton_adopt_ai/ros2_ws
colcon build --symlink-install --packages-select ugv_description ugv_simulation
source install/setup.bash

# Syntax-only check (does not verify runtime launch behavior).
python3 -m py_compile src/ugv_simulation/launch/simulation.launch.py

# Show available launch arguments.
ros2 launch ugv_simulation simulation.launch.py --show-args

# Start Gazebo and spawn patrol.
ros2 launch ugv_simulation simulation.launch.py

# In a second terminal, check the description topic.
ros2 topic list
ros2 topic echo /robot_description --once --qos-durability transient_local

# Inspect the Gazebo world names/services.
gz service -l | grep '/world/'
```

For an `ament_cmake` simulation package, remember to install assets in `CMakeLists.txt`:

```cmake
install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})
```

## 7. Useful APIs for the next milestone

| API | Import | When to use it |
|---|---|---|
| `IfCondition()` | `launch.conditions` | Optional RViz/sensors when `use_rviz:=true`. |
| `UnlessCondition()` | `launch.conditions` | Inverse condition. |
| `TimerAction()` | `launch.actions` | Delayed actions (not a substitute for a true readiness check). |
| `RegisterEventHandler()` | `launch.actions` | React to process lifecycle events. |
| `OnProcessExit()` | `launch.event_handlers` | Trigger actions after a process exits. |
| `SetEnvironmentVariable()` | `launch.actions` | Configure environment for launched processes. |
| `PathJoinSubstitution()` | `launch.substitutions` | Build paths containing runtime substitutions. |
| `FindPackageShare()` | `launch_ros.substitutions` | Resolve package paths at launch runtime. |

## 8. Official references

- [ROS 2 Jazzy — Creating Launch Files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [ROS 2 Jazzy — Using Substitutions](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-Substitutions.html)
- [ROS 2 Jazzy — Using Event Handlers](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-Event-Handlers.html)
- [Gazebo Harmonic — Launch Gazebo from ROS 2](https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/)
- [Gazebo Harmonic — Spawn a Model from ROS 2](https://gazebosim.org/docs/harmonic/ros2_spawn_model/)

---

**Convention for this repository:** `.launch.py` for orchestration, `.yaml` for ROS/controller parameters, `.urdf.xacro` for robot descriptions, and `.sdf` for Gazebo worlds.
