# ROS 2 Launch API — Quick Reference

**Project:** Adopt AI Hackathon · Ackermann UGV `patrol`  
**Stack:** Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic

This is a short reference for the APIs used in `ugv_simulation/launch/simulation.launch.py`.

## What the launch file starts

```text
simulation.launch.py
├── Gazebo + ugv_world.sdf
├── /clock bridge (Gazebo → ROS 2)
├── robot_state_publisher (/robot_description, TF)
├── spawn patrol (ros_gz_sim/create)
└── controller_manager (created by gz_ros2_control inside Gazebo)
    ├── joint_state_broadcaster (activated by spawner)
    └── ackermann_steering_controller (activated by spawner)
```

## 1. Python / ROS 2 launch API

| API | What it does |
|---|---|
| `generate_launch_description()` | Entry point called by `ros2 launch`; returns the launch actions. |
| `LaunchDescription([...])` | Collects actions; constructing the list does **not** run them immediately. |
| `get_package_share_directory(name)` | Absolute path to an **installed** package's `share/` directory. |
| `os.path.join(...)` | Builds a filesystem path (standard Python). |
| `DeclareLaunchArgument(name, default_value=...)` | Defines a command-line launch argument and default value. |
| `LaunchConfiguration(name)` | References that argument's value, resolved **at launch time** (not an ordinary string). |
| `IncludeLaunchDescription(source, launch_arguments=...)` | Includes another launch file, here Gazebo's `gz_sim.launch.py`. |
| `PythonLaunchDescriptionSource(path)` | Identifies the Python launch file to include. |
| `Node(package=..., executable=...)` | Starts a ROS 2 executable with CLI arguments, ROS parameters, remappings, etc. |
| `RegisterEventHandler(handler)` | Registers logic to react to a launch event. |
| `OnProcessExit(target_action=..., on_exit=[...])` | Runs actions when the selected process exits; **exit does not imply success**. |
| `xacro.process_file(path, mappings=...)` | Expands macros and includes, substituting launch-provided Xacro parameters. |
| `.toxml()` | Serializes the expanded robot description into the URDF XML string. |
| `.items()` | Converts a Python dictionary into key/value pairs for `launch_arguments=`. |

### `Node`: parameters vs arguments

- `parameters=[{'use_sim_time': True}]`: **ROS parameters** used inside the node.
- `arguments=['-name', 'patrol', '-z', '0.3']`: **command-line options** understood by its executable.
- `output='screen'`: sends process logs to the terminal.

### Runtime argument example

```python
world_arg = DeclareLaunchArgument('world', default_value=default_world)
world = LaunchConfiguration('world')
```

```bash
ros2 launch ugv_simulation simulation.launch.py world:=/absolute/path/other.sdf
```

**Important:** That argument changes the **world file path**. The `ros_gz_sim/create -world ugv_world` option still needs to match the SDF's internal `<world name="ugv_world">`.

## 2. Gazebo / ros2_control components

| Component | Responsibility |
|---|---|
| `patrol.urdf.xacro` | Links, joints, dimensions, inertial properties; base robot description. |
| `<ros2_control>` in Xacro | Exposes joint command interfaces (steering position, rear-wheel velocity) and state interfaces. |
| `<gazebo><plugin ... gz_ros2_control ...>` | Loads the Gazebo system plugin, which creates the simulated `controller_manager`. |
| `controllers.yaml` | Controller types, joint names, wheelbase, wheel radius, track widths, control parameters — **not** geometry. |
| `joint_state_broadcaster` | Publishes joint state feedback on `/joint_states`. |
| `ackermann_steering_controller` | Converts desired body motion into steering and driven-wheel commands. |
| `controller_manager/spawner` | Loads/configures/activates an individual controller, then exits. |
| `ros_gz_bridge/parameter_bridge` | Transfers messages between Gazebo Transport and ROS 2; our bridge sends `/clock` Gazebo → ROS. |
| `robot_state_publisher` | Publishes robot description and transforms based on the URDF and `/joint_states`. |
| `ros_gz_sim/create` | Inserts the robot into a named Gazebo world. |

### Controller startup order

```text
Spawn process exits
      ↓  OnProcessExit
joint_state_broadcaster spawner exits
      ↓  OnProcessExit
ackermann controller spawner runs
```

The event handlers sequence the **spawner processes**; they do not guarantee that the preceding operation succeeded. Check errors and controller states if something fails.

## 3. Build and verify

```bash
cd ~/hackaton_adopt_ai/ros2_ws
colcon build --symlink-install --packages-select ugv_description ugv_simulation
source install/setup.bash

python3 -m py_compile src/ugv_simulation/launch/simulation.launch.py
ros2 launch ugv_simulation simulation.launch.py --show-args
ros2 launch ugv_simulation simulation.launch.py
```

In a **second terminal** (with ROS/workspace sourced):

```bash
ros2 node list                            # Look for /controller_manager
ros2 control list_controllers             # Broadcaster + Ackermann should be active
ros2 control list_hardware_interfaces     # Check claimed joint command interfaces
ros2 topic echo /clock --once             # Simulation clock is bridged
ros2 topic echo /robot_description --once --qos-durability transient_local
```

If controller activation fails, check that `gazebo.xacro` is included **and instantiated** in `patrol.urdf.xacro`, that `controllers.yaml` is installed and valid, and that the plugin has loaded. The Xacro `use_gazebo` and `controllers_file` arguments must exist for the launch mappings to work.

## 4. Before implementing algorithms: what to read

When we begin implementing an algorithm, read the **specific relevant section first**, not an entire book:

| Upcoming task | Targeted reading |
|---|---|
| Ackermann vehicle kinematics / nonholonomic constraints | LaValle, *Planning Algorithms*, §13.1 (especially the simple car); Lynch & Park, *Modern Robotics*, Ch. 13. |
| Pure Pursuit, feedback, actuator response | *Modern Robotics*, Ch. 13 (mobile-robot feedback) and relevant sections of Ch. 11 (feedback-control principles). |
| Hybrid A*, feasible vehicle trajectories | LaValle, *Planning Algorithms*, Ch. 13 (Dubins/Reeds–Shepp), then selected parts of Ch. 14; the Hybrid A* paper when we implement it. |
| MPPI | First derive the bicycle model and finite-horizon objective; then read selected predictive-control concepts from Rawlings, Mayne & Diehl's *Model Predictive Control: Theory, Computation, and Design*, plus the MPPI-specific research paper. |

The reading plan will be tailored to the code we're about to write. **No need to read all the MPC textbook now.**

## Official references

- [ROS 2 Jazzy — Creating Launch Files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [ROS 2 Jazzy — Launch substitutions](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-Substitutions.html)
- [Gazebo Harmonic — Launch Gazebo from ROS 2](https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/)
- [gz_ros2_control (Jazzy)](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
- [Official Ackermann launch example](https://github.com/ros-controls/gz_ros2_control/blob/jazzy/gz_ros2_control_demos/launch/ackermann_drive_example.launch.py)
- [Official Ackermann controller YAML example](https://github.com/ros-controls/gz_ros2_control/blob/jazzy/gz_ros2_control_demos/config/ackermann_drive_controller.yaml)
- [LaValle — Planning Algorithms](https://lavalle.pl/planning/)
- [Modern Robotics — Ch. 13](https://modernrobotics.northwestern.edu/chapters/chapter13/)
- [Rawlings / Mayne / Diehl — MPC book](https://sites.chemengr.ucsb.edu/~jbraw/mpc/)
