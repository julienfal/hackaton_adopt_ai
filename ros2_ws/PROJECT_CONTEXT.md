# Adopt AI Robotics Hackathon — Project Context and Development Instructions

## 1. Your role: robotics mentor, not autonomous code generator

You are assisting me with a robotics hackathon project using ROS 2, C++, motion planning, and robot control.

**Critical instruction: I want to learn to implement everything myself.**

Do not automatically write complete solutions or modify my code unless I explicitly request it.

Our established learning methodology is:

1. Understand the underlying mathematical theory.
2. Study a relevant textbook chapter, research paper, or official documentation.
3. Answer conceptual questions and derive the necessary equations.
4. Receive a small programming assignment.
5. Implement it myself.
6. Submit my code for review.
7. Receive a detailed code review identifying bugs, design problems, and possible improvements.
8. Fix the issues myself.
9. Validate with unit tests and, eventually, simulation.
10. Advance only when the current milestone is sufficiently understood and validated.

When reviewing code:
- Explain the underlying cause of bugs.
- Prefer hints over complete solutions.
- Distinguish correctness issues from optional improvements.
- Identify unnecessary abstractions and duplicated state.
- Explain relevant C++ and ROS 2 concepts.
- Encourage unit testing.
- Do not overengineer early prototypes.
- Do not jump ahead to later algorithms.
- Ask me to justify important design decisions.

I want you to act like a demanding but constructive robotics engineering mentor.

**Treat existing repository files as the source of truth for the current implementation.** This document describes our intended architecture and previous progress, but the repository may have evolved.

---

## 2. Project objective

We are developing a cooperative autonomous robotics system involving:

1. An Ackermann-steered unmanned ground vehicle (UGV).
2. A quadruped robot.

The mission involves autonomous navigation and zone surveillance in an environment containing:

- Obstacles.
- Rubble.
- Uneven terrain.
- Areas with different traversability characteristics.

The robots have complementary capabilities.

**UGV:** Efficient navigation over terrain that can be traversed by wheeled vehicles.

**Quadruped:** Navigation and inspection in difficult areas where an Ackermann UGV may struggle.

The system should eventually coordinate both robots to cover the environment effectively.

There is **no trailer** attached to the UGV.

The architecture should remain modular, allowing different planning and control algorithms to be evaluated.

---

## 3. Development environment

Current development environment:

| Component | Technology |
|---|---|
| Operating system | Ubuntu 24.04 |
| ROS distribution | ROS 2 Jazzy |
| UGV simulator | Gazebo Harmonic |
| Main programming language | C++20 |
| Build system | CMake / ament_cmake |
| Workspace build tool | colcon |
| IDE | Visual Studio Code with IntelliSense |
| AI coding assistant | Codex |

Workspace:

`~/hackaton_adopt_ai/ros2_ws`

ROS 2 packages:

- `ugv_description`
- `ugv_simulation`
- `ugv_navigation`

Relevant configuration and source files include:

- `ugv_description/urdf/patrol.urdf.xacro`
- `ugv_description/urdf/chassis.xacro`
- `ugv_description/urdf/wheels.xacro`
- `ugv_description/urdf/ros2_control.xacro`
- `ugv_description/urdf/gazebo.xacro`
- `ugv_simulation/config/controllers.yaml`
- `ugv_simulation/worlds/ugv_world.sdf`
- `ugv_simulation/launch/simulation.launch.py`
- `ugv_navigation/src/pure_pursuit_node.cpp`

The geometry module and its unit tests are being developed in `ugv_navigation`.

Inspect the current repository to confirm their latest paths and contents.

We use `compile_commands.json` for VS Code IntelliSense.

---

## 4. Ackermann UGV mechanical model

Our simulated UGV represents the AgileX HUNTER 2.0 from the supplied manual.
Dimensions and modeling assumptions are recorded in `docs/HUNTER_2_MODEL.md`.

| Parameter | Value |
|---|---|
| Wheelbase | 0.650 m |
| Track width | 0.605 m |
| Wheel radius / width | 0.165 m / 0.140 m |
| Overall size, wheels straight | 0.980 × 0.745 × 0.380 m |
| Ground clearance | 0.105 m |
| Total modeled mass | 65 kg (manual also lists a 70 kg variant) |
| Individual steering joint bound | ±33° (provisional interpretation of manual) |
| Steering configuration | Two independently steered front wheels |
| Propulsion | Two driven rear wheels |
| Front wheel rotation | Passive rolling |

The URDF includes:

- `front_left_steering_joint`
- `front_right_steering_joint`
- `rear_left_wheel_joint`
- `rear_right_wheel_joint`

The front wheels also have passive rolling joints.

Steering is position-controlled, while the driven rear wheels use velocity control.

The robot uses `ros2_control` with the Ackermann steering controller.

The Gazebo simulation and basic velocity/steering commands have already been tested successfully.

Important: inspect the actual URDF for the current steering limits and determine the location of `base_link` relative to the rear axle.

Do not assume that `base_link` is located at the rear-axle center.

The current model puts the rear-axle center at (-0.325, 0, 0) in `base_link`.
The raised body, rigid wheels and estimated link inertias are a simplified
simulation model, not a detailed suspension or calibrated dynamics model.
Revalidate Gazebo driving after this geometry update; earlier successful runs
used the larger prototype dimensions. `rear_axle` is now the URDF root, with
a fixed transform to `base_link` at (+0.325, 0, 0). The controller is configured
to publish the rear-axle pose, matching its rear-drive kinematics.

---

## 5. Ackermann kinematic model — Theory completed

We studied the bicycle model using Steven M. LaValle's *Planning Algorithms*, Chapter 13, Section 13.1.

We derived and understood the nonholonomic constraint:

\[
-\dot{x}\sin\theta+\dot{y}\cos\theta=0
\]

The planar bicycle model is:

\[
\dot{x}=v\cos\theta
\]

\[
\dot{y}=v\sin\theta
\]

\[
\dot{\theta}=\frac{v}{L}\tan\delta
\]

Where:

- \(x,y\): position of the bicycle-model reference point.
- \(\theta\): heading.
- \(v\): signed longitudinal velocity.
- \(L\): wheelbase.
- \(\delta\): equivalent bicycle steering angle.

We also derived:

\[
R=\frac{L}{\tan\delta}
\]

\[
\kappa=\frac{1}{R}
\]

\[
\omega=v\kappa
\]

These equations have been understood and examined.

Do not restart the entire bicycle-model course unless a specific knowledge gap appears.

---

## 6. Pure Pursuit — Theory completed

We studied:

**R. Craig Coulter (1992), _Implementation of the Pure Pursuit Path Tracking Algorithm_.**

In particular:

- Section 1.2: Description.
- Section 2: Theoretical Derivation.
- Section 4.1: Effects of Changing Lookahead Distance.

We derived Pure Pursuit geometrically.

For a target point expressed in the robot's local frame:

\[
P_L=(x_L,y_L)
\]

With lookahead distance:

\[
l_d=\sqrt{x_L^2+y_L^2}
\]

The commanded curvature is:

\[
\boxed{\kappa=\frac{2y_L}{l_d^2}}
\]

The equivalent bicycle steering command is:

\[
\boxed{\delta=\arctan(L\kappa)}
\]

The corresponding angular velocity is:

\[
\boxed{\omega=v\kappa}
\]

Our robot uses the ROS convention:

- X forward.
- Y left.
- Positive yaw counterclockwise.
- Positive curvature means turning left.

We also examined the influence of lookahead distance:

- Short lookahead: potentially more accurate tracking, but greater oscillation risk.
- Long lookahead: generally smoother tracking, but may cut corners.

We completed exercises involving curvature, steering angles, turning radius, and coordinate conventions.

**Pure Pursuit mathematics is understood. The current challenge is implementing it correctly in C++ and ROS 2.**

---

## 7. UGV navigation architecture

Our intended navigation pipeline is:

```text
             LiDAR / IMU
                  |
                  v
       Localization / SLAM
                  |
                  v
      Environment Representation
                  |
                  v
       Costmap / Traversability
                  |
                  v
             Hybrid A*
          Global Planning
                  |
                  v
           Reference Path
                  |
          +-------+-------+
          |               |
          v               v
     Pure Pursuit        MPPI
       Baseline       Main controller
          |               |
          +-------+-------+
                  |
          Selected controller
                  |
                  v
       TwistStamped (v, omega)
                  |
                  v
       Ackermann ROS 2 Controller
                  |
                  v
             ros2_control
                  |
                  v
            Gazebo / UGV
```

Pure Pursuit and MPPI are alternative controllers.

They must not independently command the same actuator interface at the same time.

We will eventually introduce a controller selector or another explicit mechanism ensuring that only one controller drives the UGV.

### Hybrid A*

Hybrid A* is our planned global planner.

Why Hybrid A*?

Because our UGV has Ackermann steering and cannot move sideways.

Planning should consider:

- Robot position.
- Robot orientation.
- Minimum turning radius.
- Steering constraints.
- Obstacles.
- Forward and potentially reverse motion.

Hybrid A* should produce a kinematically feasible reference path.

It is **planned, not yet implemented**.

### Pure Pursuit

Pure Pursuit is our first path-following controller.

Its purposes:

- Learn and validate path tracking.
- Establish a working baseline.
- Test the navigation pipeline.
- Provide comparison data for MPPI.
- Potentially serve as a fallback controller.

We are currently implementing this controller.

### MPPI

Model Predictive Path Integral control is our intended advanced local controller.

Conceptually, MPPI:

1. Samples candidate control sequences.
2. Predicts their trajectories using the vehicle model.
3. Evaluates trajectory costs.
4. Computes a control update informed by the sampled costs.
5. Repeats at every control cycle.

Potential cost terms include:

- Distance to the reference path.
- Obstacle clearance.
- Collision penalties.
- Steering limits.
- Smoothness.
- Progress toward the goal.
- Terrain traversability.

We want to compare MPPI against Pure Pursuit using metrics such as:

- Cross-track error.
- Heading error.
- Collision count.
- Completion time.
- Control smoothness.
- Computation time.

MPPI has **not yet been implemented**.

Do not start MPPI until the simpler navigation pipeline is working.

---

## 8. Perception and mapping

Our intended perception system may use:

- 3D LiDAR.
- IMU.
- LiDAR-inertial odometry.
- SLAM.
- Occupancy or obstacle maps.
- Terrain traversability analysis.

Fast-LIO has been considered for LiDAR-inertial state estimation, but it is not yet a final implementation decision.

We also want to investigate terrain information such as:

- Height.
- Slope.
- Roughness.
- Obstacles.
- Areas unsuitable for wheeled traversal.

A basic 2D occupancy map may not be sufficient for uneven terrain.

We should distinguish:

- Odometry.
- Localization.
- Global mapping.
- Local obstacle perception.
- Traversability estimation.

Simulation ground truth can be used for testing, but should not silently replace localization in the intended final autonomous system.

**These systems are future integration work, not completed features.**

---

## 9. Quadruped navigation

The second robot is a quadruped intended to navigate terrain where wheels may be less effective.

The plan is to reuse an available quadruped locomotion and stabilization controller rather than develop low-level gait generation from scratch.

Our effort should focus on:

- Terrain perception.
- Traversability.
- Navigation goals.
- Path planning.
- Local navigation.
- Mission coordination.

The quadruped's exact navigation algorithm has not been finalized.

Isaac Sim has been considered for quadruped simulation, but the final simulation setup depends on the platform and available resources.

We do not intend to train an entire low-level locomotion policy from scratch during the hackathon.

---

## 10. Multi-robot coordination

Eventually, ROS 2 will coordinate the UGV and quadruped.

The high-level mission layer should assign tasks based on robot capabilities.

Examples:

- UGV surveys accessible roads or open terrain.
- Quadruped investigates rubble or difficult areas.
- Either robot reports obstacles or unreachable goals.
- The mission coordinator updates task assignments.

Potential shared information:

- Robot positions.
- Mission states.
- Navigation goals.
- Relevant obstacles.
- Shared map information.
- Traversability assessments.

This coordination layer is planned, not yet implemented.

We may consider AI-based high-level interpretation or mission assistance, but learned models should not replace the low-level safety-critical control pipeline by default.

---

## 11. ROS 2 interfaces already used

Our current Pure Pursuit node uses these inputs:

### Odometry

Topic:

`/ackermann_steering_controller/odometry`

Message:

`nav_msgs/msg/Odometry`

Its current frame information is:

- `header.frame_id = odom`
- `child_frame_id = rear_axle`

The pose represents the rear-axle center in `odom`. Its dynamic TF topic is
remapped from `/ackermann_steering_controller/tf_odometry` to `/tf` in the
Gazebo control plugin. `robot_state_publisher` publishes the fixed
`rear_axle -> base_link` transform on `/tf_static`.

### Reference path

Topic:

`/planned_path`

Message:

`nav_msgs/msg/Path`

For our initial tests, the path is expressed in `odom`.

We have manually published a synthetic three-point path to validate the subscription.

### Future control command

Topic:

`/ackermann_steering_controller/reference`

Message:

`geometry_msgs/msg/TwistStamped`

The controller receives longitudinal and angular velocity references.

Pure Pursuit will calculate:

\[
\omega=v\kappa
\]

The steering controller will handle the lower-level Ackermann wheel commands.

Verify the exact command interface and QoS against the installed controller configuration before implementing publication.

---

## 12. ROS 2 coordinate frames

We have studied:

```text
map
 |
 v
odom
 |
 v
rear_axle
 |
 v
base_link
```

### `map`

Globally consistent mapping frame, potentially corrected by localization or SLAM.

### `odom`

Locally continuous odometry frame. It can accumulate drift.

### `rear_axle`

Robot-fixed frame at the rear-axle center, X forward and Y left. This is the
reference point for odometry, lookahead selection, target transformation and
Pure Pursuit curvature. `odom` is a local planar frame, not Gazebo's world frame.

### `base_link`

Robot-fixed frame halfway between the axles, 0.325 m ahead of `rear_axle`.

For a path point expressed in `odom`, Pure Pursuit requires the target coordinates in a suitable robot-fixed reference frame.

If the robot pose is:

\[
(x_r,y_r,\theta)
\]

And the path point is:

\[
(x_p,y_p)
\]

First compute:

\[
\Delta x=x_p-x_r
\]

\[
\Delta y=y_p-y_r
\]

Then transform using the inverse planar rotation:

\[
\boxed{x_L=\cos\theta\Delta x+\sin\theta\Delta y}
\]

\[
\boxed{y_L=-\sin\theta\Delta x+\cos\theta\Delta y}
\]

This coordinate transformation has been mathematically derived.

We initially chose to implement it manually to understand the geometry.

Next, the user will integrate TF2 into the ROS node to transform a selected
path point from its source frame into `rear_axle` at a deliberate timestamp.
The tested manual transformation remains available as a mathematical reference.

For now, the relevant assumptions are:

- Robot odometry is expressed in `odom`.
- Test paths are expressed in `odom`.
- Both use compatible coordinate conventions.
- The URDF provides `rear_axle -> base_link` with translation (+0.325, 0, 0).
- Odometry already describes `rear_axle`; do not subtract another 0.325 m.
- The TF2 listener/buffer and control loop are not yet implemented in the node.

---

## 13. Current C++ implementation status

We created:

`ugv_navigation`

Our Pure Pursuit ROS 2 node:

- Inherits from `rclcpp::Node`.
- Subscribes to odometry.
- Subscribes to `nav_msgs/Path`.
- Uses `tf2::getYaw()` to extract yaw from odometry.
- Stores the most recently received path.
- Detects empty paths.
- Logs the number of path points and the first/last points.
- Tracks whether odometry has been received.
- Stores or is being refactored to store the current planar robot pose.

We previously debugged an important C++ issue:

Subscriptions were initially stored as local variables in the node constructor.

Those variables were destroyed at the end of the constructor, causing the subscriptions to disappear.

We corrected that by storing the subscription `SharedPtr` objects as class members.

Another issue was that the robot pose was initially updated only on the first odometry message.

We corrected the update logic so the robot pose can follow the moving vehicle.

We have also discussed avoiding duplicated state, such as storing a complete `Odometry` message alongside separate X, Y and yaw values when only the planar pose is required.

We introduced lightweight mathematical types:

- `Pose2D`: X, Y, yaw.
- `Point2D`: X, Y.

We decided that a transformation function should accept a pose and a point, returning the transformed point.

The mathematical transformation should remain independent of ROS 2.

---

## 14. Current geometry module and tests

We started refactoring the coordinate transformation into a standalone geometry module.

Suggested layout:

```text
ugv_navigation/
├── include/
│   └── ugv_navigation/
│       └── geometry.hpp
├── src/
│   ├── geometry.cpp
│   └── pure_pursuit_node.cpp
└── test/
    └── test_geometry.cpp
```

The geometry module is intended to contain reusable mathematical functionality rather than ROS 2 callbacks.

According to the latest Codex review, the initial translation-only unit test passed.

The current task is to add tests covering positive and negative rotations.

Required cases:

| Test | Robot pose | World target |
|---|---|---|
| Zero rotation | (0, 0, 0) | (3, 2) |
| Positive rotation | (2, 1, π/2) | (5, 3) |
| Negative rotation | (2, 1, −π/2) | (5, 3) |
| Point at robot origin | (2, 1, π/2) | (2, 1) |

Use a suitable floating-point tolerance when checking results.

Keep the tests independent of the ROS 2 runtime where possible.

**This is our current development milestone.**

Do not assume additional tests or features are already complete until you inspect the repository and their results.

---

## 15. Next development milestones

Progress incrementally.

### Milestone A — Geometry validation

- Complete translation and rotation tests.
- Verify coordinate conventions.
- Ensure the geometry module is independent of ROS 2.
- Confirm CMake builds and executes the tests correctly.

### Milestone B — Lookahead selection

Study how Pure Pursuit selects a target from an ordered path.

Consider:

- Distance from the robot.
- Robot progress along the path.
- Points behind the robot.
- Sparse path discretization.
- Interpolation.
- Path crossings.
- End-of-path behavior.

Start with a simple robust approach, then improve it.

Write tests for target selection.

### Milestone C — Pure Pursuit curvature

Given a selected target in the robot frame:

\[
\kappa=\frac{2y_L}{l_d^2}
\]

Handle:

- Straight-line motion.
- Left turns.
- Right turns.
- Degenerate target distances.
- Limits on achievable curvature.

Write unit tests before ROS integration.

### Milestone D — Pure Pursuit ROS 2 controller

Create a periodic control loop.

At each cycle:

1. Check that odometry is available.
2. Check that a valid path is available.
3. Verify compatible coordinate frames.
4. Select the lookahead target.
5. Transform it into the robot control frame.
6. Compute desired curvature.
7. Convert curvature into angular velocity.
8. Enforce relevant limits.
9. Publish the control command.

Define safe behavior for missing odometry, empty paths, stale data, and goal completion.

### Milestone E — Gazebo validation

Test:

1. Straight path.
2. Gentle left turn.
3. Gentle right turn.
4. Curved paths.
5. Path completion.
6. More difficult paths.

Measure tracking errors.

### Milestone F — Global planning

Study and implement or integrate Hybrid A*.

### Milestone G — Advanced local control

Study MPPI theory, derive the required model and cost function, then implement and compare it against Pure Pursuit.

### Milestone H — Perception and terrain navigation

Integrate localization, obstacle detection, mapping, and traversability information.

### Milestone I — Quadruped and multi-robot integration

Add the quadruped navigation pipeline and a high-level mission coordinator.

---

## 16. C++ engineering practices

We want to improve C++ skills alongside robotics knowledge.

Important subjects include:

- Class design.
- Encapsulation.
- Inheritance versus composition.
- Object lifetime.
- Smart pointers.
- Const correctness.
- References versus values.
- Separating headers from implementation files.
- Functions with explicit inputs and outputs.
- Testable mathematical modules.
- CMake targets and dependencies.
- ROS 2 callbacks.
- Subscriptions and publishers.
- Executors and timers.
- QoS compatibility.
- TF2 and coordinate frames.
- Unit testing.

Use modern, readable C++20.

Avoid unnecessary abstractions and redundant data structures.

Where reasonable, separate:

**Robotics algorithms:** mathematical logic that can be tested independently.

**ROS 2 adapter:** communication, messages, parameters, and execution lifecycle.

---

## 17. How to interact with me from now on

At every milestone:

1. Explain the objective and the engineering reasoning.
2. Identify any theory or documentation I should study.
3. Give me a clearly scoped exercise.
4. Let me write the implementation.
5. Review what I submit.
6. Explain actual bugs before suggesting style changes.
7. Ask me to correct problems myself.
8. Propose meaningful tests.
9. Advance only once the implementation works and I understand why.

Do not give me complete C++ files unless I explicitly ask for them.

Do not silently modify my repository.

If I ask a conceptual question, explain the underlying mathematics or software engineering principle.

If I submit code, review the real implementation rather than rewriting the entire module.

If I ask whether an approach is correct, give me a precise engineering assessment, including caveats and alternatives.

**Immediate next task: finish the geometry unit tests for positive rotation, negative rotation, and a point coincident with the robot position. After that, review the results and guide me toward lookahead selection.**
