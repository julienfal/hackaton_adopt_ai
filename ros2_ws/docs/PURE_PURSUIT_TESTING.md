# First Pure Pursuit driving tests

The current controller is a forward-only prototype for simple open paths in
`odom`. Odometry represents `rear_axle`. Both odometry and the dynamic
`odom -> rear_axle` TF must be no more than 0.5 seconds old and must not be
timestamped in the future.

Temporary settings in `pure_pursuit_node.cpp`:

| Setting | Value |
|---|---|
| Timer | 20 Hz |
| Lookahead distance | 1.5 m |
| Maximum speed | 0.2 m/s |
| Curvature limit | ±0.5 m⁻¹ |
| Slowdown distance from endpoint | 1.0 m |
| Endpoint position tolerance | 0.15 m |

When no remaining point meets the lookahead distance, the node targets the last
point. Speed is `0.2 * min(distance_to_endpoint / 1.0, 1.0)` until arrival.
Within the endpoint tolerance, it sends zero velocity and latches completion.
Every received path message starts a new tracking request and clears that latch;
publish test paths once. An empty path cancels tracking.

A target behind or directly beside the rear axle causes a stop. This version
does not reverse, align to a final heading, avoid obstacles, or track progress
through loops/crossings. Endpoint proximity is sufficient for completion on the
simple open paths used here. Odometry and TF are checked independently for age;
they are not yet synchronized to an identical measurement timestamp.

## Repeatable synthetic integration check

From `ros2_ws`, with ROS Jazzy sourced:

```bash
colcon build --packages-select ugv_navigation
colcon test --packages-select ugv_navigation
colcon test-result --test-result-base build/ugv_navigation --verbose
python3 src/ugv_navigation/test/check_control_loop.py build/ugv_navigation/ugv_navigation
```

The Python check runs separately from the geometry unit tests. It launches the
node and remaps its path, odometry, command and TF topics to unique test names.
It checks stale/future input handling, recovery, speed and curvature limits,
endpoint fallback, completion latching and new-path reset. A final ideal
straight-line vehicle integrates the published velocity to check arrival.
This does not test Gazebo wheel dynamics or physical braking distance.

## Gazebo straight-path test

Source `install/setup.bash` in each terminal. Start a fresh simulation:

```bash
ros2 launch ugv_simulation simulation.launch.py
```

In a second terminal:

```bash
ros2 run ugv_navigation ugv_navigation --ros-args -p use_sim_time:=true
```

After odometry and TF are available, publish a straight path in a third terminal.
This assumes a fresh odometry origin near `(0, 0)` with yaw near zero:

```bash
ros2 topic pub --once /planned_path nav_msgs/msg/Path "{header: {frame_id: odom}, poses: [{pose: {position: {x: 0.0}, orientation: {w: 1.0}}}, {pose: {position: {x: 1.0}, orientation: {w: 1.0}}}, {pose: {position: {x: 2.0}, orientation: {w: 1.0}}}, {pose: {position: {x: 3.0}, orientation: {w: 1.0}}}]}"
```

Expect forward motion up to 0.2 m/s, slowing within the last metre, followed by
`Path complete` and zero velocity within 0.15 m of `(3, 0)`, measured at the rear
axle. Inspect `/ackermann_steering_controller/reference` and odometry to verify
the commands and response. Only this controller should publish driving commands.

To cancel the path:

```bash
ros2 topic pub --once /planned_path nav_msgs/msg/Path "{header: {frame_id: odom}, poses: []}"
```
