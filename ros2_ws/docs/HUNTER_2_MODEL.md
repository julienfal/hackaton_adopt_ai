# HUNTER 2.0 simulation geometry

Source: the supplied **CamScanner 21-10-2026 15.45.pdf**, HUNTER 2.0
user manual. PDF page 3 (printed page 1) contains the specification table;
PDF page 18 (printed page 16) contains the dimensioned drawing.

## Dimensions used

| Quantity | Model value | Basis |
|---|---:|---|
| Wheelbase | 0.650 m | Front-to-rear axle centers |
| Front/rear track | 0.605 m | Left-to-right wheel centers |
| Wheel diameter | 0.330 m | Drawing; nominal unloaded tyre size |
| Wheel width | 0.140 m | Drawing |
| Overall length | 0.980 m | Wheelbase + wheel diameter, straight wheels |
| Overall width | 0.745 m | Track + wheel width, straight wheels |
| Overall height | 0.380 m | Specification table; drawing instead says 0.375 m |
| Body clearance | 0.105 m | Drawing and table |
| Body box length | 0.760 m | Drawing's longitudinal body dimension |
| Body box width | 0.270 m | Simplification using the dimensioned upper-body span |
| Body box height | 0.275 m | Overall height minus body clearance |
| Total mass | 65 kg | Default to lower of the manual's 65/70 kg variants |

The manufacturer's ROS 2 driver independently lists wheelbase 0.650 m,
track 0.605 m and wheel radius 0.165 m:
https://github.com/agilexrobotics/hunter_ros2/blob/humble/hunter_base/include/hunter_base/hunter_params.hpp

The box is a coarse proxy for the tapered body. Its 270 mm width is **not** a
claim that the widest part of the real body measures 270 mm. The full vehicle
envelope includes its wheels. Steering changes that envelope.

## Frames and inertia

`base_link` remains midway between the axles at wheel-center height. Front
wheel centers are at x = +0.325 m, rear wheel centers at x = -0.325 m, with
y = +/-0.3025 m. At flat-ground contact, `base_link` is 0.165 m above ground.

The body visual, collision and inertial origins are raised by 0.0775 m in
`base_link`. Its bottom is then 0.105 m above ground and its top 0.380 m.
The body is centered longitudinally as a simplifying assumption.

Wheel masses (2 kg each) and steering knuckle masses (0.5 kg each) remain
estimates. Body mass is the remainder, 56 kg, so the whole model sums to
65 kg rather than assigning the total vehicle mass to the body alone.
Body/wheel inertias use box/cylinder formulas; the actual center of mass,
mass distribution, suspension, tyre compliance and damping are not calibrated.

If the actual vehicle is the 70 kg variant, change `vehicle_mass` to 70.0 in
`patrol.urdf.xacro`; the body mass and inertia update automatically.

The massless URDF root `rear_axle` is at the rear-axle center. The fixed joint
`rear_axle_to_base_link` places `base_link` at (+wheelbase/2, 0, 0), so the
existing body and wheel coordinates relative to `base_link` are preserved.
The model's spawn pose now refers to `rear_axle`, not the chassis midpoint.

The installed steering-controller version is 4.42.1. Its rear-drive kinematics
integrate the rear-axle center and publish that pose without a midpoint offset:
https://github.com/ros-controls/ros2_controllers/blob/4.42.1/steering_controllers_library/src/steering_kinematics.cpp
https://github.com/ros-controls/ros2_controllers/blob/4.42.1/steering_controllers_library/src/steering_controllers_library.cpp

The controller's `base_frame_id` is therefore `rear_axle`. Its private
`/ackermann_steering_controller/tf_odometry` topic is remapped to `/tf` by the
Gazebo plugin, while `robot_state_publisher` publishes the fixed relationship
on `/tf_static`. The intended chain is `odom -> rear_axle -> base_link -> wheels`.
Do not publish another odom-to-base_link transform or subtract the rear-axle
offset again in the navigation node. `odom` starts at the controller's local
planar origin; its z=0 is not Gazebo ground height.

## Limits and remaining measurements

- The manual quotes maximum steering of 33 degrees. We use +/-33 degrees
  (0.57596 rad) as a **provisional individual wheel-joint bound**, not a
  bicycle/central steering angle. Confirm the actual linkage and joint limits
  on the vehicle. The manufacturer driver separately lists a central angle
  of 0.461 rad, so these quantities must not be interchanged.
- The manual quotes 1.5 m/s no-load maximum speed. This is a specification for
  the future command limiter, **not enforced by this geometry update**.
- Existing joint torque, wheel angular-velocity, steering-rate and damping
  limits remain simulation placeholders. Motor power alone does not determine
  wheel torque; measured dynamics and hardware limits are still needed.
- The table's minimum turning-radius row says "1.6" under a millimetre label.
  Its units/reference point are ambiguous, so it is not used to derive limits.
- The 150 kg payload mentioned in the manual is not included in the unloaded
  mass. Add payload geometry/mass when the sensor and equipment layout is known.

## Checking changes

Build `ugv_description` and `ugv_simulation`, source the workspace, expand
`patrol.urdf.xacro`, and validate with `check_urdf`. Check that controller
wheelbase, tracks and radii match the generated URDF. Confirm 65 kg total mass,
the straight-wheel envelope, body clearance and positive inertias.

Then validate straight driving and turning in Gazebo before using the new
model for tracking experiments. Structural URDF/SDF checks do not establish
dynamic stability or hardware fidelity.

The rear-axle frame update was also checked in an isolated headless Gazebo run:
both controllers activated, odometry reported `odom` / `rear_axle`, and a TF2
listener resolved `odom -> rear_axle -> base_link` with the +0.325 m fixed
translation. This verifies frame publication, not moving-path tracking.
