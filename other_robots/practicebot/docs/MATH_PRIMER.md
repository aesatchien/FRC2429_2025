# A Primer on Robot Geometry and Math

This document explains the core math concepts used in this codebase for robot navigation, particularly those from the `wpimath.geometry` library. Understanding these is key to writing effective autonomous and vision-guided code.

## 1. Coordinate Systems

Everything in FRC happens on a 2D plane. We use two main coordinate systems:

### Field Coordinate System (The "World Frame")
- This is the unchanging, global map of the field.
- The origin `(0, 0)` is located at the corner of the **Blue Alliance wall** at the center of the field.
- **+X** points "downfield" away from the Blue Alliance wall.
- **+Y** points to the "left" when you are standing at the Blue Alliance driver station.
- **Angles** are measured counter-clockwise (CCW), with 0 degrees pointing straight downfield along the +X axis.

> All of the robot's `Pose2d` objects from the pose estimator are in this coordinate system.

### Robot Coordinate System (The "Robot Frame")
- This coordinate system moves with the robot.
- The origin `(0, 0)` is the robot's rotational center.
- **+X** points directly **forward** from the front of the robot.
- **+Y** points directly to the **left** of the robot.
- **Angles** are measured counter-clockwise (CCW), with 0 degrees being straight ahead.

> Joystick inputs and vision target data (distance, strafe, rotation) are often calculated in this frame.

---

## 2. Core WPILib Geometry Objects

These objects are the building blocks for all our navigation math.

### `Translation2d(x, y)`
- Represents a **point** or a **vector** in 2D space.
- It's just a pair of (X, Y) coordinates.
- **Vector Math:** You can add and subtract them. This is incredibly useful.
  - `C = B - A` gives you a new `Translation2d` that is the vector pointing **from point A to point B**.

### `Rotation2d.fromDegrees(angle)`
- Represents an **angle**.
- It's more powerful than just a number because it correctly handles wrapping around 360 degrees. Subtracting 350° from 10° correctly gives 20°, not -340°.

### `Pose2d(translation, rotation)`
- This is the robot's complete **position and heading**.
- It combines a `Translation2d` (where the robot is on the field) and a `Rotation2d` (where the robot is facing).

---

## 3. Key Calculations Explained

Here are the most common geometric operations you'll see in the code.

### Finding the Vector to a Target
To figure out how to get from the robot to a gamepiece:
```python
# robot_pos and target_pos are Translation2d objects
vector_to_target = target_pos - robot_pos
```
`vector_to_target` is now a `Translation2d` that represents the direction and distance from the robot to the target.

### Finding the Angle to a Target (Robot-Relative)
This is the core of aiming. We need to know "how many degrees do I need to turn to face the target?"
```python
world_angle_to_target = vector_to_target.angle()  # Angle in the Field Coordinate System

angle_robot_needs_to_turn = world_angle_to_target - robot_heading
```
By subtracting the robot's current heading from the target's world angle, we get the error that a PID controller can use to turn the robot.

### Converting a World Vector to a Robot-Relative Vector
This is useful for calculating things like "strafe". If we have a vector in the world frame, how do we see it from the robot's perspective?
```python
# world_vector is a Translation2d in the field frame
# robot_heading is a Rotation2d
robot_relative_vector = world_vector.rotateBy(-robot_heading)
```
We rotate the world vector by the **negative** of the robot's heading. This transforms it into the robot's coordinate system.

**Example: Calculating Strafe**
```python
vec_to_gp_robot_frame = (gamepiece_pos - robot_pos).rotateBy(-robot_heading)
strafe_distance = vec_to_gp_robot_frame.Y()
```
The `Y` component of this new robot-relative vector is the lateral distance (strafe) to the target. A positive value means the target is to the robot's left; negative means to the right.