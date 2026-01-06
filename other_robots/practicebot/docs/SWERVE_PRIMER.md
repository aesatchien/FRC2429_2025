# Swerve Drive Subsystem Primer

This document explains the architecture and operation of the `Swerve` subsystem. It details how individual modules are controlled, how they work together to move the robot, and the custom features implemented for enhanced control.

## 1. The Hierarchy of Control

The swerve drive is built in layers, from the physical hardware up to the high-level commands.

### Level 1: The Swerve Module (`swervemodule_2429.py`)
Each corner of the robot has one Swerve Module. A module consists of two motors and two encoders:
-   **Drive Motor:** Controls the wheel's speed (velocity).
-   **Turn Motor:** Controls the wheel's angle (azimuth).
-   **Drive Encoder:** Measures distance traveled and speed.
-   **Turn Encoder:** Measures the absolute angle of the wheel relative to the robot frame.

**Key Responsibilities:**
-   **`setDesiredState(state)`:** This is the main input. It takes a `SwerveModuleState` (speed and angle) and uses PID controllers on the Spark Max/Flex to achieve that state.
-   **Optimization:** The module automatically optimizes the target state. If the target angle is 180 degrees away, it reverses the drive motor direction instead of spinning the wheel all the way around.

### Level 2: The Swerve Subsystem (`subsystems/swerve.py`)
This subsystem manages the four modules and the gyroscope (NavX). It acts as the interface between the robot's commands and the physical drivetrain.

**Key Responsibilities:**
-   **Kinematics:** Converts a desired robot-wide movement (ChassisSpeeds) into individual states for each of the four modules.
-   **Odometry:** Tracks the robot's position on the field by fusing encoder data with gyroscope readings.
-   **Vision Integration:** Updates the odometry with absolute field poses from AprilTags (via `addVisionMeasurement`).

---

## 2. Core Concepts & Math

### Kinematics (`SwerveDrive4Kinematics`)
Kinematics is the math that translates between "Robot Motion" and "Wheel Motion".

1.  **Inverse Kinematics (Robot -> Wheels):**
    -   Input: `ChassisSpeeds` (Forward Velocity $v_x$, Strafe Velocity $v_y$, Rotation Velocity $\omega$).
    -   Output: Four `SwerveModuleState` objects (Speed and Angle for each wheel).
    -   Used in: `drive()` method.

2.  **Forward Kinematics (Wheels -> Robot):**
    -   Input: Four `SwerveModuleState` objects (measured from sensors).
    -   Output: `ChassisSpeeds` (The robot's actual movement).
    -   Used in: Odometry updates.

### Coordinate Systems
-   **Field-Centric Control:** The driver pushes the stick "Forward" (away from them), and the robot moves downfield, regardless of which way it is facing. This requires the code to know the robot's current heading (gyro angle) to rotate the joystick inputs.
-   **Robot-Centric Control:** The driver pushes the stick "Forward", and the robot moves in the direction of its intake/front.

---

## 3. Custom Features & Non-Standard Additions

Our implementation includes several features beyond the standard WPILib examples to improve drivability and reliability.

### 1. `keep_angle` (Heading Correction)
Swerve drives can drift rotationally due to wheel scrub or uneven friction. `keep_angle` is a custom PID controller that maintains the robot's heading when the driver is not actively rotating.

-   **How it works:**
    -   If the rotation joystick input (`rot`) is non-zero, the robot rotates normally.
    -   When the driver releases the rotation stick, the code records the current heading as the `keep_angle` setpoint.
    -   If the robot drifts away from this angle while driving straight, a PID controller calculates a small rotation correction to bring it back.
    -   **Logic Location:** `perform_keep_angle()` in `swerve.py`.

### 2. Slew Rate Limiters
To prevent brownouts (voltage drops) and mechanical stress, we limit how fast the robot can accelerate.

-   **Implementation:** `SlewRateLimiter` filters are applied to the joystick inputs in the `drive()` method.
-   **Effect:** If the driver slams the stick from 0 to 100%, the limiter ramps the output up over a fraction of a second instead of applying full power instantly.

### 3. Vision-Assisted Odometry
We use the `SwerveDrive4PoseEstimator` class instead of the simpler `SwerveDriveOdometry`.

-   **Standard Odometry:** Trusts the encoders and gyro 100%. It is smooth but drifts over time.
-   **Pose Estimator:** Trusts the encoders/gyro for short-term movement but accepts "Vision Measurements" to correct long-term drift.
-   **Logic:** In `_update_vision_measurements`, we check for valid AprilTag data. If the data is fresh (low latency) and trustworthy (based on standard deviations), we pass it to the estimator to "snap" the robot's estimated position closer to reality.

### 4. QuestNav Integration
We have integrated an external localization system based on a VR headset (Quest 2/3) tracking.

-   **Role:** Acts as a secondary vision source.
-   **Logic:** The `Questnav` subsystem publishes a pose. `swerve.py` checks if this pose is valid and accepted. If so, it is added to the pose estimator just like an AprilTag measurement.

---

## 4. Key Methods in `swerve.py`

### `drive(xSpeed, ySpeed, rot, fieldRelative, rate_limited)`
The main entry point for moving the robot.
1.  Applies Slew Rate Limiters (if enabled).
2.  Applies `keep_angle` correction (if rotation stick is idle).
3.  Converts joystick inputs ($x, y, rot$) into `ChassisSpeeds`.
4.  Uses Kinematics to calculate target states for each module.
5.  Desaturates speeds (scales them down if any wheel is asked to go faster than physically possible).
6.  Sends commands to modules.

### `periodic()`
Runs every 20ms.
1.  **`_update_vision_measurements()`**: Polls cameras and QuestNav for position updates.
2.  **`_update_odometry()`**: Updates the pose estimator with the latest encoder and gyro data.
3.  **`_update_dashboard()`**: Publishes telemetry (Pose, Module States) to NetworkTables for logging and the driver dashboard.

---

## 5. Simulation Architecture

The swerve drive is fully simulated to allow testing without a robot.

-   **`simulation/swerve_sim.py`**: This file mimics the physical drivetrain.
-   **Inputs:** It listens to the `SwerveModuleState` targets set by the robot code.
-   **Physics:** It uses `physics_controller.drive()` to calculate how the robot *would* move based on those states.
-   **Feedback:** It updates the simulated Gyro (NavX) and Encoders so the robot code "thinks" it is moving.
-   **Live Tag Snapping:** If `k_use_live_tags_in_sim` is True, the simulation can snap the robot's position to match real-world AprilTag data, allowing for "Hardware-in-the-Loop" testing of vision algorithms.

---

## 6. Configuration Constants

All tuning parameters are located in `subsystems/swerve_constants.py`.

| Constant | Description |
| :--- | :--- |
| `kMaxSpeedMetersPerSecond` | The physical maximum speed of the robot (gearing dependent). |
| `kMaxAngularSpeed` | Maximum rotation speed (radians/second). |
| `kDriveKinematics` | The geometry of the robot (wheel locations relative to center). |
| `k_pose_stdevs_...` | Trust levels for vision. Smaller numbers = higher trust. |

---

## 7. Troubleshooting Common Issues

-   **Robot spins while driving straight:**
    -   Check if `keep_angle` is active.
    -   Verify the Gyro is not drifting excessively.
    -   Check if one module is dragging (mechanical issue) or has a reversed motor.
-   **"Crazy Path" in Auto:**
    -   Often caused by Vision Latency. If the robot sees a target, moves, and *then* processes the old image, it calculates the wrong target location.
    -   Check `AutoToPoseClean` debug prints for large jumps in target pose.