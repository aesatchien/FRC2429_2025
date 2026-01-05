import commands2
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from subsystems.swerve import Swerve
from subsystems.vision import Vision
from helpers.decorators import log_command

@log_command(console=True, nt=False, print_init=True, print_end=True)
class AutoTrackVisionTarget(commands2.Command):
    def __init__(self, container, camera_key='logi_front_hsv', target_distance=0.0, indent=0) -> None:
        super().__init__()
        self.setName('AutoTrackVisionTarget')
        self.container = container
        self.swerve: Swerve = container.swerve
        self.vision: Vision = container.vision
        self.camera_key = camera_key
        self.target_distance = target_distance
        self.indent = indent

        self.addRequirements(self.swerve)

        # Use a ProfiledPIDController to control acceleration and velocity.
        # This is smoother and more robust than a simple PID + SlewRateLimiter.
        forward_constraints = TrapezoidProfile.Constraints(
            2.5,  # kMaxVelocity (m/s)
            2   # kMaxAcceleration (m/s^2)
        )
        # The PID gains here help the robot follow the generated profile.
        # P gain helps it react quickly to disturbances.
        self.forward_pid = ProfiledPIDController(1.0, 0, 0, forward_constraints)

        # The tolerance is on the goal, not the profile.
        self.forward_pid.setTolerance(0.1) # Tolerance in meters

        # Rotation PID: Input is rotation (deg), Output is rotSpeed (rad/s)
        # We want to turn left (positive rot) when target is left (positive angle).
        # calculate(angle, 0) -> error = -angle. (Negative)
        # So we need negative output from calculate to get positive rot.
        self.rotation_pid = PIDController(0.05, 0, 0)
        self.rotation_pid.enableContinuousInput(-180, 180)
        self.rotation_pid.setTolerance(2)

        # State variable to track if we were seeing a target on the last loop
        self.was_tracking = False
        self.last_dist = 0

    def initialize(self):
        # Reset the state tracker at the beginning of the command
        self.was_tracking = False
        current_dist = self.vision.get_distance(self.camera_key)
        self.last_dist = current_dist if current_dist > 0 else 0
        # Reset PIDs with current values to prevent jumps on first execution
        self.forward_pid.reset(current_dist)
        self.rotation_pid.reset()

    def execute(self):
        is_tracking = self.vision.target_available(self.camera_key)
        dist = self.vision.get_distance(self.camera_key)
        
        # --- PID Reset Logic ---
        # Condition 1: We just re-acquired a target after having none.
        reset_pid = is_tracking and not self.was_tracking
        
        # Condition 2: We are at the setpoint for a target, and suddenly see a new one much further away.
        # This prevents "rocketing" when transitioning between close and far targets.
        if not reset_pid and is_tracking and (dist - self.last_dist > 0.5):
            reset_pid = True
            msg = f"New target jump detected. Old: {self.last_dist:.2f}m, New: {dist:.2f}m"
            print(msg)
            
        if reset_pid:
            self.forward_pid.reset(dist)
            self.rotation_pid.reset()

        # Update the state for the next loop
        self.was_tracking = is_tracking
        # Only update last_dist if we have a valid target to avoid using a stale value
        if is_tracking:
            self.last_dist = dist

        # Check if target is visible
        if not is_tracking:
            # Stop if no target is visible to prevent running away
            self.swerve.drive(0, 0, 0, False, False)
            return

        rot = self.vision.get_rotation(self.camera_key)  # returns degrees

        # The ProfiledPIDController's output is the desired velocity for this timestep
        # to follow the motion profile. We still negate it to drive forward.
        # The calculate method takes (current_measurement, goal_state).
        # For a simple distance goal, the goal is just a TrapezoidProfile.State with the target distance.
        forward_speed = -self.forward_pid.calculate(dist, self.target_distance)
        rotation_speed = -self.rotation_pid.calculate(rot, 0)

        # Clamp speeds to safe limits
        forward_speed = max(min(forward_speed, 2.0), -2.0)
        rotation_speed = max(min(rotation_speed, 3.0), -3.0)

        # Drive robot-centric (fieldRelative=False)
        # keep_angle=True should be fine - we want to maintain heading when in rotation deadband
        self.swerve.drive(forward_speed, 0, rotation_speed, fieldRelative=False, rate_limited=True, keep_angle=True)

    def isFinished(self):
        # This command is designed to run continuously until interrupted (e.g., button release)
        return False

    def end(self, interrupted):
        # Stop the robot when the command ends
        self.swerve.drive(0, 0, 0, False, False)