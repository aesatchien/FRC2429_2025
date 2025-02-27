import commands2
from wpilib import SmartDashboard
from constants import ShoulderConstants
from subsystems.climber import Climber
from wpimath.units import inchesToMeters, radiansToDegrees, degreesToRadians
from subsystems.robot_state import RobotState
import constants


class MoveClimber(commands2.Command):  # change the name for your command

    def __init__(self, container, climber: Climber,  mode='stowing', angle=degreesToRadians(90), wait_to_finish=False, indent=0) -> None:
        """
        note: relative mode doesn't support wait to finish (it will never finish)
        """
        super().__init__()
        self.setName('Move Climber')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.climber = climber
        self.mode = mode
        self.angle = angle
        self.wait_to_finish = wait_to_finish
        self.addRequirements(self.climber)  # commandsv2 version of requirements
        SmartDashboard.putNumber('pivot_cmd_goal_deg', 90)  # initialize the key we will use to run this command
        SmartDashboard.putString('pivot_cmd_mode', 'absolute')

        # sick of IDE complaining
        self.start_time = None
        self.goal = None

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)

        if self.mode == 'stowing':  # what will eventually be the norm
            self.goal = constants.ClimberConstants.k_climber_motor_stowed_angle
            self.climber.set_reference(self.goal)  # should already be in radians
        elif self.mode == 'readying':
            self.goal = constants.ClimberConstants.k_climber_motor_ready
            self.climber.set_reference(self.goal)
        elif self.mode == 'climbing':
            self.goal = constants.ClimberConstants.k_climber_motor_climb_angle
            self.climber.set_reference(self.goal)
        elif self.mode == 'incremental':  # for GUI troubleshooting
            self.goal = radiansToDegrees(self.angle)
            self.climber.move_degrees(delta_degrees=self.goal, silent=False)
        else:
            print(f'Invalid Elevator move mode: {self.mode}')

        print(f"{self.indent * '    '}** Started {self.getName()} with mode {self.mode} and goal {self.goal:.2f} at {self.start_time} s **", flush=True)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.wait_to_finish:
            return self.climber.is_at_goal()
        else:
            return True

    def end(self, interrupted: bool) -> None:
        self.climber.set_brake_mode(mode='brake')

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
