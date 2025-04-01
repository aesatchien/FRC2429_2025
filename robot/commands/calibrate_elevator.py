import commands2
from wpilib import SmartDashboard
from wpimath.units import inchesToMeters
import copy
import constants
from subsystems.elevator import Elevator
from constants import ElevatorConstants
import rev

class CalibrateElevator(commands2.Command):  # change the name for your command

    def __init__(self, container, elevator: Elevator, wait_to_finish=False, indent=0) -> None:
        super().__init__()
        self.setName('Calibrate Elevator')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.elevator = elevator
        self.wait_to_finish = wait_to_finish
        self.addRequirements(self.elevator)  # commandsv2 version of requirements

        # sick of IDE complaining
        self.start_time = None
        self.goal = None
        self.config = copy.deepcopy(constants.ElevatorConstants.k_config)
        self.rev_resets = rev.SparkMax.ResetMode.kNoResetSafeParameters
        self.rev_persists = rev.SparkMax.PersistMode.kNoPersistParameters
        self.counter = 0

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)

        print(f"{self.indent * '    '}** Started {self.getName()} at {self.start_time} s **", flush=True)

        self.config = copy.deepcopy(constants.ElevatorConstants.k_config)
        self.config.softLimit.reverseSoftLimitEnabled(False)
        self.elevator.disable()

        controller_revlib_error_source = self.elevator.motor.configure( self.config, self.rev_resets, self.rev_persists)
        controller_revlib_error_follower = self.elevator.follower.configure(ElevatorConstants.k_follower_config, self.rev_resets, self.rev_persists)
        print(f"Reconfigured elevator sparkmaxes. Controller status: \n {controller_revlib_error_source}\n {controller_revlib_error_follower}")

        self.elevator.motor.setVoltage(-1)
        self.counter = 0

    def execute(self) -> None:
        self.counter += 1
        if self.counter % 10 == 0:
            current = self.elevator.motor.getOutputCurrent()
            print()

    def isFinished(self) -> bool:
            return True


    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        end_message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        end_location = self.elevator.get_height()
        msg = f"{self.indent * '    '}** {end_message} {self.getName()} at {end_location:.3f}m after {end_time - self.start_time:.1f} s **"
        if print_end_message:
            print(msg)
            SmartDashboard.putString(f"alert", msg)
