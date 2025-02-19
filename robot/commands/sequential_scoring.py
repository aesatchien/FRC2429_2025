import math
import commands2
from commands2 import InstantCommand, WaitCommand
import rev

from commands.move_elevator import MoveElevator
from commands.move_pivot import MovePivot

class SequentialScoring(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()

        self.setName(f'Sequential Scoring')
        self.container = container

        # NOTE - STOWING SHOULD PROBABLY NOT USE THIS
        self.addCommands(MovePivot(container=self.container, pivot=self.container.pivot,
                                   mode='specified', angle=math.radians(90), use_dash=False, wait_to_finish=True, indent=indent+1).withTimeout(3))
        # move elevator to right above scoring position

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))
        # have to make commands aware of the goal because you can't do it in the CommandGroup
        # first three commands get us in position safely - should happen while we are lining up
        # TODO - DO YOU WANT TO MAKE SURE THE ARM IS STOWED FIRST?
        # move elevator above the target (if possible)
        self.addCommands(MoveElevator(container=self.container, elevator=self.container.elevator,
                                      mode='scoring', offset=0.0, use_dash=False, wait_to_finish=True, indent=indent+1).withTimeout(3))
        self.addCommands(WaitCommand(seconds=0.1))
        # TODO - HERE YOU WOULD WANT TO SET THE WRIST IF WE HAVE CLEARANCE
        # move pivot to scoring position
        self.addCommands(MovePivot(container=self.container, pivot=self.container.pivot,
                                   mode='scoring', use_dash=False, wait_to_finish=True, indent=indent+1).withTimeout(3))
        # move elevator to right above scoring position
        self.addCommands(WaitCommand(seconds=0.1))
        # self.addCommands(MoveElevator(container=self.container, elevator=self.container.elevator,
        #                               mode='scoring', use_dash=False, wait_to_finish=True, indent=indent+1))
        # TODO - LOWER ARM 10 DEGREES, POSSIBLY LOWER ELEVATOR 1 INCH, RELEASE, RESET
        self.addCommands(InstantCommand(lambda: self.container.intake.set_reference(value=3, control_type=rev.SparkMax.ControlType.kVoltage)))
        self.addCommands(WaitCommand(3))
        self.addCommands(InstantCommand(lambda: self.container.intake.set_reference(value=0, control_type=rev.SparkMax.ControlType.kVoltage)))


        # self.addCommands(... your stuff here, call other commands with indent=indent+1 ...)
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

