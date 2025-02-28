import commands2

from commands.move_pivot import MovePivot
from commands.move_elevator import MoveElevator
from commands.move_wrist import MoveWrist
from commands.move_wrist_by_joystick import MoveWristByJoystick
import constants

class GoToReefPosition(commands2.SequentialCommandGroup):
    def __init__(self, container, level: int, wrist_setpoint=None, indent=0) -> None:
        """
        note: cannot use this in teleop, as it waits for joystick input
        :param wrist_setpoint: the setpoint for the wrist. leave it as none for driver to decide.
        """
        super().__init__()

        self.setName(f'Command Group Template- change me!')

        if level not in [1, 2, 3, 4]:
            raise ValueError("there is no such level!")

        self.container = container

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))
        
        # go to safe position
        self.addCommands(commands2.ParallelCommandGroup(
            MovePivot(container=container, pivot=container.pivot, mode="specified", angle=constants.k_positions["wrist_clearing"]["shoulder_pivot"], wait_to_finish=True, indent=indent+1).withTimeout(5),
            MoveElevator(container=container, elevator=container.elevator, mode="specified", height=constants.k_positions["wrist_clearing"]["elevator"], wait_to_finish=True, indent=indent + 1).withTimeout(5)
        ))

        # have wrist do its thing
        if wrist_setpoint is None:
            self.addCommands(MoveWristByJoystick(container=container, controller=container.co_pilot_command_controller, timeout=10, wait_to_finish=True, indent=indent+1))
        else:
            self.addCommands(MoveWrist(container=container, radians=wrist_setpoint, timeout=5, wait_to_finish=True, indent=indent+1))

        # go to final position
        self.addCommands(MovePivot(container=container, pivot=self.container.pivot, mode="specified", angle=constants.k_positions[f"l{level}"]["shoulder_pivot"], wait_to_finish=True, indent=indent+1).withTimeout(5))
        self.addCommands(MoveElevator(container=container, elevator=container.elevator, mode="specified", height=constants.k_positions[f"l{level}"]["elevator"], wait_to_finish=True, indent=indent+1).withTimeout(5))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

