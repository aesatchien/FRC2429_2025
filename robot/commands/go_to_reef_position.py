import math
import commands2
from commands2.button import CommandXboxController

from commands.move_pivot import MovePivot
from commands.move_elevator import MoveElevator
from commands.move_wrist import MoveWrist
from commands.move_wrist_by_joystick import MoveWristByJoystick
import constants
from subsystems.robot_state import RobotState

class GoToReefPosition(commands2.SequentialCommandGroup):
    def __init__(self, container, level: int, wrist_setpoint_decider: float | CommandXboxController | RobotState, indent=0) -> None:
        """
        :param wrist_setpoint: the setpoint for the wrist. leave it as none for driver to decide.
        """
        super().__init__()

        self.setName(f'Command Group Template- change me!')

        if level not in [1, 2, 3, 4]:
            raise ValueError("there is no such level!")

        self.container = container

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))
        
        # go to safe position
        # self.addCommands(commands2.ParallelCommandGroup(
        #     MovePivot(container=container, pivot=container.pivot, mode="specified", angle=constants.k_positions[f"l{level}_wrist_clearing"]["shoulder_pivot"], wait_to_finish=True, indent=indent+1).withTimeout(5),
        #     MoveElevator(container=container, elevator=container.elevator, mode="specified", height=constants.k_positions[f"l{level}_wrist_clearing"]["elevator"], wait_to_finish=True, indent=indent + 1).withTimeout(5)
        # ))

        # have wrist do its thing
        if type(wrist_setpoint_decider) == float:
            self.addCommands(MoveWrist(container=container, radians=wrist_setpoint_decider, timeout=5, wait_to_finish=True, indent=indent+1))
        else:
            self.addCommands(MoveWristByJoystick(container=container, side_decider=wrist_setpoint_decider, swerve_for_field_centric=container.swerve, timeout=10, wait_to_finish=True, indent=indent+1))


        # go to final position
        self.addCommands(commands2.ParallelCommandGroup(
            MoveElevator(container=container, elevator=container.elevator, mode="specified", height=constants.k_positions[f"l{level}"]["elevator"], wait_to_finish=True, indent=indent+1).withTimeout(5),
            MovePivot(container=container, pivot=self.container.pivot, mode="specified", angle=constants.k_positions[f"l{level}"]["shoulder_pivot"], wait_to_finish=True, indent=indent+1).withTimeout(5),
            # commands2.WaitCommand(0.2).andThen(MoveWrist(container, math.radians(90), timeout=5, indent=indent+1))
        ))
        # self.addCommands(MoveWrist(container, math.radians(90), timeout=5, indent=indent+1))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

