import math
import commands2
from commands2.button import CommandXboxController
from commands2.waitcommand import WaitCommand

from commands.move_pivot import MovePivot
from commands.move_elevator import MoveElevator
from commands.move_wrist import MoveWrist
from commands.move_wrist_by_joystick import MoveWristByJoystick
import constants
from subsystems.robot_state import RobotState

class GoToReefPosition(commands2.SequentialCommandGroup):
    def __init__(self, container, level: int, wrist_setpoint_decider: float | CommandXboxController | RobotState, indent=0) -> None:
        """
        if you give it level=1, it will ignore wrist_setpoint_decider and just go to the wrist target specified in constants.py
        because there's only 1 (well actually 2 but idk if we need the second one) valid wrist position for l1
        fun fact, it actually never uses wrist setpoint decider fun fact
        """

        super().__init__()
        self.setName(f'Go to reef position')
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))
        
        if level not in [1, 2, 3, 4]:
            raise ValueError("there is no such level!")

        # go to final position
        if level == 1:
            self.addCommands(commands2.ParallelCommandGroup(
                WaitCommand(0.5).andThen(MoveWrist(container, constants.k_positions["l1"]["wrist_pivot"], 5, False, False, indent=indent+1).withTimeout(2)),
                WaitCommand(min(0, level - 2) * 0.5).andThen(MoveElevator(container=container, elevator=container.elevator, mode="specified", height=constants.k_positions[f"l{level}"]["elevator"], wait_to_finish=True, indent=indent+1)).withTimeout(5),
                MovePivot(container=container, pivot=self.container.pivot, mode="specified", angle=constants.k_positions[f"l{level}"]["shoulder_pivot"], wait_to_finish=True, indent=indent+1).withTimeout(5),
            ))

        else:
            self.addCommands(commands2.ParallelCommandGroup(
                # MoveWrist(container, math.radians(90), 5, False, False, indent=indent+1), got rid of this LHACK 3/15/25 because we want to run with whatever we stowd with so we can swap in stow
                WaitCommand(min(0, level - 2) * 0.5).andThen(MoveElevator(container=container, elevator=container.elevator, mode="specified", height=constants.k_positions[f"l{level}"]["elevator"], wait_to_finish=True, indent=indent+1)).withTimeout(5),
                MovePivot(container=container, pivot=self.container.pivot, mode="specified", angle=constants.k_positions[f"l{level}"]["shoulder_pivot"], wait_to_finish=True, indent=indent+1).withTimeout(5),
            ))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

