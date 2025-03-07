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
        if you give it level=1, it will ignore wrist_setpoint_decider and just go to the wrist target specified in constants.py
        because there's only 1 (well actually 2 but idk if we need the second one) valid wrist position for l1
        """

        super().__init__()
        self.setName(f'Go to reef position')
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))
        
        if level not in [1, 2, 3, 4]:
            raise ValueError("there is no such level!")

        # have wrist do its thing
        if level != 1:
            if type(wrist_setpoint_decider) == float:
                self.addCommands(MoveWrist(container=container, radians=wrist_setpoint_decider, timeout=5, wait_to_finish=True, indent=indent+1))
            else:
                self.addCommands(MoveWristByJoystick(container=container, side_decider=wrist_setpoint_decider, swerve_for_field_centric=container.swerve, timeout=10, wait_to_finish=True, indent=indent+1))

        else:
            self.addCommands(MoveWrist(container, constants.k_positions["l1"]["wrist_pivot"], 5, False, True, indent=indent+1))

        # go to final position
        self.addCommands(commands2.ParallelCommandGroup(
            MoveElevator(container=container, elevator=container.elevator, mode="specified", height=constants.k_positions[f"l{level}"]["elevator"], wait_to_finish=True, indent=indent+1).withTimeout(5),
            MovePivot(container=container, pivot=self.container.pivot, mode="specified", angle=constants.k_positions[f"l{level}"]["shoulder_pivot"], wait_to_finish=True, indent=indent+1).withTimeout(5),
            # commands2.WaitCommand(0.2).andThen(MoveWrist(container, math.radians(90), timeout=5, indent=indent+1))
        ))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

