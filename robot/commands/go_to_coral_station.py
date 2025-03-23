import commands2
import constants
from commands.move_wrist import MoveWrist
from commands.move_pivot import MovePivot
from commands.move_elevator import MoveElevator
from commands.run_intake import RunIntake

class GoToCoralStation(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        """
        note that this turns the intake on for you
        """
        super().__init__()

        self.setName(f'Go to coral station')
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))
        
        self.addCommands(commands2.ParallelCommandGroup(
            RunIntake(container=container, intake=container.intake, value=constants.IntakeConstants.k_coral_intaking_voltage, stop_on_end=False),
            commands2.WaitCommand(0.25).andThen(MoveWrist(container=container, radians=constants.k_positions["coral station"]["wrist_pivot"], timeout=5, wait_to_finish=True, indent=indent+1).withTimeout(5)),
            MovePivot(container=container, pivot=container.pivot, mode='specified', angle=constants.k_positions["coral station"]["shoulder_pivot"], wait_to_finish=True, indent=indent+1).withTimeout(5),
            MoveElevator(container=container, elevator=self.container.elevator, mode="specified", height=constants.k_positions["coral station"]["elevator"], wait_to_finish=True, indent=indent + 1).withTimeout(5)
                
        ))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

