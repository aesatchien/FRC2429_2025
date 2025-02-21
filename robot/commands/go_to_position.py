import math
import commands2
from commands.move_pivot import MovePivot
from commands.move_wrist import MoveWrist
import constants

from commands.move_elevator import MoveElevator

class GoToPosition(commands2.SequentialCommandGroup):
    def __init__(self, container, position: str, indent=0) -> None:
        super().__init__()

        self.setName(f'Go to position (with position {position})')
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} to {position} **"))
        self.container = container


        if position not in constants.k_positions.keys():
            raise ValueError("Unrecognized position requested while making GoToPosition!")

        if (constants.k_positions[position]["shoulder_pivot"] < constants.WristConstants.k_max_arm_angle_where_spinning_dangerous
            and constants.k_positions[position]["shoulder_pivot"] > constants.WristConstants.k_min_arm_angle_where_spinning_dangerous): # we are telling the shoulder to go to a stow position

            if (constants.k_positions[position]["wrist_pivot"] > constants.WristConstants.k_stowed_max_angle or # we are telling the wrist to go to a non-stow position
                constants.k_positions[position]["wrist_pivot"] < constants.WristConstants.k_stowed_min_angle):
                raise ValueError("Telling wrist and shoulder to go to illegal positions!")
            
            else: # we are telling the wrist to go to a stow position
                self.addCommands(MoveWrist(container=container, radians=constants.k_positions[position]["wrist_pivot"], timeout=4, wait_to_finish=True, indent=indent+1))


        if position in ["l1", "l2", "l3", "l4", "algae low", "algae high", "stow"]: # stow is in here because we often stow right after scoring
            # retract till it's safe to move elevator
            self.addCommands(commands2.ConditionalCommand(
                onTrue=MovePivot(container=container, pivot=self.container.pivot, mode="specified", angle=math.radians(90), wait_to_finish=True, indent=indent+1).withTimeout(5),
                onFalse=commands2.WaitCommand(0),
                condition=lambda: container.pivot.get_angle() < constants.ShoulderConstants.k_min_angle_to_elevate_in_front_of_reef
            ))

            # do these seperately because we don't want to hit reef till we extend pivot
            self.addCommands(MoveElevator(container=container, elevator=self.container.elevator, mode="specified", height=constants.k_positions[position]["elevator"], wait_to_finish=True, indent=indent + 1).withTimeout(5))

            self.addCommands(MovePivot(container=container, pivot=self.container.pivot, mode="specified", angle=constants.k_positions[position]["shoulder_pivot"], wait_to_finish=True, indent=indent+1).withTimeout(5))

        else:
            # safe to do these in parallel
            self.addCommands(commands2.ParallelCommandGroup(
                MovePivot(container=container, pivot=container.pivot, mode='specified', angle=constants.k_positions[position]["shoulder_pivot"], wait_to_finish=True, indent=indent+1).withTimeout(5),
                MoveElevator(container=container, elevator=self.container.elevator, mode="specified", height=constants.k_positions[position]["elevator"], wait_to_finish=True, indent=indent + 1).withTimeout(5)
            ))

        if (constants.k_positions[position]["shoulder_pivot"] > constants.WristConstants.k_max_arm_angle_where_spinning_dangerous
            or constants.k_positions[position]["shoulder_pivot"] < constants.WristConstants.k_min_arm_angle_where_spinning_dangerous):
            # the arm is going to a deployed position, so we should move the wrist last.
            if position not in ["l1", "l2", "l3", "l4"]: # for corals we should let driver choose side
                self.addCommands(MoveWrist(container=container, radians=constants.k_positions[position]["wrist_pivot"], timeout=4, wait_to_finish=True, indent=indent+1))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} to {position} **"))


