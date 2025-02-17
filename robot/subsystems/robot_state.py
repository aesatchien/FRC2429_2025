from enum import Enum
import commands2
from wpilib import SmartDashboard
import constants

# TODO -


class RobotState(commands2.Subsystem):
    """ Robot state
        One place to store our current goals (maybe should go into LED?)

        Need to know target (e.g. processor, L3, etc)
        Need to know left or right scoring for coral

        From this you can calculate:
        Target height
        Target angle
        Left or right - Wrist orientation or
    """
    class Target(Enum):
        """ Target class is for showing current goal """
        # can I do this programmatically from the constants file's list of positions
        STOW = {'name': 'stow'}
        GROUND = {'name': 'ground'}
        L1 = {'name': 'l1'}
        L2 = {'name': 'l2'}
        L3 = {'name': 'l3'}
        L4 = {'name': 'l4'}
        CORAL_STATION = {'name': 'coral station'}
        PROCESSOR = {'name': 'processor'}
        BARGE = {'name': 'barge'}
        ALGAE_LOW = {'name': 'algae low'}
        ALGAE_HIGH = {'name': 'algae high'}
        NONE = {'name': 'NONE'}

    class Side(Enum):
        """ Mode class is for showing robot's current scoring mode and is the default during teleop """
        RIGHT = {'name': "LEFT", }
        LEFT = {'name': "RIGHT", }
        NONE = {'name': "NONE", }


    def __init__(self, container):
        super().__init__()
        self.setName('Mode')
        self.container = container  # at the moment LED may want to query other subsystems, but this is not clean
        # try to start all the subsystems on a different count so they don't all do the periodic updates at the same time
        self.counter = 1

        # initialize modes and indicators
        self.target = self.Target.L3
        self.side = self.Side.RIGHT

        self.set_target(self.target)
        self.set_side(self.side)

        # this should auto-update the lists for the dashboard.  you can iterate over enums
        self.targets_dict = {target.value["name"]: target for target in self.Target}
        self.sides_dict = {side.value["name"]: side for side in self.Side}

    def set_target(self, target:Target) -> None:
        self.prev_target = self.target
        self.target = target
        SmartDashboard.putString('_target', self.target.value['name'])

    def get_target(self):
        return self.target

    def set_side(self, side:Side) -> None:
        self.side = side
        SmartDashboard.putString('_side', self.side.value['name'])

    def get_side(self):
        return self.side

    # answer if we are on the left or the right for the wrist
    def is_right(self):
        return self.side == self.Side.RIGHT

    def is_left(self):
        return self.side == self.Side.LEFT

    # return elevator and pivot targets based on current mode
    def get_elevator_goal(self):
        return constants.k_positions[self.target.value['name']]['elevator']

    def get_pivot_goal(self):
        return constants.k_positions[self.target.value['name']]['shoulder_pivot']


    def periodic(self):
        if self.counter % 5 == 0:  # Execute every 5 cycles (10Hz update rate)
            pass
        self.counter += 1  # Increment the main counter
