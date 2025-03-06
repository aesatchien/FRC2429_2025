from enum import Enum
import commands2
from wpilib import SmartDashboard
import constants
from constants import LedConstants

# TODO - do something better than putting a callback in LED


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
        # can I generate this programmatically from the constants file's list of positions? Some of it.
        STOW = {'name': 'stow', 'lit_leds': LedConstants.k_led_count, 'mode': 'keep'}
        GROUND = {'name': 'ground', 'lit_leds': LedConstants.k_led_count, 'mode': 'keep'}
        # coral modes
        L1 = {'name': 'l1', 'lit_leds': -1 + LedConstants.k_led_count // 8, 'mode': 'coral'}
        L3 = {'name': 'l3', 'lit_leds': -3 + 3 * LedConstants.k_led_count // 8, 'mode': 'coral'}
        L4 = {'name': 'l4', 'lit_leds': -4 + LedConstants.k_led_count // 2, 'mode': 'coral'}
        CORAL_STATION = {'name': 'coral station', 'lit_leds': LedConstants.k_led_count, 'mode': 'coral'}
        # algae modes
        PROCESSOR = {'name': 'processor', 'lit_leds': -1 + LedConstants.k_led_count // 8, 'mode': 'algae'}
        BARGE = {'name': 'barge', 'lit_leds': -2 + LedConstants.k_led_count // 2, 'mode': 'algae'}
        ALGAE_LOW = {'name': 'algae low', 'lit_leds': -3 + LedConstants.k_led_count // 4, 'mode': 'algae'}
        ALGAE_HIGH = {'name': 'algae high', 'lit_leds': 3 * LedConstants.k_led_count // 8, 'mode': 'algae'}

        NONE = {'name': 'NONE', 'lit_leds': LedConstants.k_led_count, 'mode': 'none'}

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
        self.counter = constants.RobotStateConstants.k_counter_offset

        self._callbacks = []  # Store functions to notify

        # initialize modes and indicators
        self.target = self.Target.L3
        self.side = self.Side.RIGHT

        self.set_target(self.target)
        self.set_side(self.side)

        # this should auto-update the lists for the dashboard.  you can iterate over enums
        self.targets_dict = {target.value["name"]: target for target in self.Target}
        self.sides_dict = {side.value["name"]: side for side in self.Side}

    # put in a callback so the logic to LED is not circular
    def register_callback(self, callback):
        """ Allow external systems (like LED) to register for updates. """
        self._callbacks.append(callback)

    def _notify_callbacks(self):
        """ Notify all registered callbacks when RobotState updates. """
        for callback in self._callbacks:
            callback(self.target, self.side)

    def set_target(self, target: Target) -> None:
        self.prev_target = self.target
        self.target = target
        self._notify_callbacks()  # Call all registered callbacks
        print(f'Target set to {target.value["name"]} at {self.container.get_enabled_time():.1f}')
        SmartDashboard.putString('_target', self.target.value['name'])

    def get_target(self):
        return self.target

    def set_side(self, side: Side) -> None:
        self.side = side
        self._notify_callbacks()  # Call all registered callbacks
        print(f'Side set to {side.value["name"]} at {self.container.get_enabled_time():.1f}')
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
        if self.counter % 10 == 0:  # Execute every 5 cycles (10Hz update rate)
            pass
        self.counter += 1  # Increment the main counter
