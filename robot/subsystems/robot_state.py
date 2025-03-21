import math
from enum import Enum
import commands2
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
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

    class ReefGoal(Enum):
        """Class for storing position information"""
        # todo - see if i can get the offset in here, because it's not the same y offset for left and right
        AB = {'name': 'ab', 'left_pose': constants.k_useful_robot_poses_blue['a'], 'right_pose': constants.k_useful_robot_poses_blue['b'],
              'lr_flip': False}
        CD = {'name': 'cd', 'left_pose': constants.k_useful_robot_poses_blue['c'], 'right_pose': constants.k_useful_robot_poses_blue['d'],
              'lr_flip': False}
        EF = {'name': 'ef', 'left_pose': constants.k_useful_robot_poses_blue['f'], 'right_pose': constants.k_useful_robot_poses_blue['e'],
              'lr_flip': True}
        GH = {'name': 'gh', 'left_pose': constants.k_useful_robot_poses_blue['h'], 'right_pose': constants.k_useful_robot_poses_blue['g'],
              'lr_flip': True}
        IJ = {'name': 'ij', 'left_pose': constants.k_useful_robot_poses_blue['j'], 'right_pose': constants.k_useful_robot_poses_blue['i'],
              'lr_flip': True}
        KL = {'name': 'kl', 'left_pose': constants.k_useful_robot_poses_blue['k'], 'right_pose': constants.k_useful_robot_poses_blue['l'],
              'lr_flip': False}


    class Target(Enum):
        """ Target class is for showing current goal """
        # can I generate this programmatically from the constants file's list of positions? Some of it.
        STOW = {'name': 'stow', 'lit_leds': LedConstants.k_led_count, 'mode': 'keep'}
        GROUND = {'name': 'ground', 'lit_leds': LedConstants.k_led_count, 'mode': 'keep'}
        # coral modes
        L1 = {'name': 'l1', 'lit_leds': -1 + LedConstants.k_led_count // 8, 'mode': 'coral'}
        L2 = {'name': 'l2', 'lit_leds': -2 + 1 * LedConstants.k_led_count // 4, 'mode': 'coral'}
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
        RIGHT = {'name': "RIGHT", }
        LEFT = {'name': "LEFT", }
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
        self.set_target(self.target)

        # try to set the side based on the joystick in RobotContainer and override this
        print('Temporary robot state until joysticks initialized...')
        self.side = self.Side.RIGHT
        self.set_side(self.side)

        self.reef_goal = self.ReefGoal.AB
        self.set_reef_goal(self.reef_goal)

        # this should auto-update the lists for the dashboard.  you can iterate over enums
        self.targets_dict = {target.value["name"]: target for target in self.Target}
        self.sides_dict = {side.value["name"]: side for side in self.Side}
        self.reef_goal_dict = {reef_goal.value["name"]: reef_goal for reef_goal in self.ReefGoal}

    # put in a callback so the logic to LED is not circular
    def register_callback(self, callback):
        """ Allow external systems (like LED) to register for updates. """
        self._callbacks.append(callback)

    def _notify_callbacks(self):
        """ Notify all registered callbacks when RobotState updates. """
        for callback in self._callbacks:
            callback(self.target, self.side)

    def set_reef_goal(self, reef_goal: ReefGoal) -> None:
        self.reef_goal = reef_goal
        # self._notify_callbacks()  # Call all registered callbacks
        print(f'ReefGoal set to {self.reef_goal.value["name"]} at {self.container.get_enabled_time():.1f}s')
        SmartDashboard.putString('_reef_goal', self.reef_goal.value['name'])

    def set_reef_goal_cmd(self, reef_goal: ReefGoal) -> commands2.InstantCommand:
        return commands2.InstantCommand(lambda: self.set_reef_goal(reef_goal)).ignoringDisable(True)

    def get_reef_goal(self):
        return self.reef_goal

    def get_reef_goal_pose(self):
        if self.is_right():
            return self.reef_goal.value['right_pose']
        else:
            return self.reef_goal.value['left_pose']

    def set_target(self, target: Target) -> None:
        self.prev_target = self.target
        self.target = target
        self._notify_callbacks()  # Call all registered callbacks
        print(f'Target set to {target.value["name"]} at {self.container.get_enabled_time():.1f}s')
        SmartDashboard.putString('_target', self.target.value['name'])

    def get_target(self):
        return self.target

    def set_side(self, side: Side) -> None:
        self.side = side
        self._notify_callbacks()  # Call all registered callbacks
        print(f'Side set to {side.value["name"]} at {self.container.get_enabled_time():.1f}s')
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

    def is_point_between_angles(self, x, y, a, b):
        """
        Determines if the point (x, y) is between angles a and b.
        
        Parameters:
          x, y: Coordinates of the point relative to the origin.
          a, b: Boundary angles in degrees (they can be in any order).
          
        Returns:
          True if the point's polar angle is within the arc defined by a to b,
          handling wrap-around at 360 degrees; otherwise, False.
        """
        # Compute the point's angle in degrees
        point_angle = math.degrees(math.atan2(y, x)) % 360
        
        # Normalize the boundary angles
        a = a % 360
        b = b % 360
    
        if a <= b:
            return a <= point_angle <= b
        else:
            # Arc spans the 0° boundary.
            return point_angle >= a or point_angle <= b
    
    def closest_hex_side(self, robot_x, robot_y, center_x, center_y):
        """
        Determines which side ('ab', 'cd', 'ef', 'gh', 'ij', or 'kl')
        of the hexagon the robot is closest to, based on its angle relative
        to the hexagon's center.
    
        The hexagon is assumed to have its sides arranged such that:
          - 'ab' covers 30° to 90°
          - 'cd' covers 90° to 150°
          - 'ef' covers 150° to 210°
          - 'gh' covers 210° to 270°
          - 'ij' covers 270° to 330°
          - 'kl' covers 330° to 30° (wrap-around)
    
        Parameters:
          robot_x, robot_y: The robot's coordinates.
          center_x, center_y: The hexagon center coordinates.
    
        Returns:
          The label of the hexagon side as a string.
        """
        # Translate the robot's position relative to the hexagon center.
        rel_x = robot_x - center_x
        rel_y = robot_y - center_y
    
        if self.is_point_between_angles(rel_x, rel_y, 30, 90):
            return "ab"
        elif self.is_point_between_angles(rel_x, rel_y, 90, 150):
            return "cd"
        elif self.is_point_between_angles(rel_x, rel_y, 150, 210):
            return "ef"
        elif self.is_point_between_angles(rel_x, rel_y, 210, 270):
            return "gh"
        elif self.is_point_between_angles(rel_x, rel_y, 270, 330):
            return "ij"
        elif self.is_point_between_angles(rel_x, rel_y, 330, 30):
            return "kl"
        else:
            # This case should not occur if all angular sectors are covered.
            return "unknown"

    def periodic(self):
        if self.counter % 10 == 0:  # Execute every 5 cycles (10Hz update rate)
            pass
        self.counter += 1  # Increment the main counter
