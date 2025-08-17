import time, enum
import wpilib
import commands2
import commands2.cmd as cmd
from commands2.button import JoystickButton, POVButton

import constants  # all of the constants except for swerve

from subsystems.drivetrain import Drivetrain
from subsystems.led import Led
from commands.move_robot import MoveRobot
from commands.auto_move_forward import AutoMoveForward
from commands.led_indicator_toggle import LedIndicatorToggle
from commands.led_mode_toggle import LedModeToggle
from commands.led_loop import LedLoop

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        self.start_time = time.time()

        # The robot's subsystems
        self.drive = Drivetrain(container=self)
        self.led = Led()
        self.drive.setDefaultCommand(MoveRobot(container=self, drive=self.drive))
        self.joystick = wpilib.Joystick(0)
        self.buttonA = JoystickButton(self.joystick, 1)
        self.buttonA.onTrue(AutoMoveForward(container=self, drive=self.drive, joystick=self.joystick))
        self.buttonB = JoystickButton(self.joystick, 2)
        self.buttonB.onTrue(LedIndicatorToggle(container=self))
        self.buttonY = JoystickButton(self.joystick, 4)
        self.buttonY.onTrue(LedModeToggle(container=self))

    def set_start_time(self):  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def get_enabled_time(self):  # call when we want to know the start/elapsed time for status and debug messages
        return time.time() - self.start_time
