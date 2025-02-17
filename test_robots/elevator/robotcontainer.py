from enum import Enum
import math
import time
import rev
import wpilib
import commands2


#import constants

from subsystems.elevator import Elevator
from subsystems.pivot import Pivot
from subsystems.led import Led
from subsystems.robot_state import RobotState

from commands.set_leds import SetLEDs
from commands.move_elevator import MoveElevator
from commands.move_pivot import MovePivot
from commands.sequential_scoring import SequentialScoring


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
        self.elevator = Elevator()
        self.pivot = Pivot()
        self.led = Led(container=self)
        self.robot_state = RobotState(container=self)

        # configure the controllers
        self.configure_joysticks()
        self.bind_driver_buttons()


        self.initialize_dashboard()

    def set_start_time(self):  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def get_enabled_time(self):  # call when we want to know the start/elapsed time for status and debug messages
        return time.time() - self.start_time

    def configure_joysticks(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # The driver's controller
        self.driver_command_controller = commands2.button.CommandXboxController(0)
        self.triggerA = self.driver_command_controller.a()
        self.triggerB = self.driver_command_controller.b()
        self.triggerX = self.driver_command_controller.x()
        self.triggerY = self.driver_command_controller.y()
        self.triggerLB = self.driver_command_controller.leftBumper()
        self.triggerRB = self.driver_command_controller.rightBumper()
        self.triggerBack = self.driver_command_controller.back()
        self.triggerStart = self.driver_command_controller.start()
        self.triggerUp = self.driver_command_controller.povUp()
        self.triggerDown = self.driver_command_controller.povDown()
        self.triggerLeft = self.driver_command_controller.povLeft()
        self.triggerRight = self.driver_command_controller.povRight()

        # self.copilot_controller = commands2.button.CommandXboxController(1)

    def bind_driver_buttons(self):
        self.triggerA.onTrue(commands2.cmd.runOnce(lambda: self.robot_state.set_target(self.robot_state.Target.L3)))
        self.triggerB.onTrue(commands2.cmd.runOnce(lambda: self.robot_state.set_target(self.robot_state.Target.L2)))

    def initialize_dashboard(self):
        # wpilib.SmartDashboard.putData(MoveLowerArmByNetworkTables(container=self, crank=self.lower_crank))
        # lots of putdatas for testing on the dash
        self.led_mode_chooser = wpilib.SendableChooser()
        [self.led_mode_chooser.addOption(key, value) for key, value in self.led.modes_dict.items()]  # add all the indicators
        self.led_mode_chooser.onChange(listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
            SetLEDs(container=self, led=self.led, mode=selected_value)))
        wpilib.SmartDashboard.putData('LED Mode', self.led_mode_chooser)
        self.led_indicator_chooser = wpilib.SendableChooser()
        [self.led_indicator_chooser.addOption(key, value) for key, value in self.led.indicators_dict.items()]  # add all the indicators
        self.led_indicator_chooser.onChange(listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
            SetLEDs(container=self, led=self.led, indicator=selected_value)))
        wpilib.SmartDashboard.putData('LED Indicator', self.led_indicator_chooser)

        wpilib.SmartDashboard.putData('SetSuccess', SetLEDs(container=self, led=self.led, indicator=Led.Indicator.kSUCCESS))
        wpilib.SmartDashboard.putData('MoveElevator', MoveElevator(container=self, elevator=self.elevator))
        wpilib.SmartDashboard.putData('MovePivot', MovePivot(container=self, pivot=self.pivot))
        wpilib.SmartDashboard.putData('SequentialScore', SequentialScoring(container=self))

        # quick way to test all scoring positions from dashboard
        self.score_test_chooser = wpilib.SendableChooser()
        [self.score_test_chooser.addOption(key, value) for key, value in self.robot_state.targets_dict.items()]  # add all the indicators
        self.score_test_chooser.onChange(
            listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
                commands2.cmd.runOnce(lambda: self.robot_state.set_target(target=selected_value))))
        wpilib.SmartDashboard.putData('Robot Scoring Mode', self.score_test_chooser)

    def get_autonomous_command(self):
        pass
        # return AutoBuilder.followPath(PathPlannerPath.fromPathFile("new patth"))
        # return self.autonomous_chooser.getSelected()
