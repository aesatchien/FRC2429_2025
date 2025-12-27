# 2429 FRC code for 2025 season - Reefscape


import wpilib
from wpimath.geometry import Pose2d
import commands2
from commands2.printcommand import PrintCommand

# pathplanner stuff
from pathplannerlib.pathfinders import LocalADStar
from pathplannerlib.pathfinding import Pathfinding
from pathplannerlib.auto import AutoBuilder, NamedCommands

# 2429 helper files
import constants

# 2429 subsystems
from subsystems import swerve_constants
from subsystems.led import Led
from subsystems.quest import Questnav
from subsystems.robot_state import RobotState
from subsystems.swerve import Swerve
from subsystems.vision import Vision
# from subsystems.questnav_2429 import QuestnavModule

# 2429 "auto" commands - just an organizational division of commands

# 2429 commands
from commands.auto_to_pose import AutoToPose
from commands.auto_strafe_to_tag import AutoStrafeToTag
from commands.drive_by_distance_swerve import DriveByVelocitySwerve
from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
from commands.pid_to_point import PIDToPoint
from commands.reset_field_centric import ResetFieldCentric
from commands.rumble_command import RumbleCommand
from commands.set_leds import SetLEDs
# from commands.swerve_test import SwerveTest


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
    little robot logic should actually be handled in the :class:`.Robot` periodic methods (other than the scheduler
    calls). Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.timer = wpilib.Timer()
        self.timer.start()

        # The robot's subsystems
        self.questnav = Questnav()  # going to break the silo convention and let the Swerve see the quest for now
        self.swerve = Swerve(questnav=self.questnav)
        self.vision = Vision()
        self.robot_state = RobotState()  # currently has a callback that LED can register
        self.led = Led(robot_state=self.robot_state)  # may want LED last because it may want to know about other systems
        # self.questnav_2429 = QuestnavModule()

        self.configure_joysticks()
        self.bind_driver_buttons()


        self.swerve.setDefaultCommand(DriveByJoystickSwerve(
            container=self,
            swerve=self.swerve,
            controller=self.driver_command_controller,
            # field_oriented=False,
            rate_limited=constants.k_swerve_rate_limited
        ))

        if not constants.k_swerve_only:
            pass

        self.register_commands()

        self.initialize_dashboard()

        Pathfinding.setPathfinder(LocalADStar())


    def configure_joysticks(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # The driver's controller
        self.driver_command_controller = commands2.button.CommandXboxController(constants.k_driver_controller_port)
        self.triggerA = self.driver_command_controller.a()
        self.triggerB = self.driver_command_controller.b()
        self.triggerX = self.driver_command_controller.x()
        self.triggerY = self.driver_command_controller.y()
        self.triggerLB = self.driver_command_controller.leftBumper()
        self.triggerRB = self.driver_command_controller.rightBumper()
        self.trigger_L_trigger = self.driver_command_controller.leftTrigger(0.5)
        self.triggerBack = self.driver_command_controller.back()
        self.triggerStart = self.driver_command_controller.start()
        self.triggerUp = self.driver_command_controller.povUp()
        self.triggerDown = self.driver_command_controller.povDown()
        self.triggerLeft = self.driver_command_controller.povLeft()
        self.triggerRight = self.driver_command_controller.povRight()

        self.copilot_controller = commands2.button.CommandXboxController(1) 

    def configure_codriver_joystick(self):

        print("Configuring codriver joystick")

        self.co_trigger_a = self.copilot_controller.a()  # 2024 way
        self.co_trigger_b = self.copilot_controller.b()
        self.co_trigger_y = self.copilot_controller.y()
        self.co_trigger_x = self.copilot_controller.x()
        self.co_trigger_rb = self.copilot_controller.rightBumper()
        self.co_trigger_lb = self.copilot_controller.leftBumper()
        self.co_trigger_r = self.copilot_controller.povRight()
        self.co_trigger_l = self.copilot_controller.povLeft()
        self.co_trigger_u = self.copilot_controller.povUp()
        self.co_trigger_d = self.copilot_controller.povDown()

        self.co_trigger_l_trigger = self.copilot_controller.leftTrigger(0.2)
        self.co_trigger_r_trigger = self.copilot_controller.rightTrigger(0.2)
        self.co_trigger_start = self.copilot_controller.start()
        self.co_trigger_back = self.copilot_controller.back()

        self.co_trigger_r_stick_positive_x = self.copilot_controller.axisGreaterThan(4, 0.5)
        self.co_trigger_r_stick_negative_x = self.copilot_controller.axisLessThan(4, -0.5)
        self.co_trigger_r_stick_positive_y = self.copilot_controller.axisGreaterThan(5, 0.5)
        self.co_trigger_r_stick_negative_y = self.copilot_controller.axisLessThan(5, -0.5)


    def initialize_dashboard(self):
        # lots of putdatas for testing on the dash
        # COMMANDS FOR GUI (ROBOT DEBUGGING) - 20250224 CJH

        # --------------   TESTING LEDS ----------------
        self.led_mode_chooser = wpilib.SendableChooser()
        [self.led_mode_chooser.addOption(key, value) for key, value in self.led.modes_dict.items()]  # add all the indicators
        self.led_mode_chooser.onChange(listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(SetLEDs(container=self, led=self.led, mode=selected_value)))
        wpilib.SmartDashboard.putData('LED Mode', self.led_mode_chooser)

        self.led_indicator_chooser = wpilib.SendableChooser()
        [self.led_indicator_chooser.addOption(key, value) for key, value in self.led.indicators_dict.items()]  # add all the indicators
        self.led_indicator_chooser.onChange(listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
            SetLEDs(container=self, led=self.led, indicator=selected_value)))
        wpilib.SmartDashboard.putData('LED Indicator', self.led_indicator_chooser)


        # experimental, not used on dash
        wpilib.SmartDashboard.putData('SetSuccess', SetLEDs(container=self, led=self.led, indicator=Led.Indicator.kSUCCESS))

        # commands for pyqt dashboard - please do not remove

        # end pyqt dashboard section

        # quick way to test all scoring positions from dashboard
        self.score_test_chooser = wpilib.SendableChooser()
        [self.score_test_chooser.addOption(key, value) for key, value in self.robot_state.targets_dict.items()]  # add all the indicators
        self.score_test_chooser.onChange(
            listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
                commands2.cmd.runOnce(lambda: self.robot_state.set_target(target=selected_value))))
        wpilib.SmartDashboard.putData('RobotScoringMode', self.score_test_chooser)

        # self.auto_chooser = AutoBuilder.buildAutoChooser('')  # this loops through the path planner deploy directory
        self.auto_chooser = wpilib.SendableChooser()  #  use this if you don't have any pathplanner autos defined
        self.auto_chooser.setDefaultOption('1:  Wait *CODE*', PrintCommand("** Running wait auto **").andThen(commands2.WaitCommand(15)))
        self.auto_chooser.addOption('2a: Drive 2s Straight *CODE*', PrintCommand("** Running drive by velocity swerve leave auto **").andThen(DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2)))
        self.auto_chooser.addOption('2b: Drive 2s To Driver Station *CODE*', PrintCommand("** Running drive by velocity swerve leave auto **").andThen(DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2.5, field_relative=True)))
        wpilib.SmartDashboard.putData('autonomous routines', self.auto_chooser)  #


    def bind_driver_buttons(self):
        print("Binding driver buttons")
        # move this to the top of the button area so we have a left and a right auto-drive
        self.triggerY.onTrue(ResetFieldCentric(container=self, swerve=self.swerve, angle=0))

        # test a setting of the swerve modules straight before running the auto to tag
        # self.triggerA.whileTrue(commands2.cmd.run(lambda: self.swerve.set_straight(), self.swerve))
        # self.triggerA.whileTrue(SwerveTest(self, self.swerve))

        # giving AJ buttons to hold for driving to a goal on the left and on the right
        self.triggerB.onTrue(commands2.cmd.runOnce(lambda: self.robot_state.set_side(side=RobotState.Side.RIGHT)))
        self.triggerX.onTrue(commands2.cmd.runOnce(lambda: self.robot_state.set_side(side=RobotState.Side.LEFT)))

        #self.triggerB.debounce(0.1).whileTrue(AutoStrafeToTag(container=self, swerve=self.swerve, hug_reef=False, location='right'))
        #self.triggerX.debounce(0.1).whileTrue(AutoStrafeToTag(container=self, swerve=self.swerve, hug_reef=True, location='left'))
        self.triggerB.debounce(0.1).whileTrue(AutoToPose(self, self.swerve, target_pose=None, nearest=True, from_robot_state=True,control_type='not_pathplanner'))
        self.triggerX.debounce(0.1).whileTrue(AutoToPose(self, self.swerve, target_pose=None, nearest=True, from_robot_state=True, control_type='not_pathplanner'))

        # set up dpad to allow slow, smooth robot-centric alignment
        dpad_output = 0.125
        self.triggerUp.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(dpad_output, 0, 0), timeout=10))
        self.triggerDown.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(-dpad_output, 0, 0), timeout=10))
        self.triggerLeft.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(0, dpad_output, 0), timeout=10))
        self.triggerRight.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(0, -dpad_output, 0), timeout=10))

        self.triggerLB.onTrue(RumbleCommand(container=self, rumble_amount=0.95, left_rumble=True, right_rumble=False, rumble_time=0.5))
        self.triggerRB.onTrue(RumbleCommand(container=self, rumble_amount=0.95, left_rumble=False, right_rumble=True, rumble_time=0.5))

    def bind_codriver_buttons(self):
        print("Binding codriver buttons")


    def register_commands(self):
        # this is for PathPlanner, so it can call our commands
        NamedCommands.registerCommand('robot state left', commands2.cmd.runOnce(lambda: self.robot_state.set_side(side=RobotState.Side.RIGHT)).ignoringDisable(True))


    def get_autonomous_command(self):
        # return DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2)
        return self.auto_chooser.getSelected()



