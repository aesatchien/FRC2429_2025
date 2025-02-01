from dataclasses import field
import math
import time, enum
from pathplannerlib.pathfinders import LocalADStar
from pathplannerlib.pathfinding import Pathfinding
from pathplannerlib.path import PathConstraints
import wpilib
import commands2
from commands2.button import Trigger
from wpimath.geometry import Pose2d
from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
import constants

from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.path import PathPlannerPath

# from subsystems.lower_crank import LowerCrank

# from commands.move_lower_arm_by_network_tables import MoveLowerArmByNetworkTables
from commands.increment_elevator_and_pivot import IncrementElevatorAndPivot
from commands.set_leds import SetLEDs

from subsystems.swerve import Swerve
from subsystems.elevator import Elevator
from subsystems.shoulder import Shoulder
from subsystems.intake import Intake
from subsystems.led import Led

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
        # self.lower_crank = LowerCrank(container=self) # I don't want to test without a sim yet
        self.swerve = Swerve()
        self.elevator = Elevator()
        self.shoulder = Shoulder()
        self.led = Led(self)
        self.intake = Intake()

        self.configure_joysticks()
        self.bind_driver_buttons()
        self.swerve.setDefaultCommand(DriveByJoystickSwerve(
            container=self,
            swerve=self.swerve,
            controller=self.driver_command_controller,
            field_oriented=False,
            rate_limited=constants.k_swerve_rate_limited
        ))

        if not constants.k_swerve_only:
            self.bind_operator_buttons()
            self.bind_keyboard_buttons()

        # self.configure_swerve_bindings()
        
        self.initialize_dashboard()

        Pathfinding.setPathfinder(LocalADStar())

        # swerve driving

        # initialize the turret
        # commands2.ScheduleCommand(TurretInitialize(container=self, turret=self.turret, samples=50)).initialize()
        # testing

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
        self.driver_command_controller = commands2.button.CommandXboxController(constants.k_driver_controller_port)
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

        self.copilot_controller = commands2.button.CommandXboxController(1)
        self.copilot_controller = commands2.button.CommandXboxController(1) 

        # co-pilot controller
        """
        self.co_driver_controller = wpilib.XboxController(constants.k_co_driver_controller_port)
        self.co_buttonA = JoystickButton(self.co_driver_controller, 1)
        self.co_buttonB = JoystickButton(self.co_driver_controller, 2)
        self.co_buttonX = JoystickButton(self.co_driver_controller, 3)
        self.co_buttonY = JoystickButton(self.co_driver_controller, 4)
        self.co_buttonLB = JoystickButton(self.co_driver_controller, 5)
        self.co_buttonRB = JoystickButton(self.co_driver_controller, 6)
        self.co_buttonBack = JoystickButton(self.co_driver_controller, 7)
        self.co_buttonStart = JoystickButton(self.co_driver_controller, 8)
        self.co_buttonUp = POVButton(self.co_driver_controller, 0)
        self.co_buttonDown = POVButton(self.co_driver_controller, 180)
        self.co_buttonLeft = POVButton(self.co_driver_controller, 270)
        self.co_buttonRight = POVButton(self.co_driver_controller, 90)
        self.co_buttonLeftAxis = AxisButton(self.co_driver_controller, 2)
        self.co_buttonRightAxis = AxisButton(self.co_driver_controller, 3)
        """

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

    def bind_driver_buttons(self):
        self.triggerX.whileTrue(AutoBuilder.followPath(PathPlannerPath.fromPathFile("new patth")))
        self.triggerX.onTrue(commands2.PrintCommand("starting pathplanner auto"))
        self.triggerX.onFalse(commands2.PrintCommand("ending pathplanner auto"))

        pathfinding_constraints = PathConstraints(
                maxVelocityMps=0.5,
                maxAccelerationMpsSq=3,
                maxAngularVelocityRps=math.radians(90),
                maxAngularAccelerationRpsSq=math.degrees(720),
                nominalVoltage=12
        )

        self.triggerY.whileTrue(
                AutoBuilder.pathfindToPoseFlipped(
                    pose=Pose2d(15, 4, 0),
                    constraints=pathfinding_constraints
                )
        )

        self.triggerA.onTrue(IncrementElevatorAndPivot(container=self, elevator=self.elevator, pivot=self.shoulder, direction="up"))
        self.triggerB.onTrue(IncrementElevatorAndPivot(container=self, elevator=self.elevator, pivot=self.shoulder, direction="down"))

        pass

    def bind_operator_buttons(self):
        pass

    def bind_keyboard_buttons(self):
        # for convenience, and just in case a controller goes down
        pass

    def get_autonomous_command(self):
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("new patth"))
        # return self.autonomous_chooser.getSelected()
