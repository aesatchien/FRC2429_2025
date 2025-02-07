import math
import time
import rev
import wpilib
import commands2

from wpimath.geometry import Pose2d

from pathplannerlib.pathfinders import LocalADStar
from pathplannerlib.pathfinding import Pathfinding
from pathplannerlib.path import PathConstraints
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath

import constants

from subsystems.swerve import Swerve
from subsystems.elevator import Elevator
from subsystems.shoulder import Shoulder
from subsystems.intake import Intake
from subsystems.led import Led
from subsystems.wrist import Wrist

from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
from commands.move_elevator import MoveElevator
from commands.move_shoulder import MoveShoulder
from commands.move_wrist import MoveWrist
from commands.run_intake import RunIntake
from commands.set_leds import SetLEDs

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
        self.wrist = Wrist()
        self.led = Led(self)
        self.intake = Intake()

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
            self.configure_codriver_joystick()
            self.bind_codriver_buttons()
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

    def configure_codriver_joystick(self):

        print("configuring codriver joystick")

        self.co_pilot_command_controller = commands2.button.CommandXboxController(constants.k_co_driver_controller_port)  # 2024 way
        self.co_trigger_left_stick_y = self.co_pilot_command_controller.axisGreaterThan(axis=1, threshold=0.5)
        self.co_trigger_a = self.co_pilot_command_controller.a()  # 2024 way
        self.co_trigger_b = self.co_pilot_command_controller.b()
        self.co_trigger_y = self.co_pilot_command_controller.y()
        self.co_trigger_x = self.co_pilot_command_controller.x()
        self.co_trigger_rb = self.co_pilot_command_controller.rightBumper()
        self.co_trigger_lb = self.co_pilot_command_controller.leftBumper()
        self.co_trigger_r = self.co_pilot_command_controller.povRight()
        self.co_trigger_l = self.co_pilot_command_controller.povLeft()
        self.co_trigger_u = self.co_pilot_command_controller.povUp()
        self.co_trigger_d = self.co_pilot_command_controller.povDown()
        self.co_trigger_l_trigger = self.co_pilot_command_controller.leftTrigger(0.2)
        self.co_trigger_r_trigger = self.co_pilot_command_controller.rightTrigger(0.2)
        self.co_trigger_start = self.co_pilot_command_controller.start()
        self.co_trigger_back = self.co_pilot_command_controller.back()



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

        self.triggerB.onTrue(ResetFieldCentric(container=self, swerve=self.swerve, angle=0))

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


    def bind_codriver_buttons(self):
        print("bidning codriver buttons")

        self.co_trigger_a.onTrue(
                commands2.PrintCommand("moving wrist to 90 degrees").andThen(
                MoveWrist(container=self, wrist=self.wrist, radians=math.radians(90), wait_to_finish=True)
        ))

        self.co_trigger_b.onTrue(
                commands2.PrintCommand("moving wrist to 0 degrees").andThen(
                MoveWrist(container=self, wrist=self.wrist, radians=math.radians(0), wait_to_finish=True)
        ))

        self.co_trigger_x.onTrue(
                commands2.PrintCommand("moving shoulder to 90 degrees").andThen(
                MoveShoulder(container=self, shoulder=self.shoulder, radians=math.radians(90), wait_to_finish=True)
        ))

        self.co_trigger_y.onTrue(
                commands2.PrintCommand("moving shoulder to 0 degrees").andThen(
                MoveShoulder(container=self, shoulder=self.shoulder, radians=math.radians(0), wait_to_finish=True)
        ))

        self.co_trigger_a.onTrue(

        )

        self.co_trigger_lb.onTrue(MoveElevator(container=self, elevator=self.elevator, target=1, wait_to_finish=True))
        self.co_trigger_rb.onTrue(MoveElevator(container=self, elevator=self.elevator, target=0, wait_to_finish=True))

        self.co_trigger_start.onTrue(RunIntake(container=self, intake=self.intake, value=12, control_type=rev.SparkMax.ControlType.kVoltage))
        self.co_trigger_back.onTrue(RunIntake(container=self, intake=self.intake, value=0, control_type=rev.SparkMax.ControlType.kVoltage))

    def bind_keyboard_buttons(self):
        # for convenience, and just in case a controller goes down
        pass

    def get_autonomous_command(self):
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("new patth"))
        # return self.autonomous_chooser.getSelected()
