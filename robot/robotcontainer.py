from enum import Enum
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

from commands.drive_by_distance_swerve import DriveByVelocitySwerve
from commands.sequential_scoring import SequentialScoring
import constants

from subsystems.robot_state import RobotState
from subsystems.swerve import Swerve
from subsystems.elevator import Elevator
from subsystems.pivot import Pivot
from subsystems.intake import Intake
from subsystems.led import Led
from subsystems.wrist import Wrist

from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
from commands.move_elevator import MoveElevator
from commands.move_pivot import MovePivot
from commands.move_wrist import MoveWrist
from commands.run_intake import RunIntake
from commands.set_leds import SetLEDs
#
from commands.go_to_position import GoToPosition
from commands.intake_sequence import IntakeSequence
from commands.reset_field_centric import ResetFieldCentric
# from commands.score import Score
# from commands.drive_by_joystick_subsystem import DriveByJoystickSubsystem

from autonomous.leave_then_score_1 import LeaveThenScore

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """


    # set robot modes
    class RobotMode(Enum): # use this instead of intake results directly because we want to be able to override intake results for testing and emergencies
        EMPTY = "e"
        HAS_CORAL = "c"
        HAS_ALGAE = "a"

    def set_robot_mode(self, mode: RobotMode): # because we can't assign inside lambdas
        self.robot_mode = mode

    def get_robot_mode(self) -> RobotMode:
        return self.robot_mode
    
    def is_robot_mode(self, mode: RobotMode) -> bool:
        return self.robot_mode == mode

    # set scoring mode


    def __init__(self) -> None:

        self.start_time = time.time()

        # The robot's subsystems
        # self.lower_crank = LowerCrank(container=self) # I don't want to test without a sim yet
        self.swerve = Swerve()
        self.elevator = Elevator()
        self.pivot = Pivot()
        self.wrist = Wrist()
        self.led = Led(self)
        self.intake = Intake()
        self.robot_state = RobotState(self)

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

        self.initialize_dashboard()

        Pathfinding.setPathfinder(LocalADStar())

        self.robot_mode = self.RobotMode.EMPTY


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

        def stick_between_degree_angles(angle_a, angle_b, stick_x, stick_y) -> bool:
            """
            returns whether the stick is between angle a (lower limit) and angle b (higher limit).
            0 <= a < b <= 360. Returns true if stick angle = a, but false if stick angle = b.
            cannot handle wraparound for now (how is there no method for this)
            """
            angle_a = math.radians(angle_a)
            angle_b = math.radians(angle_b)

            stick_angle = math.atan2(stick_x, stick_y)
            if stick_angle < 0: stick_angle = math.tau + stick_angle # compensate because atan returns -180 to 180 but we want 0 to 360

            return (angle_a <= stick_angle and stick_angle < angle_b)

        self.co_pilot_command_controller = commands2.button.CommandXboxController(constants.k_co_driver_controller_port)  # 2024 way
        
        self.co_trigger_right_stick_between_0_60_deg = commands2.button.Trigger(lambda: stick_between_degree_angles(0, 60,
                                                                            self.co_pilot_command_controller.getRightX(),
                                                                            self.co_pilot_command_controller.getRightY()))

        self.co_trigger_right_stick_between_60_120_deg = commands2.button.Trigger(lambda: stick_between_degree_angles(60, 120,
                                                                            self.co_pilot_command_controller.getRightX(),
                                                                            self.co_pilot_command_controller.getRightY()))

        self.co_trigger_right_stick_between_120_180_deg = commands2.button.Trigger(lambda: stick_between_degree_angles(120, 180,
                                                                            self.co_pilot_command_controller.getRightX(),
                                                                            self.co_pilot_command_controller.getRightY()))

        self.co_trigger_right_stick_between_180_240_deg = commands2.button.Trigger(lambda: stick_between_degree_angles(180, 240,
                                                                            self.co_pilot_command_controller.getRightX(),
                                                                            self.co_pilot_command_controller.getRightY()))

        self.co_trigger_right_stick_between_240_300_deg = commands2.button.Trigger(lambda: stick_between_degree_angles(240, 300,
                                                                            self.co_pilot_command_controller.getRightX(),
                                                                            self.co_pilot_command_controller.getRightY()))

        self.co_trigger_right_stick_between_300_360_deg = commands2.button.Trigger(lambda: stick_between_degree_angles(300, 360,
                                                                            self.co_pilot_command_controller.getRightX(),
                                                                            self.co_pilot_command_controller.getRightY()))

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

        self.co_trigger_l_stick_positive_x = self.co_pilot_command_controller.axisGreaterThan(0, 0.5)
        self.co_trigger_l_stick_negative_x = self.co_pilot_command_controller.axisLessThan(0, -0.5)
        self.co_trigger_l_stick_positive_y = self.co_pilot_command_controller.axisGreaterThan(1, 0.5)
        self.co_trigger_l_stick_negative_y = self.co_pilot_command_controller.axisLessThan(1, -0.5)

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
        wpilib.SmartDashboard.putData('MoveElevator', MoveElevator(container=self, elevator=self.elevator, mode='absolute'))
        wpilib.SmartDashboard.putData('MovePivot', MovePivot(container=self, pivot=self.pivot, mode='absolute'))
        wpilib.SmartDashboard.putData('SequentialScore', SequentialScoring(container=self))
        wpilib.SmartDashboard.putData('move wrist to -90 deg', MoveWrist(container=self, radians=math.radians(-90), timeout=4))
        wpilib.SmartDashboard.putData('move wrist to 0 deg', MoveWrist(container=self, radians=math.radians(0), timeout=4))
        wpilib.SmartDashboard.putData('move wrist to 90 deg', MoveWrist(container=self, radians=math.radians(90), timeout=4))

        # quick way to test all scoring positions from dashboard
        self.score_test_chooser = wpilib.SendableChooser()
        [self.score_test_chooser.addOption(key, value) for key, value in self.robot_state.targets_dict.items()]  # add all the indicators
        self.score_test_chooser.onChange(
            listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
                commands2.cmd.runOnce(lambda: self.robot_state.set_target(target=selected_value))))
        wpilib.SmartDashboard.putData('RobotScoringMode', self.score_test_chooser)

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

        print("Binding codriver buttons")

        self.co_trigger_a()

        #  leo's way: make a command that goes to any position. specify the position in command construction.
            # => a command object for each position
        # cory's way: make a command that goes to any position. specify the position in another subsystem that this command looks at.
            # => one command object and one subsystem
            # now we need a a command object for each position to tell that subsystem where to go
            # but we can change setpoints outside of construct-time

        # self.co_trigger_a.whileTrue(SequentialScoring(container=self))

        self.co_trigger_a.whileTrue(commands2.PrintCommand("we don't hvae a good l1 position yet"))
        
        self.co_trigger_b.whileTrue(GoToPosition(container=self, position="l2"))

        self.co_trigger_x.whileTrue(GoToPosition(container=self, position="l3"))

        self.co_trigger_y.whileTrue(GoToPosition(container=self, position="l4"))

        # trigger on true: go to the position, start intake
        # trigger on false: go to stow, stop intake

        self.co_trigger_d.or_(self.co_trigger_l).whileTrue(GoToPosition(container=self, position="coral station").andThen(
            RunIntake(container=self, intake=self.intake, value=-3, control_type=rev.SparkMax.ControlType.kVoltage, stop_on_end=False)))

        self.co_trigger_d.or_(self.co_trigger_l).onFalse(RunIntake(container=self, intake=self.intake, value=0, control_type=rev.SparkMax.ControlType.kVoltage, stop_on_end=False).andThen(
            GoToPosition(container=self, position="stow")))

        self.co_trigger_u.or_(self.co_trigger_r).whileTrue(GoToPosition(container=self, position="stow"))


        self.co_trigger_lb.whileTrue(RunIntake(container=self, intake=self.intake, value=-3, control_type=rev.SparkMax.ControlType.kVoltage))
        
        self.co_trigger_rb.whileTrue(RunIntake(container=self, intake=self.intake, value=3, control_type=rev.SparkMax.ControlType.kVoltage))

        self.co_trigger_l_stick_positive_x.whileTrue(MoveWrist(container=self, radians=math.radians(0), timeout=4))

        self.co_trigger_l_stick_positive_y.whileTrue(MoveWrist(container=self, radians=math.radians(-90), timeout=4)) # this seems backwards but is not because y-axis is inverted

        self.co_trigger_l_stick_negative_y.whileTrue(MoveWrist(container=self, radians=math.radians(90), timeout=4))

        # self.co_trigger_a.onTrue( # when trigger A is pressed, if we have coral, go to l1; else if we have algae, go to processor; else go to ground
        #         commands2.ConditionalCommand(
        #             onTrue=GoToPosition(container=self, position="l1"),
        #
        #             onFalse=commands2.ConditionalCommand(
        #                 onTrue=GoToPosition(container=self, position="processor"),
        #                 onFalse=IntakeSequence(container=self, position="ground"),
        #                 condition=lambda: self.get_robot_mode() == self.RobotMode.HAS_ALGAE
        #             ),
        #
        #             condition=lambda: self.get_robot_mode() == self.RobotMode.HAS_CORAL
        #         )
        # )
        #
        # self.co_trigger_b.onTrue(
        #         commands2.ConditionalCommand(
        #             onTrue=GoToPosition(container=self, position="l2"),
        #             onFalse=IntakeSequence(container=self, position="ground"), # can make either a or b into a different position if needed for algae and coral
        #             condition=lambda: self.get_robot_mode() == self.RobotMode.HAS_CORAL
        #         )
        # )
        #
        # self.co_trigger_x.onTrue(
        #         commands2.ConditionalCommand(
        #             onTrue=GoToPosition(container=self, position="l3"),
        #             onFalse=IntakeSequence(container=self, position="algae low"),
        #             condition=lambda: self.get_robot_mode() == self.RobotMode.HAS_CORAL
        #         )
        # )
        #
        # self.co_trigger_y.onTrue( # when trigger Y is pressed, if we have coral, go to l4; else if we have algae, go to net; else go to intake algae high
        #         commands2.ConditionalCommand(
        #             onTrue=GoToPosition(container=self, position="l4"),
        #
        #             onFalse=commands2.ConditionalCommand(
        #                 onTrue=GoToPosition(container=self, position="barge"),
        #                 onFalse=IntakeSequence(container=self, position="algae high"),
        #
        #                 condition=lambda: self.get_robot_mode() == self.RobotMode.HAS_ALGAE
        #             ),
        #
        #             condition=lambda: self.get_robot_mode() == self.RobotMode.HAS_CORAL
        #         )
        # )
        #
        # self.co_trigger_lb.onTrue(commands2.PrintCommand("** Setting robot mode to empty **").andThen(commands2.InstantCommand(lambda: self.set_robot_mode(self.RobotMode.EMPTY))))
        #
        # self.co_trigger_rb.onTrue(Score(container=self))
        # self.co_trigger_rb.whileTrue(DriveByJoystickSubsystem(container=self, controller=self.co_pilot_command_controller, subsystem=self.intake, duty_cycle_coef=0.01))
        #
        # self.co_trigger_r_trigger.onTrue(commands2.PrintCommand("** Setting robot mode to has algae **").andThen(commands2.InstantCommand(lambda: self.set_robot_mode(self.RobotMode.HAS_ALGAE))))
        #
        # self.co_trigger_u.or_(self.co_trigger_r).onTrue(commands2.PrintCommand("** Setting robot mode to has coral **").andThen(commands2.InstantCommand(lambda: self.set_robot_mode(self.RobotMode.HAS_CORAL))))

        # self.co_trigger_d.or_(self.co_trigger_l).onTrue(IntakeSequence(container=self, position="coral station"))


    def bind_keyboard_buttons(self):
        # for convenience, and just in case a controller goes down
        pass

    def get_autonomous_command(self):
        return LeaveThenScore(container=self)
        # return AutoBuilder.followPath(PathPlannerPath.fromPathFile("new patth"))
        # return self.autonomous_chooser.getSelected()
