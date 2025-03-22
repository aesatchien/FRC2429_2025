# 2429 FRC code for 2025 season - Reefscape
import math
import time
from enum import Enum
import rev
import wpilib
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d
from wpimath.units import degreesToRadians
import commands2
from commands2.printcommand import PrintCommand
from ntcore import NetworkTableInstance

# pathplanner stuff
from pathplannerlib.pathfinders import LocalADStar
from pathplannerlib.pathfinding import Pathfinding
from pathplannerlib.path import PathConstraints, PathPlannerPath
from pathplannerlib.auto import AutoBuilder, NamedCommands
from pathplannerlib.commands import PathfindingCommand

# 2429 helper files
import constants
import trajectory
from trajectory import CustomTrajectory, trajectory_L3

# 2429 subsystems
from subsystems import swerve_constants
from subsystems.robot_state import RobotState
from subsystems.swerve import Swerve
from subsystems.elevator import Elevator
from subsystems.pivot import Pivot
from subsystems.intake import Intake
from subsystems.led import Led
from subsystems.wrist import Wrist
from subsystems.climber import Climber
from subsystems.vision import Vision

# 2429 "auto" commands - just an organizational division of commands
from autonomous.one_plus_one import OnePlusOne
from autonomous.one_plus_two import OnePlusTwo
from autonomous.leave_then_score_1 import LeaveThenScore
from autonomous.one_plus_two_trough import OnePlusTwoTrough

# 2429 commands
from commands.auto_l1 import AutoL1
from commands.can_status import CANStatus
from commands.drive_by_distance_swerve import DriveByVelocitySwerve
from commands.drive_by_apriltag_swerve import DriveByApriltagSwerve
from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
from commands.follow_trajectory import FollowTrajectory
from commands.go_to_coral_station import GoToCoralStation
from commands.go_to_position import GoToPosition
from commands.go_to_reef_position import GoToReefPosition
from commands.go_to_stow import GoToStow
from commands.intake_sequence import IntakeSequence
from commands.move_climber import MoveClimber
from commands.move_elevator import MoveElevator
from commands.move_pivot import MovePivot
from commands.move_wrist import MoveWrist
from commands.move_wrist_swap import MoveWristSwap
from commands.pid_to_point import PIDToPoint
from commands.auto_to_pose import AutoToPose
from commands.reflash import Reflash
from commands.reset_field_centric import ResetFieldCentric
from commands.run_intake import RunIntake
from commands.score import Score
from commands.sequential_scoring import SequentialScoring
from commands.set_leds import SetLEDs


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    # set robot modes - TODO: this belongs in RobotState
    class RobotMode(Enum):  # use this instead of intake results directly because we want to be able to override intake results for testing and emergencies
        EMPTY = "e"
        HAS_CORAL = "c"
        HAS_ALGAE = "a"

    def set_robot_mode(self, mode: RobotMode):  # because we can't assign inside lambdas
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
        self.wrist = Wrist(self.pivot, self.elevator)
        self.climber = Climber()
        self.intake = Intake()
        self.vision = Vision()
        self.robot_state = RobotState(self)  # currently has a callback that LED can register, but
        self.led = Led(self)  # may want LED last because it may want to know about other systems

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
            # self.bind_codriver_buttons()  # it takes too long to poll all these
            self.bind_keyboard_buttons()  # it takes too long to poll all these
            if constants.k_use_bbox:
                self.bind_button_box()

        self.register_commands()

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
        self.trigger_L_trigger = self.driver_command_controller.leftTrigger(0.5)
        self.triggerBack = self.driver_command_controller.back()
        self.triggerStart = self.driver_command_controller.start()
        self.triggerUp = self.driver_command_controller.povUp()
        self.triggerDown = self.driver_command_controller.povDown()
        self.triggerLeft = self.driver_command_controller.povLeft()
        self.triggerRight = self.driver_command_controller.povRight()

        self.copilot_controller = commands2.button.CommandXboxController(1)
        self.copilot_controller = commands2.button.CommandXboxController(1) 

    def configure_codriver_joystick(self):

        print("Configuring codriver joystick")

        def stick_between_degree_angles(angle_a, angle_b, stick_x, stick_y) -> bool:
            """
            returns whether the stick is between angle a (lower limit) and angle b (higher limit).
            0 <= a < b <= 360. Returns true if stick angle = a, but false if stick angle = b.
            cannot handle wraparound for now (how is there no method for this)
            """
            angle_a = math.radians(angle_a)
            angle_b = math.radians(angle_b)

            stick_angle = math.atan2(stick_x, stick_y)
            if stick_angle < 0:
                stick_angle = math.tau + stick_angle  # compensate because atan returns -180 to 180 but we want 0 to 360

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

        self.co_trigger_r_stick_positive_x = self.co_pilot_command_controller.axisGreaterThan(4, 0.5)
        self.co_trigger_r_stick_negative_x = self.co_pilot_command_controller.axisLessThan(4, -0.5)
        self.co_trigger_r_stick_positive_y = self.co_pilot_command_controller.axisGreaterThan(5, 0.5)
        self.co_trigger_r_stick_negative_y = self.co_pilot_command_controller.axisLessThan(5, -0.5)

    def initialize_dashboard(self):
        # wpilib.SmartDashboard.putData(MoveLowerArmByNetworkTables(container=self, crank=self.lower_crank))
        # lots of putdatas for testing on the dash
        # COMMANDS FOR GUI (ROBOT DEBUGGING) - 20250224 CJH
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

        # Arshan's trajectory tests
        wpilib.SmartDashboard.putData('l3 trajectory', FollowTrajectory(container=self, current_trajectory=trajectory.trajectory_L3, wait_to_finish=True))
        wpilib.SmartDashboard.putData('l2 67 score trajectory', FollowTrajectory(container=self, current_trajectory=trajectory.l2_score_67, wait_to_finish=True))
        wpilib.SmartDashboard.putData('l3 67 score trajectory', FollowTrajectory(container=self, current_trajectory=trajectory.l3_score_67, wait_to_finish=True))
        wpilib.SmartDashboard.putData('l4 67 score trajectory', FollowTrajectory(container=self, current_trajectory=trajectory.l4_score_67, wait_to_finish=True))

        # experimental, not used on dash
        SmartDashboard.putData("Go to 60 deg pid", commands2.cmd.runOnce(lambda: self.pivot.set_goal(math.radians(60), False), self.pivot))
        SmartDashboard.putData("Go to 90 deg pid", commands2.cmd.runOnce(lambda: self.pivot.set_goal(math.radians(90), False), self.pivot))
        wpilib.SmartDashboard.putData('SetSuccess', SetLEDs(container=self, led=self.led, indicator=Led.Indicator.kSUCCESS))
        wpilib.SmartDashboard.putData('MoveElevator', MoveElevator(container=self, elevator=self.elevator, mode='absolute'))
        wpilib.SmartDashboard.putData('MovePivot', MovePivot(container=self, pivot=self.pivot, mode='absolute'))
        wpilib.SmartDashboard.putData('SequentialScore', SequentialScoring(container=self))
        wpilib.SmartDashboard.putData('Move wrist to -90 deg', MoveWrist(container=self, radians=math.radians(-90), timeout=4))
        wpilib.SmartDashboard.putData('Move wrist to 0 deg', MoveWrist(container=self, radians=math.radians(0), timeout=4))
        wpilib.SmartDashboard.putData('Move wrist to 90 deg', MoveWrist(container=self, radians=math.radians(90), timeout=4))

        # commands for pyqt dashboard - please do not remove
        wpilib.SmartDashboard.putData('MoveElevatorTop', MoveElevator(container=self, elevator=self.elevator, mode='specified', height=constants.ElevatorConstants.k_max_height-0.005 ))
        wpilib.SmartDashboard.putData('MoveElevatorUp', MoveElevator(container=self, elevator=self.elevator, mode='incremental', height=0.1 ))
        wpilib.SmartDashboard.putData('MoveElevatorDown', MoveElevator(container=self, elevator=self.elevator, mode='incremental', height=-0.1))
        wpilib.SmartDashboard.putData('MovePivotUp', MovePivot(container=self, pivot=self.pivot, mode='incremental', angle=10))
        wpilib.SmartDashboard.putData('MovePivotDown', MovePivot(container=self, pivot=self.pivot, mode='incremental', angle=-10))
        wpilib.SmartDashboard.putData('MoveWristUp', MoveWrist(container=self, incremental=True, radians=degreesToRadians(30), timeout=0.2))
        wpilib.SmartDashboard.putData('MoveWristDown', MoveWrist(container=self, incremental=True, radians=degreesToRadians(-30), timeout=0.2))
        wpilib.SmartDashboard.putData('IntakeOn', RunIntake(container=self, intake=self.intake, value=6, stop_on_end=False))
        wpilib.SmartDashboard.putData('IntakeOff', RunIntake(container=self, intake=self.intake, value=0, stop_on_end=False))
        wpilib.SmartDashboard.putData('IntakeReverse', RunIntake(container=self, intake=self.intake, value=-6, stop_on_end=False))
        wpilib.SmartDashboard.putData('Move climber up', MoveClimber(self, self.climber, 'incremental', math.radians(10)))
        wpilib.SmartDashboard.putData('Move climber down', MoveClimber(self, self.climber, 'incremental', math.radians(-10)))
        wpilib.SmartDashboard.putData('CANStatus', CANStatus(container=self))
        wpilib.SmartDashboard.putData("ResetFlex", Reflash(container=self))
        wpilib.SmartDashboard.putData('GoToScore', Score(container=self))
        wpilib.SmartDashboard.putData('GoToStow', GoToStow(container=self))
        wpilib.SmartDashboard.putData('GoToL1', commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L1)).ignoringDisable(True).andThen(GoToReefPosition(self, 1, self.robot_state)))
        wpilib.SmartDashboard.putData('GoToL2', commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L2)).ignoringDisable(True).andThen(GoToReefPosition(self, 2, self.robot_state)))
        wpilib.SmartDashboard.putData('GoToL3', commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L3)).ignoringDisable(True).andThen(GoToReefPosition(self, 3, self.robot_state)))
        wpilib.SmartDashboard.putData('GoToL4', commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L4)).ignoringDisable(True).andThen(GoToReefPosition(self, 4, self.robot_state)))
        # end pyqt dashboard section

        # quick way to test all scoring positions from dashboard
        self.score_test_chooser = wpilib.SendableChooser()
        [self.score_test_chooser.addOption(key, value) for key, value in self.robot_state.targets_dict.items()]  # add all the indicators
        self.score_test_chooser.onChange(
            listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
                commands2.cmd.runOnce(lambda: self.robot_state.set_target(target=selected_value))))
        wpilib.SmartDashboard.putData('RobotScoringMode', self.score_test_chooser)

        self.auto_chooser = AutoBuilder.buildAutoChooser()
        self.auto_chooser.setDefaultOption('Wait', PrintCommand("** Running wait auto **").andThen(commands2.WaitCommand(15)))
        self.auto_chooser.addOption('Drive by velocity leave', PrintCommand("** Running drive by velocity swerve leave auto **").andThen(DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2)))
        self.auto_chooser.addOption('1+2', OnePlusTwo(self))
        self.auto_chooser.addOption('1+1 in code', OnePlusOne(self))
        self.auto_chooser.addOption('1+2 trough', OnePlusTwoTrough(self))
        wpilib.SmartDashboard.putData('autonomous routines', self.auto_chooser)


    def bind_driver_buttons(self):

        self.triggerB.onTrue(ResetFieldCentric(container=self, swerve=self.swerve, angle=0))

        # self.triggerX.whileTrue(AutoBuilder.buildAuto("testt"))
        # self.triggerX.onTrue(commands2.PrintCommand("starting pathplanner auto"))
        # self.triggerX.onFalse(commands2.PrintCommand("ending pathplanner auto"))

        # this is for field centric
        #self.triggerLB.whileTrue(DriveByApriltagSwerve(container=self, swerve=self.swerve, target_heading=0))

        # button A for intake
        # left trigger for outtake

        if wpilib.RobotBase.isSimulation():
            #self.triggerA.onTrue(AutoBuilder.pathfindToPoseFlipped(pose=constants.k_useful_robot_poses_blue["a"], constraints=swerve_constants.AutoConstants.k_pathfinding_constraints)) # this one is convenient for testing
            self.triggerB.onTrue(PIDToPoint(self, self.swerve, constants.k_useful_robot_poses_blue["a"]))

        self.triggerRB.onTrue(Score(self))

        self.trigger_L_trigger.onTrue(
                GoToReefPosition(container=self, level=2, wrist_setpoint_decider=math.radians(90)).andThen(
                    Score(container=self)
                    )
                )
        

    def bind_codriver_buttons(self):

        print("Binding codriver buttons")

        #  leo's way: make a command that goes to any position. specify the position in command construction.
            # => a command object for each position
        # cory's way: make a command that goes to any position. specify the position in another subsystem that this command looks at.
            # => one command object and one subsystem
            # now we need a command object for each position to tell that subsystem where to go
            # but we can change setpoints outside of construct-time

        # self.co_trigger_a.whileTrue(SequentialScoring(container=self))

        self.co_trigger_a.onTrue(GoToReefPosition(container=self, level=1, wrist_setpoint_decider=self.robot_state))
        
        self.co_trigger_b.onTrue(GoToReefPosition(container=self, level=2, wrist_setpoint_decider=self.robot_state))

        self.co_trigger_x.onTrue(GoToReefPosition(container=self, level=3, wrist_setpoint_decider=self.robot_state))

        self.co_trigger_y.onTrue(GoToReefPosition(container=self, level=4, wrist_setpoint_decider=self.robot_state))

        # trigger on true: go to the position, start intake
        # trigger on false: go to stow, stop intake

        self.co_trigger_d.or_(self.co_trigger_l).whileTrue(GoToCoralStation(container=self).andThen(
            RunIntake(container=self, intake=self.intake, value=constants.IntakeConstants.k_coral_intaking_voltage, control_type=rev.SparkMax.ControlType.kVoltage, stop_on_end=False)))

        self.co_trigger_d.or_(self.co_trigger_l).onFalse(RunIntake(container=self, intake=self.intake, value=0, control_type=rev.SparkMax.ControlType.kVoltage, stop_on_end=False).andThen(
            GoToStow(container=self)))

        self.co_trigger_u.or_(self.co_trigger_r).whileTrue(GoToStow(container=self))


        self.co_trigger_lb.whileTrue(RunIntake(container=self, intake=self.intake, value=-6, control_type=rev.SparkMax.ControlType.kVoltage, stop_on_end=True))
        
        self.co_trigger_rb.onTrue(Score(container=self))

        self.co_trigger_r_stick_negative_y.onTrue(MoveWrist(container=self, radians=math.radians(0), timeout=4))

        self.co_trigger_r_stick_positive_x.onTrue(MoveWristSwap(self, self.wrist))  # this seems backwards but is not because y-axis is inverted
        self.co_trigger_r_stick_positive_x.onTrue(commands2.cmd.runOnce(lambda: self.robot_state.set_side(side=RobotState.Side.RIGHT)).ignoringDisable(True))  # this seems backwards but is not because y-axis is inverted

        self.co_trigger_r_stick_negative_x.onTrue(MoveWristSwap(self, self.wrist))
        self.co_trigger_r_stick_negative_x.onTrue(commands2.cmd.runOnce(lambda: self.robot_state.set_side(side=RobotState.Side.LEFT)).ignoringDisable(True))

        self.co_trigger_start.onTrue(commands2.InstantCommand(lambda: self.climber.set_duty_cycle(0.2), self.climber))
        self.co_trigger_start.onFalse(commands2.InstantCommand(lambda: self.climber.set_duty_cycle(0), self.climber))
        self.co_trigger_back.onTrue(commands2.InstantCommand(lambda: self.climber.set_duty_cycle(-0.2), self.climber))
        self.co_trigger_back.onFalse(commands2.InstantCommand(lambda: self.climber.set_duty_cycle(0), self.climber))

    def bind_keyboard_buttons(self):
        # for convenience, and just in case a controller goes down

        # a, b, c ... k, l: reserved for driving to respective positions
        # w: wrist go left (korean left is wen chok, also position on the keyboard)
        # o: wrist go right (korean right is orin chok, also position on the keyboard)
        # see https://digitalwerk.gitlab.io/solutions/adtf_content/adtf_base/adtf_core/page_qt_key_event_runner.html for the codes for each key
        

        self.keys_pressed_entry = NetworkTableInstance.getDefault().getEntry("SmartDashboard/keys_pressed")  # for operator control via keyboard

        self.keyboard_trigger_p = commands2.button.Trigger(lambda: 80 in self.keys_pressed_entry.getIntegerArray([])).debounce(0.06)

        self.keyboard_trigger_1 = commands2.button.Trigger(lambda: 49 in self.keys_pressed_entry.getIntegerArray([])).debounce(0.06)
        self.keyboard_trigger_2 = commands2.button.Trigger(lambda: 50 in self.keys_pressed_entry.getIntegerArray([])).debounce(0.06)
        self.keyboard_trigger_3 = commands2.button.Trigger(lambda: 51 in self.keys_pressed_entry.getIntegerArray([])).debounce(0.06)
        self.keyboard_trigger_4 = commands2.button.Trigger(lambda: 52 in self.keys_pressed_entry.getIntegerArray([])).debounce(0.06)

        self.keyboard_trigger_s = commands2.button.Trigger(lambda: 83 in self.keys_pressed_entry.getIntegerArray([])).debounce(0.06)
        self.keyboard_trigger_t = commands2.button.Trigger(lambda: 84 in self.keys_pressed_entry.getIntegerArray([])).debounce(0.06)

        self.keyboard_trigger_v = commands2.button.Trigger(lambda: 86 in self.keys_pressed_entry.getIntegerArray([])).debounce(0.06)

        self.keyboard_trigger_w = commands2.button.Trigger(lambda: 87 in self.keys_pressed_entry.getIntegerArray([])).debounce(0.06)
        self.keyboard_trigger_o = commands2.button.Trigger(lambda: 79 in self.keys_pressed_entry.getIntegerArray([])).debounce(0.06)

        # p: place (score)
        self.keyboard_trigger_p.onTrue(GoToStow(self))

        self.keyboard_trigger_1.onTrue(commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L1)).ignoringDisable(True).andThen(GoToReefPosition(self, 1, self.robot_state)))
        self.keyboard_trigger_2.onTrue(GoToReefPosition(self, 2, self.robot_state))
        self.keyboard_trigger_3.onTrue(commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L3)).ignoringDisable(True).andThen(GoToReefPosition(self, 3, self.robot_state)))
        self.keyboard_trigger_4.onTrue(commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L4)).ignoringDisable(True).andThen(GoToReefPosition(self, 4, self.robot_state)))

        self.keyboard_trigger_s.onTrue(Score(self))

        # t: human player (mnemonic: celeste Tarula, sean Toda, Take a piece)
        self.keyboard_trigger_t.whileTrue(GoToCoralStation(container=self).andThen(
            RunIntake(container=self, intake=self.intake, value=-3, control_type=rev.SparkMax.ControlType.kVoltage, stop_on_end=False)))
        self.keyboard_trigger_t.onFalse(RunIntake(container=self, intake=self.intake, value=0, control_type=rev.SparkMax.ControlType.kVoltage, stop_on_end=False).andThen(
            GoToStow(container=self)))

        # v: intake on (mnemonic: vacuum up the piece)
        self.keyboard_trigger_v.whileTrue(RunIntake(self, self.intake, -3, stop_on_end=True))

        # w: wrist go left (korean left is wen chok, also position on the keyboard)
        self.keyboard_trigger_o.onTrue(commands2.cmd.runOnce(lambda: self.robot_state.set_side(side=RobotState.Side.RIGHT)).ignoringDisable(True))

        # o: wrist go right (korean right is orin chok, also position on the keyboard)
        self.keyboard_trigger_w.onTrue(commands2.cmd.runOnce(lambda: self.robot_state.set_side(side=RobotState.Side.LEFT)).ignoringDisable(True))

    def register_commands(self):

        NamedCommands.registerCommand('robot state left', commands2.cmd.runOnce(lambda: self.robot_state.set_side(side=RobotState.Side.RIGHT)).ignoringDisable(True))
        NamedCommands.registerCommand('go to l4', GoToReefPosition(self, 4, self.robot_state))
        NamedCommands.registerCommand('go to l3', GoToReefPosition(self, 3, self.robot_state))
        NamedCommands.registerCommand('go to l1 auto', GoToReefPosition(self, 1, self.robot_state))
        NamedCommands.registerCommand('go to l1', AutoL1(self).withTimeout(2))
        NamedCommands.registerCommand('go to coral station', GoToCoralStation(self))
        NamedCommands.registerCommand('stow', GoToStow(self))
        NamedCommands.registerCommand('stow and turn off intake', RunIntake(self, self.intake, 0).andThen(GoToStow(self)))
        NamedCommands.registerCommand('score', Score(self))
        NamedCommands.registerCommand('intake spit out', RunIntake(self, self.intake, constants.IntakeConstants.k_coral_scoring_voltage, stop_on_end=True).raceWith(
            commands2.WaitCommand(0.5)
            ).andThen(
                GoToCoralStation(self)
                )
            )
        NamedCommands.registerCommand('score then go to coral station', GoToStow(self).andThen(GoToCoralStation(self)))
        NamedCommands.registerCommand('move wrist to 90 deg', MoveWrist(self, math.radians(90), 2, False, True))
        NamedCommands.registerCommand('move wrist to -90 deg', MoveWrist(self, math.radians(-90), 2, False, True))
        NamedCommands.registerCommand('move wrist to 0 deg', MoveWrist(self, math.radians(0), 2, False, True))


    def get_autonomous_command(self):
        # return DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2)
        return self.auto_chooser.getSelected()

    def bind_button_box(self):
        """
        Remember - buttons are 1-indexed, not zero
        """
        # The driver's controller
        self.bbox_1 = commands2.button.CommandJoystick(constants.k_bbox_1_port)
        self.bbox_2 = commands2.button.CommandJoystick(constants.k_bbox_2_port)

        self.bbox_TBD1 = self.bbox_1.button(3)  # top left red 1
        self.bbox_TBD2 = self.bbox_1.button(4)  # top left red 2

        self.bbox_right = self.bbox_1.button(1)  # true when selected
        self.bbox_left = self.bbox_1.button(2)  #  and true when selected

        print('Initializing robot state based on button box joystick:')
        if self.bbox_right.getAsBoolean():
            self.robot_state.set_side(self.robot_state.Side.RIGHT)
        else:
            self.robot_state.set_side(self.robot_state.Side.LEFT)

        self.bbox_human_left = self.bbox_1.button(5)
        self.bbox_human_right = self.bbox_1.button(6)

        # reef  stuff
        self.bbox_AB = self.bbox_1.button(7)
        self.bbox_CD = self.bbox_1.button(8)
        self.bbox_EF = self.bbox_1.button(9)
        self.bbox_GH = self.bbox_1.button(10)
        self.bbox_IJ = self.bbox_1.button(11)
        self.bbox_KL = self.bbox_1.button(12)

        self.bbox_L1 = self.bbox_2.button(1)
        self.bbox_L2 = self.bbox_2.button(2)
        self.bbox_L3 = self.bbox_2.button(3)
        self.bbox_L4 = self.bbox_2.button(4)
        self.bbox_reef_alga_high = self.bbox_2.button(5)
        self.bbox_reef_alga_low = self.bbox_2.button(7)
        self.bbox_net = self.bbox_2.button(6)
        self.bbox_processor = self.bbox_2.button(8)

        # actual bindings

        self.bbox_TBD1.onTrue(Score(self))
        self.bbox_TBD2.onTrue(GoToStow(self))

        self.bbox_right.onTrue(commands2.cmd.runOnce(lambda: self.robot_state.set_side(side=RobotState.Side.RIGHT)).ignoringDisable(True))
        # self.bbox_right.onFalse(MoveWristSwap(self, self.wrist))
        self.bbox_left.onTrue(commands2.cmd.runOnce(lambda: self.robot_state.set_side(side=RobotState.Side.LEFT)).ignoringDisable(True))
        # self.bbox_left.onFalse(MoveWristSwap(self, self.wrist))

        # CJH taking over some buttons for testing the swerve setup - have to take the joystick out of it
        #self.bbox_GH.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(0.2, 0, 0), timeout=2))  # go forward - shows you front
        #self.bbox_IJ.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(0, 0.2, 0), timeout=2))  # go left - shows you left
        #self.bbox_KL.whileTrue(DriveByVelocitySwerve(self, self.swerve, Pose2d(0, 0, 0.2), timeout=2))  # positive should spin CCW


        # giving AJ a button to hold for driving to a goal
        self.triggerA.whileTrue(AutoToPose(self, self.swerve, target_pose=None, from_robot_state=True, control_type='not_pathplanner'))

        # set up all six buttons on the reef for the while held conditions
        button_list = [self.bbox_AB, self.bbox_CD, self.bbox_EF, self.bbox_GH, self.bbox_IJ, self.bbox_KL]  #
        characters = ['ab', 'cd', 'ef', 'gh', 'ij', 'kl']
        states = [self.robot_state.is_left, self.robot_state.is_left, self.robot_state.is_right, self.robot_state.is_right, self.robot_state.is_right, self.robot_state.is_left]
        poses_dict = constants.k_useful_robot_poses_blue
        constraints = swerve_constants.AutoConstants.k_pathfinding_constraints
        reef_goals = self.robot_state.reef_goal_dict  # zip will give keys, but not poses

        use_pathplanner = True
        for but, state, chars, reef_goal_key in zip(button_list, states, characters, reef_goals.keys()):

            # pure lambda fails because if you change the iterator it only gets the last one in the list at runtime
            # note this is an onTrue - there is no delay in setting the goal
            but.onTrue(self.robot_state.set_reef_goal_cmd(reef_goals[reef_goal_key]))

            # set the reef pose of the robot for other auto-driving buttons, but you have to hold it down
            but.debounce(0.15).onTrue(PrintCommand(f'-- Starting AutoDriving to {reef_goals[reef_goal_key]} --'))

            if use_pathplanner:
                but.debounce(0.15).whileTrue(  # todo - wrap this in LED indicators, or make a start/end with
                    commands2.ConditionalCommand(
                        onTrue=AutoBuilder.pathfindToPoseFlipped(pose=poses_dict[chars[0]], constraints=constraints).andThen(
                            self.led.set_indicator_with_timeout(Led.Indicator.kSUCCESSFLASH, 2)),
                        onFalse=AutoBuilder.pathfindToPoseFlipped(pose=poses_dict[chars[1]], constraints=constraints).andThen(
                            self.led.set_indicator_with_timeout(Led.Indicator.kSUCCESSFLASH, 2)),
                        condition=state,
                    )
                )
            else:  # pidtopoint version - can test both versions this way
                but.debounce(0.15).whileTrue(
                    commands2.ConditionalCommand(
                        onTrue=AutoToPose(self, self.swerve, constants.k_useful_robot_poses_blue[chars[0]], control_type='not_pathplanner'),
                        onFalse=AutoToPose(self, self.swerve, constants.k_useful_robot_poses_blue[chars[1]], control_type='not_pathplanner'),
                        condition=state,
                    )
                )

        # picking up coral from the human station
        self.bbox_human_right.whileTrue(GoToCoralStation(container=self))
        self.bbox_human_right.onFalse(RunIntake(container=self, intake=self.intake, value=0, control_type=rev.SparkMax.ControlType.kVoltage, stop_on_end=False).andThen(
            GoToStow(container=self)))
        # self.bbox_human_right.onTrue(commands2.PrintCommand("Pushed BBox Human right"))

        # wrist swap
        self.bbox_human_left.onTrue(MoveWristSwap(self, self.wrist))

        #self.bbox_AB.whileTrue(PIDToPoint(self, self.swerve, Pose2d(0, 0, 0)))
        #self.bbox_GH.whileTrue(PIDToPoint(self, self.swerve, constants.k_useful_robot_poses_blue["g"]))

        # self.bbox_GH.onTrue(commands2.WaitCommand(4).andThen(Reflash(self)))
        # self.bbox_GH.onTrue(GoToStow(self))

        # L1-L4
        self.bbox_L1.onTrue(commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L1)).ignoringDisable(True).andThen(GoToReefPosition(self, 1, self.robot_state)))
        self.bbox_L2.onTrue(commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L2)).ignoringDisable(True).andThen(GoToReefPosition(self, 2, self.robot_state)))
        self.bbox_L3.onTrue(commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L3)).ignoringDisable(True).andThen(GoToReefPosition(self, 3, self.robot_state)))
        self.bbox_L4.onTrue(commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L4)).ignoringDisable(True).andThen(GoToReefPosition(self, 4, self.robot_state)))

        # Reef actions
        self.bbox_reef_alga_high.whileTrue(commands2.ParallelCommandGroup(
            GoToPosition(self, "algae high"),
            RunIntake(self, self.intake, constants.IntakeConstants.k_algae_intaking_voltage)))
        self.bbox_reef_alga_high.onFalse(
            RunIntake(container=self, intake=self.intake, value=0, control_type=rev.SparkMax.ControlType.kVoltage, stop_on_end=False).andThen(
                GoToStow(container=self)))

        self.bbox_reef_alga_low.whileTrue(commands2.ParallelCommandGroup(
            GoToPosition(self, "algae low"),
            RunIntake(self, self.intake, constants.IntakeConstants.k_algae_intaking_voltage)))
        self.bbox_reef_alga_low.onFalse(
            RunIntake(container=self, intake=self.intake, value=0, control_type=rev.SparkMax.ControlType.kVoltage, stop_on_end=False).andThen(
                GoToStow(container=self)))

        # climber actions
        self.bbox_net.onTrue(commands2.InstantCommand(lambda: self.climber.set_duty_cycle(0.2), self.climber))
        self.bbox_net.onTrue(GoToPosition(self, "climb"))
        self.bbox_net.onFalse(commands2.InstantCommand(lambda: self.climber.set_duty_cycle(0), self.climber))

        self.bbox_processor.onTrue(commands2.InstantCommand(lambda: self.climber.set_duty_cycle(-0.2), self.climber))
        self.bbox_processor.onTrue(GoToPosition(self, "climb"))
        self.bbox_processor.onFalse(commands2.InstantCommand(lambda: self.climber.set_duty_cycle(0), self.climber))

        # print commands for testing
        #self.bbox_net.onTrue(commands2.PrintCommand("Pushed BBox Net"))
        #self.bbox_processor.onTrue(commands2.PrintCommand("Pushed BBox Processor"))

