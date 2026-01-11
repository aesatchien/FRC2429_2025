# RobotContainer initializes subsystems, configures joystick/button bindings,
# and provides the autonomous command.

import wpilib
import commands2
from commands2 import PrintCommand, RunCommand, InstantCommand, ConditionalCommand

import constants
from constants import ShooterConstants as sc

from commands.drive_by_joystick import DriveByJoystick
from commands.shooting_log_command import ShootingCommandLogging
from commands.log_test import LogTest

from subsystems.drivetrain import Drivetrain
from subsystems.shooter import Shooter

wpilib.DriverStation.silenceJoystickConnectionWarning(True)  # stop annoying "no joystick" messages

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        self.timer = wpilib.Timer()
        self.timer.start()
        
        # The robot's subsystems
        self.drive = Drivetrain()
        self.shooter = Shooter()

        # Configure joysticks and buttons
        self.configure_joysticks()
        self.bind_driver_buttons()

        # set the default command for the drivetrain
        self.drive.setDefaultCommand(DriveByJoystick(self, self.driver_command_controller))

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
        self.triggerRB = self.driver_command_controller.rightBumper()
        self.triggerLB = self.driver_command_controller.leftBumper()
        self.trigger_up = self.driver_command_controller.povUp()
        self.trigger_down = self.driver_command_controller.povDown()
        self.trigger_left = self.driver_command_controller.povLeft()
        self.trigger_right = self.driver_command_controller.povRight()

    def initialize_dashboard(self):
        # wpilib.SmartDashboard.putData(MoveLowerArmByNetworkTables(container=self, crank=self.lower_crank))
        # lots of putdatas for testing on the dash
        pass

    def bind_driver_buttons(self):


        self.triggerA.whileTrue(ShootingCommandLogging(container=self, shooter=self.shooter, continuous=True, balls=0))

        self.triggerY.onTrue(ShootingCommandLogging(container=self, shooter=self.shooter, continuous=False, balls=2))

        self.trigger_down.onTrue(LogTest(container=self, indent=0).ignoringDisable(True))

        # ------------ DEMONSTRATE PRINT COMMANDS  --------------
        # easy to ready way - linear, not using method chaining
        # onTrue / onFalse means when trigger is pressed / released
        #self.triggerLB.onTrue(commands2.PrintCommand('trigger lb pressed (using PrintCommand)'))  # print command way
        #self.triggerLB.onFalse(commands2.InstantCommand(lambda : print('trigger lb released (using InstantCommand with lambda)')))  # lambda as a print

        # ------------ DEMONSTRATE METHOD CHAINING ON COMMANDS  --------------
        # METHOD CHAINING - function returns the object, presenting a "fluent interface"
        # self.triggerX.onTrue(commands2.PrintCommand('trigger x pressed')).onFalse(commands2.PrintCommand('trigger x released'))

        # ------------ DEMONSTRATE LAMBDAS IN COMMANDS WITH THE RUNONCE FACTORY --------------
        # commands2.cmd.runOnce is a command factory, one shot with no overrides (InstantCommand exposes the lifecycle)
        # One-shot brake-mode toggle; allowed while disabled - this is not an easy introduction
        (self.triggerLB
         .onTrue(commands2.cmd.runOnce(lambda: self.drive.set_brake_mode("coast")).ignoringDisable(True))
         .onFalse(commands2.cmd.runOnce(lambda: self.drive.set_brake_mode("brake")).ignoringDisable(True))
        )

        # ------------ DEMONSTRATE DRIVING THE TANK WITH "RUN" COMMANDS  --------------
        # more things we can do with triggers - RunCommand is going to execute continuously
        self.trigger_up.whileTrue(
            commands2.RunCommand(
                lambda: self.drive.tank_drive(leftSpeed=0.1, rightSpeed=0.1),self.drive,)
            .beforeStarting(lambda: (print(f"up pressed {self.timer.get():.2f}s ", end='')))
            .finallyDo(lambda interrupted: (print(f"up released {self.timer.get():.2f}s")))
        )

        # ------------ DEMONSTRATE INSTANT COMMANDS FOR SHOOTER TOGGLING WITH DECORATORS  --------------
        # RunCommand is not what we want here - an InstantCommand is the right call
        # self.triggerB.onTrue(
        #     commands2.InstantCommand(
        #         lambda: self.shooter.set_shooter_rpm(4000),self.shooter,)
        #     .beforeStarting(lambda: (print(f"Shooter B: at {self.timer.get():.1f} s ... ", end='')))
        #     .finallyDo(lambda interrupted: self.shooter.stop_shooter())  # this finallyDo function has to accept an "interrupted" parameter
        # # )
        # self.trigger_right.onTrue(
        #     commands2.InstantCommand(
        #         lambda: self.shooter.set_indexer_rpm(60),None)
        #     .beforeStarting(lambda: (print(f"Indexer PoVR: at {self.timer.get():.1f} s ... ", end='')))
        #     .finallyDo(lambda interrupted: self.shooter.stop_indexer())  # this finallyDo function has to accept an "interrupted" parameter
        # )
        # self.trigger_left.onTrue(
        #     commands2.InstantCommand(
        #         lambda: self.shooter.set_indexer_rpm(-60),None)
        #     .beforeStarting(lambda: (print(f"Indexer PoVL: at {self.timer.get():.1f} s ... ", end='')))
        # .andThen(commands2.InstantCommand(lambda: self.shooter.stop_indexer(),None)))

        self.trigger_right.onTrue(InstantCommand(lambda: print(f'Indexer PoVR: at {self.timer.get():.1f} s ...', end='')).
                              andThen(InstantCommand(lambda: self.shooter.set_indexer_rpm(-60),self.shooter,)))
        self.trigger_right.onFalse(InstantCommand(lambda: self.shooter.stop_indexer(),self.shooter,))

        self.trigger_left.onTrue(InstantCommand(lambda: print(f'Indexer PoVL: at {self.timer.get():.1f} s ...', end='')).
                              andThen(InstantCommand(lambda: self.shooter.set_indexer_rpm(60),self.shooter,)))
        self.trigger_left.onFalse(InstantCommand(lambda: self.shooter.stop_indexer(),self.shooter,))


        # ------------ DEMONSTRATE LINEAR ON/OFF TOGGLING WITH SOME MESSAGING  --------------
        # this is bad - it continually sets the shooter off because it's a RunCommand
        #self.triggerRB.onTrue(commands2.RunCommand(lambda: self.shooter.set_shooter_rpm(1000),self.shooter,))
        #self.triggerRB.onFalse(commands2.RunCommand(lambda: self.shooter.set_shooter_rpm(0.0),self.shooter,))
        # this is the right way - question for students: why am i using a lambda instead of a PrintCommand?
        self.triggerB.onTrue(InstantCommand(lambda: print(f'Shooter RB: at {self.timer.get():.1f} s ...', end='')).
                              andThen(InstantCommand(lambda: self.shooter.set_shooter_rpm(sc.k_fastest_speed),self.shooter,)))
        self.triggerB.onFalse(InstantCommand(lambda: self.shooter.set_shooter_rpm(0.0),self.shooter,))

        self.triggerRB.onTrue(InstantCommand(lambda: print(f'Shooter RB: at {self.timer.get():.1f} s ...', end='')).
                              andThen(InstantCommand(lambda: self.shooter.set_shooter_rpm(sc.k_test_speed),self.shooter,)))
        self.triggerRB.onFalse(InstantCommand(lambda: self.shooter.set_shooter_rpm(0.0),self.shooter,))

    # ------------ TODO SECTION  --------------
    # use the pov_right to rotate turret CW, and pov_left to rotate CCW (two commands)
    # start binding to actual full-lifecycle functions that we write
    # figure out an LED scheme

    def bind_operator_buttons(self):
        pass

    def get_autonomous_command(self):
        return commands2.PrintCommand("This is our autonomous command")
        # return self.autonomous_chooser.getSelected()
