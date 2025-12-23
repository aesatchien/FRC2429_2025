# RobotContainer initializes subsystems, configures joystick/button bindings,
# and provides the autonomous command.

import wpilib
import commands2

import constants

from commands.drive_by_joystick import DriveByJoystick

from subsystems.drivetrain import Drivetrain

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

    def initialize_dashboard(self):
        # wpilib.SmartDashboard.putData(MoveLowerArmByNetworkTables(container=self, crank=self.lower_crank))
        # lots of putdatas for testing on the dash
        pass

    def bind_driver_buttons(self):
        #coast
        self.triggerA.onTrue(
            commands2.cmd.runOnce(lambda: self.drive.set_brake_mode('coast'))).onFalse(
            commands2.cmd.runOnce(lambda: self.drive.set_brake_mode('brake')))

        # easy to ready way - linear
        # onFalse means when trigger's released
        self.triggerB.onTrue(commands2.PrintCommand('trigger b pushed'))
        self.triggerB.onFalse(commands2.PrintCommand('trigger b released'))

        # METHOD CHAINING - function returns the object, creating a "fluent interface"
        # and I threw in ignoringDisable()
        (self.triggerX.onTrue(commands2.PrintCommand('trigger x pushed').ignoringDisable(True)).onFalse(
            commands2.PrintCommand('trigger x released').ignoringDisable(True)))

        # more things we can do with triggers
        self.triggerY.whileTrue(
            commands2.RunCommand(
                lambda: self.drive.tank_drive(leftSpeed=1, rightSpeed=1),self.drive,)
            .beforeStarting(lambda: (print(f"Y START {self.timer.get():.2f}s")))
            .finallyDo(lambda interrupted: (print(f"Y END {self.timer.get():.2f}s")))
        )



    def bind_operator_buttons(self):
        pass

    def get_autonomous_command(self):
        return commands2.PrintCommand("This is our autonomous command")
        # return self.autonomous_chooser.getSelected()
