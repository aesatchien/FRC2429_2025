import time, enum
import wpilib
import commands2
from commands2.button import Trigger
import constants

from subsystems.lower_crank import LowerCrank

from commands.move_lower_arm_by_network_tables import MoveLowerArmByNetworkTables

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
        self.lower_crank = LowerCrank(container=self)

        self.configure_joysticks()
        self.bind_buttons()
        # self.configure_swerve_bindings()

        
        self.initialize_dashboard()
        self.bind_buttons

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
        self.driver_controller = commands2.button.CommandXboxController(constants.GeneralConstants.k_driver_controller_port)
        self.triggerA = self.driver_controller.a()
        self.triggerB = self.driver_controller.b()
        self.triggerX = self.driver_controller.x()
        self.triggerY = self.driver_controller.y()
        self.triggerLB = self.driver_controller.leftBumper()
        self.triggerRB = self.driver_controller.rightBumper()
        self.triggerBack = self.driver_controller.back()
        self.triggerStart = self.driver_controller.start()
        self.triggerUp = self.driver_controller.povUp()
        self.triggerDown = self.driver_controller.povDown()
        self.triggerLeft = self.driver_controller.povLeft()
        self.triggerRight = self.driver_controller.povRight()

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
        wpilib.SmartDashboard.putData(MoveLowerArmByNetworkTables(container=self, crank=self.lower_crank))
        # lots of putdatas for testing on the dash
        pass

    def bind_buttons(self):
        pass

    def get_autonomous_command(self):
        pass 
        # return self.autonomous_chooser.getSelected()
