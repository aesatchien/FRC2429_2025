
#!/usr/bin/env python3

import math
import typing

import commands2

from rev import SparkMax, SparkMaxConfig

# import warnings
# warnings.filterwarnings("ignore")


class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        self.test_spark = SparkMax(deviceID=0, type=SparkMax.MotorType.kBrushless)
        self.test_controller = self.test_spark.getClosedLoopController()

        self.test_spark_config = SparkMaxConfig()
        self.test_spark_config.closedLoop.pid(p=1000, i=0, d=0)
        
        # no conversion factors; magically it's geared such that 2pi encoder ticks actually turns the arm 360 degrees.

        self.test_spark.configure(self.test_spark_config, SparkMax.ResetMode.kResetSafeParameters, persistMode=SparkMax.PersistMode.kPersistParameters)

        self.counter = 0

        # autonomous chooser on the dashboard.

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        self.counter += 1

        if self.counter % 400 > 200:
            self.test_controller.setReference(value=0, ctrl=SparkMax.ControlType.kPosition)
            # self.test_spark.setVoltage(12)
        else:
            self.test_controller.setReference(value=math.pi / 2, ctrl=SparkMax.ControlType.kPosition)
            # self.test_spark.setVoltage(-12)

    def testInit(self) -> None:
        pass
        # Cancels all running commands at the start of test mode

        #def simulationInit(self):
        # make us blue for the pathplanner  ... does not seem to work
        # hal.initialize(500)
        # DriverStationSim.setAllianceStationId(hal.AllianceStationID.kBlue2)
        # DriverStationSim.notifyNewData()

