#!/usr/bin/env python3
import math
import typing
import wpilib
import commands2
import rev

class MyRobot(wpilib.TimedRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        self.sparkmax = rev.SparkMax(deviceID=26, type=rev.SparkMax.MotorType.kBrushless)

        sparkmax_config = rev.SparkMaxConfig()
        sparkmax_config.closedLoop.P(1)

        self.sparkmax.configure(sparkmax_config, rev.SparkMax.ResetMode.kResetSafeParameters, persistMode=rev.SparkMax.PersistMode.kPersistParameters)
        wpilib.SmartDashboard.putNumber("setpoint", 0)
        self.counter = 0

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        pass

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        pass

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        self.sparkmax.getClosedLoopController().setReference(value=math.sin(self.counter/50), ctrl=rev.SparkMax.ControlType.kPosition)
        wpilib.SmartDashboard.putNumber("sparkmax applied output: ", self.sparkmax.getAppliedOutput())
        wpilib.SmartDashboard.putNumber("sparkmax reference: ", math.sin(self.counter/50))
        self.counter += 1

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)
