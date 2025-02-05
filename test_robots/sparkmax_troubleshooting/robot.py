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
        ids = list(range(20, 28))
        print(f"ids: {ids}")
        self.sparkflexes = [rev.SparkFlex(deviceID=id, type=rev.SparkFlex.MotorType.kBrushless) for id in ids]
        # self.sparkflex = rev.SparkFlex(deviceID=25, type=rev.SparkFlex.MotorType.kBrushless)

        sparkflex_config = rev.SparkFlexConfig()
        sparkflex_config.closedLoop.pidf(0, 0, 0, 0.01)

        errors = []
        for sparkflex in self.sparkflexes:
            errors.append(sparkflex.configure(sparkflex_config, rev.SparkFlex.ResetMode.kResetSafeParameters, persistMode=rev.SparkFlex.PersistMode.kPersistParameters))

        print(f"errors: {errors}")

        wpilib.SmartDashboard.putNumber("setpoint", 0)
        print("** SPARKMAX TROUBLESHOOTING INITIALIZING! **")
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
        # self.sparkflex.getClosedLoopController().setReference(value=math.sin(self.counter/50), ctrl=rev.SparkFlex.ControlType.kVoltage, slot=rev.ClosedLoopSlot(0))
        for sparkflex in self.sparkflexes:
            sparkflex.getClosedLoopController().setReference(value=math.sin(self.counter/100) * 5, ctrl=rev.SparkFlex.ControlType.kVelocity, slot=rev.ClosedLoopSlot(0))
        # self.sparkflex.set(math.sin(self.counter/50))
        # wpilib.SmartDashboard.putNumber("sparkflex applied output: ", self.sparkflex.getAppliedOutput())
        wpilib.SmartDashboard.putNumber("sparkflex reference: ", math.sin(self.counter/100))
        self.counter += 1

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)
