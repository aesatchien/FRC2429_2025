#!/usr/bin/env python3

# This is REV's SmartMotion tuning tool.
# NOTE SmartMotion only uses ONE PIDF slot - the one for velocity.  You aren't actually changing position control PIDs.
# Also not the trickery about conversion factors in the SparkMax and how they affect decel and PIDFs
# (they all scale with the factor, and if you change position, also change velocity)
# Good starting values for NEO motor with no load  below.
# probably need to bump kFF first once we have a load.  With no load I'm not seeing much kp influence.
# Conversion factor = 1 :  kp 5e-5, ki 1e-6, kd 0, kiz 1e-5, kFF 1.6e-4
# Conversion factor = 0.1 :  kp 5e-5, ki 1e-6, kd 0, kiz 1e-5, kFF 1.9e-3

import rev
import math
from rev import ClosedLoopSlot, SparkMaxConfig
import wpilib
from wpilib._wpilib import SmartDashboard
from wpimath.units import inchesToMeters, lbsToKilograms
from wpimath.system.plant import DCMotor


class Robot(wpilib.TimedRobot):
    def robotInit(self):

        self.currents = [0.0] * 10

        # Small accel limit -> jerk on long trips, but not on short
        # Hi accel limit -> jerk on short trips, but not long

        k_CAN_id = 4
        k_follower_CAN_id = 5

        k_gear_ratio = 15 # 9, 12, or 15 gear ratio said victor 1/30/25
                          # we need it seperate for the sim
        k_effective_pulley_diameter = inchesToMeters(1.91) # (https://www.andymark.com/products/25-24-tooth-0-375-in-hex-sprocket) although we're using rev, rev doesn't give a pitch diameter
        k_meters_per_revolution = math.pi * 2 * k_effective_pulley_diameter / k_gear_ratio # 2 because our elevator goes twice as fast as the chain because continuous rigging

        k_min_height = inchesToMeters(8)
        k_max_height = inchesToMeters(60)

        self.k_config = SparkMaxConfig()

        self.k_config.inverted(True)

        self.k_config.encoder.positionConversionFactor(k_meters_per_revolution)
        self.k_config.encoder.velocityConversionFactor(k_meters_per_revolution)

        # self.k_config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.)
        self.k_config.closedLoop.pidf(p=1, i=0, d=4, ff=0, slot=ClosedLoopSlot(0))
        self.k_config.closedLoop.outputRange(-1, 1)

        self.k_config.closedLoop.pidf(p=0, i=0, d=0, ff=0.9, slot=ClosedLoopSlot(1))
        self.k_config.closedLoop.outputRange(-1, 1, slot=ClosedLoopSlot.kSlot1)

        self.k_config.closedLoop.maxMotion.maxVelocity(1 * 60)
        self.k_config.closedLoop.maxMotion.maxAcceleration(4 * 60)
        self.k_config.closedLoop.maxMotion.allowedClosedLoopError(0.005)

        self.k_config.closedLoop.maxMotion.maxVelocity(0.5 * 60, ClosedLoopSlot.kSlot1)
        self.k_config.closedLoop.maxMotion.maxAcceleration(4 * 60, ClosedLoopSlot.kSlot1)
        self.k_config.closedLoop.maxMotion.allowedClosedLoopError(0.005, ClosedLoopSlot.kSlot1)


        self.k_config.softLimit.forwardSoftLimit(k_max_height)
        self.k_config.softLimit.reverseSoftLimit(k_min_height)

        self.k_config.softLimit.forwardSoftLimitEnabled(True)
        self.k_config.softLimit.reverseSoftLimitEnabled(True)

        self.k_config.smartCurrentLimit(40)

        k_follower_config = SparkMaxConfig()
        k_follower_config.follow(k_CAN_id, invert=True)

        self.motor = rev.SparkMax(k_CAN_id, rev.SparkMax.MotorType.kBrushless)
        # self.motor.restoreFactoryDefaults()

        self.follower = rev.SparkMax(k_follower_CAN_id, rev.SparkMax.MotorType.kBrushless)

        self.motor.configure(self.k_config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
        self.follower.configure(k_follower_config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

        self.pid_controller = self.motor.getClosedLoopController()

        self.encoder = self.motor.getEncoder()
        self.encoder.setPosition(inchesToMeters(8))

        self.counter = 0

        wpilib.SmartDashboard.setDefaultNumber("Target Position", 0)
        wpilib.SmartDashboard.setDefaultNumber("Target Velocity", 0)
        wpilib.SmartDashboard.setDefaultBoolean("Velocity?", False)
        wpilib.SmartDashboard.setDefaultBoolean("Reset Encoder", False)

    def teleopPeriodic(self):

        if wpilib.SmartDashboard.getBoolean("Velocity?", False):
            target_vel = wpilib.SmartDashboard.getNumber("Target Velocity", 0)
            self.pid_controller.setReference(target_vel, rev.SparkMax.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1)

        else:
            target_pos = wpilib.SmartDashboard.getNumber("Target Position", inchesToMeters(8))
            self.pid_controller.setReference(target_pos, rev.SparkMax.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, arbFeedforward=0.38, arbFFUnits=rev.SparkClosedLoopController.ArbFFUnits.kVoltage)

    def robotPeriodic(self) -> None:
        self.counter += 1
        self.currents[self.counter % 10] = self.motor.getOutputCurrent()
        SmartDashboard.putNumber("Actual Position", self.encoder.getPosition())
        SmartDashboard.putNumber("Actual Velocity", self.encoder.getVelocity())
        SmartDashboard.putNumber("Current", sum(self.currents)/10)
        SmartDashboard.putNumber("Applied output", self.motor.getAppliedOutput())
        return super().robotPeriodic()

if __name__ == "__main__":
    wpilib.run(Robot)
