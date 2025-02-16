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
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig
import wpilib
from wpimath.units import inchesToMeters, lbsToKilograms
from wpimath.system.plant import DCMotor


class Robot(wpilib.TimedRobot):
    def robotInit(self):


        k_CAN_id = 4
        k_follower_CAN_id = 5

        k_gear_ratio = 15 # 9, 12, or 15 gear ratio said victor 1/30/25
                          # we need it seperate for the sim
        k_effective_pulley_diameter = inchesToMeters(1.91) # (https://www.andymark.com/products/25-24-tooth-0-375-in-hex-sprocket) although we're using rev, rev doesn't give a pitch diameter
        k_meters_per_revolution = math.pi * 2 * k_effective_pulley_diameter / k_gear_ratio # 2 because our elevator goes twice as fast as the chain because continuous rigging
        k_mass_kg = lbsToKilograms(25)
        k_plant = DCMotor.NEO(2)

        k_min_height = inchesToMeters(8)
        k_max_height = inchesToMeters(60)
        k_tolerance = 2 / 100 # 2 cm

        k_ff = 1 / 2.032
        # NOTE:
        # 2/15/25 14:37 ff of 0.6 and arbff of 0.38 v works
        # maybe decrease FF by a few hundredths

        k_sim_starting_height = 2

        self.k_config = SparkMaxConfig()

        self.k_config.inverted(True)

        self.k_config.encoder.positionConversionFactor(k_meters_per_revolution)
        self.k_config.encoder.velocityConversionFactor(k_meters_per_revolution / 60)

        # self.k_config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.)
        self.k_config.closedLoop.pidf(p=0, i=0, d=0, ff=0, slot=ClosedLoopSlot(0))
        self.k_config.closedLoop.IZone(iZone=0, slot=ClosedLoopSlot(0))
        self.k_config.closedLoop.IMaxAccum(0, slot=ClosedLoopSlot(0))
            
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

        self.kP = 0
        self.kI = 0
        self.kD = 0
        self.kIz = 0
        self.kFF = 0
        self.arbFF = 0
        self.kMaxOutput = 0
        self.kMinOutput = 0
        self.max_rpm = 0
        self.maxAccum = 0

        # Smart Motion coefficients
        self.max_vel = 0
        self.max_acc = 0
        self.allowed_err = 0.2

        self.slot1_kp = 0;
        self.slot1_ff = 0;
        self.slot1_maxaccel = 0
        self.slot1_maxvel = 0
        self.slot1_allowed_error = 0

        # TODO: if this doesn't work try making maxmotion constraints in slot 1

        wpilib.SmartDashboard.putNumber("P Gain", self.kP)
        wpilib.SmartDashboard.putNumber("I Gain", self.kI)
        wpilib.SmartDashboard.putNumber("D Gain", self.kD)
        wpilib.SmartDashboard.putNumber("I Zone", self.kIz)
        wpilib.SmartDashboard.putNumber("Feed Forward", self.kFF)
        wpilib.SmartDashboard.putNumber("Feed Forward - Arb Volts", self.arbFF)
        wpilib.SmartDashboard.putNumber("Max Output", self.kMaxOutput)
        wpilib.SmartDashboard.putNumber("Min Output", self.kMinOutput)
        wpilib.SmartDashboard.putNumber("IAccumMax", self.maxAccum)


        # wpilib.SmartDashboard.putNumber("Max Velocity", self.max_vel)
        # wpilib.SmartDashboard.putNumber("Max Acceleration", self.max_acc)
        # wpilib.SmartDashboard.putNumber("Allowed Closed Loop Error", self.allowed_err)
        # wpilib.SmartDashboard.putNumber("Slot 1 P", self.slot1_kp)
        # wpilib.SmartDashboard.putNumber("Slot 1 FF", self.slot1_ff)
        # wpilib.SmartDashboard.putNumber("Slot 1 max accel", self.slot1_maxaccel)
        # wpilib.SmartDashboard.putNumber("Slot 1 max vel", self.slot1_maxvel)
        # wpilib.SmartDashboard.putNumber("Slot 1 allowed error", self.slot1_allowed_error)
        wpilib.SmartDashboard.putNumber("Set Position", 0)
        wpilib.SmartDashboard.putNumber("Set Velocity", 0)

        wpilib.SmartDashboard.putBoolean("Velocity control?", False)
        wpilib.SmartDashboard.putBoolean("MaxMotion?", False)

    def teleopPeriodic(self):

        mode = wpilib.SmartDashboard.getBoolean("Velocity control?", False)
        self.arbFF = wpilib.SmartDashboard.getNumber("Feed Forward - Arb Volts", 0)
        if mode:
            setpoint = wpilib.SmartDashboard.getNumber("Set Velocity", 0)
            self.pid_controller.setReference(setpoint, rev.SparkMax.ControlType.kVelocity, arbFeedforward=self.arbFF, arbFFUnits=rev.SparkClosedLoopController.ArbFFUnits.kVoltage)
            pv = self.encoder.getVelocity()
        else:
            setpoint = wpilib.SmartDashboard.getNumber("Set Position", 0)
            maxmotion = wpilib.SmartDashboard.getBoolean("MaxMotion?", True)
            if maxmotion:
                self.pid_controller.setReference(setpoint, rev.SparkMax.ControlType.kMAXMotionPositionControl)
            else:
                self.pid_controller.setReference(setpoint, rev.SparkMax.ControlType.kPosition)
            pv = self.encoder.getPosition()

        # output = 0.6 + math.sin(3.14159 * self.counter / 50) / 4
        # wpilib.SmartDashboard.putNumber("output from sin", output)
        # self.pid_controller.setReference(value=output, ctrl=SparkMax.ControlType.kPosition, arbFFUnits=rev.SparkClosedLoopController.ArbFFUnits.kVoltage, arbFeedforward=0.38)

        self.counter += 1
        if self.counter % 2 == 0:

            p = wpilib.SmartDashboard.getNumber("P Gain", 0)
            i = wpilib.SmartDashboard.getNumber("I Gain", 0)
            d = wpilib.SmartDashboard.getNumber("D Gain", 0)
            iz = wpilib.SmartDashboard.getNumber("I Zone", 0)
            ff = wpilib.SmartDashboard.getNumber("Feed Forward", 0)
            max_out = wpilib.SmartDashboard.getNumber("Max Output", 0)
            min_out = wpilib.SmartDashboard.getNumber("Min Output", 0)
            # maxV = wpilib.SmartDashboard.getNumber("Max Velocity", 0)
            # maxA = wpilib.SmartDashboard.getNumber("Max Acceleration", 0)
            # allE = wpilib.SmartDashboard.getNumber("Allowed Closed Loop Error", 0)
            # slot1P = wpilib.SmartDashboard.getNumber("Slot 1 P", 0)
            # slot1FF = wpilib.SmartDashboard.getNumber("Slot 1 FF", 0)
            # iaccum = wpilib.SmartDashboard.getNumber("IAccumMax", 0)
            # slot1MaxA = wpilib.SmartDashboard.getNumber("Slot 1 max accel", 0)
            # slot1MaxV = wpilib.SmartDashboard.getNumber("Slot 1 max vel", 0)
            # slot1AllowedE = wpilib.SmartDashboard.getNumber("Slot 1 allowed error", 0)

            delta_config = SparkMaxConfig()
            
            # if iaccum != self.maxAccum:
            #     delta_config.closedLoop.IMaxAccum(iaccum)
            #     self.pid_controller.setIAccum(0)
            #     self.maxAccum = iaccum
            
            if p != self.kP:
                delta_config.closedLoop.P(p)
                self.kP = p
            
            if i != self.kI:
                delta_config.closedLoop.I(i)
                self.kI = i
            
            if d != self.kD:
                delta_config.closedLoop.D(d)
                self.kD = d
            
            if iz != self.kIz:
                delta_config.closedLoop.IZone(iz)
                self.kIz = iz
            
            if ff != self.kFF:
                delta_config.closedLoop.velocityFF(ff)
                self.kFF = ff
            
            if max_out != self.kMaxOutput or min_out != self.kMinOutput:
                delta_config.closedLoop.outputRange(min_out, max_out)
                self.kMinOutput = min_out
                self.kMaxOutput = max_out

            
            # if maxV != self.max_vel:
            #     delta_config.closedLoop.maxMotion.maxVelocity(maxV)
            #     self.max_vel = maxV
            #
            #
            # if maxA != self.max_acc:
            #     delta_config.closedLoop.maxMotion.maxAcceleration(maxA)
            #     self.max_acc = maxA
            #
            # if allE != self.allowed_err:
            #     delta_config.closedLoop.maxMotion.allowedClosedLoopError(allE)
            #     self.allowed_err = allE
            #
            # if slot1P != self.slot1_kp:
            #     delta_config.closedLoop.P(slot1P, slot=ClosedLoopSlot.kSlot1)
            #     self.slot1_kp = slot1P
            #
            # if slot1FF != self.slot1_ff:
            #     delta_config.closedLoop.velocityFF(slot1FF, slot=ClosedLoopSlot.kSlot1)
            #     self.slot1_ff = slot1FF
            #
            # if slot1MaxA != self.slot1_maxaccel:
            #     delta_config.closedLoop.maxMotion.maxAcceleration(slot1MaxA, slot=ClosedLoopSlot.kSlot1)
            #     self.slot1_maxaccel = slot1MaxA
            #
            # if slot1MaxV != self.slot1_maxvel:
            #     delta_config.closedLoop.maxMotion.maxVelocity(slot1MaxV, ClosedLoopSlot.kSlot1)
            #     self.slot1_maxvel = slot1MaxV
            #
            # if slot1AllowedE != self.slot1_allowed_error:
            #     delta_config.closedLoop.maxMotion.allowedClosedLoopError(slot1AllowedE, ClosedLoopSlot.kSlot1)
            #     self.slot1_allowed_error = slot1AllowedE

            self.k_config = self.k_config.apply(delta_config)
            self.motor.configure(self.k_config, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)

            # make old config
            #   every cycle:
            #   we make a config with only the changes (delta_config)
            #   we apply this only-changes config to our old config
            #   end goal: pass a config with the "default" settings, including those which we last applied
            #   i.e. changes should accumulate
            #       if we want changes: (i.e. if our new config is not our old config)
            #           apply changes

            # wpilib.SmartDashboard.putNumber("_SetPoint", setpoint)
            wpilib.SmartDashboard.putNumber("_POSITION", self.encoder.getPosition())
            wpilib.SmartDashboard.putNumber("_VELOCITY", self.encoder.getVelocity())
            wpilib.SmartDashboard.putNumber("_IAccum", self.pid_controller.getIAccum())


        # do these every time
        else:
            # wpilib.SmartDashboard.putNumber("_Process Variable", pv)
            wpilib.SmartDashboard.putNumber("_Output", self.motor.getAppliedOutput())
            wpilib.SmartDashboard.putNumber("_Current", self.motor.getOutputCurrent())

if __name__ == "__main__":
    wpilib.run(Robot)
