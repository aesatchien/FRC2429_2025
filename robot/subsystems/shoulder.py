import math

import wpilib
from commands2 import Subsystem
from wpilib import SmartDashboard
import rev

import constants

class Shoulder(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Shoulder')

        self.coral_positions = {key : constants.ElevatorConstants.k_positions[key]["shoulder_pivot"] for key in ["stow", "ground", "l1", "l2", "l3", "l4"]}

        #initialize motors
        motor_type = rev.SparkMax.MotorType.kBrushless
        self.shoulder_controller = rev.SparkMax(constants.ShoulderConstants.k_CAN_shoulder_id, motor_type)

        self.config = rev.SparkMaxConfig()
        self.config.absoluteEncoder.positionConversionFactor(constants.ShoulderConstants.k_conversion_factor).velocityConversionFactor(constants.ShoulderConstants.k_conversion_factor)
        self.config.closedLoop.pid(p=constants.ShoulderConstants.kP, i=constants.ShoulderConstants.kI, d=constants.ShoulderConstants.kD) #what is the ClosedLoopSpot (?)

        # set soft limits
        self.config.softLimit.forwardSoftLimitEnabled(constants.ShoulderConstants.k_forward_limit_enabled)
        self.config.softLimit.reverseSoftLimitEnabled(constants.ShoulderConstants.k_reverse_limit_enabled)

        self.config.softLimit.forwardSoftLimit(constants.ShoulderConstants.k_forward_limit)
        self.config.softLimit.reverseSoftLimit(constants.ShoulderConstants.k_reverse_limit)

        self.shoulder_controller.configure(config=self.config, 
                                           resetMode=rev.SparkMax.ResetMode.kResetSafeParameters, 
                                           persistMode=rev.SparkMax.PersistMode.kPersistParameters)


        self.shoulder_pivot = constants.ElevatorConstants.k_positions["stow"]["shoulder_pivot"]

        #initialize closed loop controller (PID controller)
        self.shoulder_pid = self.shoulder_controller.getClosedLoopController()
        # self.elevator_pid.setSmartMotionAllowedClosedLoopError(1)

        #get the encoder
        self.absoluteEncoder = self.shoulder_controller.getAbsoluteEncoder()

        #set intial shoulder pivot position
        # self.set_shoulder_pivot(90)

        # SmartDashboard.putNumber('shoulder_setpoint', self.setpoint)

            
    def get_shoulder_pivot(self):
        if wpilib.RobotBase.isReal():
            return self.absoluteEncoder.getPosition()
        else:
            return self.shoulder_pivot

    def set_shoulder_pivot(self, angle, mode='smartmotion'):

        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.shoulder_pid.setReference(angle, rev.SparkMax.ControlType.kMAXMotionPositionControl)
        elif mode == 'position':
            # just use the position PID
            self.shoulder_pid.setReference(angle, rev.SparkMax.ControlType.kPosition)

        if wpilib.RobotBase.isSimulation():
            self.shoulder_pivot = angle % 360

        SmartDashboard.putNumber('wrist_setpoint', angle)

    def deltaAngle(self, target, current): #returns the shorter angle (and sign/direction) between two angles.
        target %= 360
        current %= 360

        if target < 0 or current < 0: raise ValueError("Angles must be positive")

        if current < target:
            current1 = current
            turnAmount1 = target - current1
            current2 = current + 360
            turnAmount2 = target - current2
            return turnAmount1 if abs(turnAmount1) < abs(turnAmount2) else turnAmount2

        else:
            target1 = target
            turnAmount1 = target1 - current
            target2 = target + 360
            turnAmount2 = target2 - current
        
            return turnAmount1 if abs(turnAmount1) < abs(turnAmount2) else turnAmount2