import math

import rev
import wpilib
from wpimath.system.plant import DCMotor
from commands2 import Subsystem
from wpilib import SmartDashboard
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig, SparkMaxSim, SparkMax
from wpimath.units import inchesToMeters, radiansToDegrees, degreesToRadians
import constants

class Climber(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Climber')
        self.counter = 3

        self.sparkmax = rev.SparkMax(constants.ClimberConstants.k_CAN_id, rev.SparkMax.MotorType.kBrushless)

        self.sparkmax.configure(config=constants.ClimberConstants.k_config,
                                resetMode=SparkMax.ResetMode.kResetSafeParameters,
                                persistMode=SparkMax.PersistMode.kPersistParameters)
        
        #configure PID controller
        self.controller = self.sparkmax.getClosedLoopController()

        #get encoder
        self.encoder = self.sparkmax.getEncoder()
        self.encoder.setPosition(constants.ClimberConstants.k_climber_motor_stowed_angle)

        #indicators
        # self.is_moving = False
        self.tolerance = constants.ClimberConstants.k_tolerance  # rads equal to five degrees - then we will be "at goal"
        self.goal = constants.ClimberConstants.k_climber_motor_stowed_angle

    def set_reference(self, radians: float):
        radians = radians if radians < constants.ClimberConstants.k_climber_forward_rotation_limit else constants.ClimberConstants.k_climber_forward_rotation_limit
        radians = radians if radians > constants.ClimberConstants.k_climber_reverse_rotation_limit else constants.ClimberConstants.k_climber_reverse_rotation_limit

        self.goal = radians

        self.controller.setReference(self.goal, SparkMax.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0)

    def set_brake_mode(self, mode='brake'):
        if mode == 'brake':
            constants.ClimberConstants.k_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        else:
            constants.ClimberConstants.k_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)

        self.sparkmax.configure(config=constants.ClimberConstants.k_config,
                                resetMode=SparkMax.ResetMode.kResetSafeParameters,
                                persistMode=SparkMax.PersistMode.kPersistParameters)
        
    def get_angle(self):
        return self.encoder.getPosition()

    def move_degrees(self, delta_degrees: float, silent=True) -> None:  # way to bump up and down for testing
        current_angle = self.get_angle()
        goal = current_angle + degreesToRadians(delta_degrees)
        self.set_reference(goal)
        if not silent:
            message = f'setting {self.getName()} from {current_angle:.2f} to {self.goal:.2f}'
            print(message)

    def is_at_goal(self):
        return math.fabs(self.get_angle() - self.goal) < self.tolerance

    def is_stowed(self):
        return math.fabs(self.get_angle() - constants.ClimberConstants.k_climber_motor_stowed_angle) < self.tolerance

    def is_ready(self):
        return math.fabs(self.get_angle() - constants.ClimberConstants.k_climber_motor_ready) < self.tolerance

    def periodic(self) -> None:
        # What if we didn't call the below for a few cycles after we set the position?
        super().periodic()  # this does the automatic motion profiling in the background
        self.counter += 1
        if self.counter % 5 == 0:
            self.angle = self.encoder.getPosition()
            self.at_goal = math.fabs(self.angle - self.goal) < self.tolerance
            self.error = self.angle - self.goal

            debug = True
            if debug:
                wpilib.SmartDashboard.putBoolean(f'{self.getName()}_at_goal', self.at_goal)
                wpilib.SmartDashboard.putNumber(f'{self.getName()}_error', radiansToDegrees(self.error))
                wpilib.SmartDashboard.putNumber(f'{self.getName()}_goal', radiansToDegrees(self.goal))
                # wpilib.SmartDashboard.putNumber(f'{self.getName()}_curr_sp',) not sure how to ask for this - controller won't give it
                wpilib.SmartDashboard.putNumber(f'{self.getName()}_output', self.sparkmax.getAppliedOutput())
            self.is_moving = abs(self.encoder.getVelocity()) > 0.001  # m per second
            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_is_moving', self.is_moving)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_spark_angle', radiansToDegrees(self.angle))