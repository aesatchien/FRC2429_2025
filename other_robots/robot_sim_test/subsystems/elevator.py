import math

import wpilib
import rev

from commands2 import Subsystem
from wpilib import SmartDashboard
from playingwithfusion import TimeOfFlight

import constants

class Elevator(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Elevator')

        self.coral_positions = {key : constants.ElevatorConstants.k_positions[key] for key in ["stow", "ground", "ground_to_l1", "l1", "l2", "l2_to_l3", "l3", "l3_to_l4", "l4"]}
        
        self.counter = 5

        #initialize sparkmax controller + sparkmax configuration
        motor_type = rev.SparkMax.MotorType.kBrushless
        self.elevator_controller = rev.SparkMax(constants.ElevatorConstants.k_CAN_id, motor_type)

        self.config = rev.SparkMaxConfig()
        self.config.encoder.positionConversionFactor(constants.ElevatorConstants.k_elevator_encoder_conversion_factor).velocityConversionFactor(constants.ElevatorConstants.k_elevator_encoder_conversion_factor)
        self.config.closedLoop.pid(p=constants.ElevatorConstants.kP, i=constants.ElevatorConstants.kI, d=constants.ElevatorConstants.kD) #the ClosedLoopSpot parameter is for the target
        # self.config.closedLoop.IZone(iZone=LowerCrankConstants.kIZone, slot=ClosedLoopSlot(0))
        # self.config.closedLoop.IMaxAccum(LowerCrankConstants.kIMaxAccum, slot=ClosedLoopSlot(0))

        # set soft limits - do not let spark max put out power above/below a certain value
        self.config.softLimit.forwardSoftLimitEnabled(constants.ElevatorConstants.k_forward_limit_enabled)
        self.config.softLimit.reverseSoftLimitEnabled(constants.ElevatorConstants.k_reverse_limit_enabled)

        self.config.softLimit.forwardSoftLimit(constants.ElevatorConstants.k_forward_limit)
        self.config.softLimit.reverseSoftLimit(constants.ElevatorConstants.k_reverse_limit)

        self.elevator_controller.configure(config=self.config, 
                                           resetMode=rev.SparkMax.ResetMode.kResetSafeParameters, 
                                           persistMode=rev.SparkMax.PersistMode.kPersistParameters)

        #time of flight distance sensor:
        self.elevator_height_sensor = TimeOfFlight(constants.ElevatorConstants.k_timeofflight)
        self.elevator_height_sensor.setRangingMode(TimeOfFlight.RangingMode.kShort, 50)        

        #initialize closed loop controller (PID controller)
        self.elevator_pid = self.elevator_controller.getClosedLoopController()
        # self.elevator_pid.setSmartMotionAllowedClosedLoopError(1)

        # initialize the height of the elevator  - sensor is in mm, so stick with that
        initial_height = self.elevator_height_sensor.getRange() if wpilib.RobotBase.isReal() else constants.ElevatorConstants.k_positions["stow"]["elevator_height"] #note: range is got in mm, not in.
        self.set_height(initial_height)
        self.target_pos = "stow"
        self.has_coral = False #put in wrist later
        self.is_moving = False

        self.set_point = initial_height

        #get the encoder
        self.encoder = self.elevator_controller.getEncoder()
        self.encoder.setPosition(initial_height)

        self.elevator_pid.setReference(value=self.set_point, ctrl=rev.SparkMax.ControlType.kPosition) #js - what is pid slot?

        SmartDashboard.putNumber('elevator_setpoint', self.setpoint)
        SmartDashboard.putNumber('elevator_height', self.height)

    def get_height(self):
        if wpilib.RobotBase.isReal():
            return self.encoder.getPosition()
        else:
            return self.height


    def get_target_pos(self):
        return self.target_pos

    def set_height(self, height, mode="smartmotion"):
        """
        We can do this multiple ways -
        1) we can use wpilib with a PID controller
        2) we can make a PID controller ourselves
        3) or we can use the sparkmax's built-in PID / smartmotion
        """        

        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.elevator_pid.setReference(height, rev.SparkMax.ControlType.kSmartMotion) 
        elif mode == 'position':
            # just use the position PID
            self.elevator_pid.setReference(height, rev.SparkMax.ControlType.kPosition)

        self.setpoint = height
        SmartDashboard.putNumber('elevator_setpoint', self.setpoint)

        if wpilib.RobotBase.isSimulation():
            if height >= constants.ElevatorConstants.k_elevator_max_height:
                self.height = constants.ElevatorConstants.k_elevator_max_height
            elif height <= constants.ElevatorConstants.k_elevator_min_height:
                self.height = constants.ElevatorConstants.k_elevator_min_height
            else: self.height = height
            SmartDashboard.putNumber('elevator_height', self.height)
            
    def reset_height(self, height):
        self.height = height
        self.encoder.setPosition(height)
        SmartDashboard.putNumber('elevator_height', self.height)

    def next_pos(self, direction="down"):
        keys = list(self.coral_positions.keys())
        current_index = keys.index(self.target_pos)

        if direction == "up":
            if current_index + 1 < len(keys):
                self.target_pos = keys[current_index + 1]
                return (self.coral_positions[self.target_pos])
            return (self.coral_positions[self.target_pos])
        else:
            if current_index - 1 >= 0:
                self.target_pos = keys[current_index - 1]
                return (self.coral_positions[self.target_pos])
            return (self.coral_positions[self.target_pos])
    
    def set_has_coral(self, has_coral):
        self.has_coral = has_coral
        
    def get_has_coral(self):
        return self.has_coral
    
    def periodic(self) -> None: #overrides Subsystem periodic() method
        self.counter += 1
        if self.counter % 25 == 0:
            self.height = self.get_height()

            self.is_moving = abs(self.encoder.getVelocity()) > 1000

            SmartDashboard.putNumber('elevator_height', self.height)
            SmartDashboard.putNumber('elevator_tof', self.elevator_height_sensor.getRange())