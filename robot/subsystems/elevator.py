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

        self.coral_positions = {key : constants.ElevatorConstants.k_sim_positions_degrees[key] for key in ["stow", "ground", "l1", "l2", "l3", "l4"]}
        
        self.counter = 5

        #initialize sparkmax controller + sparkmax configuration
        motor_type = rev.SparkMax.MotorType.kBrushless

        self.elevator_controller = rev.SparkMax(constants.ElevatorConstants.k_CAN_id, motor_type)

        controller_revlib_error = self.elevator_controller.configure(config=constants.ElevatorConstants.k_config,
                                           resetMode=rev.SparkMax.ResetMode.kResetSafeParameters, 
                                           persistMode=rev.SparkMax.PersistMode.kPersistParameters)

        self.elevator_follower_controller = rev.SparkMax(constants.ElevatorConstants.k_follower_CAN_id, motor_type)

        follower_revlib_error = self.elevator_follower_controller.configure(config=constants.ElevatorConstants.k_follower_config,
                                                    resetMode=rev.SparkMax.ResetMode.kResetSafeParameters,
                                                    persistMode=rev.SparkMax.PersistMode.kPersistParameters)

        print(f"Configured elevator sparkmaxes.\nElevator controller status: {controller_revlib_error}\nFollower controller status: {follower_revlib_error}")

        #time of flight distance sensor:
        self.elevator_height_sensor = TimeOfFlight(constants.ElevatorConstants.k_timeofflight)
        self.elevator_height_sensor.setRangingMode(TimeOfFlight.RangingMode.kShort, 50)        

        #initialize closed loop controller (PID controller)
        self.elevator_pid = self.elevator_controller.getClosedLoopController()
        # self.elevator_pid.setSmartMotionAllowedClosedLoopError(1)

        # initialize the height of the elevator  - sensor is in mm, so convert to m
        initial_height = 1/100 * self.elevator_height_sensor.getRange() if wpilib.RobotBase.isReal() else constants.ElevatorConstants.k_sim_starting_height # note: range is got in mm, not in.
        self.set_height(initial_height)

        self.target_pos = "stow"

        self.set_point = initial_height

        #get the encoder
        self.encoder = self.elevator_controller.getEncoder()
        self.encoder.setPosition(initial_height)

        self.elevator_pid.setReference(value=self.set_point, ctrl=rev.SparkMax.ControlType.kPosition) #js - what is pid slot?

        self.has_coral = False

        SmartDashboard.putNumber('elevator_setpoint', self.setpoint)
        SmartDashboard.putNumber('elevator_height', self.encoder.getPosition())

    def get_height(self):
        return self.encoder.getPosition()

    def get_target_pos(self):
        return self.target_pos

    def set_height(self, height, mode="position"):
        """
        We can do this multiple ways -
        1) we can use wpilib with a PID controller
        2) we can make a PID controller ourselves
        3) or we can use the sparkmax's built-in PID / smartmotion
        """        

        if mode == 'maxmotion':
            # use smartmotion to send you there quickly
            self.elevator_pid.setReference(height, rev.SparkMax.ControlType.kMAXMotionPositionControl) 
            print(f"setting elevator maxmotion reference to {height}")
        elif mode == 'position':
            # just use the position PID
            self.elevator_pid.setReference(height, rev.SparkMax.ControlType.kPosition)
            print(f"setting elevator pid reference to {height}")
        else:
            raise ValueError("invalid control mode!")

        self.setpoint = height

        SmartDashboard.putNumber('elevator_setpoint', self.setpoint)
            
    def reset_height(self, height):
        self.height = height
        self.encoder.setPosition(height)
        SmartDashboard.putNumber('elevator_height', self.height)

    # unused 2/5/2025
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
        if self.counter % 10 == 0:
            self.height = self.get_height()

            self.is_moving = abs(self.encoder.getVelocity()) > 1000

            SmartDashboard.putNumber('elevator_height', self.height)
            SmartDashboard.putNumber('elevator_tof', self.elevator_height_sensor.getRange())
