from commands2.subsystem import Subsystem
import math
import wpilib
from rev import ClosedLoopSlot, SparkMax
from constants import WristConstants
import constants
from subsystems.elevator import Elevator
from subsystems.pivot import Pivot

class Wrist(Subsystem):

    def __init__(self, pivot: Pivot, elevator: Elevator):

        self.sparkmax = SparkMax(WristConstants.k_CAN_id, SparkMax.MotorType.kBrushless)

        controller_revlib_error = self.sparkmax.configure(config=WristConstants.k_config, 
                                resetMode=SparkMax.ResetMode.kResetSafeParameters,
                                persistMode=SparkMax.PersistMode.kPersistParameters)

        print(f"Configured wrist sparkmax. Wrist controller status: {controller_revlib_error}")

        self.encoder = self.sparkmax.getEncoder()
        # self.abs_encoder = self.sparkmax.getAbsoluteEncoder()

        # if wpilib.RobotBase.isReal():
        #     self.encoder.setPosition(self.abs_encoder.getPosition()) # may have to set offset here if the zeroOffset kParamInvalid error isn't fixed
        #                                                              # if so put the offset here into wristconstants
        # else:
        #     self.encoder.setPosition(WristConstants.k_starting_angle)

        self.pivot = pivot
        self.elevator = elevator

        self.encoder.setPosition(WristConstants.k_starting_angle)
        self.controller = self.sparkmax.getClosedLoopController()
        self.setpoint = self.encoder.getPosition()
        self.counter = constants.WristConstants.k_counter_offset

    def set_position(self, radians: float, control_type: SparkMax.ControlType=SparkMax.ControlType.kPosition, closed_loop_slot=0) -> None:

        if control_type not in [SparkMax.ControlType.kPosition, SparkMax.ControlType.kMAXMotionPositionControl]:
            raise ValueError("Commanding something other than the position of the wrist seems like a terrible idea.")

        self.setpoint = radians
        self.controller.setReference(value=self.setpoint, ctrl=control_type, slot=ClosedLoopSlot(closed_loop_slot))

    def increment_position(self, delta_radians: float, control_type: SparkMax.ControlType=SparkMax.ControlType.kPosition) -> None:
        # CJH added 20250224 for debugging VIA GUI
        if control_type not in [SparkMax.ControlType.kPosition, SparkMax.ControlType.kMAXMotionPositionControl]:
            raise ValueError("Commanding something other than the position of the wrist seems like a terrible idea.")

        self.setpoint = self.get_angle() + delta_radians
        self.controller.setReference(value=self.setpoint, ctrl=control_type, slot=ClosedLoopSlot(0))

    def set_encoder_position(self, radians: float):
        self.encoder.setPosition(radians)

    def get_angle(self) -> float:
        return self.encoder.getPosition()
        # return self.abs_encoder.getPosition()

    def get_at_setpoint(self) -> bool:
        return abs(self.encoder.getPosition() - self.setpoint) < WristConstants.k_tolerance

    def is_safe_to_move(self) -> bool:
        pivot_in_safe_position = (self.pivot.get_angle() > WristConstants.k_max_arm_angle_where_spinning_dangerous or
                                   self.pivot.get_angle() < WristConstants.k_min_arm_angle_where_spinning_dangerous)

        elevator_in_safe_position = self.elevator.get_height() > WristConstants.k_max_elevator_height_where_spinning_dangerous

        # return pivot_in_safe_position or elevator_in_safe_position
        return True  # CJH 20250302  - always True now

    def periodic(self) -> None:

        self.counter += 1
        # if (not self.is_safe_to_move() and 
        #     (self.get_angle() > WristConstants.k_stowed_max_angle or self.get_angle() < WristConstants.k_stowed_min_angle)):
        #     # the wrist is currently in a bad position, so retract it!
        #     self.set_position(constants.k_positions["stow"]["wrist_pivot"])

        if self.counter % 10 == 0:

            # wpilib.SmartDashboard.putNumber("wrist abs encoder, rad", self.abs_encoder.getPosition())
            wpilib.SmartDashboard.putNumber("wrist relative encoder, rad", self.encoder.getPosition())
            # wpilib.SmartDashboard.putNumber("wrist abs encoder, degrees", math.degrees(self.abs_encoder.getPosition()))
            wpilib.SmartDashboard.putNumber("wrist relative encoder, degrees", math.degrees(self.encoder.getPosition()))

            if constants.WristConstants.k_nt_debugging:  # extra debugging info for NT
                pass

        return super().periodic()
