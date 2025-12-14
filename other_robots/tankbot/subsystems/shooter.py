import wpilib
import navx
import ntcore
from commands2 import Subsystem
from wpilib import SmartDashboard, DriverStation
from ntcore import NetworkTableInstance
import rev
from rev import SparkBase, SparkLowLevel  # trying to save some typing
from wpilib.drive import DifferentialDrive

import constants
from constants import ShooterConstants as sc


class Shooter(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Shooter')
        self.counter = sc.k_flywheel_counter_offset  # note this should be an offset in constants
        self.default_rpm = 3000

        # --------------- add motors and shooter rpm ----------------
        
        motor_type = rev.SparkMax.MotorType.kBrushless
        self.flywheel_left_leader = rev.SparkMax(sc.k_CANID_flywheel_left_leader, motor_type)
        self.flywheel_right_follower = rev.SparkMax(sc.k_CANID_flywheel_right_follower, motor_type)
        self.indexer_motor = rev.SparkMax(sc.k_CANID_indexer, motor_type)

        # convenient list of motors if we need to query or set all of them
        self.motors = [self.flywheel_left_leader, self.flywheel_right_follower, self.indexer_motor]

        # you need a controller to set velocity
        self.flywheel_controller = self.flywheel_left_leader.getClosedLoopController()
        self.flywheel_encoder = self.flywheel_left_leader.getEncoder()

        self.indexer_controller = self.indexer_motor.getClosedLoopController()

        # default parameters for the sparkmaxes reset and persist modes -
        self.rev_resets = SparkBase.ResetMode.kResetSafeParameters
        self.rev_persists = SparkBase.PersistMode.kPersistParameters if constants.k_burn_flash \
            else SparkBase.PersistMode.kNoPersistParameters

        # put the configs in a list matching the motors list
        self.configs = sc.k_flywheel_configs + [sc.k_indexer_config]  # FIXME - make this consistent

        # this should be its own function later - we will call it whenever we change brake mode
        rev_errors = [motor.configure(config, self.rev_resets, self.rev_persists)
                      for motor, config in zip(self.motors, self.configs)]

        # networktables
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable("datatable")  # for example

        # initialize states
        self.shooter_on = False
        self.indexer_on = False
        SmartDashboard.putBoolean('shooter_on', self.shooter_on)
        SmartDashboard.putBoolean('indexer_on', self.indexer_on)


    def stop_shooter(self):
        # three different ways to stop the shooter
        self.flywheel_left_leader.set(0)  # this sets the output to zero (number between -1 and 1) - it is "dumb"
        # self.shooter_l.setVoltage(0)  # this sets the voltage to zero (number between -12 and 12) - it is also "dumb"
        # self.flywheel_controller.setReference(value=0, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=0)

        self.shooter_on = False
        self.voltage = 0  # CJH for 2024 testing
        SmartDashboard.putBoolean('shooter_on', self.shooter_on)

    def stop_indexer(self):
        self.indexer_motor.set(0)  # this sets the output to zero (number between -1 and 1) - it is "dumb"

        self.indexer_on = False
        SmartDashboard.putBoolean('indexer_on', self.indexer_on)

    def set_shooter_rpm(self, rpm=1000):
        # multiple different ways to set the shooter
        # self.flywheel_left_leader.set(rpm)
        feed_forward = min(12, 12 * rpm / 5600)  # if there is no gearing, then this gets you close
        # rev is a pain in the ass - you have to pass EXACTLY the types it wants - no using "0" for the slots anymore
        self.flywheel_controller.setReference(value=rpm, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=feed_forward)
        print(f'set flywheel rpm to {rpm:.0f}')  # want to say what time it is, but can't import the container's timer easily
        self.shooter_on = True
        self.voltage = feed_forward  # 12 * rpm / 5600  # Guess
        SmartDashboard.putBoolean('shooter_on', self.shooter_on)

    def set_indexer_rpm(self, rpm=60):
        gear_ratio = 5
        feed_forward = min(12, gear_ratio * 12 * rpm / 5600)  # geared down 5x
        # rev is a pain in the ass - you have to pass EXACTLY the types it wants - no using "0" for the slots anymore
        self.indexer_controller.setReference(value=rpm, ctrl=SparkLowLevel.ControlType.kVelocity, slot=rev.ClosedLoopSlot.kSlot0, arbFeedforward=feed_forward)
        print(f'set indexer rpm to {rpm:.0f}')  # want to say what time it is, but can't import the container's timer easily
        self.indexer_on = True
        SmartDashboard.putBoolean('indexer_on', self.indexer_on)


    def get_velocity(self):
        return self.flywheel_encoder.getVelocity()

    def toggle_shooter(self, rpm):
        if self.shooter_on:
            self.stop_shooter()
        else:
            self.rpm = self.default_rpm if rpm is None else rpm
            self.set_shooter_rpm(self.rpm)

    def periodic(self) -> None:
        self.counter += 1

        # SmartDashboard.putBoolean('shooter_enable', self.shooter_enable)
        if self.counter % 20 == 0:
            pass
            # not too often
            #SmartDashboard.putNumber('shooter_rpm', self.shooter_l.getVelocity())
            #SmartDashboard.putNumber('shooter_rpm_target', self.rpm)
            #at_velocity = math.fabs(self.get_velocity() - self.rpm) < 200  # need to figure out this tolerance
            #SmartDashboard.putBoolean('shooter_ready', at_velocity)
            #SmartDashboard.putNumber('shooter_current', self.shooter_l.getOutputCurrent())
            #SmartDashboard.putNumber('shooter_output', self.shooter_l.getAppliedOutput())




#2024 shooter code
#
# class Shooter(Subsystem):
#     def __init__(self):
#         super().__init__()
#         self.setName('Shooter')
#         self.counter = 0
#         # self.PID_dict_vel = {'kP': 0.00021, 'kI': 0, 'kD': 0, 'kIz': 0, 'kFF': 0.000192}
#         self.smartmotion_maxvel = 5001  # rpm
#         self.smartmotion_maxacc = 5001
#         self.current_limit = 35
#         self.voltage = 5
#         self.rpm = 1000
#         self.rpm_list = [1000, 2000, 3000, 4000, 5000, 4000, 3000, 2000]  # cycle through these
#         self.rpm_index = 0
#
#         # initialize motors
#         # looking from back to front
#         motor_type = rev.CANSparkFlex.MotorType.kBrushless
#         self.flywheel_lower_left = rev.CANSparkFlex(constants.k_flywheel_lower_left_neo_port, motor_type)
#         self.flywheel_upper_left = rev.CANSparkFlex(constants.k_flywheel_upper_left_neo_port, motor_type)
#
#         # the follower is inverted
#         self.flywheel_lower_left.setInverted(False)
#         self.flywheel_upper_left.setInverted(True)
#
#         # encoders
#         self.flywheel_left_encoder = self.flywheel_lower_left.getEncoder()
#
#         # controller
#         self.flywheel_lower_left_controller = self.flywheel_lower_left.getPIDController()
#         self.flywheel_lower_left_controller.setP(0.0001)
#         self.flywheel_upper_left_controller = self.flywheel_upper_left.getPIDController()
#         self.flywheel_upper_left_controller.setP(0.0001)
#         self.kFF = 1.03 * 1 / 6784  # feed forward for a spark flex shooter
#         self.flywheel_lower_left_controller.setFF(self.kFF, 0)
#         self.flywheel_upper_left_controller.setFF(self.kFF, 0)
#
#         # toggle state
#         self.shooter_on = False
#         SmartDashboard.putBoolean('shooter_on', self.shooter_on)
#
#         # TODO configure_sparkmax()
#         # configure_sparkmax()
#
#     def set_flywheel(self, rpm):
#         # self.flywheel_left_controller.setReference(rpm, rev.CANSparkLowLevel.ControlType.kSmartVelocity, 0)
#         # self.shooter_voltage = self.shooter_voltage + 1 if self.shooter_voltage < 12 else 5  # CJH increment voltage test
#         # self.shooter_rpm = self.shooter_rpm + 1000 if self.shooter_rpm < 5000 else 1000  # AEH increment rpm test
#
#         if rpm is None:  # cycle through for testing or actually pass a value
#             self.rpm_index += 1
#             self.rpm = self.rpm_list[self.rpm_index % len(self.rpm_list)]
#         else:
#             self.rpm = rpm
#
#         use_voltage = False
#         if use_voltage:
#             self.flywheel_lower_left_controller.setReference(self.voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
#             self.flywheel_upper_left_controller.setReference(self.voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
#         else:
#             self.flywheel_lower_left_controller.setReference(self.rpm, rev.CANSparkFlex.ControlType.kVelocity, 0)
#             self.flywheel_upper_left_controller.setReference(self.rpm, rev.CANSparkFlex.ControlType.kVelocity, 0)
#         # self.flywheel_left_controller.setReference(self.shooter_voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
#         # self.flywheel_right_controller.setReference(self.shooter_voltage, rev.CANSparkLowLevel.ControlType.kVoltage, 0)
#         self.shooter_on = True
#         print(f'setting rpm to {rpm} {self.voltage}')
#         SmartDashboard.putBoolean('shooter_on', self.shooter_on)
#
#     def stop_shooter(self):
#         self.flywheel_lower_left_controller.setReference(0, rev.CANSparkFlex.ControlType.kVoltage)
#         self.flywheel_upper_left_controller.setReference(0, rev.CANSparkFlex.ControlType.kVoltage)
#         # self.flywheel_left_controller.setReference(0, rev.CANSparkFlex.ControlType.kVoltage)
#         # self.flywheel_right_controller.setReference(0, rev.CANSparkLowLevel.ControlType.kVoltage)
#         self.shooter_on = False
#         self.voltage = 0  # CJH for 2024 testing
#         SmartDashboard.putBoolean('shooter_on', self.shooter_on)
#
#     def get_velocity(self):
#         return self.flywheel_left_encoder.getVelocity()
#
#     def toggle_shooter(self, rpm):
#         if self.shooter_on:
#             self.stop_shooter()
#         else:
#             self.rpm = self.rpm if rpm is None else rpm
#             self.set_flywheel(self.rpm)
#
#     def periodic(self) -> None:
#
#         self.counter += 1
#
#         # SmartDashboard.putBoolean('shooter_enable', self.shooter_enable)
#         if self.counter % 20 == 0:
#             # not too often
#             SmartDashboard.putNumber('shooter_rpm', self.flywheel_left_encoder.getVelocity())
#             SmartDashboard.putNumber('shooter_rpm_target', self.rpm)
#             at_velocity = math.fabs(self.get_velocity() - self.rpm) < 200  # need to figure out this tolerance
#             SmartDashboard.putBoolean('shooter_ready', at_velocity)
#             SmartDashboard.putNumber('shooter_current', self.flywheel_lower_left.getOutputCurrent())
#             SmartDashboard.putNumber('shooter_output', self.flywheel_lower_left.getAppliedOutput())