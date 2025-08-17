import math
import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard, DriverStation, Field2d
from ntcore import NetworkTableInstance
import rev
from rev import SparkMax, SparkFlex
from wpimath.filter import SlewRateLimiter
import ntcore

import constants

from helpers.questnav.questnav2 import QuestNav
from wpimath.geometry import Rotation2d, Transform2d, Pose2d
from wpimath.units import inchesToMeters

class Drivetrain(SubsystemBase):
    def __init__(self, container) -> None:
        super().__init__()
        self.setName('Drivetrain')
        self.container = container

        self.forward_switch = wpilib.DigitalInput(3)
        self.back_switch = wpilib.DigitalInput(2)
        self.analog_pedal = wpilib.AnalogInput(0)
        self.steering_encoder = wpilib.DutyCycleEncoder(6)
        # motors
        motor_type = rev.SparkMax.MotorType.kBrushless
        self.drive_l1 = rev.SparkMax(1, motor_type)
        self.drive_l2 = rev.SparkMax(2, motor_type)
        self.drive_r1 = rev.SparkMax(3, motor_type)
        self.drive_r2 = rev.SparkMax(4, motor_type)

        self.encoder_l = self.drive_l1.getEncoder()  # c.f. wpilib.Encoder(2, 3)
        self.encoder_r = self.drive_r1.getEncoder()
        # self.encoder_l.setPositionConversionFactor(constants.kDrivingEncoderPositionFactor)
        # self.encoder_l.setVelocityConversionFactor(constants.kDrivingEncoderVelocityFactor)
        # self.encoder_r.setPositionConversionFactor(constants.kDrivingEncoderPositionFactor)
        # self.encoder_r.setVelocityConversionFactor(constants.kDrivingEncoderVelocityFactor)


        self.drive_l1.setInverted(True)  # need to see if + pwr moves motor clockwise and therefore robot fwd
        self.drive_l2.setInverted(True)
        self.drive_r1.setInverted(False)  # need to see if + pwr moves motor clockwise and therefore robot fwd
        self.drive_r2.setInverted(False)

        self.drive_l2.follow(self.drive_l1)  # sets 2 to follow 1
        self.drive_r2.follow(self.drive_r1)

        if wpilib.RobotBase.isSimulation():  # have to do this to get the sim to update velocities, etc
            self.l1_drivingPIDController = self.drive_l1.getPIDController()
            self.r1_drivingPIDController = self.drive_r1.getPIDController()

        # class variables
        self.counter = 0
        self.thrust = 0
        self.twist = 0
        self.control_mode = constants.k_control_mode
        # set limits on how fast we can go in thrust (forward/reverse) and twist (turning)
        self.thrust_limit = constants.k_thrust_limit
        self.twist_limit = constants.k_twist_limit
        self.deadband = constants.k_deadband
        self.max_thrust_change = constants.k_thrust_slew_limit

        # slew_rate_limiter
        self.thrust_slew_rate_limiter = SlewRateLimiter(self.max_thrust_change, -self.max_thrust_change)
        self.twist_slew_rate_limiter = SlewRateLimiter(self.max_thrust_change, -self.max_thrust_change)

        # networktabl
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable("datatable")

        # publishers - call the set() function to write them
        # self.thrust_limit_pub = self.table.getDoubleTopic("thrust_limit").publish()
        self.max_thrust_change_pub = self.table.getDoubleTopic("max_thrust_change").publish()
        # self.twist_limit_pub = self.table.getDoubleTopic("twist_limit").publish()
        self.control_mode_pub = self.table.getStringTopic("control_mode").publish()
        # self.thrust_limit_pub.set(self.thrust_limit)
        # self.max_thrust_change_pub.set(self.max_thrust_change)
        # self.twist_limit_pub.set(self.twist_limit)
        self.control_mode_pub.set(self.control_mode)
        # subscribers - call the get() function to read them

        self.thrust_limit_sub = self.table.getDoubleTopic("thrust_limit").subscribe(self.thrust_limit)
        self.max_thrust_change_sub = self.table.getDoubleTopic("max_thrust_change").subscribe(self.max_thrust_change)
        self.twist_limit_sub = self.table.getDoubleTopic("twist_limit").subscribe(self.twist_limit)
        self.control_mode_sub = self.table.getStringTopic("control_mode").subscribe(self.control_mode)
        
        self.questnav = QuestNav()
        self.quest_to_robot = Transform2d(inchesToMeters(-12.5-0.55), 0, Rotation2d().fromDegrees(0))
        # self.quest_to_robot = Transform2d(inchesToMeters(4), 0, Rotation2d().fromDegrees(0))
        self.quest_field = Field2d()

    def drive(self):
        # teleop is quite simple for this robot - just set the motors
        self.drive_l1.set(self.thrust + self.twist)
        self.drive_r1.set(self.thrust - self.twist)

        if wpilib.RobotBase.isSimulation():
            max_meters_per_second = 4
            left_velocity = max_meters_per_second *  (self.thrust + self.twist)
            right_velocity = max_meters_per_second * (self.thrust - self.twist)
            self.l1_drivingPIDController.setReference(left_velocity, rev.CANSparkBase.ControlType.kVelocity)
            self.r1_drivingPIDController.setReference(right_velocity, rev.CANSparkBase.ControlType.kVelocity)


    def drive_forward(self, power):
        self.drive_l1.set(power)
        self.drive_r1.set(power)

    def periodic(self):
        # here we do all the checking of the robot state - read inputs, calculate outputs
        self.counter += 1
        # calculate outputs based on if we care controlled by driver station joystick or human in chairbot
        if self.control_mode == 'remote':  # control from driver station joystick
            self.thrust = self.container.joystick.getRawAxis(1) * -1 * self.thrust_limit
            self.twist = self.container.joystick.getRawAxis(4) * self.twist_limit
        elif self.control_mode == 'onboard':  # control from the human sitting on chairbot
            # determine if we are moving forward, backward, or in neutral based on 3-position switch
            direction = 0
            if not self.forward_switch.get():
                direction = 1
            elif not self.back_switch.get():
                direction = -1
            # get thrust from the analog pedal.  It rests at 0.8V and goes to 3.2V when floored
            # to map this from 0 to 1, subtract 0.8 and divide by (3.2-0.8)
            self.thrust = direction * max(0,
                                          self.analog_pedal.getVoltage() - 0.8) / 2.4  # math to get thrust value between 0 and 1 from pedal
            self.thrust *= self.thrust_limit
            # analog encoder is at 0.61 at neutral position (not turning) and travels .225 above and below that
            # so to scale from -1 to 1, subtract the central value and divide by .225
            # as of 080324, switched encoders, neutral is now 0.93
            self.twist = direction * (self.steering_encoder.get() - 0.926) / 0.225
            self.twist *= self.twist_limit

            # if math.fabs(self.twist) < self.deadband:
            #     self.twist = 0
            # else:
            #     self.twist = self.twist

            self.twist = 0 if math.fabs(self.twist) < self.deadband else self.twist
            # self.thrust = 0 if math.fabs(self.thrust) < self.deadband else self.thrust
        else:  # end of control mode
            pass  # no other control modes defined

        # Import pose from QuestNav.
        self.quest_periodic()

        # enforce acceleration limit
        self.thrust = self.thrust_slew_rate_limiter.calculate(self.thrust)
        self.twist = self.twist_slew_rate_limiter.calculate(self.twist)

        if self.counter % 10 == 0:
            # read networktables for updates to our mode/limit values
            nt_control_mode = self.control_mode_sub.get()
            if nt_control_mode != self.control_mode:
                if nt_control_mode in ['remote', 'onboard']:
                    self.control_mode = nt_control_mode
                    print(f'setting control_mode to {self.control_mode}')
                    wpilib.SmartDashboard.putString('control_mode', self.control_mode)
                else:
                    pass

            nt_thrust_limit = self.thrust_limit_sub.get()
            if nt_thrust_limit != self.thrust_limit:
                if 0.1 <= nt_thrust_limit <= 1:
                    self.thrust_limit = nt_thrust_limit
                    print(f'setting thrust_limit to {self.thrust_limit}')
                else:
                    pass
                    # self.thrust_limit_pub.set(self.thrust_limit)

            nt_max_thrust_change = self.max_thrust_change_sub.get()
            if nt_max_thrust_change != self.max_thrust_change:
                if 0.02 <= nt_max_thrust_change <= 10:
                    self.max_thrust_change = nt_max_thrust_change
                    print(f'setting max_thrust_change to {self.max_thrust_change}')
                    wpilib.SmartDashboard.putNumber('max_thrust_change', self.max_thrust_change)
                    # reset the slew rate limiter with the new value (if we are in this limit mode)
                    self.thrust_slew_rate_limiter = SlewRateLimiter(self.max_thrust_change, -self.max_thrust_change)
                else:
                    self.max_thrust_change_pub.set(self.max_thrust_change)

            nt_twist_limit = self.twist_limit_sub.get()
            if nt_twist_limit != self.twist_limit:
                if 0.1 <= nt_twist_limit <= 1:
                    self.twist_limit = nt_twist_limit
                    print(f'setting twist_limit to {self.twist_limit}')
                else:
                    pass
                    # self.twist_limit_pub.set(self.twist_limit)

            # update the SmartDashboard
            wpilib.SmartDashboard.putNumber('twist', self.twist)
            wpilib.SmartDashboard.putNumber('thrust', self.thrust)
            wpilib.SmartDashboard.putNumber('analog_pedal', self.analog_pedal.getVoltage())
            wpilib.SmartDashboard.putBoolean('forward_switch', self.forward_switch.get())
            wpilib.SmartDashboard.putBoolean('back_switch', self.back_switch.get())
            wpilib.SmartDashboard.putNumber('steering_encoder_abs', self.steering_encoder.getAbsolutePosition())
            wpilib.SmartDashboard.putNumber('steering_encoder', self.steering_encoder.get())

    def quest_periodic(self) -> None:
        self.questnav.command_periodic()
        SmartDashboard.putBoolean("QUEST_CONNECTED", self.questnav.is_connected())
        SmartDashboard.putBoolean("QUEST_TRACKING", self.questnav.is_tracking())
        quest_pose = self.questnav.get_pose().transformBy(self.quest_to_robot)

        SmartDashboard.putString("QUEST_POSE", str(quest_pose))
        self.quest_field.setRobotPose(quest_pose)
        SmartDashboard.putData("QUEST_FIELD", self.quest_field)
        if 0 < quest_pose.x < 17.658 and 0 < quest_pose.y < 8.131 and self.questnav.is_connected():
            SmartDashboard.putBoolean("QUEST_POSE_ACCEPTED", True)
            # print("Quest Timestamp: " + str(self.questnav.get_app_timestamp()))
            # print("System Timestamp: " + str(utils.get_system_time_seconds()))
            # if abs(self.questnav.get_data_timestamp() - utils.get_current_time_seconds()) < 5:
            #     print("Timestamp in correct epoch.")
            #self.add_vision_measurement(quest_pose,
            #                            utils.fpga_to_current_time(self.questnav.get_data_timestamp()),
            #                            (0.02, 0.02, 0.035))
        else:
            SmartDashboard.putBoolean("QUEST_POSE_ACCEPTED", False)

    def reset_pose_with_quest(self, pose: Pose2d) -> None:
        self.reset_pose(pose)
        self.questnav.set_pose(pose.transformBy(self.quest_to_robot.inverse()))

    def reset_odometry(self):
        """Reset robot odometry at the Subwoofer."""
        if True: #DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.reset_pose(Pose2d(14.337, 4.020, Rotation2d.fromDegrees(0)))
            #self.set_operator_perspective_forward(Rotation2d.fromDegrees(180))
            self.questnav.set_pose(Pose2d(14.337, 4.020, Rotation2d.fromDegrees(0)).transformBy(self.quest_to_robot.inverse()))
        else:
            self.reset_pose(Pose2d(3.273, 4.020, Rotation2d.fromDegrees(180)))
            #self.set_operator_perspective_forward(Rotation2d.fromDegrees(0))
            self.questnav.set_pose(Pose2d(3.273, 4.020, Rotation2d.fromDegrees(180)).transformBy(self.quest_to_robot.inverse()))
