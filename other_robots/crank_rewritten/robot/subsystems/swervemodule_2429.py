import wpilib
from rev import CANSparkMax, CANSparkFlex
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpilib import AnalogEncoder, AnalogPotentiometer
from wpimath.controller import PIDController
from misc.configure_controllers import configure_sparkmax
import math

import constants
from .swerve_constants import ModuleConstants
from .swerve_constants import DriveConstants as dc


class SwerveModule:
    def __init__(self, drivingCANId: int, turningCANId: int, encoder_analog_port: int, turning_encoder_offset: float,
                 driving_inverted=False, turning_inverted=False, label='') -> None:

        self.label = label
        self.desiredState = SwerveModuleState(0.0, Rotation2d())  # initialize desired state
        self.turning_output = 0

        # get our two motor controllers and a simulation dummy  TODO: set motor types in swerve_constants?
        self.drivingSparkMax = CANSparkFlex(drivingCANId, CANSparkFlex.MotorType.kBrushless)
        self.turningSparkMax = CANSparkMax(turningCANId, CANSparkMax.MotorType.kBrushless)
        if wpilib.RobotBase.isSimulation():  # check in sim to see if we are reacting to inputs
            pass
            # self.dummy_motor_driving = wpilib.PWMSparkMax(drivingCANId-16)
            # self.dummy_motor_turning = wpilib.PWMSparkMax(turningCANId-16)

        #  ---------------- DRIVING  SPARKMAX  ------------------
        # Factory reset, so we get the SPARKS MAX to a known state before configuring them
        if constants.k_reset_sparks_to_default:
            self.drivingSparkMax.restoreFactoryDefaults()
        self.drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode)
        self.drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
        self.drivingSparkMax.setInverted(driving_inverted)
        self.drivingSparkMax.enableVoltageCompensation(constants.k_volt_compensation)

        # Get driving encoder from the sparkmax
        self.drivingEncoder = self.drivingSparkMax.getEncoder()
        self.drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
        self.drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)
        # Configure driving PID gains for the driving motor
        self.drivingPIDController = self.drivingSparkMax.getPIDController()
        self.drivingPIDController.setFeedbackDevice(self.drivingEncoder)
        # Set/Save the configurations. If a SPARK MAX browns out during operation, it will maintain the above config
        # Saves p,i,d, ff, min/max, smart motion etc - do we need more than one slot used?  might as well always be
        configure_sparkmax(sparkmax=self.drivingSparkMax, pid_controller=self.drivingPIDController, can_id=drivingCANId, slot=0,
                               burn_flash=constants.k_burn_flash, pid_dict=ModuleConstants.k_PID_dict_vel, pid_only=False)
        # configure_sparkmax(sparkmax=self.drivingSparkMax, pid_controller=self.drivingPIDController, can_id=drivingCANId, slot=1,
        #                        burn_flash=constants.k_burn_flash, pid_dict=ModuleConstants.k_PID_dict_vel, pid_only=False)

        #  ---------------- TURNING SPARKMAX  ------------------
        self.turningSparkMax.restoreFactoryDefaults()
        self.turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode)
        self.turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)
        self.turningSparkMax.setInverted(turning_inverted)
        self.turningSparkMax.enableVoltageCompensation(constants.k_volt_compensation)

        # Setup encoders for the turning SPARKMAX - just to watch it if we need to for velocities, etc.
        # WE DO NOT USE THIS FOR ANYTHING - THE ABSOLUTE ENCODER IS USED FOR TURNING AND GOES INTO THE RIO ANALOG PORT
        self.turningEncoder = self.turningSparkMax.getEncoder()
        self.turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
        self.turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)

        if constants.k_burn_flash:  # until we make the turning motor settings a dictinary
            # self.drivingSparkMax.burnFlash()  # already done in the configure step above
            self.turningSparkMax.burnFlash()

        #  ---------------- ABSOLUTE ENCODER AND PID FOR TURNING  ------------------
        # create the AnalogPotentiometer with the offset.  TODO: this probably has to be 5V hardware but need to check
        # automatically always in radians and the turnover offset is built in, so the PID is easier
        # TODO: double check that the scale factor is the same on the new thrifty potentiometers
        self.absolute_encoder = AnalogPotentiometer(encoder_analog_port,
                                                    dc.k_analog_encoder_scale_factor * math.tau, -turning_encoder_offset)
        self.turning_PID_controller = PIDController(Kp=ModuleConstants.kTurningP, Ki=ModuleConstants.kTurningI, Kd=ModuleConstants.kTurningD)
        self.turning_PID_controller.enableContinuousInput(minimumInput=-math.pi, maximumInput=math.pi)

        # TODO: use the absolute encoder to set this - need to check the math carefully
        self.drivingEncoder.setPosition(0)
        self.turningEncoder.setPosition(self.get_turn_encoder())

        # self.chassisAngularOffset = chassisAngularOffset  # not yet
        self.desiredState.angle = Rotation2d(self.get_turn_encoder())

    def get_turn_encoder(self):
        # how we invert the absolute encoder if necessary (which it probably isn't in the standard mk4i config)
        analog_reverse_multiplier = -1 if dc.k_reverse_analog_encoders else 1
        return analog_reverse_multiplier * self.absolute_encoder.get()

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module.
        :returns: The current state of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        return SwerveModuleState(self.drivingEncoder.getVelocity(),
            Rotation2d(self.get_turn_encoder()),)

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module.
        :returns: The current position of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        return SwerveModulePosition(self.drivingEncoder.getPosition(),
            Rotation2d(self.get_turn_encoder()),)

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module.
        :param desiredState: Desired state with speed and angle.

        """

        # Apply chassis angular offset to the desired state.
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle

        # Optimize the reference state to avoid spinning further than 90 degrees.
        optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, Rotation2d(self.get_turn_encoder()))

        # don't let wheels servo back if we aren't asking the module to move
        if math.fabs(desiredState.speed) < 0.002:  # need to see what is this minimum m/s that makes sense
            optimizedDesiredState.speed = 0
            optimizedDesiredState.angle = self.getState().angle

        # Command driving and turning SPARKS MAX towards their respective setpoints.
        self.drivingPIDController.setReference(optimizedDesiredState.speed, dc.k_drive_controller_type.ControlType.kVelocity)

        # calculate the PID value for the turning motor  - use the roborio instead of the sparkmax
        # self.turningPIDController.setReference(optimizedDesiredState.angle.radians(), CANSparkMax.ControlType.kPosition)
        self.turning_output = self.turning_PID_controller.calculate(self.get_turn_encoder(), optimizedDesiredState.angle.radians())
        # clean up the turning Spark LEDs by cleaning out the noise - 20240226 CJH
        self.turning_output = 0 if math.fabs(self.turning_output) < 0.01 else self.turning_output
        self.turningSparkMax.set(self.turning_output)

        # CJH added for debugging and tuning
        if wpilib.RobotBase.isSimulation():
            if constants.k_swerve_state_messages:  # only do this when debugging - it's pretty intensive
                wpilib.SmartDashboard.putNumberArray(f'{self.label}_target_vel_angle',
                                    [optimizedDesiredState.speed, optimizedDesiredState.angle.radians()])
                wpilib.SmartDashboard.putNumberArray(f'{self.label}_actual_vel_angle',
                                    [self.drivingEncoder.getVelocity(), self.turningEncoder.getPosition()])
                wpilib.SmartDashboard.putNumberArray(f'{self.label}_volts',
                                    [self.drivingSparkMax.getAppliedOutput(), self.turningSparkMax.getAppliedOutput()])

        self.desiredState = desiredState

    def resetEncoders(self) -> None:
        """
        Zeroes all the SwerveModule encoders.
        """
        self.drivingEncoder.setPosition(0)

    def stop(self):
        pass
