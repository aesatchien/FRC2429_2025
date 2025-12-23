import math

import wpilib
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpilib import AnalogPotentiometer, SmartDashboard
from wpimath.controller import PIDController
from rev import SparkFlexConfig, SparkFlex

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
        self.drivingSparkFlex = SparkFlex(drivingCANId, SparkFlex.MotorType.kBrushless)
        self.turningSparkFlex = SparkFlex(turningCANId, SparkFlex.MotorType.kBrushless)

        #  ---------------- DRIVING  SPARKMAX  ------------------
        self._configure_spark(self.drivingSparkFlex, ModuleConstants.k_driving_config, driving_inverted, drivingCANId)

        # Get driving encoder from the sparkflex
        self.drivingEncoder = self.drivingSparkFlex.getEncoder()
        self.drivingClosedLoopController = self.drivingSparkFlex.getClosedLoopController()

        #  ---------------- TURNING SPARKMAX  ------------------
        self._configure_spark(self.turningSparkFlex, ModuleConstants.k_turning_config, turning_inverted, turningCANId)

        # Setup encoders for the turning SPARKMAX - just to watch it if we need to for velocities, etc.
        # WE DO NOT USE THIS FOR ANYTHING - THE ABSOLUTE ENCODER IS USED FOR TURNING AND GOES INTO THE RIO ANALOG PORT
        self.turningEncoder = self.turningSparkFlex.getEncoder()

        #  ---------------- ABSOLUTE ENCODER AND PID FOR TURNING  ------------------
        # create the AnalogPotentiometer with the offset.  TODO: this probably has to be 5V hardware but need to check
        # automatically always in radians and the turnover offset is built in, so the PID is easier
        # TODO: double check that the scale factor is the same on the new thrifty potentiometers
        self.absolute_encoder = AnalogPotentiometer(channel=encoder_analog_port,
                                fullRange=dc.k_analog_encoder_scale_factor * math.tau, offset= -turning_encoder_offset)
        self.turning_PID_controller = PIDController(Kp=ModuleConstants.kTurningP, Ki=ModuleConstants.kTurningI, Kd=ModuleConstants.kTurningD)
        self.turning_PID_controller.enableContinuousInput(minimumInput=-math.pi, maximumInput=math.pi)

        # TODO: use the absolute encoder to set this - need to check the math carefully
        self.drivingEncoder.setPosition(0)
        self.turningEncoder.setPosition(self.get_turn_encoder())

        # self.chassisAngularOffset = chassisAngularOffset  # not yet
        self.desiredState.angle = Rotation2d(self.get_turn_encoder())

    def _configure_spark(self, spark: SparkFlex, config_template: SparkFlexConfig, inverted: bool, can_id: int):
        """ Applies a config to a sparkmax. """
        config = SparkFlexConfig()
        config.apply(config_template)
        config.inverted(inverted)
        
        if constants.k_burn_flash:
            reset_mode = SparkFlex.ResetMode.kResetSafeParameters if constants.k_reset_sparks_to_default else SparkFlex.ResetMode.kNoResetSafeParameters
            persist_mode = SparkFlex.PersistMode.kPersistParameters
            
            error = spark.configure(config=config, resetMode=reset_mode, persistMode=persist_mode)
            print(f"Reconfigured sparkmax {can_id}. Controller status: {error}")

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

    def getDesiredState(self):
        return self.desiredState

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module.
        :param desiredState: Desired state with speed and angle.

        """

        # Apply chassis angular offset to the desired state.
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle

        # Optimize the reference state to avoid spinning further than 90 degrees
        correctedDesiredState.optimize(Rotation2d(self.get_turn_encoder()))

        # don't let wheels servo back if we aren't asking the module to move
        if math.fabs(desiredState.speed) < 0.002:  # need to see what is this minimum m/s that makes sense
            correctedDesiredState.speed = 0
            correctedDesiredState.angle = self.getState().angle

        # Command driving and turning SPARKS MAX towards their respective setpoints.
        self.drivingClosedLoopController.setReference(correctedDesiredState.speed, dc.k_drive_controller_type.ControlType.kVelocity)

        # calculate the PID value for the turning motor  - use the roborio instead of the sparkflex. todo: explain why
        # self.turningPIDController.setReference(optimizedDesiredState.angle.radians(), CANSparkFlex.ControlType.kPosition)
        self.turning_output = self.turning_PID_controller.calculate(self.get_turn_encoder(), correctedDesiredState.angle.radians())
        # clean up the turning Spark LEDs by cleaning out the noise - 20240226 CJH
        self.turning_output = 0 if math.fabs(self.turning_output) < 0.01 else self.turning_output
        self.turningSparkFlex.set(self.turning_output)

        if False: # wpilib.RobotBase.isSimulation():  # can we delete this yet?
            SmartDashboard.putNumberArray(f'{self.label}_target_vel_angle',
                                [correctedDesiredState.speed, correctedDesiredState.angle.radians()])
            SmartDashboard.putNumberArray(f'{self.label}_actual_vel_angle',
                                [self.drivingEncoder.getVelocity(), self.turningEncoder.getPosition()])
            SmartDashboard.putNumberArray(f'{self.label}_volts',
                                [self.drivingSparkFlex.getAppliedOutput(), self.turningSparkFlex.getAppliedOutput()])

        self.desiredState = desiredState

    def resetEncoders(self) -> None:
        """ Zeroes all the SwerveModule encoders. """
        self.drivingEncoder.setPosition(0)

    def stop(self):
        pass
