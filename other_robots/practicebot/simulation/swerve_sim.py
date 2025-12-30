import math
import wpilib.simulation as simlib
from wpimath.kinematics import SwerveModulePosition
import ntcore
import constants
from subsystems.swerve_constants import DriveConstants as dc

class SwerveSim:
    def __init__(self, physics_controller, robot):
        self.physics_controller = physics_controller
        self.robot = robot
        self.kinematics = dc.kDriveKinematics
        
        self.inst = ntcore.NetworkTableInstance.getDefault()
        
        # Swerve Debugging
        self.target_angles_pub = self.inst.getDoubleArrayTopic(f"{constants.sim_prefix}/target_angles").publish()
        
        # Swerve Target Subscribers
        dash_values = ['lf_target_vel_angle', 'rf_target_vel_angle', 'lb_target_vel_angle', 'rb_target_vel_angle']
        self.swerve_target_subs = [self.inst.getDoubleArrayTopic(f"/SmartDashboard/{v}").subscribe([0, 0]) for v in dash_values]

        self._initialize_sim_devices()

    def _initialize_sim_devices(self):
        # NavX
        self.navx = simlib.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")
        
        # Sparks
        self.spark_dict = {}
        self.spark_drives = ['lf_drive', 'rf_drive', 'lb_drive', 'rb_drive']
        self.spark_drive_ids = [21, 25, 23, 27]
        self.spark_turns = ['lf_turn', 'rf_turn', 'lb_turn', 'rb_turn']
        self.spark_turn_ids = [20, 24, 22, 26]
        
        self.spark_names = self.spark_drives + self.spark_turns
        self.spark_ids = self.spark_drive_ids + self.spark_turn_ids
        
        for spark_name, can_id in zip(self.spark_names, self.spark_ids):
            spark = simlib.SimDeviceSim(f'SPARK MAX [{can_id}]')
            position = spark.getDouble('Position')
            velocity = spark.getDouble('Velocity')
            output = spark.getDouble('Applied Output')
            self.spark_dict.update({spark_name: {'controller': spark, 'position': position,
                                                 'velocity': velocity, 'output': output}})

    def update(self, tm_diff):
        # Update simulated spark positions from target angles (perfect response)
        target_angles = [sub.get()[1] for sub in self.swerve_target_subs]
        for spark_turn, target_angle in zip(self.spark_turns, target_angles):
            self.spark_dict[spark_turn]['position'].set(target_angle)
            
        if constants.k_swerve_debugging_messages:
            self.target_angles_pub.set(target_angles)

        # Get desired states from robot code (for kinematics)
        module_states = self.robot.container.swerve.get_desired_swerve_module_states()
        speeds = self.kinematics.toChassisSpeeds(tuple(module_states))

        # Update physics controller
        self.physics_controller.drive(speeds, tm_diff)
        
        # Update Robot Odometry (Perfect Odometry for Sim)
        pose = self.physics_controller.get_pose()
        self.robot.container.swerve.pose_estimator.resetPosition(gyroAngle=pose.rotation(), wheelPositions=[SwerveModulePosition()] * 4, pose=pose)

        # Update NavX
        self.navx_yaw.set(self.navx_yaw.get() - math.degrees(speeds.omega * tm_diff))