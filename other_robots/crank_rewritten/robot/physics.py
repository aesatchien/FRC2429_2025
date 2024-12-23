import math
from rev import SparkMaxSim, SparkSim
from subsystems.lower_crank import LowerCrank
import wpilib
from pyfrc.physics.core import PhysicsInterface
import wpilib.simulation
from wpimath.system.plant import DCMotor
from robot import MyRobot
import constants

class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):

        self.physics_controller = physics_controller
        self.robot = robot
        
        self.crank_arm_plant = DCMotor.NEO(1)
        self.crank_arm_plant.withReduction(  )

        self.arm_sim = wpilib.simulation.SingleJointedArmSim(
                self.crank_arm_plant,
                constants.LowerCrankConstants.k_gear_ratio,
                wpilib.simulation.SingleJointedArmSim.estimateMOI(length=constants.LowerCrankConstants.k_length_meters,
                                                                  mass=constants.LowerCrankConstants.k_mass_kg),
                armLength=constants.LowerCrankConstants.k_length_meters,
                minAngle=math.radians(40),
                maxAngle=math.radians(116),
                simulateGravity=True,
                startingAngle=constants.LowerCrankConstants.k_lower_crank_sim_starting_angle
        )

        self.arm_spark_sim = SparkMaxSim(self.robot.container.lower_crank.sparkmax, self.crank_arm_plant)
        self.arm_spark_sim.setPosition(constants.LowerCrankConstants.k_lower_crank_sim_starting_angle)
        self.arm_spark_sim.enable()
        # self.arm_encoder_sim = self.arm_spark_sim.getRelativeEncoderSim()

        self.arm_mech2d = wpilib.Mechanism2d(60, 60)
        self.mech2d_root = self.arm_mech2d.getRoot(name="root", x=40, y=10)
        self.base_mech2d = self.mech2d_root.appendLigament("frame", length=-20, angle=0)
        self.crank_mech2d = self.base_mech2d.appendLigament(name="crank", length=20, angle=self.arm_sim.getAngle())

        self.base_mech2d.setColor(wpilib.Color8Bit(200, 200, 200))

        wpilib.SmartDashboard.putData("arm mech 2d", self.arm_mech2d)

        # self.arm_motor: SimCANSparkMax = self.robot.container.lower_crank.sparkmax
        # self.arm_motor_sim = wpilib.simulation.PWMSim(self.arm_motor)


    def update_sim(self, now, tm_diff):

        print(f"voltage: {wpilib.RobotController.getInputVoltage()}")
        print(f"now: {now}")
        if now > 10:
            self.arm_sim.setInput(0, self.arm_spark_sim.getAppliedOutput() * wpilib.RobotController.getInputVoltage())
        else:
            self.arm_sim.setInput(0, 0.5)

        self.arm_sim.update(tm_diff)

        wpilib.simulation.RoboRioSim.setVInVoltage(
                wpilib.simulation.BatterySim.calculate([self.arm_sim.getCurrentDraw()])
        )

        voltage = wpilib.simulation.RoboRioSim.getVInVoltage()
        print(f"rio sim voltage: {wpilib.simulation.RoboRioSim.getVInVoltage()}")
        
        # self.arm_encoder_sim.setPosition(self.arm_sim.getAngle())
        # self.arm_spark_sim.setPosition(self.arm_sim.getAngle())
        self.arm_spark_sim.iterate(math.radians(self.arm_sim.getVelocityDps()), voltage, tm_diff)
        self.robot.container.lower_crank.set_encoder_position(self.arm_sim.getAngle())

        self.crank_mech2d.setAngle(math.degrees(self.arm_sim.getAngle()))

        # l_encoder = self.drivetrain.wheelSpeeds.left * tm_diff
