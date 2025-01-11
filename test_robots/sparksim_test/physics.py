import math
import hal.simulation
from pyfrc.physics import drivetrains
from rev import SparkMaxSim
from wpilib.simulation import BatterySim, RoboRioSim, SingleJointedArmSim
import wpilib
from wpimath.system.plant import DCMotor

from robot import MyRobot

class PhysicsEngine:

    def __init__(self, physics_controller, robot: MyRobot):

        self.physics_controller = physics_controller

        self.robot = robot

        self.arm_plant = DCMotor.NEO(1)

        self.arm_sim = SingleJointedArmSim(
            gearbox=self.arm_plant,
            gearing=5 * 5 * 3 * 4.44,
            moi=SingleJointedArmSim.estimateMOI(length=20 * 0.0254, mass=8),
            armLength=20 * 0.0254,
            minAngle=40,
            maxAngle=116,
            simulateGravity=True,
            startingAngle=45
        )

        self.spark_sim = SparkMaxSim(self.robot.test_spark, self.arm_plant)
        self.spark_sim.setPosition(math.radians(60))
        self.spark_sim.enable()

        self.arm_mech2d = wpilib.Mechanism2d(60, 60)
        self.mech2d_root = self.arm_mech2d.getRoot(name="root", x=40, y=10)
        self.base_mech2d = self.mech2d_root.appendLigament("frame", length=-20, angle=0)
        self.crank_mech2d = self.base_mech2d.appendLigament(name="crank", length=20, angle=self.arm_sim.getAngle())

        self.base_mech2d.setColor(wpilib.Color8Bit(200, 200, 200))

        wpilib.SmartDashboard.putData("arm mech 2d", self.arm_mech2d)



    def update_sim(self, now, tm_diff):

        print(f"tm diff: {tm_diff}")
        
        # self.arm_sim.setInput(0, self.spark_sim.getAppliedOutput() * RoboRioSim.getVInVoltage())
        self.arm_sim.setInput(0, 12)
        
        self.arm_sim.update(tm_diff)

        self.spark_sim.iterate(velocity=self.arm_sim.getVelocity(), vbus=RoboRioSim.getVInVoltage(), dt=tm_diff)

        RoboRioSim.setVInVoltage(BatterySim.calculate([self.arm_sim.getCurrentDraw()]))

        self.crank_mech2d.setAngle(self.arm_sim.getAngle())
        # print(f"now: {now}")
        # if (now % 2 > 1):
        #     self.crank_mech2d.setAngle(80)
        # else:
        #     self.crank_mech2d.setAngle(150)
        print(f"armsim angle: {self.arm_sim.getAngle()}")

        print(f"riosim vinvoltage: {RoboRioSim.getVInVoltage()}")

        print(f"is the armsim at its limit? {self.arm_sim.hasHitLowerLimit() or self.arm_sim.hasHitUpperLimit()}")

        # optional: compute encoder
        # l_encoder = self.drivetrain.wheelSpeeds.left * tm_diff
