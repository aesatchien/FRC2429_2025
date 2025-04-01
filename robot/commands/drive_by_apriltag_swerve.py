from math import dist
import math
import commands2
from wpilib import SmartDashboard
import ntcore
import wpilib
from wpimath.controller import PIDController

from subsystems.swerve import Swerve


class DriveByApriltagSwerve(commands2.Command):  # change the name for your command

    def __init__(self, container, swerve: Swerve, target_heading: float,  indent=0) -> None:
        super().__init__()
        self.setName('drive by apriltag swerve')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.target_heading = target_heading
        self.heading_pid = PIDController(0.3, 0, 0)
        self.distance_pid = PIDController(0.1, 0, 0)
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.swerve = swerve

        self.tag_vector_subscriber = self.inst.getDoubleArrayTopic("vision/top_pi/tag_vectors").subscribe([])
        # while running:
            # get bearing, distance
            # run pid on bearing, distance
        self.addRequirements(swerve)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{self.indent * '    '}** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.heading_pid.setSetpoint(self.target_heading)
        self.distance_pid.setSetpoint(0)

        self.done_strafing = False
        self.done_turning = False # unused but we can use this in the future.

    def execute(self) -> None:
        
        tag_vectors = self.tag_vector_subscriber.get()

        tag_distance = tag_heading = 0

        for idx, tag_info_bit in enumerate(tag_vectors):
            if idx == 2:
                 tag_distance = tag_info_bit
            elif idx == 3:
                tag_heading = tag_info_bit


        #wpilib.SmartDashboard.putNumber("_atag heading", tag_heading)
        #wpilib.SmartDashboard.putNumber("_atag distance", tag_distance)

        heading_out = self.heading_pid.calculate(tag_heading)
        distance_out = self.distance_pid.calculate(tag_distance)

        heading_out += math.copysign(0.015, heading_out)

        #wpilib.SmartDashboard.putNumber("_aheading out", heading_out)
        #wpilib.SmartDashboard.putNumber("_adistance out", distance_out)

        if abs(tag_heading) < math.radians(0.5):
            self.done_strafing = True

        # if self.done_strafing:
        #     self.swerve.drive(xSpeed=-0.1, ySpeed=0, rot=0, fieldRelative=False, rate_limited=False, keep_angle=False)
        # else:
        self.swerve.drive(xSpeed=heading_out, ySpeed=0.1, rot=0, fieldRelative=False, rate_limited=False, keep_angle=False)


    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
