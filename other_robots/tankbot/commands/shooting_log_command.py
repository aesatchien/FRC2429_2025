import commands2
from commands.log_command import log_command  # outsource explicit logging clutter to a single line

from wpilib import SmartDashboard

import constants
from constants import ShooterConstants as sc
from subsystems.shooter import Shooter

@log_command(console=True, nt=False, print_init=True, print_end=True)  # will print start and end messages
class ShootingCommandLogging(commands2.Command):  # change the name for your command


    def __init__(self, container, shooter: Shooter, indent=0, continuous=True, balls=0) -> None:
        super().__init__()
        self.continuous = continuous
        self.setName('Firing continuous')  if self.continuous else self.setName('Firing n times')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.shooter = shooter
        self.addRequirements(self.shooter)  # commandsv2 version of requirements
        self.counter = 0
        self.target = 0
        self.balls = balls
        self.extra_log_info = None
        # self.counter = 0  # add a counter if you need to track iterations, remember to initialize in below
        # self.addRequirements(self.container.??)  # commands2 version of requirements - add the subsystems you need

    def initialize(self) -> None:
        # Called just before each time this Command runs
        # if you wish to add more information to the console logger, change self.extra_log_info
        # self.extra_log_info = "Target=7"  # (for example)
        self.counter = 0
        self.shooter.set_shooter_rpm(sc.k_test_speed)
        if not self.continuous: # if its not continuous, set target to current position + 1/balls per rotation
            self.target = self.shooter.get_indexer_position() + ((1 / sc.k_indexer_balls_per_rotation) * self.balls)
        self.shooter.stop_indexer()
        self.extra_log_info = "Balls=continuous" if self.continuous else "Balls=" + str(self.balls)

    def execute(self) -> None:
        self.counter += 1
        if self.counter > 10 and not self.shooter.indexer_on:
            self.shooter.set_indexer_rpm(sc.k_test_rpm)
        # runs 50x per second, so be careful about messages and timing

    def isFinished(self) -> bool:
        # True: fire once and end; False: run forever until interrupted; logic has it end when code returns True
        return False if self.continuous else self.shooter.get_indexer_position() > self.target # returns false for continuous, or evaluates current position > target position for one shot
        
    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        self.shooter.stop_shooter()
        self.shooter.stop_indexer()
