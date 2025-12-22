"""
This module serves as a template for creating new commands for the robot.

To create a new command:
1. Copy this file and rename it to something descriptive (e.g., `shooter_continuous_fire.py`).
2. Change the class name from `CommandTemplate` to a name that reflects the command's purpose (e.g., `FireShooter`).
3. In the `__init__` method:
    - Set a descriptive name for the command using `self.setName()` - this will show up in the start/end messages.
    - Add any required subsystems using `self.addRequirements()`.
4. Implement the command's logic in the following methods:
    - `initialize()`: Called once when the command is scheduled. Use it for one-time setup.
    - `execute()`: Called repeatedly while the command is running. This is where the main logic goes.
    - `isFinished()`: Called repeatedly to check if the command has completed. Return `True` when it's done.
    - `end(interrupted)`: Called once when the command finishes (or is interrupted). Use it for cleanup.

This template provides a basic structure with logging to the console and SmartDashboard
to aid in debugging.
"""
import commands2



from wpilib import SmartDashboard

import constants
from constants import ShooterConstants as sc
from subsystems.shooter import Shooter


class ShooterFireOnce(commands2.Command):  # change the name for your command

    def __init__(self, container, shooter: Shooter, indent=0) -> None:
        super().__init__()
        self.setName('Firing once')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.shooter = shooter
        self.addRequirements(self.shooter)  # commandsv2 version of requirements
        self.counter = 0
        self.target = 0

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = self.container.timer.get()
        print(f"{self.indent * '    '}** Started {self.getName()} at {self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time:2.1f} s **")
        self.counter = 0
        self.shooter.set_shooter_rpm(sc.k_test_speed)
        self.target = self.shooter.get_indexer_position() + 1 / sc.k_indexer_balls_per_rotation
        self.shooter.stop_indexer()

    def execute(self) -> None:
        self.counter += 1
        if self.counter > 10 and not self.shooter.indexer_on:
            self.shooter.set_indexer_rpm(sc.k_test_rpm)


    def isFinished(self) -> bool:
        return self.shooter.get_indexer_position() > self.target

    def end(self, interrupted: bool) -> None:
        end_time = self.container.timer.get()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            duration = end_time - self.start_time
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {duration:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {duration:.1f} s **")
        self.shooter.stop_shooter()
        self.shooter.stop_indexer()
