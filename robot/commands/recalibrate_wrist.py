import math

import commands2
from rev import SparkMax
from wpilib import SmartDashboard, Timer
from constants import WristConstants
from subsystems.wrist import Wrist


class RecalibrateWrist(commands2.Command):

    def __init__(self, container,  timeout=10,  wait_to_finish=False,  indent=0) -> None:

        super().__init__()
        self.setName('RecalibrateWrist')
        self.indent = indent
        self.container = container
        self.wrist: Wrist = self.container.wrist

        self.timeout = timeout
        self.wait_to_finish = wait_to_finish
        self.timer = Timer()
        self.addRequirements(self.wrist)
        self.counter = 0
        # set wrist to -10 degrees and then back
        self.cal_positions = list(range(0, -100, -2)) + list(range(-100, 0, 2))
        self.cal_positions = [math.radians(pos/10) for pos in self.cal_positions]

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        msg = self.indent * "    " + f"** Started {self.getName()} at {self.start_time} s **"
        print(msg, flush=True)
        SmartDashboard.putString("alert", msg)

        self.wrist.set_position(radians=0)  # set wrist to zero
        self.counter = 0

    def execute(self) -> None:

        if self.counter < len(self.cal_positions):
            self.wrist.set_position(radians=self.cal_positions[self.counter])
        self.counter += 1


    def isFinished(self) -> bool:
        return self.counter > len(self.cal_positions)

    def runsWhenDisabled(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:

        # update the wrist encoder
        abs_position = self.wrist.abs_encoder.getPosition()
        abs_offset = abs_position - WristConstants.k_abs_encoder_readout_when_at_zero_position
        abs_offset_rad = abs_offset * math.tau
        self.wrist.encoder.setPosition(abs_offset_rad)
        print(f'After calibration, reset wrist encoder to {math.degrees(abs_offset_rad):.1f}°')

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
