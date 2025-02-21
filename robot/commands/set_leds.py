import commands2
import wpilib
import typing

from wpilib import SmartDashboard
from subsystems.led import Led

class SetLEDs(commands2.Command):
    """Command to test the LED modes and indicators"""
    def __init__(self, container, led: Led, indicator: typing.Union[None, Led.Indicator] = None,
                 mode: typing.Union[None, Led.Mode] = None, indicator_timeout=3) -> None:
        super().__init__()
        self.setName('Set Leds')
        self.container = container
        self.led = led
        self.mode = mode
        self.indicator = indicator
        self.indicator_timeout = indicator_timeout

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""

        msg_mode = 'None'
        msg_indicator = 'None'
        if self.indicator is not None:
            if self.indicator_timeout is not None:
                commands2.CommandScheduler.getInstance().schedule(
                    self.led.set_indicator_with_timeout(self.indicator, self.indicator_timeout))
            else:
                self.led.set_indicator(self.indicator)
            msg_indicator = self.indicator.value['name']
        if self.mode is not None:
            # in this case we will turn off the indicator so we can see the mode
            self.led.set_indicator(self.led.Indicator.kNONE)
            self.led.set_mode(self.mode)
            msg_mode = self.mode.value['name']

        self.start_time = round(self.container.get_enabled_time(), 2)
        msg = f"** Started {self.getName()} at {self.start_time} s with mode {msg_mode} and indicator {msg_indicator} **"
        print("\n" + msg, flush=True)
        SmartDashboard.putString("alert", msg)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted: bool) -> None:        
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        # print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
