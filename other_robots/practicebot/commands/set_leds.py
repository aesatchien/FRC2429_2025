import commands2
import typing

from subsystems.led import Led
from helpers.decorators import log_command


@log_command(console=True, nt=False, print_init=True, print_end=False)
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
        self.extra_log_info = None  # for logging

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

        self.extra_log_info = f'mode {msg_mode} and indicator {msg_indicator}'

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True
    
    def end(self, interrupted: bool) -> None:        
        pass
