import commands2
from wpilib import SmartDashboard
from subsystems.led import Led


class LedIndicatorToggle(commands2.Command):

    counter = 0

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('LedIndicatorToggle')
        self.container = container
        self.addRequirements(container.led)
        self.modes = [
            'cone',
            'cube',
        ]
        self.indicators = [
            self.container.led.indicator.PICKUP_COMPLETE,
            self.container.led.indicator.VISION_TARGET_FAILURE,
            self.container.led.indicator.VISION_TARGET_SUCCESS,
            self.container.led.indicator.AUTO_STRAFE_COMPLETE,
            self.container.led.indicator.RAINBOW,
            self.container.led.indicator.RSL,
            self.container.led.indicator.NONE,
        ]

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)

        self.counter += 1
        active_indicator = self.indicators[self.counter % len(self.indicators)]
        #active_mode = self.modes[self.counter % len(self.modes)]
        self.container.game_piece_mode = active_indicator
        # if active_mode == 'cone':
        #   self.container.led.set_mode(Led.Mode.CONE)
        # elif active_mode == 'cube':
        #   self.container.led.set_mode(Led.Mode.CUBE)

        print("\n" + f"** Firing {self.getName()} with active mode {active_indicator} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        #  Decide how to send out the changes to the LED subsystem -
        # if you want to just set the active mode
        # self.container.led.set_indicator(active_mode)

        # if you want to run with a timeout, it is currently a command
        self.container.led.set_indicator_with_timeout(active_indicator, 5).schedule()




    def execute(self) -> None:
        pass


    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        #print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        #SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")


