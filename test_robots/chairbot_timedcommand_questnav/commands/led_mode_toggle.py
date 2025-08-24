import commands2
from wpilib import SmartDashboard
from subsystems.led import Led


class LedModeToggle(commands2.Command):

    counter = 0

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('LedModeToggle')
        self.container = container
        self.addRequirements(container.led)
        self.modes = [
            'cone',
            'cube',
        ]

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)

        self.counter += 1

        mode = self.container.led.get_mode()
        new_mode = self.container.led.mode.CONE if mode == self.container.led.mode.CUBE else self.container.led.mode.CUBE
        self.container.led.set_mode(new_mode)

        print("\n" + f"** Firing {self.getName()} with mode {new_mode} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")



    def execute(self) -> None:
        pass


    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        #print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        #SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")


