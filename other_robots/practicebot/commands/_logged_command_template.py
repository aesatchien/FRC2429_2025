import commands2
from helpers.decorators import log_command  # outsource explicit logging clutter to a single line


@log_command(console=True, nt=False, print_init=True, print_end=True)  # will print start and end messages
class LoggedCommandTemplate(commands2.Command):  # change the name for your command

    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName('LoggedCommandTemplate')  # change this to whatever you named it above
        self.indent = indent
        self.container = container
        self.extra_log_info = None
        # self.counter = 0  # add a counter if you need to track iterations, remember to initialize in below
        # self.addRequirements(self.container.??)  # commands2 version of requirements - add the subsystems you need

    def initialize(self) -> None:
        # Called just before each time this Command runs
        # if you wish to add more information to the console logger, change self.extra_log_info
        # self.extra_log_info = "Target=7"  # (for example)
        pass

    def execute(self) -> None:
        # runs 50x per second, so be careful about messages and timing
        pass

    def isFinished(self) -> bool:
        # True: fire once and end; False: run forever until interrupted; logic has it end when code returns True
        return True

    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        pass