import commands2
from helpers.decorators import log_command  # outsource explicit logging clutter to a single line


@log_command(console=True, nt=False, print_init=True, print_end=False)  # will print start and end messages
class MoveTrainingBox(commands2.Command):  # change the name for your command

    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName('MoveTrainingBox')  # change this to whatever you named it above
        self.indent = indent
        self.container = container
        self.extra_log_info = None
        # self.counter = 0  # add a counter if you need to track iterations, remember to initialize in below
        self.addRequirements(self.container.vision)  # commands2 version of requirements - add the subsystems you need

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        """
        Called once when the command is scheduled.
        Reads the current training box, increments the X value, and writes it back.
        """
        # Get the current training box values
        box_data = self.container.vision.training_box_entry.get()

        # Increment the X fraction (the first element)
        new_x = box_data[0] + 0.1

        # Wrap around if it exceeds the limit
        if new_x >= 0.91:
            new_x = 0.1

        # Update the array and publish it back
        box_data[0] = round(new_x, 2)  # Round to prevent floating point issues
        self.container.vision.set_training_box(box_data)
        self.extra_log_info = f"Set training box X to {new_x:.1f}"

    def execute(self) -> None:
        # runs 50x per second, so be careful about messages and timing
        pass

    def isFinished(self) -> bool:
        # True: fire once and end; False: run forever until interrupted; logic has it end when code returns True
        return True

    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        pass