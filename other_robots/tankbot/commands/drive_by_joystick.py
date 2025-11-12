import commands2
from wpilib import SmartDashboard
from commands2.button import CommandXboxController



class DriveByJoystick(commands2.Command):

    def __init__(self, container, controller: CommandXboxController) -> None:
        super().__init__()
        self.setName('drive_by_joystick')
        self.indent = 1
        self.container = container
        self.drive = self.container.drive
        self.controller: CommandXboxController = controller
        self.addRequirements(self.container.drive)  # commandsv2 version of requirements
        self.counter = 0

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = self.container.timer.get()
        print(f"{self.indent * '    '}** Started {self.getName()} at {self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time:2.1f} s **")

    def execute(self) -> None:
        speed_limit = 0.7
        turn_limit = 0.5
        left_y = self.controller.getHID().getLeftY()
        right_y = self.controller.getHID().getRightY()
        right_x = self.controller.getHID().getRightX()
        #self.drive.tank_drive(-left_y, -right_y)  # could do arcade drive instead
        self.drive.arcade_drive(-left_y * speed_limit, -right_x * turn_limit, True)
        self.counter +=1
        if self.counter % 100 == 0:
            pass
            # print(f'debugging: {left_y} {right_y}')


    def isFinished(self)-> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.timer.get()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            duration = end_time - self.start_time
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {duration:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {duration:.1f} s **")
