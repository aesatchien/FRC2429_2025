import commands2
import ntcore
from helpers.log_command import log_command  # outsource explicit logging clutter to a single line
import constants


@log_command(console=True, nt=False, print_init=True, print_end=False)  # will print start and end messages
class SimShowFOV(commands2.Command):  # change the name for your command

    def __init__(self, container, cameras=None, indent=0) -> None:
        super().__init__()
        self.setName('SimShowFOV')
        self.indent = indent
        self.container = container
        self.extra_log_info = None

        if cameras is None:
            self.cameras = list(constants.k_cameras.keys())
        else:
            self.cameras = cameras

        self.inst = ntcore.NetworkTableInstance.getDefault()
        sim_prefix = constants.sim_prefix
        self.fov_pubs = {
            key: self.inst.getBooleanTopic(f"{sim_prefix}/FOV/{key}_show_fov").publish()
            for key in self.cameras
        }

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        self.extra_log_info = f"Cameras: {self.cameras}"
        for key in self.cameras:
            self.fov_pubs[key].set(True)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        # This command should run until interrupted
        return False

    def end(self, interrupted: bool) -> None:
        for key in self.cameras:
            self.fov_pubs[key].set(False)