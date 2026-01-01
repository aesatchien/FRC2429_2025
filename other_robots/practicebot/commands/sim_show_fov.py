import commands2
import ntcore
from wpimath.geometry import Pose2d, Transform2d
from helpers.decorators import log_command  # outsource explicit logging clutter to a single line
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
        auto_prefix = constants.auto_prefix

        self.fov_pubs = {
            key: self.inst.getBooleanTopic(f"{sim_prefix}/FOV/{key}_show_fov").publish()
            for key in self.cameras
        }
        
        # Publishers for vision targets
        self.targets_pub = self.inst.getStructArrayTopic(f"{auto_prefix}/_vision_target_poses", Pose2d).publish()
        self.show_targets_pub = self.inst.getBooleanTopic(f"{auto_prefix}/_show_vision_targets").publish()
        self.show_targets_pub.set(False)

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        self.extra_log_info = f"Cameras: {self.cameras}"
        for key in self.cameras:
            self.fov_pubs[key].set(True)
        self.show_targets_pub.set(True)

    def execute(self) -> None:
        current_pose = self.container.swerve.get_pose()
        target_poses = []
        
        for cam in self.cameras:
            # Get nearest target relative to robot
            relative_pose = self.container.vision.nearest_to_cam(cam)
            
            if relative_pose is not None:
                # Transform to field pose
                rel_tf = Transform2d(relative_pose.translation(), relative_pose.rotation())
                field_pose = current_pose.transformBy(rel_tf)
                target_poses.append(field_pose)
        
        self.targets_pub.set(target_poses)

    def isFinished(self) -> bool:
        # This command should run until interrupted
        return False

    def end(self, interrupted: bool) -> None:
        for key in self.cameras:
            self.fov_pubs[key].set(False)
        self.show_targets_pub.set(False)
        self.targets_pub.set([])