import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard, DriverStation
import ntcore
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
import math

import constants


class Vision(SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Vision')
        self.counter = constants.SimConstants.k_counter_offset
        self.ntinst = NetworkTableInstance.getDefault()

        # Initialize dictionary with logical keys from constants
        # set up a dictionary of cams to go through
        # If one physical camera does both, we treat it as two cameras but with the same topic
        self.camera_dict = {key: {} for key in constants.k_cameras.keys()}
        self.camera_values = {}
        for key in self.camera_dict.keys():
            self.camera_values[key] = {}
            self.camera_values[key].update({'id': 0, 'targets': 0, 'distance': 0, 'rotation': 0, 'strafe': 0})

        self._init_networktables()

    def _init_networktables(self):
        self.inst = NetworkTableInstance.getDefault()
        vision_prefix = constants.vision_prefix

        # ------------- Publishers (Efficiency) -------------
        self.match_time_pub = self.inst.getDoubleTopic(f"/SmartDashboard/match_time").publish()

        # Training Box Entry - allows read/write from dashboard and robot
        self.training_box_entry = self.inst.getDoubleArrayTopic(f"{constants.camera_prefix}/_training_box").getEntry([0]*2)

        # Status Publishers - Map internal keys to dashboard names for all the allowed cameras
        self.status_pubs = {}
        for ix, key in enumerate(constants.k_cameras.keys()):
            self.status_pubs[key] = self.inst.getBooleanTopic(f"{vision_prefix}/{key}_targets_exist").publish()
            if constants.VisionConstants.k_print_config:
                print(f"vision's status pubs {ix}: {key}: {self.status_pubs[key]}")

        # in case we're messing around with photonvision
        self.status_pubs['photoncam'] = self.inst.getBooleanTopic(
            f"{vision_prefix}/photoncam_targets_exist").publish()  # Used in sim

        # ------------- Subscribers -------------
        # Explicit mapping of camera keys to NetworkTable paths
        for key, config in constants.k_cameras.items():
            cam_name = config['topic_name']
            cam_type = config.get('label', config['type'])
            table_path = f"{constants.camera_prefix}/{cam_name}"
            base_topic = f"{table_path}/{cam_type}"

            # Timestamp is the camera heartbeat returned by the pi, located at the camera root (deprecated, now using NT only)
            # other topics fall under a /tags or /orange subfolder.
            self.camera_dict[key]['id_entry'] = self.inst.getDoubleTopic(f"{base_topic}/id").subscribe(0)
            self.camera_dict[key]['targets_entry'] = self.inst.getDoubleTopic(f"{base_topic}/targets").subscribe(0)
            self.camera_dict[key]['distance_entry'] = self.inst.getDoubleTopic(f"{base_topic}/distance").subscribe(0)
            self.camera_dict[key]['strafe_entry'] = self.inst.getDoubleTopic(f"{base_topic}/strafe").subscribe(0)
            self.camera_dict[key]['rotation_entry'] = self.inst.getDoubleTopic(f"{base_topic}/rotation").subscribe(0)

        if constants.VisionConstants.k_print_config:
            print('\n*** VISION.PY CAMERA DICT ***')
            for key, item in self.camera_dict.items():
                print(f'{key}: {item}')
            print()

    def target_available(self, camera_key='arducam_high'):
        # Use getAtomic to get the value and the timestamp of the last update
        atomic_targets = self.camera_dict[camera_key]['targets_entry'].getAtomic()
        target_exists = atomic_targets.value > 0

        # Check latency (in microseconds). 1,000,000 us = 1 second.
        latency_us = ntcore._now() - atomic_targets.time
        time_stamp_good = latency_us < 1000000

        if target_exists and not time_stamp_good:
            print(f"Vision Warning: Stale target on '{camera_key}'. Latency: {latency_us / 1000:.1f} ms")

        return target_exists and time_stamp_good

    def get_strafe(self, camera_key: str) -> float:
        if wpilib.RobotBase.isSimulation() and constants.k_cameras[camera_key]['type'] == 'tags':  # trick the sim into thinking we are on tag 18
            drive_y = SmartDashboard.getNumber('drive_y', 0)
            y_dist = drive_y - 4.025900  # will be positive if we are left of tag,right if center
            # pretend that we get 100% change over a meter
            if math.fabs(y_dist) > 0.5:  # todo - make this dist from tag
                return 0
            else:
                # mimic the behavior of a tag in a camera - if we are on the left of the tag (y above tag 18)
                # we add to the return value a positive number, so the "tag" is on the right side of the camera
                cam_fraction = 0.5 + y_dist
                return cam_fraction  # should go from 0 to 1, with 0.5 being centered

        if self.target_available(camera_key):
            return self.camera_dict[camera_key]['strafe_entry'].get()
        return 0

    def get_distance(self, camera_key: str) -> float:
        if self.target_available(camera_key):
            return self.camera_dict[camera_key]['distance_entry'].get()
        return 0

    def get_rotation(self, camera_key: str) -> float:
        if self.target_available(camera_key):
            return self.camera_dict[camera_key]['rotation_entry'].get()
        return 0

    def nearest_to_cam(self, camera_key: str, max_dist: float = 5.0) -> Pose2d | None:
        """
        Returns the robot-relative Pose2d of the nearest target seen by the specified camera.
        The rotation of the returned Pose2d is the angle to the target.
        """
        if not self.target_available(camera_key):
            return None

        dist = self.get_distance(camera_key)
        if dist > max_dist or dist <= 0:
            return None

        # Get camera rotation from constants
        cam_config = constants.k_cameras.get(camera_key)
        if not cam_config:
            return None

        cam_rot_deg = cam_config.get('rotation', 0)

        # Rotation from NT is relative to camera mount
        nt_rot = self.get_rotation(camera_key)

        # 1. Define where the camera is relative to the robot center
        # TODO: Add x/y offsets to k_cameras so this is accurate!
        camera_to_robot = Transform2d(Translation2d(0, 0), Rotation2d.fromDegrees(cam_rot_deg))

        # 2. Define where the target is relative to the camera
        # We assume the camera sees the target at 'dist' distance and 'nt_rot' angle
        target_translation = Translation2d(dist, 0).rotateBy(Rotation2d.fromDegrees(nt_rot))
        target_to_camera = Transform2d(target_translation, Rotation2d.fromDegrees(nt_rot))

        # 3. Combine them: Robot -> Camera -> Target
        # This creates a Pose2d representing the target in the Robot's coordinate frame
        pose = Pose2d().transformBy(camera_to_robot).transformBy(target_to_camera)

        # print(f"Vision's nearest_to_cam returned {pose}")

        return pose

    def nearest_to_robot(self, camera_list: list[str] = None, max_dist: float = 5.0) -> Pose2d | None:
        """
        Returns the robot-relative Pose2d of the nearest target seen by the specified cameras.
        If camera_list is None, checks all cameras.
        """
        if camera_list is None:
            camera_list = list(self.camera_dict.keys())

        # Get all valid poses from specified cameras
        poses = [self.nearest_to_cam(key, max_dist) for key in camera_list if key in self.camera_dict]
        valid_poses = [p for p in poses if p is not None]

        if not valid_poses:
            return None

        # Return the one with the minimum distance (norm of translation)
        return min(valid_poses, key=lambda p: p.translation().norm())

    def set_training_box(self, data: list[float]) -> None:
        """Update the training box parameters on the coprocessor."""
        self.training_box_entry.set(data)

    def periodic(self) -> None:
        self.counter += 1

        # update x times a second
        if self.counter % 10 == 0:
            if wpilib.RobotBase.isSimulation():
                self.match_time_pub.set(wpilib.Timer.getFPGATimestamp())
            else:
                self.match_time_pub.set(DriverStation.getMatchTime())

            for ix, key in enumerate(self.camera_dict.keys()):
                self.camera_values[key]['targets'] = self.camera_dict[key]['targets_entry'].get()
                self.camera_values[key]['distance'] = self.camera_dict[key]['distance_entry'].get()
                self.camera_values[key]['rotation'] = self.camera_dict[key]['rotation_entry'].get()
                self.camera_values[key]['strafe'] = self.camera_dict[key]['strafe_entry'].get()

            # Update publishers based on target availability - this is for the GUI
            # Works for both Real (actual data) and Sim (physics.py data)
            for key, pub in self.status_pubs.items():
                if key in self.camera_dict:
                    pub.set(self.target_available(key))

            if constants.VisionConstants.k_nt_debugging:  # extra debugging info for NT
                pass
