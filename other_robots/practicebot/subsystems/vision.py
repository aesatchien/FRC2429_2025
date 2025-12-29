import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard, DriverStation
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d
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
            cam_type = config['type']
            table_path = f"{constants.camera_prefix}/{cam_name}"
            base_topic = f"{table_path}/{cam_type}"

            # Timestamp is the camera heartbeat returned by the pi, located at the camera root
            # other topics fall under a /tags or /orange subfolder.
            self.camera_dict[key]['timestamp_entry'] = self.inst.getDoubleTopic(f"{table_path}/_timestamp").subscribe(0)
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
        target_available = self.camera_dict[camera_key]['targets_entry'].get() > 0
        time_stamp_good = wpilib.Timer.getFPGATimestamp() - self.camera_dict[camera_key]['timestamp_entry'].get() < 1
        return target_available and time_stamp_good

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
