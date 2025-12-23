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
        self.counter = constants.VisionConstants.k_counter_offset
        self.ntinst = NetworkTableInstance.getDefault()

        # set up a dictionary of cams to go through
        self.camera_dict = {'ardu_high_tags': {}, 'ardu_back_tags': {}, 'genius_low_tags': {}, 'logi_reef_tags': {}}
        self.camera_values = {}

        self.arducam_back_table = NetworkTableInstance.getDefault().getTable('/Cameras/ArducamBack')  # arducam for tags
        self.arducam_high_table = NetworkTableInstance.getDefault().getTable('/Cameras/ArducamHigh')  # arducam for high tags
        # TODO - figure out how to have two tag cameras play nice
        self.genius_low_table = NetworkTableInstance.getDefault().getTable('/Cameras/GeniusLow')  # logitech for tags
        self.logitech_reef_table = NetworkTableInstance.getDefault().getTable('/Cameras/LogitechReef')  # logitech for reef

        self.tables = [self.arducam_high_table, self.arducam_back_table, self.genius_low_table,  self.logitech_reef_table]

        for ix, key in enumerate(self.camera_dict.keys()):  # colors on top
            table = self.tables[ix]
            nt_key = 'orange' if key == 'orange' else 'tags'
            self.camera_dict[key].update({'timestamp_entry': table.getDoubleTopic(f"_timestamp").subscribe(0)})
            self.camera_dict[key].update({'id_entry': table.getDoubleTopic(f"{nt_key}/id").subscribe(0)})
            self.camera_dict[key].update({'targets_entry': table.getDoubleTopic(f"{nt_key}/targets").subscribe(0)})
            self.camera_dict[key].update({'distance_entry': table.getDoubleTopic(f"{nt_key}/distance").subscribe(0)})
            self.camera_dict[key].update({'strafe_entry': table.getDoubleTopic(f"{nt_key}/strafe").subscribe(0)})
            self.camera_dict[key].update({'rotation_entry': table.getDoubleTopic(f"{nt_key}/rotation").subscribe(0)})
            self.camera_values[key] = {}
            self.camera_values[key].update({'id': 0, 'targets': 0, 'distance': 0, 'rotation': 0, 'strafe': 0})


    def target_available(self, target='ardu_high_tags'):
        target_available = self.camera_dict[target]['targets_entry'].get() > 0
        time_stamp_good = wpilib.Timer.getFPGATimestamp() - self.camera_dict[target]['timestamp_entry'].get() < 1
        return target_available and time_stamp_good

    def get_tag_strafe(self, target='ardu_high_tags'):

        if wpilib.RobotBase.isSimulation():  # trick the sim into thinking we are on tag 18
            drive_y = SmartDashboard.getNumber('drive_y', 0)
            y_dist = drive_y - 4.025900	# will be positive if we are left of tag,right if center
            # pretend that we get 100% change over a meter
            if math.fabs(y_dist) > 0.5:  # todo - make this dist from tag
                return 0
            else:
                # mimic the behavior of a tag in a camera - if we are on the left of the tag (y above tag 18)
                # we add to the return value a positive number, so the "tag" is on the right side of the camera
                cam_fraction = 0.5 + y_dist
                return cam_fraction # should go from 0 to 1, with 0.5 being centered

        tag_available = self.target_available(target)
        if tag_available > 0:
            return self.camera_dict[target]['strafe_entry'].get()
        else:
            return 0  # it would do this anyway because it defaults to zero

    def get_orange_strafe(self):
        orange_available = self.target_available('orange')
        if orange_available > 0:
            return self.camera_dict['orange']['strafe_entry'].get()
        else:
            return 0  # it would do this anyway because it defaults to zero
        
    def get_tag_dist(self, target='ardu_high_tags'):
        tag_available = self.target_available(target)
        if tag_available > 0:
            return self.camera_dict[target]['distance_entry'].get()
        else:
            return 0
        
    def get_orange_dist(self):
        orange_available = self.target_available('orange')
        if orange_available > 0:
            return self.camera_dict['orange']['distance_entry'].get()
        else:
            return 0

    def get_orange_rotation(self):
        orange_available = self.target_available('orange')
        if orange_available > 0:
            return self.camera_dict['orange']['rotation_entry'].get()
        else:
            return 0

    def periodic(self) -> None:
        self.counter += 1

        # update x times a second
        if self.counter % 10 == 0:
            if wpilib.RobotBase.isSimulation():
                SmartDashboard.putNumber('match_time', wpilib.Timer.getFPGATimestamp())
            else:
                SmartDashboard.putNumber('match_time', DriverStation.getMatchTime())

            for ix, key in enumerate(self.camera_dict.keys()):
                nt_key = 'orange' if key == 'orange' else 'tags'
                self.camera_values[key]['targets'] = self.camera_dict[key]['targets_entry'].get()
                self.camera_values[key]['distance'] = self.camera_dict[key]['distance_entry'].get()
                self.camera_values[key]['rotation'] = self.camera_dict[key]['rotation_entry'].get()
                self.camera_values[key]['strafe'] = self.camera_dict[key]['strafe_entry'].get()

            if wpilib.RobotBase.isReal():
                # orange will crash at the moment
                # wpilib.SmartDashboard.putBoolean('orange_targets_exist', self.target_available('orange'))
                wpilib.SmartDashboard.putBoolean('arducam_high_targets_exist', self.target_available('ardu_high_tags'))
                wpilib.SmartDashboard.putBoolean('genius_low_targets_exist', self.target_available('genius_low_tags'))
                wpilib.SmartDashboard.putBoolean('arducam_back_targets_exist', self.target_available('ardu_back_tags'))
                wpilib.SmartDashboard.putBoolean('logitech_reef_targets_exist', self.target_available('logi_reef_tags'))
            else:  # test the keys
                wpilib.SmartDashboard.putBoolean('arducam_high_targets_exist', 0 < self.counter % 600 < 110 )
                wpilib.SmartDashboard.putBoolean('genius_low_targets_exist', 100 < self.counter % 600 < 210)
                wpilib.SmartDashboard.putBoolean('arducam_back_targets_exist', 200 < self.counter % 600 < 310)
                wpilib.SmartDashboard.putBoolean('logitech_reef_targets_exist', 300 < self.counter % 600 < 410)
                wpilib.SmartDashboard.putBoolean('photoncam_targets_exist', 400 < self.counter % 600 < 510)
            # wpilib.SmartDashboard.putNumber('tag_strafe', self.camera_dict['tags']['strafe_entry'].get())

            if constants.VisionConstants.k_nt_debugging:  # extra debugging info for NT
                pass
