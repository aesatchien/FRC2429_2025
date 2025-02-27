import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard, DriverStation
from ntcore import NetworkTableInstance

import constants


class Vision(SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Vision')
        self.counter = 8
        self.ntinst = NetworkTableInstance.getDefault()

        # set up a dictionary of cams to go through
        self.camera_dict = {'ardu_reef_tags': {}, 'ardu_back_tags': {}, 'logi_high_tags': {}, 'logi_tags_tags': {}}
        self.camera_values = {}


        self.arducam_back_table = NetworkTableInstance.getDefault().getTable('/Cameras/ArducamBack')  # lifecam for rings
        self.arducam_reef_table = NetworkTableInstance.getDefault().getTable('/Cameras/ArducamReef')  # logitech for tags
        # TODO - figure out how to have two tag cameras play nice
        self.logitech_high_table = NetworkTableInstance.getDefault().getTable('/Cameras/LogitechHigh')  # logitech for tags
        self.logitech_tags_table = NetworkTableInstance.getDefault().getTable('/Cameras/LogitechTags')  # logitech for tags

        self.tables = [self.arducam_back_table, self.logitech_high_table, self.arducam_reef_table, self.logitech_tags_table]

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


    def target_available(self, target='ardu_reef_tags'):
        target_available = self.camera_dict[target]['targets_entry'].get() > 0
        time_stamp_good = wpilib.Timer.getFPGATimestamp() - self.camera_dict[target]['timestamp_entry'].get() < 1
        return target_available and time_stamp_good

    def get_tag_strafe(self, target='ardu_reef_tags'):
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
        
    def get_tag_dist(self, target='ardu_reef_tags'):
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
                wpilib.SmartDashboard.putBoolean('arducam_reef_targets_exist', self.target_available('ardu_reef_tags'))
                wpilib.SmartDashboard.putBoolean('logitech_high_targets_exist', self.target_available('logi_high_tags'))
                wpilib.SmartDashboard.putBoolean('arducam_back_targets_exist', self.target_available('ardu_back_tags'))
                wpilib.SmartDashboard.putBoolean('logitech_tags_targets_exist', self.target_available('logi_tags_tags'))
            else:  # test the keys
                wpilib.SmartDashboard.putBoolean('arducam_reef_targets_exist', self.counter % 600 > 350)
                wpilib.SmartDashboard.putBoolean('logitech_high_targets_exist', self.counter % 600 < 250)
                wpilib.SmartDashboard.putBoolean('arducam_back_targets_exist', self.counter % 600 > 100)
                wpilib.SmartDashboard.putBoolean('logitech_tags_targets_exist', self.counter % 600 < 500)
            # wpilib.SmartDashboard.putNumber('tag_strafe', self.camera_dict['tags']['strafe_entry'].get())

