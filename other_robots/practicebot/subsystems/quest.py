import wpilib
from commands2 import SubsystemBase, InstantCommand
from wpilib import SmartDashboard, DriverStation, Timer, Field2d
from wpimath.units import inchesToMeters
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Pose3d, Rotation3d, Translation3d, Transform2d, Transform3d
from ntcore import NetworkTableInstance

from helpers.questnav.questnav2 import QuestNav as Metaquestnav

import constants

# TODO - clean and speed this up
class Questnav(SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Quest')
        self.counter = constants.VisionConstants.k_counter_offset
        self.ntinst = NetworkTableInstance.getDefault()

        self.questnav = Metaquestnav()

        self.quest_pose_accepted = False  # validate our location on a field

        self.quest_to_robot = Transform2d(inchesToMeters(-8.35), inchesToMeters(-10.50), Rotation2d().fromDegrees(270)) #10.50 -8.35
        # This is quest-centric coordinate. X is robot center position -8.35 inch as seen rom Quest. y is robot center -10.50 inches as seen from Quest
        # self.quest_to_robot = Transform2d(inchesToMeters(4), 0, Rotation2d().fromDegrees(0))
        self.quest_field = Field2d()

        self.quest_has_synched = False  # use this to check in disabled whether to update the quest with the robot odometry
        SmartDashboard.putBoolean('questnav_synched', self.quest_has_synched)

        self.use_quest = constants.k_use_quest_odometry
        SmartDashboard.putBoolean('questnav_in_use', self.use_quest)

        # note - have Risaku standardize these with the rest of the putDatas
        SmartDashboard.putData('QuestResetOdometry', InstantCommand(lambda: self.quest_reset_odometry()).ignoringDisable(True))
        SmartDashboard.putData('QuestSyncOdometry', InstantCommand(lambda: self.quest_sync_odometry()).ignoringDisable(True))
        # SmartDashboard.putData('QuestUnSync', InstantCommand(lambda: self.quest_unsync_odometry()).ignoringDisable(True))
        SmartDashboard.putData('QuestEnableToggle', InstantCommand(lambda: self.quest_enabled_toggle()).ignoringDisable(True))
        SmartDashboard.putData('QuestSyncToggle', InstantCommand(lambda: self.quest_sync_toggle()).ignoringDisable(True))

    def reset_pose_with_quest(self, pose: Pose2d) -> None:
        #self.reset_pose(pose)
        self.questnav.set_pose(pose.transformBy(self.quest_to_robot.inverse()))

    def quest_reset_odometry(self) -> None:
        """Reset robot odometry at the Subwoofer."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.questnav.set_pose(Pose2d(14.337, 4.020, Rotation2d.fromDegrees(0)).transformBy(self.quest_to_robot.inverse()))
        else:
            self.questnav.set_pose(Pose2d(3.273, 4.020, Rotation2d.fromDegrees(180)).transformBy(self.quest_to_robot.inverse()))
        print(f"Reset questnav at {Timer.getFPGATimestamp():.1f}s")
        self.quest_unsync_odometry()

    def quest_sync_odometry(self) -> None:
        self.quest_has_synched = True  # let the robot know we have been synched so we don't automatically do it again
        self.questnav.set_pose(self.get_pose().transformBy(self.quest_to_robot.inverse()))
        print(f'Synched quest at {Timer.getFPGATimestamp():.1f}s')
        SmartDashboard.putBoolean('questnav_synched', self.quest_has_synched)

    def quest_unsync_odometry(self) -> None:
        self.quest_has_synched = False  # let the robot know we have been synched so we don't automatically do it again
        print(f'Unsynched quest at {Timer.getFPGATimestamp():.1f}s')
        SmartDashboard.putBoolean('questnav_synched', self.quest_has_synched)

    def quest_enabled_toggle(self, force=None):  # allow us to stop using quest if it is a problem - 20251014 CJH
        if force is None:
            self.use_quest = not self.use_quest  # toggle the boolean
        elif force == 'on':
            self.use_quest = True
        elif force == 'off':
            self.use_quest = False
        else:
            self.use_quest = False

        print(f'swerve use_quest updated to {self.use_quest} at {Timer.getFPGATimestamp():.1f}s')
        SmartDashboard.putBoolean('questnav_in_use', self.use_quest)

    def quest_sync_toggle(self, force=None):  # toggle sync state for dashboard - 20251014 CJH
        current_state = self.quest_has_synched
        if force is None:
            current_state = not current_state  # toggle the boolean
        elif force == 'on':
            current_state = True
        elif force == 'off':
            current_state = False
        else:
            current_state = False

        if current_state:
            self.quest_sync_odometry()
        else:
            self.quest_unsync_odometry()
        # reporting done by the sync/unsync functions

    def is_quest_enabled(self):
        return self.use_quest

    def periodic(self) -> None:
        self.counter += 1

        self.questnav.command_periodic()
        quest_pose = self.questnav.get_pose().transformBy(self.quest_to_robot)
        self.quest_field.setRobotPose(quest_pose)

        if self.counter % 10 == 0:
            SmartDashboard.putData("QUEST_FIELD", self.quest_field)
            if 0 < quest_pose.x < 17.658 and 0 < quest_pose.y < 8.131 and self.questnav.is_connected():
                self.quest_pose_accepted = True
            else:
                self.quest_pose_accepted = False
            SmartDashboard.putBoolean("QUEST_POSE_ACCEPTED", self.quest_pose_accepted)

            SmartDashboard.putNumberArray("Quest_Pose2D_AdvScope", [quest_pose.x, quest_pose.y, quest_pose.rotation().radians()])
            SmartDashboard.putBoolean("QUEST_CONNECTED", self.questnav.is_connected())
            SmartDashboard.putBoolean("QUEST_TRACKING", self.questnav.is_tracking())
            SmartDashboard.putNumber("Quest_Battery_%", self.questnav.get_battery_percent())
            SmartDashboard.putNumber("Quest_Latency", self.questnav.get_latency())
            SmartDashboard.putNumber("Quest_Tracking_lost_count", self.questnav.get_tracking_lost_counter())
            SmartDashboard.putNumber("Quest_frame_count", self.questnav.get_frame_count())

    def is_pose_accepted(self):
        return self.quest_pose_accepted

