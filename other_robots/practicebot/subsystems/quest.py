import wpilib
from commands2 import SubsystemBase, InstantCommand
from wpilib import SmartDashboard, DriverStation, Timer, Field2d
from wpimath.units import inchesToMeters
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Pose3d, Rotation3d, Translation3d, Transform2d, Transform3d
from ntcore import NetworkTableInstance

from helpers.questnav.questnav import QuestNav as Metaquestnav
from wpilib import DataLogManager

import constants

# TODO - clean and speed this up
class Questnav(SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Quest')
        self.counter = constants.VisionConstants.k_counter_offset

        self.questnav = Metaquestnav()

        self.quest_pose_accepted = False  # validate our location on a field

        self.quest_to_robot = Transform2d(inchesToMeters(-8.35), inchesToMeters(-10.50), Rotation2d().fromDegrees(270)) #10.50 -8.35
        # This is quest-centric coordinate. X is robot center position -8.35 inch as seen rom Quest. y is robot center -10.50 inches as seen from Quest
        # self.quest_to_robot = Transform2d(inchesToMeters(4), 0, Rotation2d().fromDegrees(0))
        self.quest_field = Field2d()

        self.quest_has_synched = False  # use this to check in disabled whether to update the quest with the robot odometry
        self.use_quest = constants.k_use_quest_odometry
        self.quest_pose = Pose2d(-10, -10, Rotation2d.fromDegrees(0)) # initial pose if not connected / tracking

        self._init_networktables()

        if not wpilib.RobotBase.isSimulation():
            DataLogManager.start()  # start wpilib datalog for AdvantageScope

    def _init_networktables(self):
        self.inst = NetworkTableInstance.getDefault()

        # ------------- Publishers (Efficiency) -------------
        self.quest_synched_pub = self.inst.getBooleanTopic("/QuestNav/questnav_synched").publish()
        self.quest_in_use_pub = self.inst.getBooleanTopic("/QuestNav/questnav_in_use").publish()
        
        self.quest_accepted_pub = self.inst.getBooleanTopic("/QuestNav/QUEST_POSE_ACCEPTED").publish()
        self.quest_connected_pub = self.inst.getBooleanTopic("/QuestNav/QUEST_CONNECTED").publish()
        self.quest_tracking_pub = self.inst.getBooleanTopic("/QuestNav/QUEST_TRACKING").publish()
        
        # Use StructPublisher for Pose2d - matches Swerve implementation and works with AdvantageScope
        self.quest_pose_pub = self.inst.getStructTopic("/QuestNav/Quest_Pose2D_AdvScope", Pose2d).publish()
        
        self.quest_battery_pub = self.inst.getDoubleTopic("/QuestNav/Quest_Battery_%").publish()
        self.quest_latency_pub = self.inst.getDoubleTopic("/QuestNav/Quest_Latency").publish()
        self.quest_lost_count_pub = self.inst.getDoubleTopic("/QuestNav/Quest_Tracking_lost_count").publish()
        self.quest_frame_count_pub = self.inst.getDoubleTopic("/QuestNav/Quest_frame_count").publish()

        # ------------- Subscribers -------------
        # Subscribe to the drive_pose published by Swerve (now a Struct)
        self.drive_pose2d_sub = self.inst.getStructTopic("/SmartDashboard/Swerve/drive_pose2d", Pose2d).subscribe(Pose2d())

        # ------------- Initial Values & Buttons -------------
        self.quest_synched_pub.set(self.quest_has_synched)
        self.quest_in_use_pub.set(self.use_quest)

        # note - have Risaku standardize these with the rest of the putDatas
        SmartDashboard.putData('Quest/ResetOdometry', InstantCommand(lambda: self.quest_reset_odometry()).ignoringDisable(True))
        SmartDashboard.putData('Quest/SyncOdometry', InstantCommand(lambda: self.quest_sync_odometry()).ignoringDisable(True))
        SmartDashboard.putData('Quest/EnableToggle', InstantCommand(lambda: self.quest_enabled_toggle()).ignoringDisable(True))
        SmartDashboard.putData('Quest/SyncToggle', InstantCommand(lambda: self.quest_sync_toggle()).ignoringDisable(True))
        
        # Put Field2d once (*** it updates itself internally ***)
        SmartDashboard.putData("Quest/Field", self.quest_field)

    def set_quest_pose(self, pose: Pose2d) -> None:
        # set the pose of the Questnav, transforming from robot center top questnav coordinate
        self.questnav.set_pose(Pose3d(pose.transformBy(self.quest_to_robot.inverse())))

    def reset_pose_with_quest(self, pose: Pose2d) -> None:
        #self.reset_pose(pose)
        self.set_quest_pose(pose)

    def quest_reset_odometry(self) -> None:
        """Reset robot odometry at the Subwoofer."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.set_quest_pose(Pose2d(14.337, 4.020, Rotation2d.fromDegrees(0)))
        else:
            self.set_quest_pose(Pose2d(3.273, 4.020, Rotation2d.fromDegrees(180)))
        print(f"Reset questnav at {Timer.getFPGATimestamp():.1f}s")
        self.quest_unsync_odometry()

    def quest_sync_odometry(self) -> None:
        self.quest_has_synched = True  # let the robot know we have been synched so we don't automatically do it again
        
        # Efficiently get the pose from the subscriber (returns a Pose2d object)
        self.set_quest_pose(self.drive_pose2d_sub.get())
        self.quest_synched_pub.set(self.quest_has_synched)

    def quest_unsync_odometry(self) -> None:
        self.quest_has_synched = False  # let the robot know we have been synched so we don't automatically do it again
        print(f'Unsynched quest at {Timer.getFPGATimestamp():.1f}s')
        self.quest_synched_pub.set(self.quest_has_synched)

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
        self.quest_in_use_pub.set(self.use_quest)

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

        frames = self.questnav.get_all_unread_pose_frames()
        for frame in frames:
            if self.questnav.is_connected and self.questnav.is_tracking():
                try:
                    self.quest_pose = frame.quest_pose_3d.toPose2d().transformBy(self.quest_to_robot)
                    quest_pose_old = self.quest_pose
                except Exception as e:
                    print(f"Error converting QuestNav Pose3d to Pose2d: {e}")
                    self.quest_pose = quest_pose_old  # use last good pose in cases of error
                    continue

        # quest_pose = self.questnav.get_pose().transformBy(self.quest_to_robot)
        self.quest_field.setRobotPose(self.quest_pose)

        if self.counter % 10 == 0:
            if 0 < self.quest_pose.x < 17.658 and 0 < self.quest_pose.y < 8.131 and self.questnav.is_connected():
                self.quest_pose_accepted = True
            else:
                self.quest_pose_accepted = False
            
            self.quest_accepted_pub.set(self.quest_pose_accepted)
            self.quest_pose_pub.set(self.quest_pose)
            self.quest_connected_pub.set(self.questnav.is_connected())
            self.quest_tracking_pub.set(self.questnav.is_tracking())
            self.quest_battery_pub.set(self.questnav.get_battery_percent())
            self.quest_latency_pub.set(self.questnav.get_latency())
            self.quest_lost_count_pub.set(self.questnav.get_tracking_lost_counter())
            self.quest_frame_count_pub.set(self.questnav.get_frame_count())

    def is_pose_accepted(self):
        return self.quest_pose_accepted
