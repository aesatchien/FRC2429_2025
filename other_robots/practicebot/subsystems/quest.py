import wpilib
import random
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
        self.counter = constants.QuestConstants.k_counter_offset

        self.questnav = Metaquestnav()

        self.quest_pose_accepted = False  # validate our location on a field

        self.quest_to_robot = constants.QuestConstants.quest_to_robot  # 10.50 -8.35
        # This is quest-centric coordinate. X is robot center position -8.35 inch as seen rom Quest. y is robot center -10.50 inches as seen from Quest
        # self.quest_to_robot = Transform2d(inchesToMeters(4), 0, Rotation2d().fromDegrees(0))
        # self.quest_field = Field2d()

        # ping the quest - it seems to respond to is_connected only after you get frames ~ 2s after we connect
        # and although the headset says it is connected to the sim, is_connected() stays False unless you get frames
        self._ping_connection()

        self.quest_has_synched = False  # use this to check in disabled whether to update the quest with the robot odometry
        self.use_quest = constants.k_use_quest_odometry
        self.quest_pose = Pose2d(-10, -10, Rotation2d.fromDegrees(0)) # initial pose if not connected / tracking

        # Simulation variables
        # Start with a large error to verify syncing works (e.g., Quest thinks world origin off by x=2, y=2)
        self.sim_offset_from_truth = Transform2d(Translation2d(2, 2), Rotation2d.fromDegrees(30))
        # random walk - using uniform distribution, so sigma step ~ a/sqrt(3) and in n steps magnitude is sqrt n * sigma step
        self.walk_xy = 0.005  # meters per loop, 0.005 per step at 50x/second should be 0.02 m/sec
        self.walk_deg = 0.1  # degrees per loop

        self._init_networktables()

    def _ping_connection(self):
        connected_before = self.questnav.is_connected()
        frames = self.questnav.get_all_unread_pose_frames()  # this causes a networked to quest to ACK
        connected_after = self.questnav.is_connected()
        if connected_after != connected_before:
            # let us know if the situation changes
            print(f'*** {Timer.getFPGATimestamp():04.1f}s: Questnav connection status {connected_before} before ping and {connected_after} after  ***')

    def _init_networktables(self):
        self.inst = NetworkTableInstance.getDefault()
        quest_prefix = constants.quest_prefix
        swerve_prefix = constants.swerve_prefix
        sim_prefix = constants.sim_prefix

        # ------------- Publishers (Efficiency) -------------
        self.quest_synched_pub = self.inst.getBooleanTopic(f"{quest_prefix}/questnav_synched").publish()
        self.quest_in_use_pub = self.inst.getBooleanTopic(f"{quest_prefix}/questnav_in_use").publish()
        
        self.quest_accepted_pub = self.inst.getBooleanTopic(f"{quest_prefix}/quest_pose_accepted").publish()
        self.quest_connected_pub = self.inst.getBooleanTopic(f"{quest_prefix}/quest_connected").publish()
        self.quest_tracking_pub = self.inst.getBooleanTopic(f"{quest_prefix}/quest_tracking").publish()
        
        # Use StructPublisher for Pose2d - matches Swerve implementation and works with AdvantageScope
        self.quest_pose_pub = self.inst.getStructTopic(f"{quest_prefix}/quest_pose", Pose2d).publish()
        # self.pose_pub = self.inst.getDoubleArrayTopic(f"{quest_prefix}/QUEST_POSE").publish()  # legacy GUI dashboard
        
        self.quest_battery_pub = self.inst.getDoubleTopic(f"{quest_prefix}/quest_Battery_pct").publish()
        self.quest_latency_pub = self.inst.getDoubleTopic(f"{quest_prefix}/quest_Latency").publish()
        self.quest_lost_count_pub = self.inst.getDoubleTopic(f"{quest_prefix}/quest_tracking_lost_count").publish()
        self.quest_frame_count_pub = self.inst.getDoubleTopic(f"{quest_prefix}/quest_frame_count").publish()

        # ------------- Subscribers -------------
        # Subscribe to the drive_pose published by Swerve (now a Struct)
        self.drive_pose_sub = self.inst.getStructTopic(f"{swerve_prefix}/drive_pose", Pose2d).subscribe(Pose2d())

        # Subscribe to ground truth from Physics (for Sim) - doesn't really matter if it doesn't exist when real
        self.ground_truth_sub = self.inst.getStructTopic(f"{sim_prefix}/ground_truth", Pose2d).subscribe(Pose2d())

        # ------------- Initial Values & Buttons -------------
        self.quest_synched_pub.set(self.quest_has_synched)
        self.quest_in_use_pub.set(self.use_quest)

        # note - may not want these buried one deeper.  Also,, we may want to put them in robotcontainer instead of here
        command_prefix = constants.command_prefix
        SmartDashboard.putData(f'{command_prefix}/QuestResetOdometry', InstantCommand(lambda: self.quest_reset_odometry()).ignoringDisable(True))
        SmartDashboard.putData(f'{command_prefix}/QuestSyncOdometry', InstantCommand(lambda: self.quest_sync_odometry()).ignoringDisable(True))
        SmartDashboard.putData(f'{command_prefix}/QuestEnableToggle', InstantCommand(lambda: self.quest_enabled_toggle()).ignoringDisable(True))
        SmartDashboard.putData(f'{command_prefix}/QuestSyncToggle', InstantCommand(lambda: self.quest_sync_toggle()).ignoringDisable(True))
        
        # Put Field2d once (*** it updates itself internally ***)
        # TODO - this needs to be broadcast based on the prefixes in constants
        # SmartDashboard.putData("Quest/Field", self.quest_field)

    def set_quest_pose(self, pose: Pose2d) -> None:
        # set the pose of the Questnav, transforming from robot center top questnav coordinate
        self.questnav.set_pose(Pose3d(pose.transformBy(self.quest_to_robot.inverse())))
        
        if wpilib.RobotBase.isSimulation() and not self.questnav.is_connected():
            # In Sim, calculate the error needed so that (Truth + Error) = TargetPose
            # This allows us to "reset" the quest to a specific field location even if the robot isn't there
            ground_truth = self.ground_truth_sub.get()
            self.sim_offset_from_truth = Transform2d(
                pose.X() - ground_truth.X(),
                pose.Y() - ground_truth.Y(),
                pose.rotation() - ground_truth.rotation()
            )
            print(f"Sim Quest Reset: Error reset to {self.sim_offset_from_truth.X():.2f}, {self.sim_offset_from_truth.Y():.2f}")

    def reset_pose_with_quest(self, pose: Pose2d) -> None:
        # this came over from swerve, maybe we don't need it anymore
        #self.reset_pose(pose)
        self.set_quest_pose(pose)

    def quest_reset_odometry(self) -> None:
        """Reset robot odometry at the Subwoofer."""
        red_pose = Pose2d(14.0, 4.00, Rotation2d.fromDegrees(0))
        blue_pose = Pose2d(3., 4.00, Rotation2d.fromDegrees(180))

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            new_pose = red_pose
        else:
            new_pose = blue_pose

        self.set_quest_pose(new_pose)
        print(f"Reset questnav at {Timer.getFPGATimestamp():.1f}s")
        self.quest_unsync_odometry()

    def quest_sync_odometry(self) -> None:
        self.quest_has_synched = True  # let the robot know we have been synched so we don't automatically do it again
        
        if wpilib.RobotBase.isSimulation() and not self.questnav.is_connected():
            # In Sim, "Sync" means agree with Ground Truth (remove all drift/error)
            self.set_quest_pose(self.ground_truth_sub.get())
        else:
            # In Real life, "Sync" means agree with the Robot's Odometry
            # Although, really, the swerve sim is basically serving that same sim ground truth
            self.set_quest_pose(self.drive_pose_sub.get())

        self.quest_synched_pub.set(self.quest_has_synched)
        print(f'  synched quest to {self.quest_pose}\nusing               {self.drive_pose_sub.get()}')

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

    def is_pose_accepted(self):
        return self.quest_pose_accepted

    def periodic(self) -> None:
        self.counter += 1

        self.questnav.command_periodic()

        # todo - this should be simplified
        if wpilib.RobotBase.isReal() or self.questnav.is_connected():
            frames = self.questnav.get_all_unread_pose_frames()
            for frame in frames:
                if self.questnav.is_connected() and self.questnav.is_tracking():  # i think RT had a typo here and was checking True vs self.questnav.is_connected (the fn, not the return val)
                    try:
                        self.quest_pose = frame.quest_pose_3d.toPose2d().transformBy(self.quest_to_robot)
                        quest_pose_old = self.quest_pose
                    except Exception as e:
                        print(f"Error converting QuestNav Pose3d to Pose2d: {e}")
                        self.quest_pose = quest_pose_old  # use last good pose in cases of error
                        continue

        else:  # simulate a pose read ground truth from sim and apply the current "drift/error" of the Quest
            
            # Add a random walk to the error to simulate drift
            self.sim_offset_from_truth = Transform2d(
                self.sim_offset_from_truth.X() + (random.random() - 0.5) * 2 * self.walk_xy,
                self.sim_offset_from_truth.Y() + (random.random() - 0.5) * 2 * self.walk_xy,
                Rotation2d.fromDegrees(self.sim_offset_from_truth.rotation().degrees() + (random.random() - 0.5) * 2 * self.walk_deg)
            )

            ground_truth:Pose2d = self.ground_truth_sub.get()
            # Apply the offset (Transform) to the ground truth Pose
            # FIX: Apply offset field-relatively (simple addition) instead of robot-relatively (transformBy)
            self.quest_pose = Pose2d(
                ground_truth.X() + self.sim_offset_from_truth.X(),
                ground_truth.Y() + self.sim_offset_from_truth.Y(),
                ground_truth.rotation() + self.sim_offset_from_truth.rotation()
            )

        # does this belong here?  not sure what it's for - CJH
        # self.quest_pose = self.get_pose().transformBy(self.quest_to_robot)

        # why do we need more than one field?  does this get read by the quest itself
        # self.quest_field.setRobotPose(self.quest_pose)

        if self.counter % 10 == 0:
            if 0 < self.quest_pose.x < 17.658 and 0 < self.quest_pose.y < 8.131 and self.questnav.is_connected():
                self.quest_pose_accepted = True
            else:
                self.quest_pose_accepted = False
            
            # poses used in gui and advantagescope
            self.quest_pose_pub.set(self.quest_pose)
            #self.pose_pub.set([self.quest_pose.X(), self.quest_pose.Y(), self.quest_pose.rotation().degrees()])  # legacy GUI version


            self.quest_accepted_pub.set(self.quest_pose_accepted)  # GUI uses as VALID
            self.quest_connected_pub.set(self.questnav.is_connected())  # GUI uses as heartbeat
            self.quest_tracking_pub.set(self.questnav.is_tracking())  # GUI follows to see if tracking

            self.quest_battery_pub.set(self.questnav.get_battery_percent())
            self.quest_latency_pub.set(self.questnav.get_latency())
            self.quest_lost_count_pub.set(self.questnav.get_tracking_lost_counter())
            self.quest_frame_count_pub.set(self.questnav.get_frame_count())

        # ping the quest every few seconds in sim to check to get a connection to physical hardware
        if wpilib.RobotBase.isSimulation() and self.counter % 200 == 0 and not self.questnav.is_connected():
            self._ping_connection()
