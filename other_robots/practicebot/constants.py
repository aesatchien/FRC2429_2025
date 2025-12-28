import math
import rev
import robotpy_apriltag
import wpilib
 
from rev import ClosedLoopSlot, SparkClosedLoopController, SparkFlexConfig, SparkMax, SparkMaxConfig
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import inchesToMeters, lbsToKilograms
from wpimath.system.plant import DCMotor
from wpilib.simulation import SingleJointedArmSim
from subsystems.swerve_constants import DriveConstants

# TODO - organize this better
k_enable_logging = False  # allow logging from Advantagescope (in swerve.py), but really we may as well start it here

# starting position for odometry
k_start_x, k_start_y  = 0, 0

# ------------  joysticks and other input ------------
k_driver_controller_port = 0
k_co_driver_controller_port = 1

# should be fine to burn on every reboot, but we can turn this off
k_burn_flash = True

#  ----------  network tables organization - one source for truth in publishing
# systems outside the robot
camera_prefix = r'/Cameras'  # from the pis
quest_prefix = r'/QuestNav'  # putting this on par with the cameras as an external system

# Dictionary mapping Logical Name -> NetworkTables Camera Name in /Cameras
k_cameras = {
    'ardu_high_tags': 'ArducamHigh',
    'ardu_back_tags': 'ArducamBack',
    'genius_low_tags': 'GeniusLow',
    'logi_reef_tags': 'LogitechReef',
}

# systems inside/from the robot
status_prefix = r'/SmartDashboard/RobotStatus'  # the default for any status message
vision_prefix = r'/SmartDashboard/Vision'  # from the robot
swerve_prefix = r'/SmartDashboard/Swerve'  # from the robot
sim_prefix = r'/SmartDashboard/Sim'  # from the sim (still from the robot)
command_prefix = r'Command'  # SPECIAL CASE: the SmartDashboard.putData auto prepends /SmartDashboard to the key


k_swerve_debugging_messages = True
# multiple attempts at tags this year - TODO - use l/r or up/down tilted cameras again, gives better data
k_use_quest_odometry = True
k_use_photontags = False  # take tags from photonvision camera
k_use_CJH_tags = True  # take tags from the pis
k_swerve_only = False
k_swerve_rate_limited = True
k_field_oriented = True  # is there any reason for this at all?



# TODO - this whole apriltag section belongs somewhere else - maybe in helpers/utilities.py
# ----------   APRILTAG HELPERS  -----------
# Load the AprilTag field layout
layout = robotpy_apriltag.AprilTagFieldLayout.loadField(robotpy_apriltag.AprilTagField.k2025ReefscapeWelded)

# Dictionary to store robot poses
k_useful_robot_poses_blue = {}
# Tag-to-branch name mapping
branch_names = ["cd", "ab", "kl", "ij", "gh", "ef"]

# Store tag positions for plotting
tag_positions = {}

# Compute useful robot poses
for tag_id in range(17, 23):
    branch_name_idx = tag_id - 17

    this_face_tag_pose = layout.getTagPose(tag_id)

    if this_face_tag_pose:
        # Get the tag's position and rotation
        tag_translation = this_face_tag_pose.translation().toTranslation2d()
        tag_yaw = Rotation2d(this_face_tag_pose.rotation().Z())

        # Store the tag position for plotting
        tag_positions[tag_id] = (tag_translation.X(), tag_translation.Y())

        # Compute robot rotation and offsets
        robot_rotation = tag_yaw + Rotation2d(math.radians(-90))  # CJH changed this to get new orientation right
        # imagine the tag is at the origin facing in +x. this is your reference frame for these offsets.
        # see ../resources/plots/useful_robot_locations.ipynb
        coral_center_offset = 0.0  # the center of the arm is not the center of the robot - this is in y because we rotated 90
        x_offset = 0.47 # how far back from the tag the center of the robot should be - DEFINITELY POSITIVE
        right_y_offset = 0.15  # POSITIVE  - because of tag yaw we have to add the right  was 0.19 on 20251006 - CJH changed to 0.15
        left_y_offset = 0.17  # POSITIVE  - because of tag yaw we have to subtract the left below
        robot_offset_left = Translation2d(x_offset, -left_y_offset - coral_center_offset).rotateBy(tag_yaw)
        robot_offset_right = Translation2d(x_offset - coral_center_offset, +right_y_offset - coral_center_offset).rotateBy(tag_yaw)

        # Compute robot positions
        left_branch_position = tag_translation + robot_offset_left
        right_branch_position = tag_translation + robot_offset_right

        # Get branch names
        left_branch_name = branch_names[branch_name_idx][0]
        right_branch_name = branch_names[branch_name_idx][1]

        # Store poses
        k_useful_robot_poses_blue[left_branch_name] = Pose2d(left_branch_position, robot_rotation)
        k_useful_robot_poses_blue[right_branch_name] = Pose2d(right_branch_position, robot_rotation)

        # print(f'tag:{tag_id}: Trans: {tag_translation}  Theta: {tag_yaw}')  # CJH trying to debug this stuff - this is correct
        # print(f'tag:{tag_id}:  rot: {robot_rotation.degrees():.1f} R: {k_useful_robot_poses_blue[right_branch_name]}  L: {k_useful_robot_poses_blue[left_branch_name] }')

# ----------   END APRILTAG HELPERS  -----------


class SimConstants:
    k_counter_offset = 1

class VisionConstants:

    k_counter_offset = 2
    k_nt_debugging = False  # print extra values to NT for debugging
    k_pi_names = ["top_pi"]

    k_valid_tags = list(range(1, 23))


class QuestConstants:
    k_counter_offset = 3


class LedConstants:

    k_counter_offset = 4
    k_nt_debugging = False  # print extra values to NT for debugging
    k_led_count = 40  # correct as of 2025 0305
    k_led_count_ignore = 4  # flat ones not for the height indicator
    k_led_pwm_port = 0  # correct as of 2025 0305

class RobotStateConstants:

    k_counter_offset = 5
    k_nt_debugging = False  # print extra values to NT for debugging

class DrivetrainConstants:

    k_counter_offset = 6
    k_nt_debugging = False  # print extra values to NT for debugging
    # these are for the apriltags.  For the most part, you want to trust the gyro, not the tags for angle
    # based on https://www.chiefdelphi.com/t/swerve-drive-pose-estimator-and-add-vision-measurement-using-limelight-is-very-jittery/453306/13
    k_pose_stdevs_large = (2, 2
                           , 10)  # use when you don't trust the april tags - stdev x, stdev y, stdev theta
    k_pose_stdevs_disabled = (1, 1, 2)  # use when we are disabled to quickly get updates
    k_pose_stdevs_small = (0.1, 0.1, 10)  # use when you do trust the tags
