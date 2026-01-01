import math
import robotpy_apriltag
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

# This data is initialized once when the module is first imported.
layout = robotpy_apriltag.AprilTagFieldLayout.loadField(robotpy_apriltag.AprilTagField.k2025ReefscapeWelded)

# Mapping from letter to tag ID and which side of the tag it corresponds to.
# Note: 'e' is the right side of tag 22, 'f' is the left side, etc.
letter_map = {
    'a': {'tag_id': 18, 'side': 'left'}, 'b': {'tag_id': 18, 'side': 'right'},
    'c': {'tag_id': 17, 'side': 'left'}, 'd': {'tag_id': 17, 'side': 'right'},
    'e': {'tag_id': 22, 'side': 'right'}, 'f': {'tag_id': 22, 'side': 'left'},
    'g': {'tag_id': 21, 'side': 'right'}, 'h': {'tag_id': 21, 'side': 'left'},
    'i': {'tag_id': 20, 'side': 'right'}, 'j': {'tag_id': 20, 'side': 'left'},
    'k': {'tag_id': 19, 'side': 'left'}, 'l': {'tag_id': 19, 'side': 'right'},
}

# Pre-calculate tag positions for plotting or other uses
tag_positions = {tag_id: layout.getTagPose(tag_id).translation().toTranslation2d()
                 for tag_id in range(17, 23) if layout.getTagPose(tag_id) is not None}

def get_reefscape_scoring_pose(letter: str) -> Pose2d:
    """
    Calculates the ideal robot pose to score on a specific branch (a-l) of the Reef.
    This encapsulates the geometric calculations based on the 2025 AprilTag field layout.

    :param letter: The single character name of the scoring branch ('a' through 'l').
    :return: A Pose2d representing the robot's target position and orientation,
             or a default Pose2d(0,0,0) if the letter is invalid or tag is not found.
    """
    letter = letter.lower()
    if letter not in letter_map:
        return Pose2d()

    map_info = letter_map[letter]
    tag_id = map_info['tag_id']
    side = map_info['side']

    this_face_tag_pose = layout.getTagPose(tag_id)
    if not this_face_tag_pose:
        return Pose2d()

    # --- Geometric calculations moved from constants.py ---
    tag_translation = this_face_tag_pose.translation().toTranslation2d()
    tag_yaw = Rotation2d(this_face_tag_pose.rotation().Z())

    robot_rotation = tag_yaw + Rotation2d(math.radians(-90))
    coral_center_offset = 0.0
    x_offset = 0.47
    right_y_offset = 0.15
    left_y_offset = 0.17
    robot_offset_left = Translation2d(x_offset, -left_y_offset - coral_center_offset).rotateBy(tag_yaw)
    robot_offset_right = Translation2d(x_offset - coral_center_offset, +right_y_offset - coral_center_offset).rotateBy(tag_yaw)

    left_branch_position = tag_translation + robot_offset_left
    right_branch_position = tag_translation + robot_offset_right

    if side == 'left':
        return Pose2d(left_branch_position, robot_rotation)
    else:  # side == 'right'
        return Pose2d(right_branch_position, robot_rotation)


# Dictionary to store robot poses for reefscape autos
k_useful_robot_poses_blue = {letter: get_reefscape_scoring_pose(letter) for letter in 'abcdefghijkl'}
