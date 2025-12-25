# repo for utility functions anyone can use

# figure out the nearest stage - or any tag, I suppose if we pass in a list
import math

from wpimath.geometry import Pose2d, Translation2d, Rotation2d
import robotpy_apriltag as ra

# Optimization: Load the field layout once on boot, not every time we ask for a tag
field_layout = ra.AprilTagFieldLayout.loadField(ra.AprilTagField.k2025ReefscapeWelded)

def get_nearest_tag(current_pose, destination='stage'):

    if destination == 'reef':
        # get all distances to the stage tags
        tags = [6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21,
                22]  # the ones we can see from driver's station - does not matter if red or blue
        x_offset, y_offset = -0.10, 0.10  # subtracting translations below makes +x INTO the tage, +y LEFT of tag
        robot_offset = Pose2d(Translation2d(x_offset, y_offset), Rotation2d(0))
        face_tag = True  # do we want to face the tag?
    else:
        raise ValueError('  location for get_nearest tag must be in ["stage", "amp"] etc')

    poses = [field_layout.getTagPose(tag).toPose2d() for tag in tags]
    distances = [current_pose.translation().distance(pose.translation()) for pose in poses]

    # sort the distances
    combined = list(zip(tags, distances))
    combined.sort(key=lambda x: x[1])  # sort on the distances
    sorted_tags, sorted_distances = zip(*combined)
    nearest_pose = field_layout.getTagPose(sorted_tags[0])  # get the pose of the nearest stage tag

    # transform the tag pose to our specific needs
    tag_pose = nearest_pose.toPose2d()  # work with a 2D pose
    tag_rotation = tag_pose.rotation()  # we are either going to match this or face opposite
    robot_offset_corrected = robot_offset.rotateBy(tag_rotation)  # rotate our offset so we align with the tag
    updated_translation = tag_pose.translation() - robot_offset_corrected.translation()  # careful with these signs
    updated_rotation = tag_rotation + Rotation2d(math.pi) if face_tag else tag_rotation  # choose if we flip
    updated_pose = Pose2d(translation=updated_translation, rotation=updated_rotation)  # drive to here

    return sorted_tags[0]  # changed this in 2025 instead of updated_pose