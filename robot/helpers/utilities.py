# repo for utility functions anyone can use

# figure out the nearest stage - or any tag, I suppose if we pass in a list
import math

from wpimath.geometry import Pose2d, Translation2d, Rotation2d
import robotpy_apriltag as ra

# Optimization: Load the field layout once on boot, not every time we ask for a tag
field_layout = ra.AprilTagFieldLayout.loadField(ra.AprilTagField.k2025ReefscapeWelded)


def get_nearest_tag(current_pose, destination='stage'):
    """ Return the nearest allowed tag to a given pose """
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


from rev import ClosedLoopSlot
def compare_motors(motor_a, motor_b, name_a="Motor A", name_b="Motor B"):
    """
    Compares two REV Spark motors using the 2025 configAccessor API.
    All numeric values are limited to 6 significant digits.
    """

    def get_motor_state(motor):
        ca = motor.configAccessor
        s0 = ClosedLoopSlot.kSlot0

        # Follower logic: Return ID or 'None'
        leader_id = ca.getFollowerModeLeaderId()
        follower_status = f"ID {leader_id}" if leader_id > 0 else "None"

        # Using descriptive, snake_case keys (Pythonic)
        return {
            "device_id": motor.getDeviceId(),
            "is_inverted": ca.getInverted(),
            "idle_mode": str(ca.getIdleMode()).split('.')[-1],
            "follower_leader_id": follower_status,
            "smart_current_limit_amps": ca.getSmartCurrentLimit(),
            "position_conversion": ca.encoder.getPositionConversionFactor(),
            "velocity_conversion": ca.encoder.getVelocityConversionFactor(),
            "pid_p_gain_slot_0": ca.closedLoop.getP(s0),
            "pid_i_gain_slot_0": ca.closedLoop.getI(s0),
            "pid_d_gain_slot_0": ca.closedLoop.getD(s0),
            "pid_ff_gain_slot_0": ca.closedLoop.getFF(s0),
            "max_output_slot_0": ca.closedLoop.getMaxOutput(s0),
            "min_output_slot_0": ca.closedLoop.getMinOutput(s0),
        }

    def format_value(value):
        """
        Descriptive Pythonic formatting:
        - 5 decimal places for floats - Strips unnecessary trailing zeros - Leaves strings/ints as is
        """
        if isinstance(value, float):
            # format to 5 decimal places, then strip trailing zeros and potential dot
            return f"{value:.5f}".rstrip('0').rstrip('.')
        return str(value)

    # 1. Collect Data
    try:
        data_a = get_motor_state(motor_a)
        data_b = get_motor_state(motor_b)
    except Exception as e:
        print(f"Error: 2025 API Access Failed. {e}")
        return

    # 2. Table Setup
    col_width_label = 30
    col_width_data = 25

    header = f"{'Setting Name':<{col_width_label}} | {name_a:<{col_width_data}} | {name_b:<{col_width_data}}"
    separator = "-" * len(header)

    print(f"\n{header}\n{separator}")

    # 3. Iterative Comparison
    for key in data_a.keys():
        val_a = format_value(data_a[key])
        val_b = format_value(data_b[key])

        # Mark differences for quick scanning
        marker = "!" if val_a != val_b else " "

        # Print row with clean descriptive name
        print(f"{marker} {key:<{col_width_label - 2}} | {val_a:<{col_width_data}} | {val_b:<{col_width_data}}")
    print()

# Example usage:
# compare_motors_2025(left_drive, right_drive, "Left_Master", "Right_Master")