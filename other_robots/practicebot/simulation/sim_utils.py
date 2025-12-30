import math
from wpimath.geometry import Pose2d, Translation2d


def get_distance(pos1: Translation2d, pos2: Translation2d) -> float:
    """
    Calculate the Euclidean distance between two positions.
    Useful for determining how far the robot is from a target.

    :param pos1: The first Translation2d
    :param pos2: The second Translation2d
    :return: Distance in meters
    """
    return pos1.distance(pos2)


def distance_to_gamepiece(robot_pose: Pose2d, gamepiece_position: Translation2d) -> tuple[float, float]:
    """
    Calculate the distance and rotation from the robot's current position to a gamepiece.

    :param robot_pose: The current Pose2d of the robot.
    :param gamepiece_position: The Translation2d of the gamepiece.
    :return: Tuple (distance_in_meters, rotation_needed_in_degrees)
    """
    distance = get_distance(robot_pose.translation(), gamepiece_position)

    to_target = gamepiece_position - robot_pose.translation()
    rotation_needed = to_target.angle() - robot_pose.rotation()

    return distance, rotation_needed.degrees()


def is_on_gamepiece(robot_pose: Pose2d, gamepiece_position: Translation2d, threshold: float = 0.5) -> bool:
    """
    Check if the robot is "on" (close enough to interact with) a gamepiece.

    :param robot_pose: The current Pose2d of the robot.
    :param gamepiece_position: The Translation2d of the gamepiece.
    :param threshold: The distance threshold in meters (default 0.5m).
    :return: True if within threshold, False otherwise.
    """
    dist, _ = distance_to_gamepiece(robot_pose, gamepiece_position)
    return dist < threshold


def get_closest_gamepiece(robot_pose: Pose2d, active_gamepieces: list[Translation2d]) -> tuple[float, float, float, Pose2d]:
    """
    Find the closest gamepiece from a list of active gamepieces.

    :param robot_pose: The current Pose2d of the robot.
    :param active_gamepieces: A list of Translation2d objects for active gamepieces.
    :return: A tuple containing (distance, rotation, strafe, target_pose).
             Returns (inf, 0, 0, Pose2d()) if the list is empty.
    """
    if not active_gamepieces:
        return float('inf'), 0, 0, Pose2d()

    robot_translation = robot_pose.translation()

    # Find the gamepiece with the minimum distance to the robot
    closest = min(active_gamepieces, key=lambda gp: get_distance(robot_translation, gp))

    # Calculate properties
    dist, rot = distance_to_gamepiece(robot_pose, closest)

    # Calculate strafe (Y component in robot frame)
    vec_robot_frame = (closest - robot_translation).rotateBy(-robot_pose.rotation())
    strafe = vec_robot_frame.Y()

    # Target Pose: Location of gamepiece, rotated to face the robot
    target_rotation = (closest - robot_translation).angle()
    target_pose = Pose2d(closest, target_rotation)

    return dist, rot, strafe, target_pose