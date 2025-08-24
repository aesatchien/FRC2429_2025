from wpimath.geometry import Pose2d, Quaternion, Rotation2d, Translation2d
from ntcore import NetworkTableInstance
from wpilib import Timer
from wpimath.units import seconds


class QuestNav:
    def __init__(self):
        self.status = {"READY": 0,
                  "HEADING_RESET_COMPLETE": 99,
                  "POSE_RESET_COMPLETE": 98,
                  "PING_RESPONSE": 97}

        self.command = {"CLEAR": 0,
                   "RESET_HEADING": 1,
                   "RESET_POSE": 2,
                   "PING": 3}

        self.nt4instance = NetworkTableInstance.getDefault()
        self.nt4Table = self.nt4instance.getTable("questnav")
        self.quest_miso = self.nt4Table.getIntegerTopic("miso").subscribe(0)
        self.quest_mosi = self.nt4Table.getIntegerTopic("mosi").publish()

        self.quest_timestamp = self.nt4Table.getDoubleArrayTopic("timestamp").subscribe([-1.0])

        self.quest_position = self.nt4Table.getFloatArrayTopic("position").subscribe([-1.0, -1.0, -1.0])

        self.quest_quaternion = self.nt4Table.getFloatArrayTopic("quaternion").subscribe([-1.0, -1.0, -1.0, -1.0])

        self.quest_euler_angles = self.nt4Table.getFloatArrayTopic("eulerAngles").subscribe([-1.0, -1.0, -1.0])

        self.quest_frame_count = self.nt4Table.getIntegerTopic("frameCount").subscribe(-1)

        self.quest_battery_percent = self.nt4Table.getDoubleTopic("device/batteryPercent").subscribe(-1.0)

        self.quest_is_tracking = self.nt4Table.getBooleanTopic("device/isTracking").subscribe(False)

        self.quest_tracking_lost_count = self.nt4Table.getIntegerTopic("device/trackingLostCounter").subscribe(-1)

        self.reset_pose_pub = self.nt4Table.getDoubleArrayTopic("resetpose").publish()

        self.heartbeat_request_sub = self.nt4Table.getDoubleTopic("heartbeat/quest_to_robot").subscribe(-1.0)

        self.heartbeat_response_pub = self.nt4Table.getDoubleTopic("heartbeat/robot_to_quest").publish()

        self.last_processed_heartbeat_id = 0

    def set_pose(self, pose: Pose2d) -> None:
        """Set the Quest's pose."""
        self.reset_pose_pub.set(
            [
                pose.x,
                pose.y,
                pose.rotation().degrees()
            ]
        )
        self.quest_mosi.set(self.command["RESET_POSE"])

    def process_heartbeat(self) -> None:
        """Process heartbeat updates."""
        request_id = self.heartbeat_request_sub.get()

        if request_id > 0 and request_id != self.last_processed_heartbeat_id:
            self.heartbeat_response_pub.set(request_id)
            self.last_processed_heartbeat_id = request_id

    def get_battery_percent(self) -> float:
        """Gets the battery percentage of the headset."""
        return self.quest_battery_percent.get()

    def get_tracking_status(self) -> bool:
        return self.quest_is_tracking.get()

    def get_frame_count(self) -> int:
        return self.quest_frame_count.get()

    def get_tracking_lost_counter(self) -> int:
        return self.quest_tracking_lost_count.get()

    # TODO Unclear if this conversion is right
    def get_connected(self) -> seconds:
        return seconds(Timer.getTimestamp() - (self.quest_timestamp.getLastChange() / 1000000)).__lt__(0.25)

    def get_quaternion(self) -> Quaternion:
        qq_floats = self.quest_quaternion.get()
        return Quaternion(qq_floats[0], qq_floats[1], qq_floats[2], qq_floats[3])

    def get_timestamp(self) -> int:
        return self.quest_timestamp.getAtomic().serverTime

    def cleanup_responses(self) -> None:
        if self.quest_miso.get() != self.status["READY"]:
            if int(self.quest_miso.get()) == self.status["POSE_RESET_COMPLETE"]:
                self.quest_mosi.set(self.command["CLEAR"])
            elif int(self.quest_miso.get()) == self.status["HEADING_RESET_COMPLETE"]:
                self.quest_mosi.set(self.command["CLEAR"])
            elif int(self.quest_miso.get()) == self.status["PING_RESPONSE"]:
                self.quest_mosi.set(self.command["CLEAR"])

    def get_yaw(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-self.quest_euler_angles.get()[1])

    def get_translation(self) -> Translation2d:
        questnav_position = self.quest_position.get()
        return Translation2d(questnav_position[2], -questnav_position[0])

    def get_pose(self) -> Pose2d:
        return Pose2d(self.get_translation(), self.get_yaw())
