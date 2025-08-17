from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpilib import Timer, SmartDashboard
from ntcore import NetworkTableInstance
import helpers.questnav.protos.generated.commands_pb2 as cpb2
import helpers.questnav.protos.generated.geometry2d_pb2 as gpb2
import helpers.questnav.protos.generated.data_pb2 as dpb2

# --- QuestNav Class Conversion ---
class QuestNav:
    """
    The QuestNav class provides an interface to communicate with an Oculus/Meta Quest VR headset for
    robot localization and tracking purposes. It uses NetworkTables to exchange data between the
    robot and the Quest device.
    """

    def __init__(self):
        """Creates a new QuestNav implementation."""
        # Initialize NetworkTables
        self.nt4_instance = NetworkTableInstance.getDefault()
        self.quest_nav_table = self.nt4_instance.getTable("QuestNav")

        # Protobuf instances
        self.command_response_proto = cpb2.ProtobufQuestNavCommandResponse()
        self.command_proto = cpb2.ProtobufQuestNavCommand()
        self.pose2d_proto = gpb2.ProtobufPose2d()
        self.device_data_proto = dpb2.ProtobufQuestNavDeviceData()
        self.frame_data_proto = dpb2.ProtobufQuestNavFrameData()

        # Subscribers and Publishers using RawTopic for protobuf data
        # Data is sent/received as JSON strings in this mock implementation
        self.response_topic = self.quest_nav_table.getRawTopic("response")
        self.response_subscriber = self.response_topic.subscribe("proto:questnav.protos.commands.ProtobufQuestNavCommandResponse", b"")  # Subscribe to raw bytes (empty default)

        self.frame_data_topic = self.quest_nav_table.getRawTopic("frameData")
        self.frame_data_subscriber = self.frame_data_topic.subscribe("proto:questnav.protos.data.ProtobufQuestNavFrameData", b"")

        self.device_data_topic = self.quest_nav_table.getRawTopic("deviceData")
        self.device_data_subscriber = self.device_data_topic.subscribe("proto:questnav.protos.data.ProtobufQuestNavDeviceData", b"")

        self.request_topic = self.quest_nav_table.getRawTopic("request")
        self.request_publisher = self.request_topic.publish("raw")

        # Cached requests to lessen object creation (as in Java)
        self.cached_command_request = cpb2.ProtobufQuestNavCommand()
        self.cached_pose_reset_payload = cpb2.ProtobufQuestNavPoseResetPayload()
        self.cached_proto_pose = gpb2.ProtobufPose2d()

        self.last_sent_request_id = 0
        self.last_processed_response_id = 0

    def set_pose(self, pose: Pose2d):
        """
        Sets the field-relative pose of the Quest. This is the position of the Quest, not the robot.
        Make sure you correctly offset back from the center of your robot first.

        Args:
            pose: The field relative position of the Quest
        """
        # self.cached_proto_pose.Clear()
        # self.pose2d_proto.Pack(self.cached_proto_pose, pose)
        # pose_proto = gpb2.ProtobufPose2d()
        # pose_proto.translation.x = pose.translation().x
        # pose_proto.translation.y = pose.translation().y
        # pose_proto.rotation.value = pose.rotation().radians()

        # self.pose2d_proto.CopyFrom(pose_proto)

        self.cached_command_request.Clear()
        self.last_sent_request_id += 1

        self.cached_command_request.type = cpb2.QuestNavCommandType.POSE_RESET
        self.cached_command_request.command_id = self.last_sent_request_id
        payload = self.cached_command_request.pose_reset_payload
        payload.target_pose.translation.x = pose.translation().x
        payload.target_pose.translation.y = pose.translation().y
        payload.target_pose.rotation.value = pose.rotation().radians()

        self.request_publisher.set(self.cached_command_request.SerializeToString())

    def get_battery_percent(self) -> int:
        """
        Returns the Quest's battery level (0-100%), or -1 if no data is available.

        Returns:
            The battery percentage as an int, or -1 if no data is available
        """
        raw_data = self.device_data_subscriber.get()
        if not raw_data:
            return -1
        try:
            latest_device_data = dpb2.ProtobufQuestNavDeviceData.FromString(raw_data)
            return latest_device_data.battery_percent
        except Exception as e:
            return -1

    def is_tracking(self) -> bool:
        """
        Gets the current tracking state of the Quest headset.

        Returns:
            Boolean indicating if the Quest is currently tracking (true) or not (false)
        """
        raw_data = self.device_data_subscriber.get()
        if not raw_data:
            return False
        try:
            # Assuming raw_data is binary Protobuf, not JSON
            latest_device_data = dpb2.ProtobufQuestNavDeviceData.FromString(raw_data)
            # Then check a specific field for tracking state
            return bool(latest_device_data.currently_tracking)  # Or whatever field represents tracking
        except Exception as e:
            return False

    def get_frame_count(self) -> int:
        """
        Gets the current frame count from the Quest headset.

        Returns:
            The frame count value
        """
        raw_data = self.frame_data_subscriber.get()
        if not raw_data:
            return -1
        try:
            latest_frame_data = dpb2.ProtobufQuestNavFrameData.FromString(raw_data)
            return latest_frame_data.frame_count
        except Exception as e:
            return -1

    def get_tracking_lost_counter(self) -> int:
        """
        Gets the number of tracking lost events since the Quest connected to the robot.

        Returns:
            The tracking lost counter value
        """
        raw_data = self.device_data_subscriber.get()
        if not raw_data:
            return -1
        try:
            # Assuming raw_data is binary Protobuf, not JSON
            latest_device_data = dpb2.ProtobufQuestNavDeviceData.FromString(raw_data)
            # Then check a specific field for tracking state
            return latest_device_data.tracking_lost_counter  # Or whatever field represents tracking
        except Exception as e:
            return -1

    def is_connected(self) -> bool:
        """
        Determines if the Quest headset is currently connected to the robot. Connection is determined
        by how stale the last received frame from the Quest is.

        Returns:
            Boolean indicating if the Quest is connected (true) or not (false)
        """
        # NetworkTables.get_server_time_us() provides the server time in microseconds
        # entry.last_change() provides the last change time in microseconds
        current_time_us = Timer.getTimestamp()
        last_change_us = self.frame_data_subscriber.getLastChange()

        # If last_change_us is 0, it means no data has been received yet
        if last_change_us == 0:
            return False

        # Convert to milliseconds for comparison (50 ms threshold)
        latency_ms = (current_time_us - last_change_us) / 1000.0
        return latency_ms < 50.0

    def get_latency(self) -> float:
        """
        Gets the latency of the Quest > Robot Connection. Returns the latency between the current time
        and the last frame data update.

        Returns:
            The latency in milliseconds
        """
        current_time_us = Timer.getTimestamp()
        last_change_us = self.frame_data_subscriber.getLastChange()

        if last_change_us == 0:
            return -1.0  # Indicate no data

        return (current_time_us - last_change_us) / 1000.0  # Latency in milliseconds

    def get_app_timestamp(self) -> float:
        """
        Returns the Quest app's uptime timestamp. For integration with a pose estimator, use
        `get_data_timestamp()` instead!

        Returns:
            The timestamp as a double value
        """
        raw_data = self.frame_data_subscriber.get()
        if not raw_data:
            return -1
        try:
            latest_frame_data = dpb2.ProtobufQuestNavFrameData.FromString(raw_data)
            return latest_frame_data.timestamp
        except Exception as e:
            return -1

    def get_data_timestamp(self) -> float:
        """
        Gets the NT timestamp of when the last frame data was sent. This is the value which should be
        used with a pose estimator.

        Returns:
            The timestamp as a double value in seconds
        """
        # The Java code uses frameData.getAtomic().serverTime which is a NetworkTables internal timestamp.
        # In pynetworktables, the subscriber's last_change() gives the timestamp in microseconds.
        # We convert it to seconds.
        last_change_us = self.frame_data_subscriber.getLastChange()
        if last_change_us == 0:
            return -1.0
        return last_change_us / 1_000_000.0  # Convert microseconds to seconds

    def get_pose(self) -> Pose2d:
        """
        Returns the current pose of the Quest on the field. This will only return the field-relative
        pose if `set_pose(pose)` has been called at least once.

        Returns:
            Pose2d representing the Quest's location on the field
        """
        raw_data = self.frame_data_subscriber.get()
        if not raw_data:
            return Pose2d(-100, -100, -100)
        try:
            latest_frame_data = dpb2.ProtobufQuestNavFrameData.FromString(raw_data)
            # return self.pose2d_proto.unpack(latest_frame_data.pose2d)
            # print(str(latest_frame_data.pose2d.translation))
            xval = float(str(latest_frame_data.pose2d.translation)[3:str(latest_frame_data.pose2d.translation).index("\n")])
            yval = float(str(latest_frame_data.pose2d.translation)[str(latest_frame_data.pose2d.translation).index("\n") + 3:-1])
            rot = float(str(latest_frame_data.pose2d.rotation)[7:-1])
            return Pose2d(Translation2d(xval, yval), Rotation2d(rot))
        except Exception as e:
            return Pose2d(-100, -100, -100)  # Return kZero if no data available

    def command_periodic(self):
        """Cleans up QuestNav responses after processing on the headset."""
        raw_response = self.response_subscriber.get()
        if not raw_response:
            return

        latest_command_response = cpb2.ProtobufQuestNavCommandResponse.FromString(raw_response)

        # if we don't have data or for some reason the response we got isn't for the command we sent,
        # skip for this loop
        if latest_command_response.command_id != self.last_sent_request_id:
            return

        if self.last_processed_response_id != latest_command_response.command_id:
            if not latest_command_response.success:
                print(f"ERROR: QuestNav command failed!\n{latest_command_response.error_message}")
            # don't double process
            self.last_processed_response_id = latest_command_response.command_id

