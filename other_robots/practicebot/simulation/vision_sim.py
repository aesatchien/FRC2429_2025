import math
import wpilib
import ntcore
from ntcore import PubSubOptions
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import constants
from helpers import apriltag_utils

class VisionSim:
    def __init__(self, field: wpilib.Field2d):
        self.field = field
        self.inst = ntcore.NetworkTableInstance.getDefault()
        
        # Configuration
        self.cam_list = list(constants.k_cameras.keys())
        self.physical_cameras = sorted(list(set(c['topic_name'] for c in constants.k_cameras.values())))

        self.camera_dict = {}
        self._init_networktables()
        self._init_field_objects()
        self._init_apriltags()

    def _init_networktables(self):
        sim_prefix = constants.sim_prefix
        
        # FOV Show/Hide Subscribers
        self.show_fov_subs = {
            key: self.inst.getBooleanTopic(f"{sim_prefix}/FOV/{key}_show_fov").subscribe(False)
            for key in self.cam_list
        }

        # skip the rest of the publishers if we want real hardware to connect to the sim
        if constants.SimConstants.k_disable_vision_sim:
            return

        # note this will be skipped if we told it to disable vision - so update will loop through nothing
        for ix, (key, config) in enumerate(constants.k_cameras.items()):
            cam_topic = config['topic_name']
            cam_type = config.get('label', config['type'])
            base = f'/Cameras/{cam_topic}/{cam_type}'
            
            self.camera_dict[key] = {
                'offset': ix,
                'frames': 0,
                'frames_pub': self.inst.getIntegerTopic(f"/Cameras/{cam_topic}/_frames").publish(),
                'targets_pub': self.inst.getDoubleTopic(f"{base}/targets").publish(PubSubOptions(keepDuplicates=True)),  # otherwise targets get stale
                'distance_pub': self.inst.getDoubleTopic(f"{base}/distance").publish(),
                'strafe_pub': self.inst.getDoubleTopic(f"{base}/strafe").publish(),
                'rotation_pub': self.inst.getDoubleTopic(f"{base}/rotation").publish()
            }

    def _init_field_objects(self):
        # Pre-fetch Field2d objects for FOV visualization
        self.fov_objects = {}
        for idx, key in enumerate(self.cam_list):
            self.fov_objects[key] = self.field.getObject(f"FOV_{idx}")

    def _init_apriltags(self):
        self.tag_translations = []
        self.tag_poses = []
        
        # Load tags from the layout utility
        for tag in apriltag_utils.layout.getTags():
            pose3d = apriltag_utils.layout.getTagPose(tag.ID)
            if pose3d is not None:
                pose2d = pose3d.toPose2d()
                self.tag_poses.append(pose2d)
                self.tag_translations.append(pose2d.translation())
        
        self.field.getObject("AprilTags").setPoses(self.tag_poses)


    def update(self, robot_pose: Pose2d, gamepieces: list[dict]):
        now = wpilib.Timer.getFPGATimestamp()

        # 1. Manual Override: Use External Cameras
        # If True: Do not simulate targets, do not blink test. Just draw FOV.
        if constants.SimConstants.k_use_external_cameras:
            for key in self.cam_list:
                self._update_fov_visualization(key, robot_pose, constants.k_cameras[key], self.show_fov_subs[key].get())
            return

        # 2. Blink Test
        if constants.SimConstants.k_do_blink_test:
            self._run_blink_test(now)
            return

        # 3. Normal Simulation
        for key in self.cam_list:
            config = constants.k_cameras[key]

            if key in self.camera_dict:
                cam_data = self.camera_dict[key]
                topic = config['topic_name']
                cam_rot = config.get('rotation', 0)
                cam_fov = config.get('fov', 90)

                # 1. Calculate Real Visibility & Data
                targets = 0
                dist = 0
                rot = 0
                strafe = 0

                visible_targets = []
                
                # Determine candidates based on camera type
                candidates = []
                if config['type'] == 'hsv':
                    candidates = [{'pos': gp['pos'], 'facing': None} for gp in gamepieces if gp['active']]
                elif config['type'] == 'tags':
                    candidates = [{'pos': p.translation(), 'facing': p.rotation()} for p in self.tag_poses]

                # Unified detection logic
                for candidate in candidates:
                    target_pos = candidate['pos']
                    target_facing = candidate['facing']

                    # Check tag visibility angle if applicable
                    if target_facing is not None:
                        # Vector from tag to robot
                        vec_tag_to_robot = robot_pose.translation() - target_pos
                        angle_to_robot = vec_tag_to_robot.angle()
                        
                        # Angle difference between tag normal and vector to robot
                        angle_diff = (angle_to_robot - target_facing).degrees()
                        angle_diff = (angle_diff + 180) % 360 - 180
                        
                        if abs(angle_diff) > constants.SimConstants.k_tag_visibility_angle:
                            continue

                    vec_to_target = target_pos - robot_pose.translation()
                    
                    angle_robot_relative = (vec_to_target.angle() - robot_pose.rotation()).degrees()
                    angle_robot_relative = (angle_robot_relative + 180) % 360 - 180

                    angle_cam_relative = angle_robot_relative - cam_rot
                    angle_cam_relative = (angle_cam_relative + 180) % 360 - 180

                    d = vec_to_target.norm()
                    if abs(angle_cam_relative) < (cam_fov / 2.0) and d < constants.SimConstants.k_cam_distance_limit:
                        s = d * math.sin(math.radians(angle_cam_relative))
                        visible_targets.append({'dist': d, 'rot': angle_cam_relative, 'strafe': s})

                if visible_targets:
                    closest = min(visible_targets, key=lambda x: x['dist'])
                    targets = len(visible_targets)
                    dist = closest['dist']
                    rot = closest['rot']
                    strafe = closest['strafe']

                cam_data['frames'] += 1
                cam_data['frames_pub'].set(cam_data['frames'])
                cam_data['targets_pub'].set(targets)
                cam_data['distance_pub'].set(dist)
                cam_data['strafe_pub'].set(strafe)
                cam_data['rotation_pub'].set(rot)

                # NOTE: We do NOT publish to the 'poses' topic here with the other data.
                # This ensures that swerve_sim.py (which listens for live tags) doesn't
                # get confused and try to snap the robot to these simulated targets.

            # 3. Update FOV Visualization
            self._update_fov_visualization(key, robot_pose, config, self.show_fov_subs[key].get())

    def _update_fov_visualization(self, key, robot_pose, config, show_fov):
        if not constants.SimConstants.k_draw_camera_fovs or not show_fov:
            self.fov_objects[key].setPoses([])
            return

        robot_pos = robot_pose.translation()
        robot_rot_rad = robot_pose.rotation().radians()
        fov_dist = constants.SimConstants.k_cam_distance_limit

        cam_rot_rad = math.radians(config.get('rotation', 0))
        cam_fov_rad = math.radians(config.get('fov', 90))

        left_edge_angle = robot_rot_rad + cam_rot_rad - (cam_fov_rad / 2)
        right_edge_angle = robot_rot_rad + cam_rot_rad + (cam_fov_rad / 2)

        p1 = robot_pos
        p2 = robot_pos + Translation2d(fov_dist * math.cos(left_edge_angle), fov_dist * math.sin(left_edge_angle))
        p3 = robot_pos + Translation2d(fov_dist * math.cos(right_edge_angle), fov_dist * math.sin(right_edge_angle))

        self.fov_objects[key].setPoses([Pose2d(p1, Rotation2d()), Pose2d(p2, Rotation2d()), Pose2d(p3, Rotation2d())])

    def _run_blink_test(self, now):
        # 60s cycle for camera disconnection
        num_physical = len(self.physical_cameras)
        if num_physical > 0:
            time_per_cam = 60.0 / num_physical
            off_index = int(now / time_per_cam) % num_physical
            topic_off = self.physical_cameras[off_index]
        else:
            topic_off = None

        # Cycle through logical cameras one at a time (2s per camera)
        if len(self.cam_list) > 0:
            key_target_on = self.cam_list[int(now / 2.0) % len(self.cam_list)]
        else:
            key_target_on = None

        for key in self.cam_list:
            if key not in self.camera_dict:
                continue

            cam_data = self.camera_dict[key]
            config = constants.k_cameras[key]
            topic = config['topic_name']

            is_connected = (topic != topic_off)


            pub_list = [cam_data['targets_pub'], cam_data['distance_pub'],
                        cam_data['strafe_pub'], cam_data['rotation_pub']]

            if is_connected:
                cam_data['frames'] += 1
                cam_data['frames_pub'].set(cam_data['frames'])

                if key == key_target_on:  # set all the target data to the position in the list
                    data = cam_data['offset'] + 1
                    for pub in pub_list:
                        pub.set(data)
                else:
                    for pub in pub_list:  # set all the target data to 0
                        pub.set(0)
            else:
                # Disconnected: freeze frames, clear targets
                for pub in pub_list:  # set all the target data to 0
                    pub.set(0)