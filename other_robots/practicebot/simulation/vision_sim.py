import math
import wpilib
import ntcore
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import constants

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

    def _init_networktables(self):
        sim_prefix = constants.sim_prefix
        
        # FOV Show/Hide Subscribers
        self.show_fov_subs = {
            key: self.inst.getBooleanTopic(f"{sim_prefix}/FOV/{key}_show_fov").subscribe(False)
            for key in self.cam_list
        }

        # skip the rest of the publishers if we want real hardware to connect to the sim
        if constants.SimConstants.k_disable_vision:
            return

        for ix, (key, config) in enumerate(constants.k_cameras.items()):
            cam_topic = config['topic_name']
            cam_type = config.get('label', config['type'])
            base = f'/Cameras/{cam_topic}/{cam_type}'
            
            self.camera_dict[key] = {
                'offset': ix,
                'timestamp_pub': self.inst.getDoubleTopic(f"/Cameras/{cam_topic}/_timestamp").publish(),
                'targets_pub': self.inst.getDoubleTopic(f"{base}/targets").publish(),
                'distance_pub': self.inst.getDoubleTopic(f"{base}/distance").publish(),
                'strafe_pub': self.inst.getDoubleTopic(f"{base}/strafe").publish(),
                'rotation_pub': self.inst.getDoubleTopic(f"{base}/rotation").publish()
            }

    def _init_field_objects(self):
        # Pre-fetch Field2d objects for FOV visualization
        self.fov_objects = {}
        for idx, key in enumerate(self.cam_list):
            self.fov_objects[key] = self.field.getObject(f"FOV_{idx}")

    def update(self, robot_pose: Pose2d, gamepieces: list[dict]):
        now = wpilib.Timer.getFPGATimestamp()
        
        # Determine blink test state
        topic_off = None
        key_target_on = None
        if constants.SimConstants.k_do_blink_test:
            # Rotate which physical camera is disconnected (15s per camera)
            topic_off = self.physical_cameras[int(now / 15.0) % len(self.physical_cameras)]
            # Rotate which logical camera sees targets (2s per camera) - ONE ON AT A TIME
            key_target_on = self.cam_list[int(now / 2.0) % len(self.cam_list)]

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

                visible_gps = []
                if config['type'] == 'hsv':  # Only simulate gamepieces for HSV cameras
                    for gp_data in gamepieces:
                        if not gp_data['active']: continue
                        gp = gp_data['pos']
                        vec_to_gp = gp - robot_pose.translation()
                        
                        angle_robot_relative = (vec_to_gp.angle() - robot_pose.rotation()).degrees()
                        angle_robot_relative = (angle_robot_relative + 180) % 360 - 180

                        angle_cam_relative = angle_robot_relative - cam_rot
                        angle_cam_relative = (angle_cam_relative + 180) % 360 - 180

                        d = vec_to_gp.norm()
                        if abs(angle_cam_relative) < (cam_fov / 2.0) and d < constants.SimConstants.k_cam_distance_limit:
                            s = d * math.sin(math.radians(angle_cam_relative))
                            visible_gps.append({'dist': d, 'rot': angle_cam_relative, 'strafe': s})

                if visible_gps:
                    closest = min(visible_gps, key=lambda x: x['dist'])
                    targets = len(visible_gps)
                    dist = closest['dist']
                    rot = closest['rot']
                    strafe = closest['strafe']

                # 2. Apply Blink Test Overrides
                is_connected = True
                if constants.SimConstants.k_do_blink_test:
                    is_connected = (topic != topic_off)
                    if is_connected and key != key_target_on:
                        targets = 0
                
                cam_data['timestamp_pub'].set(now if is_connected else now - 5)
                cam_data['targets_pub'].set(targets if is_connected else 0)
                cam_data['distance_pub'].set(dist)
                cam_data['strafe_pub'].set(strafe)
                cam_data['rotation_pub'].set(rot)

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