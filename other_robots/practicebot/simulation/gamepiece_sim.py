import wpilib
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from simulation import sim_utils

class GamepieceSim:
    def __init__(self, field: wpilib.Field2d):
        self.field = field
        
        # Initial locations
        self.gamepiece_locations = [(2.89, 7.0), (2.89, 5.57), (2.89, 4.1), (8.28, 7.46), (8.28, 5.76), (8.28, 4.1),
                                    (8.28, 2.42), (8.28, 0.76), (13.68, 7.0), (13.68, 5.57), (13.68, 4.1)]
        self.gamepieces = [{'pos': Translation2d(gl), 'active': True} for gl in self.gamepiece_locations]
        
        self.gamepiece_obj = self.field.getObject("Gamepieces")
        self.update_field()

    def update(self, robot_pose: Pose2d):
        # Check consumption
        changed = False
        for gp in self.gamepieces:
            if gp['active']:
                if sim_utils.is_on_gamepiece(robot_pose, gp['pos']):
                    gp['active'] = False
                    print(f"Simulation consumed gamepiece at {gp['pos']}")
                    changed = True
        
        # Auto-reset if all consumed
        if len([gp for gp in self.gamepieces if gp['active']]) == 0:
            self.reset_gamepieces()
            changed = True
            
        if changed:
            self.update_field()

    def reset_gamepieces(self):
        for gp in self.gamepieces:
            gp['active'] = True
        print(f"Simulation reset all game pieces")
        self.update_field()

    def update_field(self):
        active_poses = [Pose2d(gp['pos'], Rotation2d()) for gp in self.gamepieces if gp['active']]
        self.gamepiece_obj.setPoses(active_poses)

    def get_active_gamepieces(self):
        return self.gamepieces