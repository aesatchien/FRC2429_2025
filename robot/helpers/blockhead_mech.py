import math
import wpilib
from wpilib import Color, Color8Bit
import constants

class BlockheadMech:
    """
    A class to manage the Mechanism2d visualizations for the robot.
    This replaces the script-based simmech.py and provides an API for
    updating mechanism states (elevator height, shoulder angle, etc.).
    """

    def __init__(self):
        """
        Initialize the Mechanism2d objects and publish them to SmartDashboard.
        This method sets up the canvas dimensions and creates the two main views:
        1. Front View: Shows the elevator moving up and down.
        2. Side View: Shows the elevator, shoulder, climber, and field elements.
        """
        # Dimensions
        self.win_len = constants.ElevatorConstants.k_window_length
        self.win_height = constants.ElevatorConstants.k_window_height
        
        # Create Mechanisms
        self.mech_front = wpilib.Mechanism2d(self.win_len, self.win_height)
        self.mech_side = wpilib.Mechanism2d(self.win_len, self.win_height)

        # Visual Constants
        self.chassis_len = 28
        self.chassis_bottom = 2
        self.chassis_offset = 6
        self.bar_width = 10
        
        # Elevator Constants
        self.elev_width = constants.ElevatorConstants.k_elevator_sim_width
        self.elev_max_height = constants.ElevatorConstants.k_max_height
        self.elev_sim_max_height = constants.ElevatorConstants.k_elevator_sim_max_height
        
        # Initialize Views
        self._init_front_view()
        self._init_side_view()

        # Publish to SmartDashboard
        wpilib.SmartDashboard.putData("Elevator Front View", self.mech_front)
        wpilib.SmartDashboard.putData("Elevator Side View", self.mech_side)

    def _init_front_view(self):
        """
        Constructs the Front View of the robot.
        This view visualizes the elevator carriage as a box that expands/contracts vertically.
        It is useful for seeing the elevator's height relative to the chassis.
        """
        # Root: Chassis
        root_chassis = self.mech_front.getRoot("chassisBase", self.chassis_offset, self.chassis_bottom)
        root_chassis.appendLigament("chassis", self.chassis_len, 0, 12, Color8Bit(Color.kGray))

        # Root: Elevator
        # elevator_offset_x = 10, elevator_offset_y = 2 * chassis_bottom (4)
        root_elev = self.mech_front.getRoot("elevator", 10, self.chassis_bottom * 2)
        
        # Build the box
        # Bottom (Static)
        self.lig_front_elev_bottom = root_elev.appendLigament("elevator_bottom", self.elev_width, 0, self.bar_width, Color8Bit(Color.kRed))
        
        # Right (Vertical - Variable Length)
        self.lig_front_elev_right = self.lig_front_elev_bottom.appendLigament("elevator_right", self.elev_max_height, 90, self.bar_width, Color8Bit(Color.kRed))
        
        # Top (Horizontal - Attached to Right)
        self.lig_front_elev_top = self.lig_front_elev_right.appendLigament("elevator_top", self.elev_width, 90, self.bar_width, Color8Bit(Color.kRed))
        
        # Left (Vertical - Attached to Top - Variable Length)
        self.lig_front_elev_left = self.lig_front_elev_top.appendLigament("elevator_left", self.elev_max_height, 90, self.bar_width, Color8Bit(Color.kRed))

    def _init_side_view(self):
        """
        Constructs the Side View of the robot.
        This view is more detailed and includes:
        - The Chassis base
        - The Climber mechanism
        - The Elevator (side profile)
        - The Shoulder and Coral manipulator
        - Static guide bars for reference
        """
        # Root: Chassis
        root_chassis = self.mech_side.getRoot("chassisBase", self.chassis_offset, self.chassis_bottom)
        self.lig_side_chassis = root_chassis.appendLigament("chassis", self.chassis_len, 0, 12, Color8Bit(Color.kGray))

        # Climber (Attached to Chassis Base)
        # Note: In simmech, climber_base was attached to chassisBase root, but here we can attach to the ligament or root.
        # simmech: climber_base = side_elevator.appendLigament("chassisBase", "climber_base", ...)
        # Since chassisBase is a root, we append to the root's existing ligament or create a new one? 
        # Mechanism2d roots can have multiple ligaments.
        self.lig_climber_base = self.lig_side_chassis.appendLigament("climber_base", 5, 90, 12, Color8Bit(Color.kGray))
        self.lig_climber = self.lig_climber_base.appendLigament("climber", constants.ClimberConstants.k_length_meters * 39.3701, 90, 12, Color8Bit(Color.kGray))

        # Root: Elevator Main
        elev_offset_x = self.win_len / 2
        elev_offset_y = self.chassis_bottom * 2
        root_elev = self.mech_side.getRoot("elevator", elev_offset_x, elev_offset_y)
        
        # Elevator Side (Vertical - Variable Length)
        self.lig_side_elev = root_elev.appendLigament("elevator_side", self.elev_max_height, 90, self.bar_width * 1.5, Color8Bit(Color.kRed))
        
        # Shoulder (Attached to Elevator Side)
        shoulder_len = constants.ElevatorConstants.k_shoulder_length_sim
        self.lig_shoulder = self.lig_side_elev.appendLigament("shoulder", shoulder_len, constants.k_positions["l1"]["shoulder_pivot"], self.bar_width/3, Color8Bit(Color.kYellow))
        
        # Coral (Attached to Shoulder)
        self.lig_coral = self.lig_shoulder.appendLigament("coral_left", 6, 90, self.bar_width, Color8Bit(Color.kWhite))

        # Elevator Bars (Visual guides)
        root_bar1 = self.mech_side.getRoot("elevator_bar1", elev_offset_x - 5, elev_offset_y)
        self.lig_bar_left = root_bar1.appendLigament("elevator_bar_left", self.elev_max_height + 5, 90, self.bar_width/3, Color8Bit(Color.kGreen))
        self.lig_bar_left.appendLigament("elevator_bar_top", 10, 270, self.bar_width/3, Color8Bit(Color.kGreen))
        
        root_bar2 = self.mech_side.getRoot("elevator_bar2", elev_offset_x + 5, elev_offset_y)
        self.lig_bar_right = root_bar2.appendLigament("elevator_bar_right", self.elev_max_height + 5, 90, self.bar_width/3, Color8Bit(Color.kGreen))

        # Coral Branch (Static Field Element)
        self._init_coral_branch()

    def _init_coral_branch(self):
        """
        Draws the static Coral Branch (Reef) field element in the Side View.
        This helps visualize where the robot is relative to the scoring targets.
        """
        # Side view for coral branch
        small_branch_length = 12
        branch_width = 1.66
        branch_color = Color8Bit(Color.kPurple)
        top_stems = 15.87
        
        # Positioned at the far right of the window
        root_branch = self.mech_side.getRoot("branch_base", self.win_len, 0)
        
        base_stem = root_branch.appendLigament("base_stem", 17.88, 90, branch_width, branch_color)
        base_stem.appendLigament("l1_branch", 12.84, 90, branch_width, Color8Bit(Color.kGray))
        
        l2_stem = base_stem.appendLigament("l2_stem", 5.2, 0, branch_width, branch_color)
        l2_stem.appendLigament("l2_branch", small_branch_length, 55, branch_width, branch_color)
        
        l3_stem = l2_stem.appendLigament("l3_stem", top_stems, 0, branch_width, branch_color)
        l3_stem.appendLigament("13_branch", small_branch_length, 55, branch_width, branch_color)
        
        l4_stem = l3_stem.appendLigament("l4_stem", top_stems, 0, branch_width, branch_color)
        bent_branch = l4_stem.appendLigament("bent_branch", 16.14, 55, branch_width, branch_color)
        bent_branch.appendLigament("top_branch", 6.8, 305, branch_width, branch_color)

    # ---------------- Update Methods ----------------

    def update_elevator(self, height_meters):
        """Updates the elevator length based on physical height in meters."""
        # Scale meters to simulation units
        sim_height = height_meters * (self.elev_sim_max_height / self.elev_max_height)
        
        # Update Front View
        self.lig_front_elev_right.setLength(sim_height)
        self.lig_front_elev_left.setLength(sim_height)
        
        # Update Side View
        self.lig_side_elev.setLength(sim_height)
        self.lig_bar_left.setLength(sim_height + 5)
        self.lig_bar_right.setLength(sim_height + 5)

    def update_shoulder(self, angle_radians):
        """Updates the shoulder angle based on physical angle in radians."""
        # Physics calc: 180 - degrees(angle) - 90
        # Simplified: 90 - degrees(angle)
        angle_deg = math.degrees(angle_radians)
        visual_angle = 90 - angle_deg
        self.lig_shoulder.setAngle(visual_angle)

    def update_climber(self, angle_radians):
        """Updates the climber angle based on physical angle in radians."""
        # Physics calc: degrees(angle) - 90
        angle_deg = math.degrees(angle_radians)
        visual_angle = angle_deg - 90
        self.lig_climber.setAngle(visual_angle)