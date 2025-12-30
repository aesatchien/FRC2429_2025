import math
import wpilib
from wpilib import Color, Color8Bit
import constants

class BlockheadMech:
    """
    A class to manage the Mechanism2d visualizations for the robot.
    This replaces the script-based simmech.py and provides an API for
    updating mechanism states.
    """

    def __init__(self, num_arms=4):
        """
        Initialize the Mechanism2d objects and publish them to SmartDashboard.
        """
        # Dimensions
        self.win_len = 20
        self.win_height = 20
        
        # Create Mechanisms
        self.mech_indexer = wpilib.Mechanism2d(self.win_len, self.win_height)

        # Visual Constants
        self.bar_width = 10
        self.num_arms = num_arms
        self.arm_length = 8
        
        # Initialize Views
        self._init_indexer_view()

        # Publish to SmartDashboard
        wpilib.SmartDashboard.putData("Indexer View", self.mech_indexer)

    def _init_indexer_view(self):
        """
        Constructs the Indexer View of the robot.
        """
        # Root: Center of the window
        self.root_indexer = self.mech_indexer.getRoot("indexer", self.win_len / 2, self.win_height / 2)
        
        self.indexer_arms = []
        colors = [Color.kRed, Color.kGreen, Color.kBlue, Color.kYellow, Color.kPurple, Color.kOrange]
        
        angle_step = 360 / self.num_arms
        
        for i in range(self.num_arms):
            color = colors[i % len(colors)]
            # Initial angle is distributed around the circle
            angle = i * angle_step
            arm = self.root_indexer.appendLigament(f"arm_{i}", self.arm_length, angle, self.bar_width, Color8Bit(color))
            self.indexer_arms.append(arm)

    # ---------------- Update Methods ----------------

    def update_indexer(self, position_rotations):
        """
        Updates the indexer rotation based on physical position in rotations.
        """
        # Convert rotations to degrees
        angle_deg = position_rotations * 360
        
        angle_step = 360 / self.num_arms
        
        for i, arm in enumerate(self.indexer_arms):
            # Update angle: base offset + rotation
            # Note: Mechanism2d angles are absolute relative to parent (root), 
            # but since we want them to spin together, we can just update their angle.
            # However, appendLigament angle is relative to parent. Root has no angle.
            # So we set the angle directly.
            
            initial_angle = i * angle_step
            current_angle = initial_angle + angle_deg
            arm.setAngle(current_angle)
