# CJH trying to put fairly complex mechanism definitions outside physics.py
# 2024 1228
import wpilib
import constants


# Simplified dictionary wrapper for Mechanism2d
#UNIT: INCHES
class MechTracker:
    def __init__(self, *, length, width, height, view):
        """
        Initialize the MechTracker with dimensions and a specific view (side or top).

        :param length: Length of the mechanism.
        :param width: Width of the mechanism (for top view).
        :param height: Height of the mechanism (for side view).
        :param view: View of the mechanism ('side' or 'top').
        """
        self.length = length
        self.width = width
        self.height = height
        if view == 'side':
            self.mechanism = wpilib.Mechanism2d(length, height)
        elif view == 'top':
            self.mechanism = wpilib.Mechanism2d(length, width)
        else:
            raise ValueError("View must be 'side' or 'top'.")

        self.components = {}

    def getRoot(self, name, x, y):
        root = self.mechanism.getRoot(name, x, y)
        if name in self.components:
            raise ValueError(f"Component '{name}' already exists in the components dictionary.")
        self.components[name] = {'root': root, 'ligaments': []}
        print(f"DEBUG: Added root '{name}' at ({x}, {y})")
        return root

    def appendLigament(self, parent_name, name, length, angle, weight, color):
        if parent_name not in self.components:
            raise ValueError(f"Parent '{parent_name}' not found.")

        parent = self.components[parent_name].get('root') or self.components[parent_name].get('ligament')
        if not parent:
            raise ValueError(f"Parent '{parent_name}' does not have a valid root or ligament.")
        
        ligament = parent.appendLigament(name, length, angle, weight, wpilib.Color8Bit(color))
        self.components[parent_name]['ligaments'].append(name)
        self.components[name] = {
            'ligament': ligament,
            'ligaments': [],
            'length': length,  # Store the initial length
            'angle': angle,  # Store the initial angle
            'weight': weight,  # Store the weight
            'color': color  # Store the color
        }
        print(
            f"DEBUG: Added ligament '{name}' under parent '{parent_name}' with length {length}, angle {angle}, weight {weight}, color {color}")
        return ligament


# Initialize the mechanism trackers
length = constants.ElevatorConstants.k_window_length
width = constants.ElevatorConstants.k_window_width
height = constants.ElevatorConstants.k_window_height
front_elevator = MechTracker(length=length, width=width, height=height, view='side')
side_elevator = MechTracker(length=length, width=width, height=height, view='side')

# front view
chassis_length = 28
bar_width = 12
chassis_bottom = 2
chassis_offset = 6
chassis_base_side = front_elevator.getRoot("chassisBase", chassis_offset, chassis_bottom)
front_elevator.appendLigament("chassisBase", "chassis", chassis_length, 0, bar_width, wpilib.Color.kGray)

#elevator mechanism lengths (sim)
elevator_length = constants.ElevatorConstants.k_elevator_sim_length
elevator_width = constants.ElevatorConstants.k_elevator_sim_width
elevator_height = constants.ElevatorConstants.k_max_height
bar_width = 10  # this is a percentage of the mech screen, so scales with the mech dimensions

# shoulder length (sim)
shoulder_length = constants.ElevatorConstants.k_shoulder_length_sim

# front view of elevator mechanism
elevator_offset_x = 10
elevator_offset_y = 2 * chassis_bottom
elevator_base = front_elevator.getRoot("elevator", elevator_offset_x, elevator_offset_y)

front_elevator.appendLigament("elevator", "elevator_bottom", elevator_width, 0, bar_width, wpilib.Color.kRed)
front_elevator.appendLigament("elevator_bottom", "elevator_right", elevator_height, 90, bar_width, wpilib.Color.kRed)
front_elevator.appendLigament("elevator_right", "elevator_top", elevator_width, 90, bar_width, wpilib.Color.kRed)
front_elevator.appendLigament("elevator_top", "elevator_left", elevator_height, 90, bar_width, wpilib.Color.kRed)

# side view of elevator mechanism
chassis_base_side = side_elevator.getRoot("chassisBase", chassis_offset, chassis_bottom)
side_elevator.appendLigament("chassisBase", "chassis", chassis_length, 0, 12, wpilib.Color.kGray)

elevator_offset_x = width/2
elevator_base = side_elevator.getRoot("elevator", elevator_offset_x, elevator_offset_y)
side_elevator.appendLigament("elevator", "elevator_side", elevator_height, 90, bar_width * 1.5, wpilib.Color.kRed)
side_elevator.appendLigament("elevator_side", "shoulder", shoulder_length, constants.ElevatorConstants.k_sim_positions_degrees["l1"]["shoulder_pivot"], bar_width/3, wpilib.Color.kYellow)
side_elevator.appendLigament("shoulder", "coral_left", 6, 90, bar_width, wpilib.Color.kWhite)  # CJH added 20250218
side_elevator.appendLigament("shoulder", "coral_right", 6, -90, bar_width, wpilib.Color.kWhite)

# top view - looking down front is on the right,  right is on the bottom.  remember - angles are relativeto parent!
# ...

#side view for coral branch
small_branch_length = 12
bar_width = 1.66
bar_color = wpilib.Color.kPurple
top_stems = 15.87
branch_offset_x = length
branch_offset_y = 0
branch_base = side_elevator.getRoot("branch_base", branch_offset_x, branch_offset_y)
side_elevator.appendLigament("branch_base", "base_stem", 17.88, 90, bar_width, bar_color)
side_elevator.appendLigament("base_stem", "l1_branch", 12.84, 90, bar_width, wpilib.Color.kGray)
side_elevator.appendLigament("base_stem", "l2_stem", 5.2, 0, bar_width, bar_color)
side_elevator.appendLigament("l2_stem", "l2_branch", small_branch_length, 55, bar_width, bar_color)
side_elevator.appendLigament("l2_stem", "l3_stem", top_stems, 0, bar_width, bar_color)
side_elevator.appendLigament("l3_stem", "13_branch", small_branch_length, 55, bar_width, bar_color)
side_elevator.appendLigament("l3_stem", "l4_stem", top_stems, 0, bar_width, bar_color)
side_elevator.appendLigament("l4_stem", "bent_branch", 16.14, 55, bar_width, bar_color)
side_elevator.appendLigament("bent_branch", "top_branch", 6.8, 305, bar_width, bar_color)

# Push to SmartDashboard
wpilib.SmartDashboard.putData("Elevator Front View", front_elevator.mechanism)
wpilib.SmartDashboard.putData("Elevator Side View", side_elevator.mechanism)

# Debugging output
print("DEBUG: Elevator Front View Components:")
for name, data in front_elevator.components.items():
    print(f"side:  {name}: {data}")

# print("DEBUG: Top Mechanism Components:")
# for name, data in side_elevator.components.items():
#     print(f"top:  {name}: {data}")
