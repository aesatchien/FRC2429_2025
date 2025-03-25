import math
import numpy as np
np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
import constants

class CustomTrajectory:
    def __init__(self, waypoints, duration, interpolation_type="linear", velocity_constraints=None,
                 acceleration_constraints=None, name='test'):
        self.name = name
        self.duration = duration
        self.interpolation_type = interpolation_type  # Store interpolation method
        self.time_steps = np.linspace(0, duration, int(duration / 0.01) + 1)
        self.keys = ['elevator', 'pivot', 'wrist', 'intake']
        self.servo_columns = {'intake'}  # Default servo column
        self.waypoints = dict(sorted(waypoints.items()))  # Store waypoints for later reuse
        self.trajectory = {key: [] for key in self.keys}
        self.velocity_constraints = velocity_constraints if velocity_constraints else {key: 0 for key in self.keys}
        self.acceleration_constraints = acceleration_constraints if acceleration_constraints else {key: 0 for key in
                                                                                                   self.keys}
        self.generate_trajectory()  # Generate the initial trajectory
        # self.check_constraints()

    def _smooth_trajectory_simple(self, data, window_size=9):
        """Apply a moving average filter while keeping the array size unchanged."""
        if window_size < 2:
            return data  # No smoothing if the window is too small

        # Pad edges symmetrically to prevent boundary distortion
        pad_size = window_size // 2
        padded = np.pad(data, (pad_size, pad_size), mode='edge')

        # Compute the moving average
        smoothed = np.convolve(padded, np.ones(window_size) / window_size, mode='same')

        # Properly unpad symmetrically
        return smoothed[pad_size:-pad_size]

    def _smooth_trajectory_gaussian(self, data, window_size=15, sigma=1.5):
        """Apply Gaussian smoothing while keeping the array size unchanged."""
        if window_size < 2:
            return data  # No smoothing if the window is too small

        # Create Gaussian kernel
        x = np.linspace(-1, 1, window_size)
        kernel = np.exp(-0.5 * (x / sigma) ** 2)
        kernel /= kernel.sum()  # Normalize

        # Pad the data symmetrically
        pad_size = window_size // 2
        padded = np.pad(data, (pad_size, pad_size), mode='edge')

        # Apply convolution
        smoothed = np.convolve(padded, kernel, mode='same')

        # Properly unpad symmetrically
        return smoothed[pad_size:-pad_size]

    def _cubic_hermite_interp(self, x, y, x_new):  # no scipy on the rio, but this is a crappy cubic
        """
        Approximate PCHIP behavior using NumPy.
        Computes derivatives using finite differences and performs interpolation.
        """
        # Compute finite differences
        dydx = np.zeros_like(y)
        dydx[1:-1] = (y[2:] - y[:-2]) / (x[2:] - x[:-2])  # Central differences
        dydx[0] = (y[1] - y[0]) / (x[1] - x[0])  # Forward diff
        dydx[-1] = (y[-1] - y[-2]) / (x[-1] - x[-2])  # Backward diff

        # Interpolation step
        x_new = np.asarray(x_new)
        y_new = np.zeros_like(x_new)

        for i in range(len(x) - 1):
            mask = (x_new >= x[i]) & (x_new < x[i+1])
            h = x[i+1] - x[i]
            t = (x_new[mask] - x[i]) / h

            # Hermite basis functions
            h00 = (1 + 2 * t) * (1 - t) ** 2
            h10 = t * (1 - t) ** 2
            h01 = t ** 2 * (3 - 2 * t)
            h11 = t ** 2 * (t - 1)

            y_new[mask] = (h00 * y[i] + h10 * h * dydx[i] +
                           h01 * y[i+1] + h11 * h * dydx[i+1])

        return y_new

    def generate_trajectory(self):
        times = np.array(list(self.waypoints.keys()))
        for key in self.keys:
            values = np.array([self.waypoints[t][key] for t in times]).astype(float)

            if key in self.servo_columns:
                # Servo behavior: stepwise constant values
                step_values = np.zeros_like(self.time_steps)
                for i, t in enumerate(self.time_steps):
                    step_values[i] = values[np.searchsorted(times, t, side='right') - 1]
                self.trajectory[key] = step_values.astype(float)
            else:
                # Normal interpolation using numpy
                if self.interpolation_type == "linear":
                    self.trajectory[key] = np.interp(self.time_steps, times, values).astype(float)
                elif self.interpolation_type == "smoothed":
                    self.trajectory[key] = np.interp(self.time_steps, times, values).astype(float)
                    # Apply smoothing filter
                    self.trajectory[key] = self._smooth_trajectory_simple(self.trajectory[key])
                elif self.interpolation_type == "gaussian":
                    self.trajectory[key] = np.interp(self.time_steps, times, values).astype(float)
                    # Apply smoothing filter
                    self.trajectory[key] = self._smooth_trajectory_gaussian(self.trajectory[key])
                elif self.interpolation_type == "cubic":
                    self.trajectory[key] = self._cubic_hermite_interp(times, values, self.time_steps)
                else:
                    raise ValueError(f"Unsupported interpolation type: {self.interpolation_type}")


    def set_constraints(self, velocity_constraints=None, acceleration_constraints=None):
        """ Adds constraints of the form
         velocity_constraints = {'elevator':1, 'pivot':1, 'wrist':1, 'intake'0 }
         acceleration_constraints = {'elevator':1, 'pivot':1, 'wrist':1, 'intake'0 }
        """
        if velocity_constraints:
            self.velocity_constraints = velocity_constraints
        if acceleration_constraints:
            self.acceleration_constraints = acceleration_constraints
        self.check_constraints()

    def check_constraints(self):
        """ Checks if velocity and acceleration constraints are violated and records violations """
        self.waypoint_violations = {t: {key: None for key in self.keys} for t in self.waypoints}
        self.speeds = {key: {'velocity': [], 'acceleration': []} for key in self.keys}
        for key in self.keys:
            if len(self.trajectory[key]) > 1:
                vel = np.gradient(self.trajectory[key], self.time_steps)
                acc = np.gradient(vel, self.time_steps)
                vel[0] = 0  # Ensure initial condition matches waypoint
                acc[0] = 0  # Ensure initial condition matches waypoint
                self.speeds[key]['velocity'] = vel
                self.speeds[key]['acceleration'] = acc
                for i in range(1, len(self.time_steps)):
                    for t in self.waypoints:
                        if self.time_steps[i] >= t:
                            if self.velocity_constraints[key] and abs(vel[i]) > self.velocity_constraints[key]:
                                if self.waypoint_violations[t][key] is None:
                                    self.waypoint_violations[t][key] = []
                                self.waypoint_violations[t][key].append('vel')
                            if self.acceleration_constraints[key] and abs(acc[i]) > self.acceleration_constraints[key]:
                                if self.waypoint_violations[t][key] is None:
                                    self.waypoint_violations[t][key] = []
                                self.waypoint_violations[t][key].append('acc')
                for t in self.waypoints:
                    if self.waypoint_violations[t][key]:
                        self.waypoint_violations[t][key] = set(self.waypoint_violations[t][key])

        violations_exist = any(any(v is not None for v in self.waypoint_violations[t].values()) for t in self.waypoints)
        if violations_exist:
            print("Waypoint Violations:")
            print("Time      | " + " | ".join(self.keys))
            print("-" * (12 + 4 * len(self.keys)))
            for t, violations in self.waypoint_violations.items():
                row = [f"{t:8.2f}"] + [', '.join(violations[key]) if violations[key] else "None" for key in self.keys]
                print(" | ".join(row))
        else:
            print("No Waypoint Violations:")

    def set_interpolation_type(self, new_type):
        """ Change interpolation type and regenerate trajectory """
        self.interpolation_type = new_type
        self.generate_trajectory()  # Recompute the trajectory with new method

    def get_value(self, t):
        """ Returns a dictionary of setpoint values for any given time in the trajectory """
        if t < 0: t = 0
        if t > self.duration: t = self.duration
        idx = np.searchsorted(self.time_steps, t)
        if idx == 0:
            return {key: float(self.trajectory[key][0]) for key in self.keys}
        elif idx >= len(self.time_steps):
            return {key: float(self.trajectory[key][-1]) for key in self.keys}
        else:
            t1, t2 = self.time_steps[idx - 1], self.time_steps[idx]
            values = {}
            for key in self.keys:
                if key in self.servo_columns:
                    values[key] = float(self.trajectory[key][idx])
                else:
                    v1, v2 = self.trajectory[key][idx - 1], self.trajectory[key][idx]
                    values[key] = float(v1 + (v2 - v1) * (t - t1) / (t2 - t1))
            return values

    def check_trajectory(self, wrist_safe_pivot=(-70, 70), wrist_safe_elevator=1):
        """ See if trajectory ever violates wrist safety constraints"""
        collisions = []
        for i, t in enumerate(self.time_steps):
            pivot, elevator, wrist = self.trajectory['pivot'][i], self.trajectory['elevator'][i], \
            self.trajectory['wrist'][i]
            if wrist != 0 and not (
                    pivot < wrist_safe_pivot[0] or pivot > wrist_safe_pivot[1] or elevator > wrist_safe_elevator):
                collisions.append(t)
        return collisions

    def fix_trajectory(self, wrist_safe_pivot=(-70, 70), wrist_safe_elevator=1):
        """ Adjusts the trajectory to ensure the wrist does not move until it is safe.
            NOTE - does not work yet  """
        fixed_trajectory = {key: self.trajectory[key].copy().astype(float) for key in self.keys}

        for i, t in enumerate(self.time_steps):
            pivot, elevator, wrist = fixed_trajectory['pivot'][i], fixed_trajectory['elevator'][i], \
            fixed_trajectory['wrist'][i]
            if wrist != 0 and not (
                    pivot < wrist_safe_pivot[0] or pivot > wrist_safe_pivot[1] or elevator > wrist_safe_elevator):
                fixed_trajectory['wrist'][i] = fixed_trajectory['wrist'][i - 1] if i > 0 else 0.0

        self.trajectory = fixed_trajectory

    def rescale_trajectory(self, new_duration):
        """ Rescales the trajectory to fit a new duration while maintaining relative timing of waypoints. """
        scale_factor = new_duration / self.duration
        new_waypoints = {t * scale_factor: v for t, v in self.waypoints.items()}
        self.duration = new_duration
        self.time_steps = np.linspace(0, new_duration, int(new_duration / 0.01) + 1)
        self.waypoints = dict(sorted(new_waypoints.items()))
        self.generate_trajectory()

# make a test trajectory, call it L3
waypoints = {
    0: {'elevator': 0.21, 'pivot': 90, 'wrist': 0, 'intake': 2},  # start
    0.25: {'elevator': 0.3, 'pivot': 90, 'wrist': 0, 'intake': 2},  # start
    0.5: {'elevator': 0.5, 'pivot': 70, 'wrist': 0, 'intake': 2},  # get to safe wrist
    1: {'elevator': 1.2, 'pivot': 50, 'wrist': 90, 'intake': 2},  # get to scoring wrist while raising elevator
    1.5: {'elevator': 1.17, 'pivot': 40, 'wrist': 90, 'intake': -3},  # move pivot while scoring
    2.5: {'elevator': 0.2, 'pivot': 70, 'wrist': 0, 'intake': 0},  # return home with wrist safe
    3.0: {'elevator': 0.2, 'pivot': 90, 'wrist': 0, 'intake': 0},  # come down to bottom
}
velocity_constraints = {'elevator':1.5, 'pivot':1000, 'wrist':90, 'intake':0 }
acceleration_constraints = {'elevator': 20, 'pivot':1000, 'wrist':180, 'intake':0 }
# linear, pchip, cubic
trajectory_L3 = CustomTrajectory(waypoints, duration=3, interpolation_type="smoothed",
                              velocity_constraints=velocity_constraints, acceleration_constraints=acceleration_constraints)
# initialize waypoints for dashboard commands
waypoints = {
    0: {'elevator': 0.21, 'pivot': 90, 'wrist': 0, 'intake': 0},  # start
    1: {'elevator': 1.1, 'pivot': 90, 'wrist': 0, 'intake': 0},  # get to scoring wrist
    2: {'elevator': 0.8, 'pivot': 50, 'wrist': 90, 'intake': 3},  # return home with wrist safe
}

score_waypoint_dict = { #TODO: READ CURRENT WRIST POSITION!
    'l2': {
        0: {
            'elevator': constants.k_positions["l2"]["elevator"] + 0.035,
            'pivot': constants.k_positions["l2"]["shoulder_pivot"],
            'wrist': constants.k_positions["l2"]["wrist_pivot"],
            'intake': 0
        },
        0.4: {
            'elevator': constants.k_positions["l2"]["elevator"] + 0.035,
            'pivot': math.degrees(constants.k_positions["l2"]["shoulder_pivot"]) + 7.25,
            'wrist': constants.k_positions["l2"]["wrist_pivot"],
            'intake': 1
        },
        0.8: {
            'elevator': constants.k_positions["l2"]["elevator"] + 0.035,
            'pivot': math.degrees(constants.k_positions["l2"]["shoulder_pivot"]) + 14.5,
            'wrist': constants.k_positions["l2"]["wrist_pivot"],
            'intake': 2
        }
    },

    'l3': {
        0: {
            'elevator': constants.k_positions["l3"]["elevator"] + 0.035,
            'pivot': constants.k_positions["l3"]["shoulder_pivot"],
            'wrist': constants.k_positions["l3"]["wrist_pivot"],
            'intake': 0
        },
        0.4: {
            'elevator': constants.k_positions["l3"]["elevator"] + 0.035,
            'pivot': math.degrees(constants.k_positions["l3"]["shoulder_pivot"]) + 7.25,
            'wrist': constants.k_positions["l3"]["wrist_pivot"],
            'intake': 1
        },
        0.8: {
            'elevator': constants.k_positions["l3"]["elevator"] + 0.035,
            'pivot': math.degrees(constants.k_positions["l3"]["shoulder_pivot"]) + 14.5,
            'wrist': constants.k_positions["l3"]["wrist_pivot"],
            'intake': 2
        }
    },

    'l4': {
        0: {
            'elevator': constants.k_positions["l4"]["elevator"] + 0.035,
            'pivot': constants.k_positions["l4"]["shoulder_pivot"],
            'wrist': constants.k_positions["l4"]["wrist_pivot"],
            'intake': 0
        },
        0.4: {
            'elevator': constants.k_positions["l4"]["elevator"] + 0.035,
            'pivot': math.degrees(constants.k_positions["l4"]["shoulder_pivot"]) + 5.0,
            'wrist': constants.k_positions["l4"]["wrist_pivot"],
            'intake': 1
        },
        0.8: {
            'elevator': constants.k_positions["l4"]["elevator"] + 0.035,
            'pivot': math.degrees(constants.k_positions["l4"]["shoulder_pivot"]) + 10.0,
            'wrist': constants.k_positions["l4"]["wrist_pivot"],
            'intake': 2
        }
    }
}
